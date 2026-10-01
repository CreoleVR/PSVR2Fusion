#include "imu_source.h"

#include "imu.h"
#include "log.h"

#include <MinHook.h>
#include <windows.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <type_traits>

namespace psvr2fusion {
namespace ImuSource {

// Sony's driver_playstation_vr2.dll, the build PSVR2 Toolkit supports (the
// toolkit hooks the same addresses). Checked against the code bytes below
// before anything is touched.
constexpr size_t kUsbManagerGetterRva = 0x124C90; // CaesarManager singleton getter
constexpr size_t kReadPipeRva = 0x127D60;         // CaesarUsbThread::ReadPipe
constexpr size_t kStatusThreadOffset = 0xB8;      // CaesarManager -> status/IMU USB thread
constexpr uint8_t kUsbManagerGetterCode[] = {0x48, 0x83, 0xEC, 0x28, 0x48, 0x8B, 0x05}; // sub rsp,28h; mov rax,[rip+x]
constexpr uint8_t kReadPipeCode[] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x30};               // push rbx; sub rsp,30h

// Minimum samples before the IMU counts as live; and how stale the newest
// sample may get before it stops counting (with hysteresis).
constexpr uint32_t kMinSamples = 64;
constexpr double kFreshAge = 0.25;
constexpr double kStaleAge = 1.0;

// Single-writer sequence lock: the USB thread publishes, the pose thread reads
// without ever blocking it.
template <typename T> class SeqLock {
    static_assert(std::is_trivially_copyable<T>::value && sizeof(T) % 8 == 0, "");
    static constexpr size_t kWords = sizeof(T) / 8;

public:
    void Store(const T &value) {
        uint64_t w[kWords];
        std::memcpy(w, &value, sizeof(T));
        uint32_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        for (size_t i = 0; i < kWords; ++i) words_[i].store(w[i], std::memory_order_relaxed);
        seq_.store(s + 2, std::memory_order_release);
    }

    T Load() const {
        uint64_t w[kWords];
        for (;;) {
            uint32_t s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1) continue;
            for (size_t i = 0; i < kWords; ++i) w[i] = words_[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == s1) break;
        }
        T value;
        std::memcpy(&value, w, sizeof(T));
        return value;
    }

private:
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint64_t> words_[kWords] = {};
};

struct Sample {
    Quat orientation;
    Vec3 angularVelocity;
    int64_t qpc; // when it was measured, QueryPerformanceCounter ticks
};

using ReadPipeFn = int (*)(void *self, uint8_t pipeId, char *buffer, size_t length);

static double g_qpcSeconds = 0.0; // seconds per QPC tick
static double g_qpcTicks = 0.0;   // QPC ticks per second

static std::mutex g_lifecycle; // Start/Stop
static std::thread g_attachThread;
static std::atomic<bool> g_attaching{false};
static std::atomic<bool> g_hooked{false};
static void *g_target = nullptr;
static ReadPipeFn g_original = nullptr;
static std::atomic<void *> g_statusThread{nullptr};
static std::atomic<int> g_inFlight{0};

static std::mutex g_stateMutex;
static std::string g_state = "not started";

// Written by the USB thread only.
static GyroIntegrator g_integrator;
static SeqLock<Sample> g_sample;
static std::atomic<uint32_t> g_accepted{0};
static std::atomic<uint64_t> g_packets{0};
static std::atomic<uint64_t> g_samples{0};
static std::atomic<uint64_t> g_rejected{0};
static std::atomic<bool> g_sawPacket{false};
static std::atomic<int> g_packetBytes{0};
static std::atomic<int> g_packetRecords{0};
static std::atomic<int> g_pipeId{0};
static std::atomic<double> g_rateHz{0.0};
static int64_t g_rateQpc = 0;
static uint64_t g_rateSamples = 0;

static std::atomic<bool> g_fast{false};

static int64_t Qpc() {
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

static void SetState(const std::string &state) {
    {
        std::lock_guard<std::mutex> lock(g_stateMutex);
        if (g_state == state) return;
        g_state = state;
    }
    Log("imu: %s", state.c_str());
}

static void OnStatusTransfer(const uint8_t *buffer, int bytes) {
    int count = ImuRecordCount(bytes);
    if (count <= 0) return;
    int64_t now = Qpc();

    for (int i = 0; i < count; ++i) {
        GyroIntegrator::Result r;
        if (!g_integrator.Add(ReadImuRecord(buffer, i), &r)) {
            g_rejected.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        if (g_accepted.load(std::memory_order_relaxed) < kMinSamples) g_accepted.fetch_add(1);
        // A transfer holds the newest samples, 0.5 ms apart.
        int64_t age = (int64_t)((count - 1 - i) * kImuPeriodSeconds * g_qpcTicks);
        g_sample.Store({r.orientation, r.angularVelocity, now - age});
    }

    uint64_t samples = g_samples.fetch_add(count) + count;
    g_packets.fetch_add(1);
    if (!g_sawPacket.load()) {
        g_packetBytes = bytes;
        g_packetRecords = count;
        g_sawPacket = true;
    }
    if (g_rateQpc != 0) {
        double elapsed = (now - g_rateQpc) * g_qpcSeconds;
        if (elapsed < 1.0) return;
        g_rateHz = (samples - g_rateSamples) / elapsed;
    }
    g_rateSamples = samples;
    g_rateQpc = now;
}

static int ReadPipeHook(void *self, uint8_t pipeId, char *buffer, size_t length) {
    g_inFlight.fetch_add(1);
    int bytes = g_original(self, pipeId, buffer, length);
    if (bytes > 0 && buffer && self == g_statusThread.load(std::memory_order_relaxed) &&
        ImuRecordCount(bytes) > 0) {
        if (!g_sawPacket.load()) g_pipeId = pipeId;
        OnStatusTransfer(reinterpret_cast<const uint8_t *>(buffer), bytes);
    }
    g_inFlight.fetch_sub(1);
    return bytes;
}

// A jump someone (PSVR2 Toolkit) already put at the start of the function.
static bool LooksHooked(const uint8_t *p) {
    if (p[0] == 0xE9 || p[0] == 0xEB) return true;                 // jmp rel32 / rel8
    if (p[0] == 0xFF && p[1] == 0x25) return true;                  // jmp [rip+x]
    return p[0] == 0x48 && p[1] == 0xB8 && p[10] == 0xFF && p[11] == 0xE0; // mov rax, imm64; jmp rax
}

static bool InImage(HMODULE module, size_t rva, size_t size) {
    auto dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(module);
    auto nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(reinterpret_cast<const uint8_t *>(module) + dos->e_lfanew);
    return rva + size <= nt->OptionalHeader.SizeOfImage;
}

static void AttachLoop() {
    using Clock = std::chrono::steady_clock;
    bool waitingForToolkit = false;
    Clock::time_point toolkitWaitStart;

    while (g_attaching.load()) {
        // With PSVR2 Toolkit installed, Sony's DLL is renamed and loaded by
        // the toolkit's own driver_playstation_vr2.dll.
        HMODULE module = GetModuleHandleW(L"driver_playstation_vr2_orig.dll");
        if (!module) module = GetModuleHandleW(L"driver_playstation_vr2_orig.dll.dll");
        bool toolkit = module != nullptr;
        if (!module) module = GetModuleHandleW(L"driver_playstation_vr2.dll");
        if (!module) {
            SetState("waiting for Sony's PSVR2 driver to load");
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        auto base = reinterpret_cast<uint8_t *>(module);
        if (!InImage(module, kUsbManagerGetterRva, 16) || !InImage(module, kReadPipeRva, 16) ||
            std::memcmp(base + kUsbManagerGetterRva, kUsbManagerGetterCode, sizeof(kUsbManagerGetterCode)) != 0) {
            SetState(Format("Sony driver build not recognised (signature mismatch at +0x%zX) - raw IMU disabled, "
                            "falling back to the tracker pose",
                            kUsbManagerGetterRva));
            return;
        }

        uint8_t *readPipe = base + kReadPipeRva;
        if (toolkit) {
            // Let the toolkit hook ReadPipe first, then chain in front of it.
            if (!LooksHooked(readPipe)) {
                if (!waitingForToolkit) {
                    SetState("waiting for PSVR2Toolkit to install its USB hooks");
                    waitingForToolkit = true;
                    toolkitWaitStart = Clock::now();
                }
                if (Clock::now() - toolkitWaitStart < std::chrono::seconds(30)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
            }
        } else if (std::memcmp(readPipe, kReadPipeCode, sizeof(kReadPipeCode)) != 0 && !LooksHooked(readPipe)) {
            SetState("ReadPipe prologue not recognised - raw IMU disabled");
            return;
        }

        auto getManager = reinterpret_cast<uint8_t **(*)()>(base + kUsbManagerGetterRva);
        uint8_t **manager = getManager();
        if (!manager || !*manager) {
            SetState("waiting for the PSVR2 USB manager to come up");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        void *statusThread = *reinterpret_cast<void **>(reinterpret_cast<uint8_t *>(manager) + kStatusThreadOffset);
        if (!statusThread) {
            SetState("waiting for the PSVR2 IMU/status USB thread to start");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        g_statusThread = statusThread;
        g_target = readPipe;
        MH_STATUS st = MH_Initialize();
        if (st != MH_OK && st != MH_ERROR_ALREADY_INITIALIZED) {
            SetState(Format("MinHook init failed (%d) - raw IMU disabled", (int)st));
            return;
        }
        st = MH_CreateHook(g_target, reinterpret_cast<void *>(&ReadPipeHook), reinterpret_cast<void **>(&g_original));
        if (st != MH_OK) {
            SetState(Format("MH_CreateHook failed (%d) - raw IMU disabled", (int)st));
            return;
        }
        st = MH_EnableHook(g_target);
        if (st != MH_OK) {
            MH_RemoveHook(g_target);
            SetState(Format("MH_EnableHook failed (%d) - raw IMU disabled", (int)st));
            return;
        }
        g_hooked = true;
        SetState(Format("attached to %s at +0x%zX%s", toolkit ? "PSVR2Toolkit's USB transport" : "Sony's USB transport",
                        kReadPipeRva, toolkit ? " (chained in front of the toolkit)" : ""));
        return;
    }
}

void Start() {
    std::lock_guard<std::mutex> lock(g_lifecycle);
    if (g_attaching.exchange(true)) return;
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    g_qpcTicks = (double)f.QuadPart;
    g_qpcSeconds = 1.0 / g_qpcTicks;
    SetState("attaching");
    g_attachThread = std::thread(AttachLoop);
}

void Stop() {
    std::lock_guard<std::mutex> lock(g_lifecycle);
    if (!g_attaching.exchange(false)) return;
    if (g_attachThread.joinable()) g_attachThread.join();
    if (g_hooked.exchange(false)) {
        MH_DisableHook(g_target);
        // Let a ReadPipe that's inside the hook finish before freeing it.
        for (int i = 0; i < 200 && g_inFlight.load() != 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        MH_RemoveHook(g_target);
        MH_Uninitialize();
    }
    SetState("stopped");
}

// Samples are arriving: the newest is younger than 0.25 s to become live,
// and stays live until it is older than 1 s.
static bool Fresh(Sample *sample) {
    if (!g_hooked.load() || g_accepted.load() < kMinSamples) return false;
    *sample = g_sample.Load();
    if (sample->qpc == 0) return false;
    double age = (Qpc() - sample->qpc) * g_qpcSeconds;
    bool fresh = age < (g_fast.load() ? kStaleAge : kFreshAge);
    g_fast = fresh;
    return fresh;
}

bool Read(double predictSeconds, Quat *orientation, Vec3 *angularVelocity, double *ageSeconds) {
    Sample s;
    if (!Fresh(&s)) return false;
    double age = (Qpc() - s.qpc) * g_qpcSeconds;
    if (ageSeconds) *ageSeconds = age;
    // Carry the newest sample forward to now at its angular velocity.
    double dt = std::fmin(0.15, std::fmax(-0.05, age + predictSeconds));
    Quat q = s.orientation;
    if (dt != 0.0) q = Normalize(Mul(Integrate(s.angularVelocity, dt), q));
    *orientation = q;
    *angularVelocity = s.angularVelocity;
    return true;
}

std::string Status() {
    std::string state;
    {
        std::lock_guard<std::mutex> lock(g_stateMutex);
        state = g_state;
    }
    if (!g_hooked.load()) return state;
    Sample s;
    bool live = Fresh(&s);
    return Format("%s | %llu packets, %llu samples (%.0f Hz), %llu rejected; packet=%d bytes = 32 + %d x 24 on pipe "
                  "0x%02X; %s",
                  state.c_str(), (unsigned long long)g_packets.load(), (unsigned long long)g_samples.load(),
                  g_rateHz.load(), (unsigned long long)g_rejected.load(), g_packetBytes.load(), g_packetRecords.load(),
                  g_pipeId.load(), live ? "LIVE" : "no samples yet");
}

} // namespace ImuSource
} // namespace psvr2fusion
