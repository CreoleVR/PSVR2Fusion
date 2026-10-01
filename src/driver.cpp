// PSVR2Fusion: makes the PSVR2 a lighthouse-tracked SteamVR headset.
//
// A Vive tracker (or any SteamVR-tracked device) is mounted on the PSVR2.
// This driver adds a virtual tracker, psvr2fusion0, whose pose is that
// tracker's pose with orientation fused with the PSVR2's own 2 kHz IMU, and a
// SteamVR tracking override puts that pose on the PSVR2 headset. Sony's
// driver keeps doing everything else (display, controllers, eye tracking).
#include "fusion.h"
#include "imu_source.h"
#include "log.h"
#include "room_setup.h"
#include "settings.h"

#include <openvr_driver.h>
#include <windows.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

namespace psvr2fusion {

using Clock = std::chrono::steady_clock;

constexpr char kSerial[] = "psvr2fusion0";
constexpr char kOverrideFrom[] = "/devices/psvr2fusion/psvr2fusion0";
constexpr char kOverrideTo[] = "/devices/playstation_vr2/playstation_vr2_headset";
constexpr uint32_t kMaxDevices = vr::k_unMaxTrackedDeviceCount;

static bool IEquals(const std::string &a, const char *b) { return _stricmp(a.c_str(), b) == 0; }

enum class Hmd { None, Psvr2, Other };

// What SteamVR's HMD (device 0) is.
static Hmd ActiveHmd() {
    vr::CVRPropertyHelpers *props = vr::VRProperties();
    vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
    if (c == vr::k_ulInvalidPropertyContainer) return Hmd::None;
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    int32_t cls = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
    if (err != vr::TrackedProp_Success || cls != vr::TrackedDeviceClass_HMD) return Hmd::None;
    std::string serial = props->GetStringProperty(c, vr::Prop_SerialNumber_String);
    std::string system = props->GetStringProperty(c, vr::Prop_TrackingSystemName_String);
    if (IEquals(serial, "playstation_vr2_headset") || IEquals(system, "playstation_vr2")) return Hmd::Psvr2;
    return Hmd::Other;
}

// The device with this serial (any kind but an HMD), or -1.
static int FindDevice(const std::string &serial) {
    if (serial.empty()) return -1;
    vr::CVRPropertyHelpers *props = vr::VRProperties();
    for (uint32_t i = 0; i < kMaxDevices; ++i) {
        vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(i);
        if (c == vr::k_ulInvalidPropertyContainer) continue;
        if (props->GetInt32Property(c, vr::Prop_DeviceClass_Int32) == vr::TrackedDeviceClass_HMD) continue;
        if (IEquals(props->GetStringProperty(c, vr::Prop_SerialNumber_String), serial.c_str())) return (int)i;
    }
    return -1;
}

static vr::DriverPose_t InvalidPose() {
    vr::DriverPose_t p{};
    p.qWorldFromDriverRotation.w = 1.0;
    p.qDriverFromHeadRotation.w = 1.0;
    p.qRotation.w = 1.0;
    p.result = vr::TrackingResult_Running_OutOfRange;
    p.poseIsValid = false;
    p.deviceIsConnected = true;
    return p;
}

static vr::DriverPose_t ToDriverPose(const HeadPose &h) {
    vr::DriverPose_t p = InvalidPose();
    if (!h.valid) return p;
    p.vecPosition[0] = h.position.x;
    p.vecPosition[1] = h.position.y;
    p.vecPosition[2] = h.position.z;
    p.vecVelocity[0] = h.velocity.x;
    p.vecVelocity[1] = h.velocity.y;
    p.vecVelocity[2] = h.velocity.z;
    Quat q = Normalize(h.orientation);
    p.qRotation = {q.w, q.x, q.y, q.z};
    Vec3 w = Rotate(Conj(q), h.angularVelocity); // OpenVR wants it in the head frame
    p.vecAngularVelocity[0] = w.x;
    p.vecAngularVelocity[1] = w.y;
    p.vecAngularVelocity[2] = w.z;
    p.result = vr::TrackingResult_Running_OK;
    p.poseIsValid = true;
    return p;
}

static TrackerPose ToTrackerPose(const vr::TrackedDevicePose_t &p) {
    TrackerPose t;
    t.valid = true;
    const auto &m = p.mDeviceToAbsoluteTracking.m;
    t.position = {m[0][3], m[1][3], m[2][3]};
    t.orientation = FromMatrix(m);
    t.velocity = {p.vVelocity.v[0], p.vVelocity.v[1], p.vVelocity.v[2]};
    t.angularVelocity = {p.vAngularVelocity.v[0], p.vAngularVelocity.v[1], p.vAngularVelocity.v[2]};
    return t;
}

// Sleeps to a deadline with sub-millisecond precision where Windows allows.
class Sleeper {
public:
    Sleeper() { timer_ = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS); }
    ~Sleeper() {
        if (timer_) CloseHandle(timer_);
    }
    void Until(Clock::time_point deadline) {
        auto left = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - Clock::now()).count();
        if (left <= 0) return;
        if (timer_) {
            LARGE_INTEGER due;
            due.QuadPart = -(left / 100 > 0 ? left / 100 : 1); // relative, 100 ns units
            if (SetWaitableTimerEx(timer_, &due, 0, nullptr, nullptr, nullptr, 0)) {
                WaitForSingleObject(timer_, INFINITE);
                return;
            }
        }
        std::this_thread::sleep_until(deadline);
    }

private:
    HANDLE timer_ = nullptr;
};

class HeadTracker : public vr::ITrackedDeviceServerDriver {
public:
    vr::EVRInitError Activate(uint32_t id) override {
        if (ActiveHmd() != Hmd::Psvr2) {
            Log("PSVR2 is not the active HMD; declining virtual tracker activation");
            return vr::VRInitError_Driver_HmdUnknown;
        }
        id_ = id;
        settings_ = LoadSettings();

        vr::CVRPropertyHelpers *props = vr::VRProperties();
        vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(id);
        props->SetStringProperty(c, vr::Prop_TrackingSystemName_String, "psvr2fusion");
        props->SetStringProperty(c, vr::Prop_ModelNumber_String, "psvr2fusion virtual head");
        props->SetStringProperty(c, vr::Prop_SerialNumber_String, kSerial);
        props->SetStringProperty(c, vr::Prop_ManufacturerName_String, "psvr2fusion");
        props->SetStringProperty(c, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
        props->SetStringProperty(c, vr::Prop_ControllerType_String, "vive_tracker");
        props->SetStringProperty(c, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        props->SetInt32Property(c, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
        props->SetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
        props->SetBoolProperty(c, vr::Prop_WillDriftInYaw_Bool, false);
        props->SetBoolProperty(c, vr::Prop_DeviceIsWireless_Bool, true);
        props->SetBoolProperty(c, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        props->SetBoolProperty(c, vr::Prop_Identifiable_Bool, false);

        Log("activate id=%u tracker='%s'%s fusion=%d tau=%.2f hz=%d (identity tracker offset; no Sony pose fallback)", id,
            settings_.trackerSerial.c_str(), SerialSource().c_str(), settings_.fusion ? 1 : 0, settings_.tauSeconds,
            settings_.updateHz);
        if (settings_.trackerSerial.empty()) NoTrackerHint();
        if (settings_.fusion) ImuSource::Start();
        else Log("'fusion' is false -> passing the tracker pose straight through.");

        running_ = true;
        thread_ = std::thread([this] { Run(); });
        return vr::VRInitError_None;
    }

    void Deactivate() override { Stop(); }

    void EnterStandby() override {}

    void *GetComponent(const char *) override { return nullptr; }

    void DebugRequest(const char *, char *response, uint32_t size) override {
        if (size) response[0] = '\0';
    }

    vr::DriverPose_t GetPose() override {
        std::lock_guard<std::mutex> lock(poseMutex_);
        return pose_;
    }

    void Stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        ImuSource::Stop();
    }

private:
    std::string SerialSource() const {
        return settings_.trackerSerialSource.empty() ? "" : " (from " + settings_.trackerSerialSource + ")";
    }

    void NoTrackerHint() const {
        std::wstring path = TrackerFilePath();
        Log("no tracker set: put the serial of the tracker on the headset in %ls", path.c_str());
    }

    void Publish(const vr::DriverPose_t &pose) {
        {
            std::lock_guard<std::mutex> lock(poseMutex_);
            pose_ = pose;
        }
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose, sizeof(pose));
    }

    // tracker.txt was edited: switch trackers without restarting SteamVR.
    void ReloadSerial() {
        Settings fresh = LoadSettings();
        if (fresh.trackerSerial == settings_.trackerSerial) return;
        settings_.trackerSerial = fresh.trackerSerial;
        settings_.trackerSerialSource = fresh.trackerSerialSource;
        trackerIndex_ = -1;
        fusion_.Reset();
        Log("tracker is now '%s'%s", settings_.trackerSerial.c_str(), SerialSource().c_str());
        if (settings_.trackerSerial.empty()) NoTrackerHint();
    }

    void LogDevices(const vr::TrackedDevicePose_t *poses, bool trackerValid) {
        vr::CVRPropertyHelpers *props = vr::VRProperties();
        std::string line = "seen devices:";
        for (uint32_t i = 0; i < kMaxDevices; ++i) {
            vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(i);
            if (c == vr::k_ulInvalidPropertyContainer) continue;
            vr::ETrackedPropertyError err = vr::TrackedProp_Success;
            int32_t cls = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
            if (err != vr::TrackedProp_Success || cls == vr::TrackedDeviceClass_Invalid) continue;
            line += Format(" [%u cls=%d '%s' conn=%d valid=%d res=%d]", i, cls,
                           props->GetStringProperty(c, vr::Prop_SerialNumber_String).c_str(),
                           poses[i].bDeviceIsConnected ? 1 : 0, poses[i].bPoseIsValid ? 1 : 0,
                           (int)poses[i].eTrackingResult);
        }
        Log("%s", line.c_str());
        Log("want tracker '%s' -> idx=%d pose-valid=%d%s", settings_.trackerSerial.c_str(), trackerIndex_,
            trackerValid ? 1 : 0, trackerValid ? "" : " (no Sony pose fallback: the headset has no pose until it's seen)");
    }

    void Run() {
        const auto period = std::chrono::nanoseconds(1000000000LL / settings_.updateHz);
        const auto never = Clock::now() - std::chrono::hours(24);
        auto next = Clock::now();
        auto nextRoomCheck = next;
        auto nextFileCheck = next + std::chrono::seconds(1);
        auto lastDevicesLog = never;
        auto lastStatusLog = never;
        bool lastTrackerValid = true;
        bool imuLive = false;
        double imuAge = 0.0;

        fusion_.useImu = settings_.fusion;
        fusion_.tauSeconds = settings_.tauSeconds;

        RoomSetup room;
        TrackerFileWatcher trackerFile;
        Sleeper sleeper;
        vr::TrackedDevicePose_t poses[kMaxDevices];

        while (running_) {
            next += period;

            if (ActiveHmd() != Hmd::Psvr2) {
                Publish(InvalidPose());
                ImuSource::Stop();
                Log("PSVR2 is no longer active; stopped fusion and IMU");
                return;
            }

            auto now = Clock::now();
            if (now >= nextRoomCheck) {
                room.Enforce(settings_);
                nextRoomCheck = now + std::chrono::milliseconds(100);
            }
            if (now >= nextFileCheck) {
                nextFileCheck = now + std::chrono::seconds(1);
                if (trackerFile.Changed()) ReloadSerial();
            }

            if (trackerIndex_ < 0) trackerIndex_ = FindDevice(settings_.trackerSerial);
            vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, kMaxDevices);

            TrackerPose tracker;
            int trackerResult = -1;
            if (trackerIndex_ >= 0) {
                const vr::TrackedDevicePose_t &p = poses[trackerIndex_];
                trackerResult = (int)p.eTrackingResult;
                if (!p.bDeviceIsConnected) {
                    trackerIndex_ = -1; // look for it again, it may come back elsewhere
                } else if (p.bPoseIsValid) {
                    tracker = ToTrackerPose(p);
                }
            }

            now = Clock::now();
            if (tracker.valid != lastTrackerValid ||
                (!tracker.valid && now - lastDevicesLog > std::chrono::seconds(3))) {
                LogDevices(poses, tracker.valid);
                lastDevicesLog = now;
                lastTrackerValid = tracker.valid;
            }
            if (now - lastStatusLog > std::chrono::seconds(10)) {
                Log("imu %s | mode=%s age=%.2fms resid=%.2fdeg trackerResult=%d", ImuSource::Status().c_str(),
                    imuLive ? "rawimu" : "tracker-passthrough", imuAge * 1000.0, fusion_.ResidualDeg(),
                    trackerResult);
                lastStatusLog = now;
            }

            ImuPose imu;
            if (settings_.fusion) {
                imu.fresh = ImuSource::Read(0.0, &imu.orientation, &imu.angularVelocity, &imuAge);
                if (imu.fresh && !imuLive) {
                    Log("raw PSVR2 IMU is live -> %s", ImuSource::Status().c_str());
                    imuLive = true;
                    fusion_.Reset();
                } else if (!imu.fresh && imuLive) {
                    Log("raw PSVR2 IMU unavailable (%s) -> tracker pose passthrough", ImuSource::Status().c_str());
                    imuLive = false;
                }
            }

            int64_t nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
            Publish(ToDriverPose(fusion_.Step(tracker, imu, nowNs)));

            now = Clock::now();
            if (next < now) next = now; // fell behind: don't burst to catch up
            sleeper.Until(next);
        }
    }

    uint32_t id_ = vr::k_unTrackedDeviceIndexInvalid;
    Settings settings_;
    Fusion fusion_;
    int trackerIndex_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex poseMutex_;
    vr::DriverPose_t pose_ = InvalidPose();
};

class Provider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext *context) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(context);
        Log("Init");

        // Put psvr2fusion0's pose on the PSVR2 headset, and only on it.
        vr::EVRSettingsError err = vr::VRSettingsError_None;
        char current[256] = {};
        vr::VRSettings()->GetString(vr::k_pch_TrackingOverride_Section, kOverrideFrom, current, sizeof(current), &err);
        if (err != vr::VRSettingsError_None || std::strcmp(current, kOverrideTo) != 0) {
            err = vr::VRSettingsError_None;
            vr::VRSettings()->SetString(vr::k_pch_TrackingOverride_Section, kOverrideFrom, kOverrideTo, &err);
            if (err != vr::VRSettingsError_None) {
                Log("cannot scope tracking override to PSVR2 (%d); refusing activation", (int)err);
                return vr::VRInitError_None;
            }
            Log("tracking override targets only %s", kOverrideTo);
        }

        if (ActiveHmd() == Hmd::Other) {
            Log("another HMD is active; not registering a virtual tracker");
            return vr::VRInitError_None;
        }

        tracker_ = std::make_unique<HeadTracker>();
        if (!vr::VRServerDriverHost()->TrackedDeviceAdded(kSerial, vr::TrackedDeviceClass_GenericTracker, tracker_.get()))
            Log("TrackedDeviceAdded failed");
        return vr::VRInitError_None;
    }

    void Cleanup() override {
        if (tracker_) tracker_->Stop();
        ImuSource::Stop();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    void RunFrame() override {}
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    std::unique_ptr<HeadTracker> tracker_;
};

static Provider g_provider;

} // namespace psvr2fusion

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *interfaceName, int *returnCode) {
    if (std::strcmp(interfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) return &psvr2fusion::g_provider;
    if (returnCode) *returnCode = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
