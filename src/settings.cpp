#include "settings.h"

#include <openvr_driver.h>

#include <cstdlib>
#include <fstream>

namespace psvr2fusion {

static const char *const kSection = "driver_psvr2fusion";

static std::wstring ModuleDir() {
    HMODULE self = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&ModuleDir), &self);
    wchar_t path[MAX_PATH];
    DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
    std::wstring dir(path, n);
    size_t slash = dir.find_last_of(L"\\/");
    return slash == std::wstring::npos ? L"." : dir.substr(0, slash);
}

std::wstring TrackerFilePath() { return ModuleDir() + L"\\tracker.txt"; }

static std::string Trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r\n\xEF\xBB\xBF"); // also a UTF-8 BOM
    if (a == std::string::npos) return {};
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool ReadTrackerFile(std::string *serial) {
    std::ifstream f(TrackerFilePath());
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#' || line.rfind("//", 0) == 0) continue;
        *serial = line;
        return true;
    }
    return false;
}

TrackerFileWatcher::TrackerFileWatcher() { Changed(); }

bool TrackerFileWatcher::Changed() {
    WIN32_FILE_ATTRIBUTE_DATA data;
    bool exists = GetFileAttributesExW(TrackerFilePath().c_str(), GetFileExInfoStandard, &data) != 0;
    FILETIME write = exists ? data.ftLastWriteTime : FILETIME{};
    bool changed = exists != exists_ || CompareFileTime(&write, &lastWrite_) != 0;
    exists_ = exists;
    lastWrite_ = write;
    return changed;
}

static bool GetBool(const char *key, bool fallback) {
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    bool v = vr::VRSettings()->GetBool(kSection, key, &err);
    return err == vr::VRSettingsError_None ? v : fallback;
}

static std::string GetString(const char *key, const std::string &fallback) {
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    char buf[256] = {};
    vr::VRSettings()->GetString(kSection, key, buf, sizeof(buf), &err);
    return err == vr::VRSettingsError_None ? std::string(buf) : fallback;
}

Settings LoadSettings() {
    Settings s;
    s.fusion = GetBool("fusion", s.fusion);
    s.isolateRoomSetup = GetBool("isolateRoomSetup", s.isolateRoomSetup);

    std::string universe = GetString("roomUniverseId", std::to_string(s.roomUniverseId));
    s.roomUniverseId = std::strtoull(universe.c_str(), nullptr, 10);

    vr::EVRSettingsError err = vr::VRSettingsError_None;
    float tau = vr::VRSettings()->GetFloat(kSection, "tauSeconds", &err);
    if (err == vr::VRSettingsError_None) s.tauSeconds = tau;

    err = vr::VRSettingsError_None;
    int hz = vr::VRSettings()->GetInt32(kSection, "updateHz", &err);
    if (err == vr::VRSettingsError_None) s.updateHz = hz;
    if (s.updateHz < 30) s.updateHz = 30;
    if (s.updateHz > 1000) s.updateHz = 1000;

    if (ReadTrackerFile(&s.trackerSerial)) {
        s.trackerSerialSource = "tracker.txt";
    } else {
        s.trackerSerial = GetString("trackerSerial", "");
        if (!s.trackerSerial.empty()) s.trackerSerialSource = "steamvr.vrsettings";
    }
    return s;
}

} // namespace psvr2fusion
