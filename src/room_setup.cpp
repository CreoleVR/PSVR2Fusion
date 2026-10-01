#include "room_setup.h"

#include "log.h"

#include <openvr_driver.h>

#include <filesystem>
#include <system_error>

namespace psvr2fusion {

namespace fs = std::filesystem;

void RoomSetup::Enforce(const Settings &settings) {
    if (!settings.isolateRoomSetup || failed_) return;

    vr::CVRPropertyHelpers *props = vr::VRProperties();
    vr::PropertyContainerHandle_t hmd = props->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
    if (hmd == vr::k_ulInvalidPropertyContainer) return;

    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    std::string current = props->GetStringProperty(hmd, vr::Prop_DriverProvidedChaperonePath_String, &err);

    if (isolatedPath_.empty()) {
        // Sony's driver hasn't named its room setup file yet.
        if (current.empty()) return;
        fs::path dir = fs::u8path(current).parent_path();
        if (dir.filename() != "psvr2fusion") dir = dir.parent_path() / "psvr2fusion";
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) {
            Log("could not create isolated room-setup directory '%s' (%d)", dir.u8string().c_str(), ec.value());
            failed_ = true;
            return;
        }
        isolatedPath_ = (dir / "chaperone_info.vrchap").u8string();
    }

    if (current != isolatedPath_) {
        props->SetStringProperty(hmd, vr::Prop_DriverProvidedChaperonePath_String, isolatedPath_.c_str());
        props->SetBoolProperty(hmd, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
        Log("isolated SteamVR room setup at '%s'", isolatedPath_.c_str());
    }

    if (settings.roomUniverseId != 0) {
        err = vr::TrackedProp_Success;
        uint64_t universe = props->GetUint64Property(hmd, vr::Prop_CurrentUniverseId_Uint64, &err);
        if (err != vr::TrackedProp_Success) universe = 0;
        if (universe != settings.roomUniverseId) {
            props->SetUint64Property(hmd, vr::Prop_CurrentUniverseId_Uint64, settings.roomUniverseId);
            Log("pinned Lighthouse room universe %llu", (unsigned long long)settings.roomUniverseId);
        }
    }
}

} // namespace psvr2fusion
