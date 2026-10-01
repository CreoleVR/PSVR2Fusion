// Gives the PSVR2 its own SteamVR room setup in the lighthouse universe.
//
// The tracker puts the PSVR2 in the lighthouse universe, but Sony's driver
// still points SteamVR at its own room setup file. This repoints the headset
// at config\psvr2fusion\chaperone_info.vrchap (a sibling of Sony's folder) and
// pins the universe id, so the room setup stays put and the lighthouse
// universe never takes over another headset's room.
//
// SteamVR reads that file but can't save into it, so Room Setup changes last
// one session; edit the file to change the play area for good.
#pragma once

#include "settings.h"

#include <string>

namespace psvr2fusion {

class RoomSetup {
public:
    // Call periodically while the PSVR2 is the HMD.
    void Enforce(const Settings &settings);

private:
    std::string isolatedPath_;
    bool failed_ = false;
};

} // namespace psvr2fusion
