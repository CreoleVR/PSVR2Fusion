// Settings: the tracker serial from tracker.txt next to the DLL, everything
// else (and the serial too, if there is no tracker.txt) from the
// "driver_psvr2fusion" section of steamvr.vrsettings.
#pragma once

#include <windows.h>

#include <cstdint>
#include <string>

namespace psvr2fusion {

struct Settings {
    std::string trackerSerial;       // the tracker mounted on the headset
    std::string trackerSerialSource; // where trackerSerial came from, for the log
    bool fusion = true;              // fuse with the PSVR2's raw IMU (false: tracker only)
    bool isolateRoomSetup = true;    // keep the PSVR2's room setup in its own file
    uint64_t roomUniverseId = 1788272693;
    double tauSeconds = 1.0;         // how fast the tracker corrects the gyro
    int updateHz = 1000;             // pose updates per second (30-1000)
};

Settings LoadSettings();

// <folder of driver_psvr2fusion.dll>\tracker.txt
std::wstring TrackerFilePath();

// The serial in tracker.txt: its first line that isn't empty or a comment.
// False if the file is missing or names no tracker.
bool ReadTrackerFile(std::string *serial);

// Notices edits to tracker.txt.
class TrackerFileWatcher {
public:
    TrackerFileWatcher();
    bool Changed(); // true once after each change (including creation/removal)

private:
    FILETIME lastWrite_{};
    bool exists_ = false;
};

} // namespace psvr2fusion
