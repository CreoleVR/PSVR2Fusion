// The PSVR2's raw IMU, read from inside Sony's SteamVR driver.
//
// Sony's driver (or PSVR2 Toolkit's wrapper of it) reads the headset's status
// interface on its own USB thread. A hook on that thread's ReadPipe sees every
// transfer; the ones from the status thread carry the 2 kHz IMU records,
// which are integrated on the spot. Nothing is sent to the headset.
#pragma once

#include "quat.h"

#include <string>

namespace psvr2fusion {

namespace ImuSource {

// Starts attaching to Sony's driver (in the background; it may not be loaded
// yet). Does nothing if already started.
void Start();

// Removes the hook. Safe to call when not started.
void Stop();

// The newest IMU orientation, predicted forward to now + predictSeconds.
// False while samples aren't arriving.
bool Read(double predictSeconds, Quat *orientation, Vec3 *angularVelocity, double *ageSeconds);

// One-line state and statistics for the log.
std::string Status();

} // namespace ImuSource

} // namespace psvr2fusion
