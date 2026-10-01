// Tracker + IMU orientation fusion.
//
// The tracker gives an absolute pose in the lighthouse universe at a few
// hundred Hz. The PSVR2's gyro gives orientation at 2 kHz, but in its own
// drifting frame. A complementary filter joins them: the rotation that maps
// the IMU's frame onto the tracker's ("offset") follows the tracker with time
// constant tau, so the gyro carries everything faster than tau and the
// tracker everything slower. Position and linear velocity are the tracker's.
//
// The tracker is taken to sit on the headset with an identity offset: its
// pose is the head pose.
#pragma once

#include "quat.h"

#include <cmath>
#include <cstdint>

namespace psvr2fusion {

struct TrackerPose {
    bool valid = false;
    Vec3 position;        // m
    Quat orientation;
    Vec3 velocity;        // m/s
    Vec3 angularVelocity; // rad/s, world frame
};

struct ImuPose {
    bool fresh = false;   // samples are arriving
    Quat orientation;     // in the IMU's own world
    Vec3 angularVelocity; // rad/s, in the IMU's own world
};

struct HeadPose {
    bool valid = false;
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity; // rad/s, world frame
};

class Fusion {
public:
    bool useImu = true;   // false: tracker only
    double tauSeconds = 1.0;

    HeadPose Step(const TrackerPose &tracker, const ImuPose &imu, int64_t nowNs) {
        HeadPose out;
        if (!tracker.valid) return out;
        out.valid = true;
        out.position = tracker.position;
        out.velocity = tracker.velocity;

        if (!useImu || !imu.fresh) {
            out.orientation = Normalize(tracker.orientation);
            out.angularVelocity = tracker.angularVelocity;
            return out;
        }

        Quat target = Normalize(Mul(tracker.orientation, Conj(imu.orientation)));
        if (!haveOffset_) {
            offset_ = target;
            haveOffset_ = true;
        } else {
            double dt = std::fmax(0.0, (nowNs - lastNs_) / 1e9);
            double tau = tauSeconds > 0.0 ? tauSeconds : 1.0;
            offset_ = Normalize(Slerp(offset_, target, 1.0 - std::exp(-dt / tau)));
        }
        lastNs_ = nowNs;

        out.orientation = Normalize(Mul(offset_, imu.orientation));
        out.angularVelocity = Rotate(offset_, imu.angularVelocity);
        residualDeg_ = AngleDeg(Mul(tracker.orientation, Conj(out.orientation)));
        return out;
    }

    // Snap to the tracker on the next fused step (new IMU stream or tracker).
    void Reset() { haveOffset_ = false; }

    // Angle between the fused and the tracker orientation at the last step.
    double ResidualDeg() const { return residualDeg_; }

private:
    Quat offset_;
    bool haveOffset_ = false;
    int64_t lastNs_ = 0;
    double residualDeg_ = 0.0;
};

} // namespace psvr2fusion
