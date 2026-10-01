// The PSVR2's raw IMU, as it arrives on the headset's status interface.
//
// Sony's driver reads USB interface 7 (endpoint 0x88) with interrupt
// transfers. Each transfer is a 32-byte status header followed by IMU records
// of 24 bytes, sampled at 2 kHz. The record layout, scales and axes are the
// ones Monado's psvr2 driver uses (src/xrt/drivers/psvr2/psvr2_protocol.h),
// which give the gyro and accelerometer in the head frame OpenVR expects.
//
// This file has no Windows or OpenVR dependencies so it can be tested alone.
#pragma once

#include "quat.h"

#include <cstdint>
#include <cstring>

namespace psvr2fusion {

#pragma pack(push, 1)
struct ImuRecord {
    uint32_t vtsUs;     // device timestamp, microseconds
    int16_t accel[3];   // raw sensor axes
    int16_t gyro[3];    // raw sensor axes
    uint16_t dpFrameCount;
    uint16_t dpLineCount;
    uint16_t imuTsUs;
    uint16_t status;
};
#pragma pack(pop)
static_assert(sizeof(ImuRecord) == 24, "PSVR2 IMU records are 24 bytes");

constexpr int kStatusHeaderBytes = 32;
constexpr double kImuPeriodSeconds = 1.0 / 2000.0;
constexpr double kGyroScale = 2000.0 / 32767.0 * kDegToRad; // rad/s per LSB
constexpr double kAccelScale = 4.0 * 9.80665 / 32767.0;      // m/s^2 per LSB
constexpr double kMaxGyro = 40.0;                             // rad/s, above is a bad sample
constexpr double kMaxAccel = 8.0 * 9.80665;                   // m/s^2, above is a bad sample

// Number of IMU records in a status transfer of this size, or 0 if the size
// doesn't fit the status layout (so it isn't a status transfer).
inline int ImuRecordCount(int bytes) {
    int payload = bytes - kStatusHeaderBytes;
    if (payload < (int)sizeof(ImuRecord) || payload % (int)sizeof(ImuRecord) != 0) return 0;
    return payload / (int)sizeof(ImuRecord);
}

inline Vec3 GyroOf(const ImuRecord &r) {
    return {-r.gyro[1] * kGyroScale, r.gyro[2] * kGyroScale, -r.gyro[0] * kGyroScale};
}

inline Vec3 AccelOf(const ImuRecord &r) {
    return {-r.accel[1] * kAccelScale, r.accel[2] * kAccelScale, -r.accel[0] * kAccelScale};
}

// Integrates the gyro into an orientation. Its world frame is wherever the
// headset pointed at the first sample; the fusion step maps it onto the
// tracker's world. The accelerometer is only used to reject bad samples.
class GyroIntegrator {
public:
    struct Result {
        Quat orientation;     // head orientation in the IMU's world
        Vec3 angularVelocity; // rad/s, in the IMU's world
    };

    // Adds one record. Returns false (and changes nothing) if it is rejected.
    bool Add(const ImuRecord &r, Result *out) {
        Vec3 w = GyroOf(r);
        if (!(Norm(w) < kMaxGyro) || !(Norm(AccelOf(r)) < kMaxAccel)) return false;

        double dt = 0.0;
        if (haveTime_) dt = (uint32_t)(r.vtsUs - lastVtsUs_) * 1e-6;
        lastVtsUs_ = r.vtsUs;
        haveTime_ = true;

        // A gap of 100 ms or more means samples were lost; don't integrate
        // across it.
        if (dt > 0.0 && dt < 0.1) q_ = Normalize(Mul(q_, Integrate(w, dt)));

        out->orientation = q_;
        out->angularVelocity = Rotate(q_, w);
        return true;
    }

    void Reset() { *this = GyroIntegrator(); }

private:
    Quat q_;
    uint32_t lastVtsUs_ = 0;
    bool haveTime_ = false;
};

inline ImuRecord ReadImuRecord(const uint8_t *transfer, int index) {
    ImuRecord r;
    std::memcpy(&r, transfer + kStatusHeaderBytes + index * sizeof(ImuRecord), sizeof(r));
    return r;
}

} // namespace psvr2fusion
