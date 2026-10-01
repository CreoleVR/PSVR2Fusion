// Tests for the parts that don't need SteamVR: IMU parsing, gyro integration
// and the fusion filter.
#include "fusion.h"
#include "imu.h"
#include "quat.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace psvr2fusion;

static int g_failures = 0;

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                               \
            ++g_failures;                                                                                              \
        }                                                                                                              \
    } while (0)

static bool Near(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

static Quat AxisAngle(Vec3 axis, double deg) {
    double n = Norm(axis);
    double h = deg * kDegToRad / 2;
    return {std::cos(h), axis.x / n * std::sin(h), axis.y / n * std::sin(h), axis.z / n * std::sin(h)};
}

// Angle between two orientations, degrees.
static double Between(const Quat &a, const Quat &b) { return AngleDeg(Mul(a, Conj(b))); }

static ImuRecord Record(uint32_t vtsUs, Vec3 gyroHead) {
    // Inverse of GyroOf: head (x, y, z) = (-g1, g2, -g0).
    ImuRecord r{};
    r.vtsUs = vtsUs;
    r.gyro[0] = (int16_t)std::lround(-gyroHead.z / kGyroScale);
    r.gyro[1] = (int16_t)std::lround(-gyroHead.x / kGyroScale);
    r.gyro[2] = (int16_t)std::lround(gyroHead.y / kGyroScale);
    r.accel[2] = (int16_t)std::lround(9.80665 / kAccelScale); // 1 g up
    return r;
}

static void TestRecordCount() {
    CHECK(ImuRecordCount(32) == 0);
    CHECK(ImuRecordCount(32 + 24) == 1);
    CHECK(ImuRecordCount(32 + 48) == 2);
    CHECK(ImuRecordCount(32 + 24 * 41) == 41);
    CHECK(ImuRecordCount(33 + 24) == 0);
    CHECK(ImuRecordCount(1024) == 0);
    CHECK(ImuRecordCount(0) == 0);
    CHECK(ImuRecordCount(-5) == 0);
}

static void TestAxes() {
    ImuRecord r{};
    r.gyro[0] = 100;
    r.gyro[1] = 200;
    r.gyro[2] = 300;
    r.accel[0] = 10;
    r.accel[1] = 20;
    r.accel[2] = 30;
    Vec3 g = GyroOf(r), a = AccelOf(r);
    CHECK(Near(g.x, -200 * kGyroScale, 1e-12) && Near(g.y, 300 * kGyroScale, 1e-12) && Near(g.z, -100 * kGyroScale, 1e-12));
    CHECK(Near(a.x, -20 * kAccelScale, 1e-12) && Near(a.y, 30 * kAccelScale, 1e-12) && Near(a.z, -10 * kAccelScale, 1e-12));
    CHECK(Near(kGyroScale * 32767, 2000 * kDegToRad, 1e-9));
}

static void TestIntegration() {
    // 90 deg/s about +Y (yaw) for 1 s at 2 kHz.
    GyroIntegrator imu;
    GyroIntegrator::Result r;
    Vec3 w = {0, 90 * kDegToRad, 0};
    for (int i = 0; i <= 2000; ++i) CHECK(imu.Add(Record(1000000u + i * 500u, w), &r));
    CHECK(Near(Between(r.orientation, AxisAngle({0, 1, 0}, 90)), 0, 0.05));
    CHECK(Near(r.angularVelocity.y, 90 * kDegToRad, 1e-3));

    // The device clock wraps without a hiccup.
    GyroIntegrator wrap;
    for (int i = 0; i <= 2000; ++i) wrap.Add(Record(0xFFFFFFFFu - 400000u + i * 500u, w), &r);
    CHECK(Near(Between(r.orientation, AxisAngle({0, 1, 0}, 90)), 0, 0.05));

    // A gap of 100 ms or more isn't integrated across.
    GyroIntegrator gap;
    gap.Add(Record(0, w), &r);
    gap.Add(Record(200000, w), &r);
    CHECK(Near(AngleDeg(r.orientation), 0, 1e-9));

    // Full scale on one gyro axis (2000 deg/s) and on every accelerometer
    // axis (4 g each) passes; every gyro axis saturated at once (over
    // 40 rad/s) is garbage, and is dropped without touching the state.
    GyroIntegrator limits;
    ImuRecord max{};
    max.gyro[1] = -32768;
    for (int i = 0; i < 3; ++i) max.accel[i] = -32768;
    CHECK(limits.Add(max, &r));
    for (int i = 0; i < 3; ++i) max.gyro[i] = -32768;
    max.vtsUs = 500;
    CHECK(!limits.Add(max, &r));
}

static void TestMath() {
    Quat a = AxisAngle({1, 2, 3}, 40), b = AxisAngle({-1, 0, 2}, 130);
    CHECK(Near(Between(Slerp(a, b, 0), a), 0, 1e-6));
    CHECK(Near(Between(Slerp(a, b, 1), b), 0, 1e-6));
    CHECK(Near(Between(Slerp(a, b, 0.5), a), Between(a, b) / 2, 1e-6));
    Quat negB = {-b.w, -b.x, -b.y, -b.z}; // same rotation, other hemisphere
    CHECK(Near(Between(Slerp(a, negB, 0.5), Slerp(a, b, 0.5)), 0, 1e-6));

    // Rotation matrix round trip, through each branch of FromMatrix.
    Quat qs[] = {AxisAngle({1, 2, 3}, 40), AxisAngle({1, 0, 0}, 179), AxisAngle({0, 1, 0}, 179), AxisAngle({0, 0, 1}, 179)};
    for (const Quat &q : qs) {
        float m[3][4] = {};
        Vec3 ex = Rotate(q, {1, 0, 0}), ey = Rotate(q, {0, 1, 0}), ez = Rotate(q, {0, 0, 1});
        m[0][0] = (float)ex.x, m[1][0] = (float)ex.y, m[2][0] = (float)ex.z;
        m[0][1] = (float)ey.x, m[1][1] = (float)ey.y, m[2][1] = (float)ey.z;
        m[0][2] = (float)ez.x, m[1][2] = (float)ez.y, m[2][2] = (float)ez.z;
        CHECK(Near(Between(Normalize(FromMatrix(m)), q), 0, 1e-3));
    }
}

static void TestFusion() {
    const int64_t ms = 1000000;
    TrackerPose tracker;
    tracker.valid = true;
    tracker.position = {1, 2, 3};
    tracker.orientation = AxisAngle({0, 1, 0}, 30);
    ImuPose imu;
    imu.fresh = true;
    imu.orientation = AxisAngle({1, 0, 0}, 70); // the IMU's world is unrelated to the tracker's

    Fusion f;
    HeadPose h = f.Step(tracker, imu, 0);
    CHECK(h.valid && Near(h.position.z, 3, 1e-12));
    CHECK(Near(Between(h.orientation, tracker.orientation), 0, 1e-9)); // first step snaps to the tracker

    // Head turns; tracker and gyro agree: the output follows exactly.
    Quat turn = AxisAngle({0, 1, 0}, 20);
    tracker.orientation = Mul(turn, tracker.orientation);
    imu.orientation = Mul(Mul(imu.orientation, Conj(AxisAngle({0, 1, 0}, 30))), Mul(turn, AxisAngle({0, 1, 0}, 30)));
    h = f.Step(tracker, imu, 1 * ms);
    CHECK(Near(Between(h.orientation, tracker.orientation), 0, 1e-6));

    // Gyro drifts 10 degrees away; the tracker pulls it back with tau = 1 s.
    Fusion g;
    g.tauSeconds = 1.0;
    TrackerPose t2 = tracker;
    ImuPose i2;
    i2.fresh = true;
    g.Step(t2, i2, 0);
    i2.orientation = AxisAngle({0, 1, 0}, 10);
    int64_t now = 0;
    h = g.Step(t2, i2, now += ms);
    CHECK(Near(Between(h.orientation, t2.orientation), 10, 0.1)); // fast: the gyro wins
    while (now < 1000 * ms) h = g.Step(t2, i2, now += ms);
    CHECK(Near(Between(h.orientation, t2.orientation), 10 * std::exp(-1.0), 0.1)); // one tau: 1/e left
    while (now < 6000 * ms) h = g.Step(t2, i2, now += ms);
    CHECK(Between(h.orientation, t2.orientation) < 0.05);
    CHECK(Near(g.ResidualDeg(), Between(h.orientation, t2.orientation), 1e-6));

    // No fresh IMU or fusion off: the tracker passes straight through.
    i2.fresh = false;
    h = g.Step(t2, i2, now += ms);
    CHECK(Near(Between(h.orientation, t2.orientation), 0, 1e-9));
    Fusion off;
    off.useImu = false;
    imu.fresh = true;
    h = off.Step(tracker, imu, 0);
    CHECK(Near(Between(h.orientation, tracker.orientation), 0, 1e-9));

    // No tracker: no pose (there is no fallback to Sony's tracking).
    tracker.valid = false;
    CHECK(!f.Step(tracker, imu, 2 * ms).valid);
}

int main() {
    TestRecordCount();
    TestAxes();
    TestIntegration();
    TestMath();
    TestFusion();
    if (g_failures == 0) std::printf("all tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
