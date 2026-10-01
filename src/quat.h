// Small double-precision vector and quaternion helpers.
//
// Quaternions are (w, x, y, z) with the Hamilton product, the same layout as
// vr::HmdQuaternion_t. A quaternion q rotates a vector v as q * v * conj(q).
#pragma once

#include <algorithm>
#include <cmath>

namespace psvr2fusion {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Quat {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

inline double Norm(const Vec3 &v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

inline Vec3 Scale(const Vec3 &v, double s) { return {v.x * s, v.y * s, v.z * s}; }

inline Quat Conj(const Quat &q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quat Mul(const Quat &a, const Quat &b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// Unit quaternion, or identity when q is too small to have a direction.
inline Quat Normalize(const Quat &q) {
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) return {};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Vec3 Rotate(const Quat &q, const Vec3 &v) {
    Quat r = Mul(Mul(q, {0.0, v.x, v.y, v.z}), Conj(q));
    return {r.x, r.y, r.z};
}

// The rotation that turns at angular velocity w (rad/s) for dt seconds.
inline Quat Integrate(const Vec3 &w, double dt) {
    Vec3 h = Scale(w, 0.5 * dt); // half the rotation vector
    double a = Norm(h);
    if (a < 1e-12) return Normalize({1.0, h.x, h.y, h.z});
    double s = std::sin(a) / a;
    return {std::cos(a), h.x * s, h.y * s, h.z * s};
}

// Shortest-path spherical interpolation from a (t = 0) to b (t = 1).
inline Quat Slerp(const Quat &a, Quat b, double t) {
    double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d < 0.0) {
        d = -d;
        b = {-b.w, -b.x, -b.y, -b.z};
    }
    if (d > 0.9995) {
        return Normalize({a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
                          a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t});
    }
    double theta = std::acos(d);
    double s = std::sin(theta);
    double ka = std::sin((1.0 - t) * theta) / s;
    double kb = std::sin(t * theta) / s;
    return Normalize({a.w * ka + b.w * kb, a.x * ka + b.x * kb,
                      a.y * ka + b.y * kb, a.z * ka + b.z * kb});
}

// How far q rotates, in degrees (0..180).
inline double AngleDeg(const Quat &q) {
    Quat n = Normalize(q);
    return 2.0 * std::acos(std::min(1.0, std::fabs(n.w))) * kRadToDeg;
}

// Rotation part of a row-major 3x4 pose matrix (vr::HmdMatrix34_t::m).
inline Quat FromMatrix(const float m[3][4]) {
    double tr = m[0][0] + m[1][1] + m[2][2];
    Quat q;
    if (tr > 0.0) {
        double s = std::sqrt(tr + 1.0) * 2.0;
        q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
    } else if (m[1][1] > m[2][2]) {
        double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
    } else {
        double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
    }
    return Normalize(q);
}

} // namespace psvr2fusion
