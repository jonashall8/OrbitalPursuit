#pragma once

#include "vec3.h"
#include <cmath>

/// Quaternion for gimbal-lock-free 3D rotation.
/// Used for spacecraft attitude representation and control.
struct Quaternion {
    double w, x, y, z;

    /// Default constructor — identity rotation (no rotation).
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // --- Factory methods ---

    static Quaternion identity() { return {1, 0, 0, 0}; }

    /// Create quaternion from axis-angle representation.
    /// @param axis  Unit vector defining the rotation axis.
    /// @param angle Rotation angle in radians.
    static Quaternion from_axis_angle(const Vec3& axis, double angle) {
        Vec3 n = axis.normalized();
        double half = angle * 0.5;
        double s = std::sin(half);
        return {std::cos(half), n.x * s, n.y * s, n.z * s};
    }

    // --- Operations ---

    /// Hamilton product (quaternion multiplication).
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quaternion normalized() const {
        double n = norm();
        if (n < 1e-12) return identity();
        return {w / n, x / n, y / n, z / n};
    }

    /// Rotate a 3D vector by this quaternion: v' = q * v * q*
    Vec3 rotate(const Vec3& v) const {
        Quaternion qv(0, v.x, v.y, v.z);
        Quaternion result = (*this) * qv * conjugate();
        return {result.x, result.y, result.z};
    }

    /// Spherical linear interpolation between two quaternions.
    /// @param t Interpolation parameter [0, 1].
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t) {
        double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

        // Take the shorter arc
        Quaternion b2 = b;
        if (dot < 0) {
            b2 = {-b.w, -b.x, -b.y, -b.z};
            dot = -dot;
        }

        // Fall back to linear interpolation for nearly-equal quaternions
        if (dot > 0.9995) {
            return Quaternion{
                a.w + t * (b2.w - a.w),
                a.x + t * (b2.x - a.x),
                a.y + t * (b2.y - a.y),
                a.z + t * (b2.z - a.z)
            }.normalized();
        }

        double theta = std::acos(dot);
        double sin_theta = std::sin(theta);
        double wa = std::sin((1.0 - t) * theta) / sin_theta;
        double wb = std::sin(t * theta) / sin_theta;

        return Quaternion{
            wa * a.w + wb * b2.w,
            wa * a.x + wb * b2.x,
            wa * a.y + wb * b2.y,
            wa * a.z + wb * b2.z
        }.normalized();
    }

    /// Convert to Euler angles (roll, pitch, yaw) in radians.
    Vec3 to_euler() const {
        double sinr = 2.0 * (w * x + y * z);
        double cosr = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr, cosr);

        double sinp = 2.0 * (w * y - z * x);
        double pitch = (std::abs(sinp) >= 1)
            ? std::copysign(M_PI / 2, sinp)
            : std::asin(sinp);

        double siny = 2.0 * (w * z + x * y);
        double cosy = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny, cosy);

        return {roll, pitch, yaw};
    }

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]";
        return os;
    }
};
