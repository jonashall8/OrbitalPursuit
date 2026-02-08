#pragma once

#include <cmath>
#include <iostream>

/// Lightweight 3D vector for orbital mechanics and position tracking.
/// No external dependencies — suitable for constrained embedded targets.
struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    // --- Arithmetic operators ---

    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }

    // --- Vector operations ---

    double dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    Vec3 cross(const Vec3& v) const {
        return {
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        };
    }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    double length_sq() const { return x * x + y * y + z * z; }

    Vec3 normalized() const {
        double len = length();
        if (len < 1e-12) return {0, 0, 0};
        return *this / len;
    }

    // --- Utility ---

    static double distance(const Vec3& a, const Vec3& b) {
        return (a - b).length();
    }

    static Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
        return a + (b - a) * t;
    }

    static Vec3 zero() { return {0, 0, 0}; }

    // --- Friend operators ---

    friend Vec3 operator*(double s, const Vec3& v) { return v * s; }

    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};
