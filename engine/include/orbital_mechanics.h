#pragma once

#include "vec3.h"

/// Clohessy-Wiltshire (CW) relative-motion model and transfer planning.
///
/// All positions and velocities are in the LVLH (Hill) frame
/// relative to a circular reference orbit:
///   x — radial (outward from Earth)
///   y — along-track (in direction of orbital velocity)
///   z — cross-track (completes right-hand system)
class OrbitalMechanics {
public:
    /// Propagate relative state using the CW state-transition matrix.
    /// @param pos Position vector (meters), updated in place.
    /// @param vel Velocity vector (m/s), updated in place.
    /// @param n   Orbital rate of the reference orbit (rad/s).
    /// @param dt  Time step (seconds).
    static void propagate_cw(Vec3& pos, Vec3& vel, double n, double dt);

    /// Compute the delta-v required for a two-impulse CW transfer
    /// from (pos0, vel0) to pos_target over a given duration.
    /// @return Required initial velocity change (m/s).
    static Vec3 compute_transfer_dv(
        const Vec3& pos0,
        const Vec3& vel0,
        const Vec3& pos_target,
        double n,
        double transfer_time
    );

    /// Fuel consumed for a maneuver (Tsiolkovsky rocket equation).
    /// @param dv_ms   Delta-v magnitude (m/s).
    /// @param mass_kg Spacecraft wet mass (kg).
    /// @param isp_s   Specific impulse (seconds).
    /// @return Propellant consumed (kg).
    static double fuel_cost(double dv_ms, double mass_kg, double isp_s);

    /// Check whether a straight-line path between two points is clear
    /// of all obstacles by at least min_clearance_m.
    static bool is_path_clear(
        const Vec3& from,
        const Vec3& to,
        const Vec3* obstacles,
        int num_obstacles,
        double min_clearance_m
    );
};
