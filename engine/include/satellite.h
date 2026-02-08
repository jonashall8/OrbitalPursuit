#pragma once

#include "vec3.h"
#include "quaternion.h"
#include <string>

/// Health status of a compute satellite.
enum class SatelliteStatus {
    NOMINAL,    // All systems green
    WARNING,    // Approaching limits
    CRITICAL,   // Immediate attention needed
    OFFLINE     // Shut down / unreachable
};

const char* status_to_string(SatelliteStatus s);

/// Represents a single compute satellite in the constellation.
/// Each satellite runs AI workloads and has thermal, power, and fuel state.
struct ComputeSatellite {
    int         id;
    std::string name;

    // ── Orbital state (LVLH frame, meters relative to reference orbit) ──
    Vec3        position;
    Vec3        velocity;

    // ── Attitude ──
    Quaternion  attitude;
    Vec3        angular_velocity;   // rad/s

    // ── Health parameters ──
    double      gpu_temp_c;            // GPU temperature (Celsius)
    double      radiator_efficiency;   // 0.0–1.0 (degrades over time)
    double      fuel_kg;               // station-keeping fuel remaining
    double      compute_load;          // 0.0–1.0 workload fraction

    // ── Power ──
    double      solar_power_w;         // current solar generation
    double      power_draw_w;          // current consumption
    bool        in_eclipse;            // in Earth's shadow?

    // ── Status ──
    SatelliteStatus status;
    double      time_since_service_s;  // seconds since last servicing

    /// Set initial healthy state for a new satellite.
    void init(int satellite_id, const Vec3& pos);

    /// Advance satellite state by dt seconds.
    void update(double dt, double sim_time);

    /// Compute urgency score (higher = needs service sooner).
    double urgency_score() const;

    /// Human-readable one-line status.
    std::string status_description() const;
};
