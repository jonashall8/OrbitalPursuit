#pragma once

#include <cmath>

/// Central configuration for all simulation parameters.
/// Tunable constants for orbital mechanics, thermal, power, and mission logic.
namespace Config {

    // ─── Simulation ────────────────────────────────────────────────
    constexpr double TIMESTEP            = 0.1;     // seconds per physics step
    constexpr double REALTIME_FACTOR     = 10.0;    // simulation speed multiplier
    constexpr int    TELEMETRY_INTERVAL_MS = 100;   // telemetry output interval (ms)

    // ─── Orbital Parameters ────────────────────────────────────────
    constexpr double EARTH_RADIUS_KM     = 6371.0;
    constexpr double EARTH_MU            = 398600.4418;  // km^3/s^2
    constexpr double ORBIT_ALTITUDE_KM   = 550.0;       // LEO constellation altitude
    constexpr double ORBIT_RADIUS_KM     = EARTH_RADIUS_KM + ORBIT_ALTITUDE_KM;
    // Orbital rate n = sqrt(mu / r^3), with r in km → result in rad/s
    constexpr double ORBITAL_RATE        = 0.00114;      // ~rad/s for 550 km LEO

    // ─── Constellation ─────────────────────────────────────────────
    constexpr int    NUM_SATELLITES       = 8;           // compute satellites in fleet
    constexpr double FORMATION_SPACING_KM = 5.0;        // spacing between satellites

    // ─── Thermal ───────────────────────────────────────────────────
    constexpr double SOLAR_FLUX_W_M2     = 1361.0;      // solar constant at 1 AU
    constexpr double STEFAN_BOLTZMANN    = 5.67e-8;      // W/(m^2 K^4)
    constexpr double RADIATOR_AREA_M2    = 4.0;         // radiator surface area per sat
    constexpr double SAT_ABSORPTIVITY    = 0.3;         // solar absorptivity
    constexpr double RADIATOR_EMISSIVITY = 0.9;         // infrared emissivity
    constexpr double GPU_HEAT_W          = 300.0;       // heat per GPU module (W)
    constexpr int    GPUS_PER_SAT        = 8;           // GPU modules per satellite
    constexpr double TEMP_WARNING_C      = 75.0;        // warning threshold
    constexpr double TEMP_CRITICAL_C     = 90.0;        // critical threshold
    constexpr double TEMP_SHUTDOWN_C     = 100.0;       // auto-shutdown threshold
    constexpr double AMBIENT_SPACE_K     = 2.7;         // cosmic background temperature

    // ─── Power ─────────────────────────────────────────────────────
    constexpr double SOLAR_PANEL_AREA_M2 = 10.0;        // per satellite
    constexpr double SOLAR_EFFICIENCY    = 0.30;         // 30% panel efficiency
    constexpr double MAX_SOLAR_POWER_W   = SOLAR_FLUX_W_M2 * SOLAR_PANEL_AREA_M2 * SOLAR_EFFICIENCY;
    constexpr double COMPUTE_POWER_W     = 2400.0;      // full-load GPU power draw
    constexpr double HOUSEKEEPING_POWER_W = 200.0;      // comms, sensors, ADCS
    constexpr double THRUSTER_POWER_W    = 500.0;       // electric propulsion draw

    // ─── Servicer Vehicle ──────────────────────────────────────────
    constexpr double SERVICER_FUEL_KG    = 50.0;        // initial fuel mass
    constexpr double SERVICER_MASS_KG    = 200.0;       // dry mass
    constexpr double SERVICER_ISP_S      = 1500.0;      // specific impulse (electric)
    constexpr double APPROACH_SPEED_MS   = 0.5;         // max proximity approach (m/s)
    constexpr double DOCKING_DIST_M      = 2.0;         // docking proximity threshold
    constexpr double SERVICE_DURATION_S  = 300.0;       // time to complete a service

    // ─── Rendezvous Ranges ─────────────────────────────────────────
    constexpr double FAR_RANGE_M         = 1000.0;      // far rendezvous threshold
    constexpr double CLOSE_RANGE_M       = 50.0;        // proximity ops threshold
    constexpr double FINAL_RANGE_M       = 10.0;        // final approach threshold
}
