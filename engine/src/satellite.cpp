#include "satellite.h"
#include "config.h"
#include "thermal_model.h"

#include <cmath>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const char* status_to_string(SatelliteStatus s) {
    switch (s) {
        case SatelliteStatus::NOMINAL:  return "NOMINAL";
        case SatelliteStatus::WARNING:  return "WARNING";
        case SatelliteStatus::CRITICAL: return "CRITICAL";
        case SatelliteStatus::OFFLINE:  return "OFFLINE";
    }
    return "UNKNOWN";
}

void ComputeSatellite::init(int satellite_id, const Vec3& pos) {
    id   = satellite_id;
    name = "COMPUTE-" + std::to_string(satellite_id);

    position         = pos;
    velocity         = Vec3::zero();
    attitude         = Quaternion::identity();
    angular_velocity = Vec3(0.001, 0.0005, 0.002);  // slight natural tumble

    // Introduce variance so satellites degrade at different rates
    gpu_temp_c           = 45.0 + (satellite_id * 3.0);
    radiator_efficiency  = 1.0  - (satellite_id * 0.05);
    fuel_kg              = 10.0 - (satellite_id * 0.5);
    compute_load         = 0.6  + (satellite_id * 0.04);

    solar_power_w = Config::MAX_SOLAR_POWER_W;
    power_draw_w  = Config::HOUSEKEEPING_POWER_W
                  + compute_load * Config::COMPUTE_POWER_W;
    in_eclipse    = false;

    status               = SatelliteStatus::NOMINAL;
    time_since_service_s = satellite_id * 3600.0;  // staggered service history
}

void ComputeSatellite::update(double dt, double sim_time) {
    // ── Eclipse calculation ──
    double orbital_period = 2.0 * M_PI / Config::ORBITAL_RATE;
    double phase_offset   = id * orbital_period / Config::NUM_SATELLITES;
    double orbital_angle  = std::fmod(sim_time + phase_offset, orbital_period);
    double angle_rad      = (orbital_angle / orbital_period) * 2.0 * M_PI;
    in_eclipse = ThermalModel::is_in_eclipse(angle_rad);

    // ── Power ──
    solar_power_w = in_eclipse ? 0.0 : Config::MAX_SOLAR_POWER_W;
    power_draw_w  = Config::HOUSEKEEPING_POWER_W
                  + compute_load * Config::COMPUTE_POWER_W;

    // ── Degradation over time ──
    compute_load        = std::min(1.0, compute_load + 0.001 * dt);
    radiator_efficiency = std::max(0.3, radiator_efficiency - 0.00001 * dt);
    fuel_kg             = std::max(0.0, fuel_kg - 0.0001 * dt);

    // ── Thermal ──
    gpu_temp_c = ThermalModel::compute_temperature(*this, dt, gpu_temp_c);

    // ── Attitude drift ──
    double angle = angular_velocity.length() * dt;
    if (angle > 1e-10) {
        Quaternion delta = Quaternion::from_axis_angle(
            angular_velocity.normalized(), angle);
        attitude = (delta * attitude).normalized();
    }

    // ── Status determination ──
    time_since_service_s += dt;

    if (gpu_temp_c >= Config::TEMP_SHUTDOWN_C || fuel_kg <= 0.0) {
        status = SatelliteStatus::OFFLINE;
    } else if (gpu_temp_c >= Config::TEMP_CRITICAL_C || fuel_kg < 1.0) {
        status = SatelliteStatus::CRITICAL;
    } else if (gpu_temp_c >= Config::TEMP_WARNING_C
            || fuel_kg < 3.0
            || radiator_efficiency < 0.5) {
        status = SatelliteStatus::WARNING;
    } else {
        status = SatelliteStatus::NOMINAL;
    }
}

double ComputeSatellite::urgency_score() const {
    if (status == SatelliteStatus::OFFLINE) return 1000.0;

    double score = 0.0;

    // Temperature (exponential urgency near critical)
    if (gpu_temp_c > Config::TEMP_WARNING_C) {
        double ratio = (gpu_temp_c - Config::TEMP_WARNING_C)
                     / (Config::TEMP_SHUTDOWN_C - Config::TEMP_WARNING_C);
        score += 100.0 * ratio * ratio;
    }

    // Fuel
    if (fuel_kg < 3.0) {
        score += 50.0 * (1.0 - fuel_kg / 3.0);
    }

    // Radiator degradation
    if (radiator_efficiency < 0.6) {
        score += 30.0 * (1.0 - radiator_efficiency);
    }

    // Time since last service (10 hours → +1 point)
    score += time_since_service_s / 36000.0;

    return score;
}

std::string ComputeSatellite::status_description() const {
    std::ostringstream oss;
    oss << name
        << " [" << status_to_string(status) << "]"
        << " Temp:" << static_cast<int>(gpu_temp_c) << "C"
        << " Fuel:" << static_cast<int>(fuel_kg) << "kg"
        << " Rad:"  << static_cast<int>(radiator_efficiency * 100) << "%"
        << " Load:" << static_cast<int>(compute_load * 100) << "%";
    return oss.str();
}
