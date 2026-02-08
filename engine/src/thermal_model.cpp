#include "thermal_model.h"
#include "satellite.h"
#include "config.h"

#include <cmath>
#include <fstream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double ThermalModel::compute_temperature(
    const ComputeSatellite& sat,
    double dt,
    double current_temp_c)
{
    double temp_k = current_temp_c + 273.15;

    // ── Heat inputs ──
    double q_solar = solar_heat_input(
        Config::SAT_ABSORPTIVITY,
        Config::SOLAR_PANEL_AREA_M2,
        sat.in_eclipse
    );

    double q_compute = compute_heat(
        sat.compute_load,
        Config::GPUS_PER_SAT,
        Config::GPU_HEAT_W
    );

    // ── Heat output (Stefan-Boltzmann radiative cooling) ──
    double q_radiated = radiative_dissipation(
        Config::RADIATOR_EMISSIVITY,
        Config::RADIATOR_AREA_M2,
        temp_k,
        sat.radiator_efficiency
    );

    // ── Net heat flow ──
    double q_net = q_solar + q_compute - q_radiated;

    // Simplified thermal mass: aluminum structure ~900 J/(kg·K), ~100 kg
    double thermal_mass = 900.0 * 100.0;  // 90,000 J/K
    double dT = (q_net / thermal_mass) * dt;

    double new_c = (temp_k + dT) - 273.15;

    // Clamp to physically reasonable range
    return std::max(-50.0, std::min(150.0, new_c));
}

double ThermalModel::read_pi_cpu_temp() {
    // Raspberry Pi exposes CPU temp at this sysfs path.
    // Returns -1 on non-Pi platforms (Windows, macOS, etc.).
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (f.is_open()) {
        std::string s;
        std::getline(f, s);
        f.close();
        try {
            return std::stod(s) / 1000.0;  // millidegrees → degrees
        } catch (...) {
            return -1.0;
        }
    }
    return -1.0;
}

double ThermalModel::solar_heat_input(
    double absorptivity, double area_m2, bool in_eclipse)
{
    if (in_eclipse) return 0.0;
    return absorptivity * Config::SOLAR_FLUX_W_M2 * area_m2;
}

double ThermalModel::radiative_dissipation(
    double emissivity, double area_m2, double temp_k, double radiator_efficiency)
{
    // Stefan-Boltzmann: P = ε σ A (T⁴ − T_space⁴)
    double t_bg = Config::AMBIENT_SPACE_K;
    double power = emissivity * Config::STEFAN_BOLTZMANN * area_m2
                 * (std::pow(temp_k, 4) - std::pow(t_bg, 4));
    return power * radiator_efficiency;
}

double ThermalModel::compute_heat(
    double compute_load, int num_gpus, double heat_per_gpu)
{
    return compute_load * num_gpus * heat_per_gpu;
}

bool ThermalModel::is_in_eclipse(double orbital_angle) {
    // Simplified: Earth subtends ~120° at 550 km LEO → ~35% of orbit in shadow.
    double a = std::fmod(orbital_angle, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;

    double center     = M_PI;
    double half_width = 0.35 * M_PI;

    return std::abs(a - center) < half_width;
}
