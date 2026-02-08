#pragma once

#include "constellation.h"
#include "mission_manager.h"
#include <string>

/// Builds JSON telemetry strings for the frontend (no external dependencies).
class Telemetry {
public:
    /// Serialize full simulation state to a JSON string.
    static std::string build_telemetry(
        const Constellation& constellation,
        const MissionManager& mission,
        double sim_time,
        double pi_cpu_temp
    );

    /// Write a JSON string to a file (atomic-ish for frontend polling).
    static void write_to_file(const std::string& json, const std::string& filepath);

private:
    static std::string satellite_to_json(const ComputeSatellite& sat);
    static std::string servicer_to_json(const Servicer& servicer);
    static std::string vec3_to_json(const Vec3& v);
    static std::string quat_to_json(const Quaternion& q);
    static std::string escape_json(const std::string& s);
};
