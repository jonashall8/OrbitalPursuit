#include "telemetry.h"
#include "thermal_model.h"

#include <sstream>
#include <fstream>
#include <iomanip>

std::string Telemetry::build_telemetry(
    const Constellation& constellation,
    const MissionManager& mission,
    double sim_time,
    double pi_cpu_temp)
{
    std::ostringstream j;
    j << std::fixed << std::setprecision(4);

    j << "{\n";

    j << "  \"sim_time\": " << sim_time << ",\n";
    j << "  \"pi_cpu_temp\": " << pi_cpu_temp << ",\n";

    // ── Mission state ──
    j << "  \"mission\": {\n";
    j << "    \"phase\": \""      << phase_to_string(mission.get_phase()) << "\",\n";
    j << "    \"target_id\": "    << mission.get_target_id() << ",\n";
    j << "    \"mission_time\": " << mission.get_mission_time() << ",\n";
    j << "    \"phase_progress\": " << mission.get_phase_progress() << ",\n";
    j << "    \"status\": \""     << escape_json(mission.get_status_message()) << "\"\n";
    j << "  },\n";

    // ── Servicer ──
    j << "  \"servicer\": " << servicer_to_json(constellation.get_servicer()) << ",\n";

    // ── Satellites ──
    j << "  \"satellites\": [\n";
    const auto& sats = constellation.get_satellites();
    for (size_t i = 0; i < sats.size(); i++) {
        j << "    " << satellite_to_json(sats[i]);
        if (i < sats.size() - 1) j << ",";
        j << "\n";
    }
    j << "  ],\n";

    // ── Trajectory prediction ──
    j << "  \"trajectory\": [";
    int tc = mission.get_trajectory_count();
    const Vec3* tpts = mission.get_trajectory();
    for (int i = 0; i < tc; i++) {
        j << vec3_to_json(tpts[i]);
        if (i < tc - 1) j << ",";
    }
    j << "],\n";

    // ── Decision reasoning ──
    j << "  \"decision\": {\n";
    j << "    \"reason\": \"" << escape_json(mission.get_decision_reason()) << "\",\n";
    j << "    \"scores\": [";
    int sc = mission.get_score_count();
    const SatelliteScore* scores = mission.get_scores();
    for (int i = 0; i < sc; i++) {
        const auto& s = scores[i];
        j << "{\"id\":" << s.id
          << ",\"temp\":" << s.temp_score
          << ",\"fuel\":" << s.fuel_score
          << ",\"rad\":"  << s.rad_score
          << ",\"time\":" << s.time_score
          << ",\"total\":" << s.total << "}";
        if (i < sc - 1) j << ",";
    }
    j << "]\n";
    j << "  },\n";

    // ── Event log ──
    j << "  \"events\": [";
    const auto& evts = mission.get_events();
    for (size_t i = 0; i < evts.size(); i++) {
        j << "{\"t\":" << evts[i].time
          << ",\"msg\":\"" << escape_json(evts[i].message) << "\"}";
        if (i < evts.size() - 1) j << ",";
    }
    j << "]\n";

    j << "}\n";
    return j.str();
}

void Telemetry::write_to_file(
    const std::string& json, const std::string& filepath)
{
    std::ofstream f(filepath);
    if (f.is_open()) {
        f << json;
        f.close();
    }
}

// ─── Serializers ───────────────────────────────────────────────────

std::string Telemetry::satellite_to_json(const ComputeSatellite& sat) {
    std::ostringstream j;
    j << std::fixed << std::setprecision(4);
    j << "{";
    j << "\"id\":"           << sat.id << ",";
    j << "\"name\":\""       << escape_json(sat.name) << "\",";
    j << "\"pos\":"          << vec3_to_json(sat.position) << ",";
    j << "\"vel\":"          << vec3_to_json(sat.velocity) << ",";
    j << "\"att\":"          << quat_to_json(sat.attitude) << ",";
    j << "\"gpu_temp\":"     << sat.gpu_temp_c << ",";
    j << "\"radiator_eff\":" << sat.radiator_efficiency << ",";
    j << "\"fuel\":"         << sat.fuel_kg << ",";
    j << "\"compute_load\":" << sat.compute_load << ",";
    j << "\"solar_power\":"  << sat.solar_power_w << ",";
    j << "\"power_draw\":"   << sat.power_draw_w << ",";
    j << "\"in_eclipse\":"   << (sat.in_eclipse ? "true" : "false") << ",";
    j << "\"status\":\""     << status_to_string(sat.status) << "\",";
    j << "\"urgency\":"      << sat.urgency_score();
    j << "}";
    return j.str();
}

std::string Telemetry::servicer_to_json(const Servicer& svc) {
    std::ostringstream j;
    j << std::fixed << std::setprecision(4);
    j << "{";
    j << "\"pos\":"       << vec3_to_json(svc.position) << ",";
    j << "\"vel\":"       << vec3_to_json(svc.velocity) << ",";
    j << "\"att\":"       << quat_to_json(svc.attitude) << ",";
    j << "\"fuel\":"      << svc.fuel_kg << ",";
    j << "\"target_id\":" << svc.target_id;
    j << "}";
    return j.str();
}

std::string Telemetry::vec3_to_json(const Vec3& v) {
    std::ostringstream j;
    j << std::fixed << std::setprecision(4);
    j << "[" << v.x << "," << v.y << "," << v.z << "]";
    return j.str();
}

std::string Telemetry::quat_to_json(const Quaternion& q) {
    std::ostringstream j;
    j << std::fixed << std::setprecision(6);
    j << "[" << q.w << "," << q.x << "," << q.y << "," << q.z << "]";
    return j.str();
}

std::string Telemetry::escape_json(const std::string& s) {
    std::ostringstream out;
    for (char c : s) {
        switch (c) {
            case '"':  out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n";  break;
            case '\r': out << "\\r";  break;
            case '\t': out << "\\t";  break;
            default:   out << c;
        }
    }
    return out.str();
}
