#include "constellation.h"
#include "mission_manager.h"
#include "thermal_model.h"
#include "telemetry.h"
#include "config.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <string>
#include <cstdio>

static volatile bool running = true;

void signal_handler(int /*signum*/) {
    running = false;
}

static void print_header() {
    std::cout << "\n";
    std::cout << "  ╔══════════════════════════════════════════════════╗\n";
    std::cout << "  ║      ORBITAL PURSUIT — Servicing Simulator      ║\n";
    std::cout << "  ║   Autonomous Space Data-Center Maintenance      ║\n";
    std::cout << "  ╚══════════════════════════════════════════════════╝\n";
    std::cout << "\n";
}

static void print_status(
    const Constellation& constellation,
    const MissionManager& mission,
    double sim_time,
    double pi_temp)
{
    // ANSI clear screen (works in modern terminals on all platforms)
    std::cout << "\033[2J\033[H";
    print_header();

    std::cout << "  Time: " << static_cast<int>(sim_time) << "s"
              << "  |  Phase: " << phase_to_string(mission.get_phase())
              << "  |  Pi CPU: "
              << (pi_temp >= 0
                    ? std::to_string(static_cast<int>(pi_temp)) + " C"
                    : "N/A")
              << "\n";
    std::cout << "  " << mission.get_status_message() << "\n\n";

    // ── Satellite table ──
    std::cout << "  ID  | Name        | Status   | Temp  | Fuel  | Rad   | Load  | Urgency\n";
    std::cout << "  ----+-------------+----------+-------+-------+-------+-------+--------\n";

    for (const auto& sat : constellation.get_satellites()) {
        char line[200];
        std::snprintf(line, sizeof(line),
            "  %2d  | %-11s | %-8s | %3.0f C | %4.1fkg| %3.0f%%  | %3.0f%%  | %.1f",
            sat.id,
            sat.name.c_str(),
            status_to_string(sat.status),
            sat.gpu_temp_c,
            sat.fuel_kg,
            sat.radiator_efficiency * 100,
            sat.compute_load * 100,
            sat.urgency_score()
        );
        std::cout << line << "\n";
    }

    // ── Servicer status ──
    const auto& svc = constellation.get_servicer();
    std::cout << "\n  Servicer: Fuel=" << static_cast<int>(svc.fuel_kg)
              << "kg  Target="
              << (svc.target_id >= 0
                    ? "COMPUTE-" + std::to_string(svc.target_id)
                    : "NONE")
              << "  Pos=(" << static_cast<int>(svc.position.x)
              << ", "      << static_cast<int>(svc.position.y)
              << ", "      << static_cast<int>(svc.position.z)
              << ") m\n";

    std::cout << "\n  [Ctrl+C to exit]  |  Telemetry → frontend/telemetry.json\n";
}

// ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
#ifdef SIGTERM
    std::signal(SIGTERM, signal_handler);
#endif

    // ── Parse CLI args ──
    std::string telemetry_path = "frontend/telemetry.json";
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-o" && i + 1 < argc) {
            telemetry_path = argv[++i];
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: orbital_pursuit [-o telemetry.json] [-v]\n"
                      << "  -o PATH   Telemetry output file\n"
                      << "  -v        Verbose console output\n";
            return 0;
        }
    }

    // ── Initialise ──
    Constellation constellation;
    constellation.init();

    MissionManager mission;
    mission.init(&constellation);

    double sim_time       = 0.0;
    double telemetry_acc  = 0.0;
    int    frame          = 0;

    print_header();
    std::cout << "  Constellation: " << Config::NUM_SATELLITES << " compute satellites\n";
    std::cout << "  Telemetry out: " << telemetry_path << "\n";
    std::cout << "  Starting simulation (Ctrl+C to stop)...\n\n";

    auto last = std::chrono::steady_clock::now();

    // ── Main loop ──
    while (running) {
        auto now     = std::chrono::steady_clock::now();
        double real_dt = std::chrono::duration<double>(now - last).count();
        last = now;

        double sim_dt = real_dt * Config::REALTIME_FACTOR;
        sim_time      += sim_dt;
        telemetry_acc += real_dt;

        // Physics
        constellation.update(sim_dt, sim_time);
        mission.update(sim_dt, sim_time);

        // Pi CPU temperature
        double pi_temp = ThermalModel::read_pi_cpu_temp();

        // Telemetry output
        if (telemetry_acc >= Config::TELEMETRY_INTERVAL_MS / 1000.0) {
            telemetry_acc = 0.0;

            std::string json = Telemetry::build_telemetry(
                constellation, mission, sim_time, pi_temp);
            Telemetry::write_to_file(json, telemetry_path);

            // Console
            if (verbose || frame % 10 == 0) {
                print_status(constellation, mission, sim_time, pi_temp);
            }
            frame++;
        }

        // ~10 Hz tick rate
        std::this_thread::sleep_for(
            std::chrono::milliseconds(Config::TELEMETRY_INTERVAL_MS));
    }

    std::cout << "\n  Simulation stopped. Total sim-time: "
              << static_cast<int>(sim_time) << " s\n";
    return 0;
}
