#pragma once

#include "constellation.h"
#include <string>

/// Phases of a servicing mission.
enum class MissionPhase {
    MONITORING,   // Watching constellation health
    TRIAGE,       // Selecting highest-priority target
    TRANSIT,      // Transferring to target satellite
    RENDEZVOUS,   // Proximity operations
    DOCKING,      // Final approach & docking
    SERVICING,    // Performing repairs
    COMPLETE      // Returning to parking orbit
};

const char* phase_to_string(MissionPhase phase);

/// State machine that drives the autonomous servicing mission.
class MissionManager {
public:
    MissionManager();

    /// Bind to a constellation (must be called before update).
    void init(Constellation* constellation);

    /// Advance mission logic by dt seconds.
    void update(double dt, double sim_time);

    // ── Accessors ──

    MissionPhase get_phase()          const;
    int          get_target_id()      const;
    double       get_mission_time()   const;
    double       get_phase_progress() const;  // 0.0–1.0
    std::string  get_status_message() const;

    /// Override: manually assign a servicing target (ground-operator command).
    void set_target(int satellite_id);

private:
    Constellation*  constellation_;
    MissionPhase    phase_;
    int             target_id_;
    double          phase_timer_;
    double          mission_timer_;

    void transition_to(MissionPhase new_phase);

    // Per-phase update logic
    void update_monitoring(double dt);
    void update_triage(double dt);
    void update_transit(double dt);
    void update_rendezvous(double dt);
    void update_docking(double dt);
    void update_servicing(double dt);
    void update_complete(double dt);
};
