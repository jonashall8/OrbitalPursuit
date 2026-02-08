#pragma once

#include "constellation.h"
#include <string>
#include <vector>

enum class MissionPhase {
    MONITORING,
    TRIAGE,
    TRANSIT,
    RENDEZVOUS,
    DOCKING,
    SERVICING,
    COMPLETE
};

const char* phase_to_string(MissionPhase phase);

/// Detailed urgency-score breakdown for a single satellite.
/// Sent to the frontend so the autonomous reasoning is visible and auditable.
struct SatelliteScore {
    int    id;
    double temp_score;
    double fuel_score;
    double rad_score;
    double time_score;
    double total;
};

/// Timestamped event for the mission log.
struct MissionEvent {
    double      time;
    std::string message;
};

/// Autonomous servicing mission state machine.
/// Now includes CW-based trajectory planning, collision avoidance,
/// decision reasoning, and an event log.
class MissionManager {
public:
    MissionManager();
    void init(Constellation* constellation);
    void update(double dt, double sim_time);

    // ── Accessors ──
    MissionPhase get_phase()          const;
    int          get_target_id()      const;
    double       get_mission_time()   const;
    double       get_phase_progress() const;
    std::string  get_status_message() const;

    // Trajectory prediction (future servicer positions)
    const Vec3*  get_trajectory()         const;
    int          get_trajectory_count()   const;

    // Decision reasoning (score breakdown per satellite)
    const SatelliteScore* get_scores()    const;
    int                   get_score_count() const;
    const std::string&    get_decision_reason() const;

    // Event log (recent autonomous decisions)
    const std::vector<MissionEvent>& get_events() const;

    // Ground-operator override
    void set_target(int satellite_id);

private:
    Constellation*  constellation_;
    MissionPhase    phase_;
    int             target_id_;
    double          phase_timer_;
    double          mission_timer_;
    double          sim_time_;

    // ── Trajectory prediction ──
    static constexpr int MAX_TRAJ = 60;
    Vec3 trajectory_[MAX_TRAJ];
    int  traj_count_;
    void compute_trajectory_prediction();

    // ── Collision avoidance ──
    static constexpr double KEEPOUT_M = 200.0;
    Vec3 avoidance_waypoint_;
    bool has_avoidance_waypoint_;
    bool check_collision(const Vec3& from, const Vec3& to, Vec3& waypoint);

    // ── Decision reasoning ──
    static constexpr int MAX_SCORES = 16;
    SatelliteScore scores_[MAX_SCORES];
    int  score_count_;
    std::string decision_reason_;

    // ── Event log ──
    std::vector<MissionEvent> events_;
    void log_event(const std::string& msg);

    // ── Phase logic ──
    void transition_to(MissionPhase p);
    void update_monitoring(double dt);
    void update_triage(double dt);
    void update_transit(double dt);
    void update_rendezvous(double dt);
    void update_docking(double dt);
    void update_servicing(double dt);
    void update_complete(double dt);
};
