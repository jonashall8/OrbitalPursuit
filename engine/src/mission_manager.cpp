#include "mission_manager.h"
#include "orbital_mechanics.h"
#include "config.h"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <iomanip>

const char* phase_to_string(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::MONITORING:  return "MONITORING";
        case MissionPhase::TRIAGE:      return "TRIAGE";
        case MissionPhase::TRANSIT:     return "TRANSIT";
        case MissionPhase::RENDEZVOUS:  return "RENDEZVOUS";
        case MissionPhase::DOCKING:     return "DOCKING";
        case MissionPhase::SERVICING:   return "SERVICING";
        case MissionPhase::COMPLETE:    return "COMPLETE";
    }
    return "UNKNOWN";
}

// ════════════════════════════════════════════════════════════════════

MissionManager::MissionManager()
    : constellation_(nullptr)
    , phase_(MissionPhase::MONITORING)
    , target_id_(-1)
    , phase_timer_(0.0)
    , mission_timer_(0.0)
    , sim_time_(0.0)
    , traj_count_(0)
    , has_avoidance_waypoint_(false)
    , score_count_(0)
{}

void MissionManager::init(Constellation* constellation) {
    constellation_  = constellation;
    phase_          = MissionPhase::MONITORING;
    target_id_      = -1;
    phase_timer_    = 0.0;
    mission_timer_  = 0.0;
    traj_count_     = 0;
    score_count_    = 0;
    has_avoidance_waypoint_ = false;
    events_.clear();
    log_event("System initialized. Monitoring constellation.");
}

// ═══ Main update ═══════════════════════════════════════════════════

void MissionManager::update(double dt, double sim_time) {
    if (!constellation_) return;
    mission_timer_ += dt;
    phase_timer_   += dt;
    sim_time_       = sim_time;

    switch (phase_) {
        case MissionPhase::MONITORING:  update_monitoring(dt);  break;
        case MissionPhase::TRIAGE:      update_triage(dt);      break;
        case MissionPhase::TRANSIT:     update_transit(dt);     break;
        case MissionPhase::RENDEZVOUS:  update_rendezvous(dt);  break;
        case MissionPhase::DOCKING:     update_docking(dt);     break;
        case MissionPhase::SERVICING:   update_servicing(dt);   break;
        case MissionPhase::COMPLETE:    update_complete(dt);    break;
    }
}

// ═══ Accessors ═════════════════════════════════════════════════════

MissionPhase MissionManager::get_phase()        const { return phase_; }
int          MissionManager::get_target_id()    const { return target_id_; }
double       MissionManager::get_mission_time() const { return mission_timer_; }

const Vec3*  MissionManager::get_trajectory()       const { return trajectory_; }
int          MissionManager::get_trajectory_count() const { return traj_count_; }

const SatelliteScore* MissionManager::get_scores()      const { return scores_; }
int                   MissionManager::get_score_count()  const { return score_count_; }
const std::string&    MissionManager::get_decision_reason() const { return decision_reason_; }

const std::vector<MissionEvent>& MissionManager::get_events() const { return events_; }

double MissionManager::get_phase_progress() const {
    switch (phase_) {
        case MissionPhase::TRANSIT: {
            if (target_id_ < 0) return 0.0;
            auto* t = constellation_->get_satellite(target_id_);
            if (!t) return 0.0;
            double dist  = Vec3::distance(constellation_->get_servicer().position, t->position);
            double start = Config::FORMATION_SPACING_KM * 1000.0 * 2.0;
            return std::max(0.0, std::min(1.0, 1.0 - dist / start));
        }
        case MissionPhase::SERVICING:
            return std::min(1.0, phase_timer_ / Config::SERVICE_DURATION_S);
        default:
            return 0.0;
    }
}

void MissionManager::set_target(int satellite_id) {
    target_id_ = satellite_id;
    constellation_->get_servicer().target_id = satellite_id;
    log_event("Manual override: target set to COMPUTE-" + std::to_string(satellite_id));
    if (phase_ == MissionPhase::MONITORING || phase_ == MissionPhase::COMPLETE) {
        transition_to(MissionPhase::TRANSIT);
    }
}

std::string MissionManager::get_status_message() const {
    std::ostringstream oss;
    switch (phase_) {
        case MissionPhase::MONITORING:
            oss << "Monitoring constellation health...";
            break;
        case MissionPhase::TRIAGE:
            oss << "Evaluating service priorities...";
            break;
        case MissionPhase::TRANSIT:
            if (target_id_ >= 0) {
                auto* t = constellation_->get_satellite(target_id_);
                if (t) {
                    double d = Vec3::distance(
                        constellation_->get_servicer().position, t->position);
                    oss << "Transit to " << t->name
                        << " | Range: " << static_cast<int>(d) << " m";
                    if (has_avoidance_waypoint_)
                        oss << " [COLLISION AVOIDANCE]";
                }
            }
            break;
        case MissionPhase::RENDEZVOUS: {
            auto* t = constellation_->get_satellite(target_id_);
            if (t) {
                double d = Vec3::distance(
                    constellation_->get_servicer().position, t->position);
                oss << "Proximity ops — " << static_cast<int>(d)
                    << " m from " << t->name;
            }
            break;
        }
        case MissionPhase::DOCKING: {
            auto* t = constellation_->get_satellite(target_id_);
            if (t) {
                double d = Vec3::distance(
                    constellation_->get_servicer().position, t->position);
                oss << "Final approach — " << std::fixed << std::setprecision(1)
                    << d << " m";
            }
            break;
        }
        case MissionPhase::SERVICING:
            oss << "Servicing " << static_cast<int>(get_phase_progress() * 100)
                << "% — restoring radiator, refueling, rebalancing load";
            break;
        case MissionPhase::COMPLETE:
            oss << "Service complete. Returning to parking orbit.";
            break;
    }
    return oss.str();
}

// ═══ Internal helpers ══════════════════════════════════════════════

void MissionManager::log_event(const std::string& msg) {
    events_.push_back({sim_time_, msg});
    // Keep last 30 events
    if (events_.size() > 30) {
        events_.erase(events_.begin());
    }
}

void MissionManager::transition_to(MissionPhase p) {
    if (p != phase_) {
        log_event(std::string("Phase: ") + phase_to_string(phase_)
                  + " -> " + phase_to_string(p));
    }
    phase_       = p;
    phase_timer_ = 0.0;
    has_avoidance_waypoint_ = false;
}

// ═══ Trajectory prediction ═════════════════════════════════════════

void MissionManager::compute_trajectory_prediction() {
    const Servicer& svc = constellation_->get_servicer();
    Vec3 pos = svc.position;
    Vec3 vel = svc.velocity;
    traj_count_ = 0;
    double step = 5.0;  // predict in 5-second increments

    for (int i = 0; i < MAX_TRAJ; i++) {
        trajectory_[i] = pos;
        traj_count_++;
        OrbitalMechanics::propagate_cw(pos, vel, Config::ORBITAL_RATE, step);
    }
}

// ═══ Collision avoidance ═══════════════════════════════════════════

bool MissionManager::check_collision(
    const Vec3& from, const Vec3& to, Vec3& waypoint)
{
    for (const auto& sat : constellation_->get_satellites()) {
        if (sat.id == target_id_) continue;  // don't avoid our target

        Vec3   dir      = to - from;
        double path_len = dir.length();
        if (path_len < 1e-6) continue;
        Vec3 d = dir.normalized();

        Vec3   to_sat = sat.position - from;
        double proj   = to_sat.dot(d);
        if (proj < 0 || proj > path_len) continue;

        Vec3   closest = from + d * proj;
        double dist    = Vec3::distance(closest, sat.position);

        if (dist < KEEPOUT_M) {
            // Compute perpendicular escape direction
            Vec3 escape = (closest - sat.position);
            if (escape.length() < 1.0) escape = Vec3(0, 0, 1);  // fallback
            escape = escape.normalized();
            waypoint = sat.position + escape * (KEEPOUT_M * 1.5);
            return true;
        }
    }
    return false;
}

// ═══ Phase implementations ═════════════════════════════════════════

void MissionManager::update_monitoring(double /*dt*/) {
    traj_count_ = 0;
    auto* urgent = constellation_->get_most_urgent();
    if (urgent) {
        transition_to(MissionPhase::TRIAGE);
    }
}

void MissionManager::update_triage(double /*dt*/) {
    // ── Compute detailed score for every satellite ──
    score_count_ = 0;
    double highest = 0;
    int    best_id = -1;
    Servicer& svc  = constellation_->get_servicer();

    for (const auto& sat : constellation_->get_satellites()) {
        if (score_count_ >= MAX_SCORES) break;
        SatelliteScore& s = scores_[score_count_];
        s.id = sat.id;

        // Temperature urgency (exponential near critical)
        s.temp_score = 0;
        if (sat.gpu_temp_c > Config::TEMP_WARNING_C) {
            double r = (sat.gpu_temp_c - Config::TEMP_WARNING_C)
                     / (Config::TEMP_SHUTDOWN_C - Config::TEMP_WARNING_C);
            s.temp_score = 100.0 * r * r;
        }
        // Fuel urgency
        s.fuel_score = (sat.fuel_kg < 3.0)
            ? 50.0 * (1.0 - sat.fuel_kg / 3.0) : 0.0;
        // Radiator degradation
        s.rad_score = (sat.radiator_efficiency < 0.6)
            ? 30.0 * (1.0 - sat.radiator_efficiency) : 0.0;
        // Time since last service
        s.time_score = sat.time_since_service_s / 36000.0;

        s.total = s.temp_score + s.fuel_score + s.rad_score + s.time_score;

        if (s.total > highest && sat.status != SatelliteStatus::NOMINAL) {
            highest = s.total;
            best_id = sat.id;
        }
        score_count_++;
    }

    if (best_id >= 0) {
        target_id_    = best_id;
        svc.target_id = best_id;

        // Build human-readable reasoning
        for (int i = 0; i < score_count_; i++) {
            if (scores_[i].id == best_id) {
                std::ostringstream r;
                r << "COMPUTE-" << best_id << " selected: "
                  << "thermal(" << static_cast<int>(scores_[i].temp_score) << ") + "
                  << "fuel("    << static_cast<int>(scores_[i].fuel_score) << ") + "
                  << "radiator("<< static_cast<int>(scores_[i].rad_score)  << ") + "
                  << "age("     << static_cast<int>(scores_[i].time_score) << ") = "
                  << static_cast<int>(scores_[i].total);
                decision_reason_ = r.str();
                break;
            }
        }
        log_event(decision_reason_);
        transition_to(MissionPhase::TRANSIT);
    } else {
        transition_to(MissionPhase::MONITORING);
    }
}

void MissionManager::update_transit(double dt) {
    if (target_id_ < 0) { transition_to(MissionPhase::MONITORING); return; }
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::MONITORING); return; }

    Servicer& svc = constellation_->get_servicer();
    Vec3   to_target = target->position - svc.position;
    double dist      = to_target.length();

    if (dist < Config::FAR_RANGE_M) {
        log_event("Entering proximity zone — range " + std::to_string((int)dist) + " m");
        svc.velocity = svc.velocity * 0.3;  // bleed off speed
        transition_to(MissionPhase::RENDEZVOUS);
        return;
    }

    // ── First frame: compute CW transfer burn ──
    if (phase_timer_ < dt * 2.0) {
        double transfer_time = std::max(60.0, dist / 25.0);
        Vec3 dv = OrbitalMechanics::compute_transfer_dv(
            svc.position, svc.velocity, target->position,
            Config::ORBITAL_RATE, transfer_time);

        double dv_mag = dv.length();
        if (dv_mag > 8.0) dv = dv.normalized() * 8.0;  // cap burn
        svc.velocity += dv;

        double fuel = OrbitalMechanics::fuel_cost(
            dv_mag, Config::SERVICER_MASS_KG, Config::SERVICER_ISP_S);
        svc.fuel_kg -= fuel;

        std::ostringstream msg;
        msg << std::fixed << std::setprecision(2)
            << "Transfer burn: dv=" << dv_mag << " m/s, fuel=" << fuel << " kg";
        log_event(msg.str());
    }

    // ── Collision avoidance ──
    Vec3 waypoint;
    if (check_collision(svc.position, target->position, waypoint)) {
        if (!has_avoidance_waypoint_) {
            has_avoidance_waypoint_ = true;
            avoidance_waypoint_ = waypoint;
            log_event("Collision avoidance: rerouting via waypoint");
        }
        // Steer toward waypoint instead
        Vec3 to_wp = avoidance_waypoint_ - svc.position;
        if (to_wp.length() < 100.0) {
            has_avoidance_waypoint_ = false;  // passed the waypoint
        } else {
            Vec3 correction = to_wp.normalized() * 0.3;
            svc.velocity += correction * dt;
        }
    } else {
        has_avoidance_waypoint_ = false;
    }

    // ── Propagate with CW equations (produces curved path) ──
    OrbitalMechanics::propagate_cw(
        svc.position, svc.velocity, Config::ORBITAL_RATE, dt);

    // Mid-course correction if not closing
    double closing = -to_target.normalized().dot(svc.velocity);
    if (closing < 0.5 && dist > Config::FAR_RANGE_M * 3) {
        Vec3 nudge = to_target.normalized() * 0.15;
        svc.velocity += nudge * dt;
    }

    // ── Predict future trajectory for visualisation ──
    compute_trajectory_prediction();
}

void MissionManager::update_rendezvous(double dt) {
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::MONITORING); return; }

    Servicer& svc = constellation_->get_servicer();
    Vec3   dir  = target->position - svc.position;
    double dist = dir.length();

    if (dist < Config::FINAL_RANGE_M) {
        log_event("Final approach — range " + std::to_string((int)dist) + " m");
        transition_to(MissionPhase::DOCKING);
        return;
    }

    // Controlled approach with velocity damping
    double desired_speed = std::min(Config::APPROACH_SPEED_MS, dist * 0.003);
    Vec3   desired_vel   = dir.normalized() * desired_speed;
    Vec3   vel_error     = desired_vel - svc.velocity;
    svc.velocity += vel_error * std::min(1.0, dt * 2.0);  // PD-like control
    svc.position += svc.velocity * dt;

    compute_trajectory_prediction();
}

void MissionManager::update_docking(double dt) {
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::MONITORING); return; }

    Servicer& svc = constellation_->get_servicer();
    Vec3   dir  = target->position - svc.position;
    double dist = dir.length();

    if (dist < Config::DOCKING_DIST_M) {
        svc.position = target->position;
        svc.velocity = Vec3::zero();
        log_event("Docked with " + target->name);
        transition_to(MissionPhase::SERVICING);
        return;
    }

    // Very slow final approach
    double speed = std::min(0.1, dist * 0.008);
    Vec3   desired_vel = dir.normalized() * speed;
    svc.velocity = Vec3::lerp(svc.velocity, desired_vel, std::min(1.0, dt * 3.0));
    svc.position += svc.velocity * dt;

    compute_trajectory_prediction();
}

void MissionManager::update_servicing(double dt) {
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::COMPLETE); return; }

    // Gradually restore satellite health
    target->radiator_efficiency  = std::min(1.0, target->radiator_efficiency + 0.001 * dt);
    target->fuel_kg              = std::min(10.0, target->fuel_kg + 0.01 * dt);
    target->compute_load         = std::max(0.5, target->compute_load - 0.001 * dt);
    target->time_since_service_s = 0.0;

    traj_count_ = 0;  // no trajectory during servicing

    if (phase_timer_ >= Config::SERVICE_DURATION_S) {
        log_event("Service of " + target->name + " complete — systems restored");
        transition_to(MissionPhase::COMPLETE);
    }
}

void MissionManager::update_complete(double /*dt*/) {
    Servicer& svc = constellation_->get_servicer();
    Vec3 parking(0, -Config::FORMATION_SPACING_KM * 1000.0 * 2.0, 0);

    Vec3   dir  = parking - svc.position;
    double dist = dir.length();

    if (dist < 100.0) {
        svc.velocity  = Vec3::zero();
        svc.target_id = -1;
        target_id_    = -1;
        traj_count_   = 0;
        log_event("Servicer parked. Resuming monitoring.");
        transition_to(MissionPhase::MONITORING);
        return;
    }

    double speed = std::min(50.0, dist * 0.01);
    svc.velocity = dir.normalized() * speed;
    svc.position += svc.velocity * Config::TIMESTEP;

    compute_trajectory_prediction();
}
