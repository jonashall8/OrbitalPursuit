#include "mission_manager.h"
#include "orbital_mechanics.h"
#include "config.h"

#include <cmath>
#include <sstream>

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

// ────────────────────────────────────────────────────────────────────

MissionManager::MissionManager()
    : constellation_(nullptr)
    , phase_(MissionPhase::MONITORING)
    , target_id_(-1)
    , phase_timer_(0.0)
    , mission_timer_(0.0)
{}

void MissionManager::init(Constellation* constellation) {
    constellation_  = constellation;
    phase_          = MissionPhase::MONITORING;
    target_id_      = -1;
    phase_timer_    = 0.0;
    mission_timer_  = 0.0;
}

// ─── Main update dispatch ──────────────────────────────────────────

void MissionManager::update(double dt, double /*sim_time*/) {
    if (!constellation_) return;

    mission_timer_ += dt;
    phase_timer_   += dt;

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

// ─── Accessors ─────────────────────────────────────────────────────

MissionPhase MissionManager::get_phase()        const { return phase_; }
int          MissionManager::get_target_id()    const { return target_id_; }
double       MissionManager::get_mission_time() const { return mission_timer_; }

double MissionManager::get_phase_progress() const {
    switch (phase_) {
        case MissionPhase::TRANSIT: {
            if (target_id_ < 0) return 0.0;
            auto* t = constellation_->get_satellite(target_id_);
            if (!t) return 0.0;
            double dist  = Vec3::distance(
                constellation_->get_servicer().position, t->position);
            double start = Config::FORMATION_SPACING_KM * 1000.0 * 2.0;
            return std::max(0.0, 1.0 - dist / start);
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
                        << " | Distance: " << static_cast<int>(d) << " m";
                }
            }
            break;

        case MissionPhase::RENDEZVOUS:
            oss << "Proximity operations — matching attitude...";
            break;

        case MissionPhase::DOCKING:
            oss << "Final approach — docking in progress...";
            break;

        case MissionPhase::SERVICING:
            oss << "Servicing in progress... "
                << static_cast<int>(get_phase_progress() * 100) << "%";
            break;

        case MissionPhase::COMPLETE:
            oss << "Service complete. Returning to parking orbit.";
            break;
    }

    return oss.str();
}

// ─── Internal helpers ──────────────────────────────────────────────

void MissionManager::transition_to(MissionPhase new_phase) {
    phase_       = new_phase;
    phase_timer_ = 0.0;
}

// ─── Phase logic ───────────────────────────────────────────────────

void MissionManager::update_monitoring(double /*dt*/) {
    auto* urgent = constellation_->get_most_urgent();
    if (urgent) {
        transition_to(MissionPhase::TRIAGE);
    }
}

void MissionManager::update_triage(double /*dt*/) {
    auto* urgent = constellation_->get_most_urgent();
    if (urgent) {
        target_id_ = urgent->id;
        constellation_->get_servicer().target_id = target_id_;
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
    Vec3   dir  = target->position - svc.position;
    double dist = dir.length();

    if (dist < Config::FAR_RANGE_M) {
        transition_to(MissionPhase::RENDEZVOUS);
        return;
    }

    // Proportional approach with speed cap
    double speed = std::min(50.0, dist * 0.01);
    svc.velocity = dir.normalized() * speed;
    svc.position += svc.velocity * dt;

    // Fuel consumption
    double dv = speed * dt * 0.001;
    svc.fuel_kg -= OrbitalMechanics::fuel_cost(
        dv, Config::SERVICER_MASS_KG, Config::SERVICER_ISP_S);
}

void MissionManager::update_rendezvous(double dt) {
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::MONITORING); return; }

    Servicer& svc = constellation_->get_servicer();
    Vec3   dir  = target->position - svc.position;
    double dist = dir.length();

    if (dist < Config::FINAL_RANGE_M) {
        transition_to(MissionPhase::DOCKING);
        return;
    }

    // Slower approach in proximity zone
    double speed = std::min(Config::APPROACH_SPEED_MS, dist * 0.005);
    svc.velocity = dir.normalized() * speed;
    svc.position += svc.velocity * dt;
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
        transition_to(MissionPhase::SERVICING);
        return;
    }

    // Very slow final approach
    double speed = std::min(0.1, dist * 0.01);
    svc.velocity = dir.normalized() * speed;
    svc.position += svc.velocity * dt;
}

void MissionManager::update_servicing(double dt) {
    auto* target = constellation_->get_satellite(target_id_);
    if (!target) { transition_to(MissionPhase::COMPLETE); return; }

    // Gradually restore satellite health
    target->radiator_efficiency = std::min(1.0, target->radiator_efficiency + 0.001 * dt);
    target->fuel_kg             = std::min(10.0, target->fuel_kg + 0.01 * dt);
    target->compute_load        = std::max(0.5, target->compute_load - 0.001 * dt);
    target->time_since_service_s = 0.0;

    if (phase_timer_ >= Config::SERVICE_DURATION_S) {
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
        transition_to(MissionPhase::MONITORING);
        return;
    }

    double speed = std::min(50.0, dist * 0.01);
    svc.velocity = dir.normalized() * speed;
    svc.position += svc.velocity * Config::TIMESTEP;
}
