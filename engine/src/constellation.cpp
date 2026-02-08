#include "constellation.h"
#include "config.h"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─── Servicer ──────────────────────────────────────────────────────

void Servicer::init() {
    // Park behind the constellation in the along-track direction
    position  = Vec3(0, -Config::FORMATION_SPACING_KM * 1000.0 * 2.0, 0);
    velocity  = Vec3::zero();
    attitude  = Quaternion::identity();
    fuel_kg   = Config::SERVICER_FUEL_KG;
    target_id = -1;
}

// ─── Constellation ─────────────────────────────────────────────────

Constellation::Constellation() {}

void Constellation::init() {
    satellites_.resize(Config::NUM_SATELLITES);

    // Distribute satellites in an elliptical formation (LVLH frame)
    for (int i = 0; i < Config::NUM_SATELLITES; i++) {
        double angle  = (2.0 * M_PI * i) / Config::NUM_SATELLITES;
        double radius = Config::FORMATION_SPACING_KM * 1000.0;  // to meters

        Vec3 pos(
            radius * std::cos(angle) * 0.3,              // radial spread
            radius * std::sin(angle),                     // along-track spread
            radius * std::cos(angle + M_PI / 4) * 0.2    // cross-track
        );

        satellites_[i].init(i, pos);
    }

    servicer_.init();
}

void Constellation::update(double dt, double sim_time) {
    for (auto& sat : satellites_) {
        sat.update(dt, sim_time);
    }
}

ComputeSatellite* Constellation::get_most_urgent() {
    ComputeSatellite* best = nullptr;
    double highest = 0.0;

    for (auto& sat : satellites_) {
        double score = sat.urgency_score();
        if (score > highest && sat.status != SatelliteStatus::NOMINAL) {
            highest = score;
            best    = &sat;
        }
    }

    return best;
}

ComputeSatellite* Constellation::get_satellite(int id) {
    if (id >= 0 && id < static_cast<int>(satellites_.size())) {
        return &satellites_[id];
    }
    return nullptr;
}

std::vector<ComputeSatellite>&       Constellation::get_satellites()       { return satellites_; }
const std::vector<ComputeSatellite>& Constellation::get_satellites() const { return satellites_; }

Servicer&       Constellation::get_servicer()       { return servicer_; }
const Servicer& Constellation::get_servicer() const { return servicer_; }
