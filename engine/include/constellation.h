#pragma once

#include "satellite.h"
#include <vector>
#include <string>

/// The autonomous servicer vehicle that travels between satellites.
struct Servicer {
    Vec3        position;
    Vec3        velocity;
    Quaternion  attitude;
    double      fuel_kg;
    int         target_id;  // -1 if no current target

    void init();
};

/// Manages the full constellation of compute satellites and the servicer.
class Constellation {
public:
    Constellation();

    /// Place satellites in formation and initialize servicer.
    void init();

    /// Advance all satellite states by dt seconds.
    void update(double dt, double sim_time);

    /// Return the satellite with the highest urgency score,
    /// or nullptr if all are NOMINAL.
    ComputeSatellite* get_most_urgent();

    /// Lookup by ID (returns nullptr if out of range).
    ComputeSatellite* get_satellite(int id);

    std::vector<ComputeSatellite>&       get_satellites();
    const std::vector<ComputeSatellite>& get_satellites() const;

    Servicer&       get_servicer();
    const Servicer& get_servicer() const;

private:
    std::vector<ComputeSatellite> satellites_;
    Servicer servicer_;
};
