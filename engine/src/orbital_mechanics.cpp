#include "orbital_mechanics.h"
#include "config.h"

#include <cmath>

void OrbitalMechanics::propagate_cw(
    Vec3& pos, Vec3& vel, double n, double dt)
{
    /*
     * Clohessy-Wiltshire state-transition matrix for relative motion
     * in the LVLH frame around a circular reference orbit.
     *
     *   x: radial, y: along-track, z: cross-track
     *
     * The x-y dynamics are coupled; z is a simple harmonic oscillator.
     */

    double nt  = n * dt;
    double snt = std::sin(nt);
    double cnt = std::cos(nt);

    double x0  = pos.x, y0  = pos.y, z0  = pos.z;
    double vx0 = vel.x, vy0 = vel.y, vz0 = vel.z;

    // ── Position ──
    pos.x = (4.0 - 3.0 * cnt) * x0
          + (snt / n) * vx0
          + (2.0 / n) * (1.0 - cnt) * vy0;

    pos.y = 6.0 * (snt - nt) * x0
          + y0
          + (2.0 / n) * (cnt - 1.0) * vx0
          + (4.0 * snt / n - 3.0 * dt) * vy0;

    pos.z = z0 * cnt + (vz0 / n) * snt;

    // ── Velocity ──
    vel.x =  3.0 * n * snt * x0
          +  cnt * vx0
          +  2.0 * snt * vy0;

    vel.y = -6.0 * n * (1.0 - cnt) * x0
          -  2.0 * snt * vx0
          + (4.0 * cnt - 3.0) * vy0;

    vel.z = -z0 * n * snt + vz0 * cnt;
}

Vec3 OrbitalMechanics::compute_transfer_dv(
    const Vec3& pos0,
    const Vec3& vel0,
    const Vec3& pos_target,
    double n,
    double transfer_time)
{
    /*
     * Two-impulse CW transfer solver.
     *
     * Given: current state (pos0, vel0) and desired final position pos_target,
     * find the initial velocity change so that after transfer_time seconds
     * the chaser reaches pos_target.
     *
     * pos_target = Φ_rr · pos0  +  Φ_rv · v_required
     * → v_required = Φ_rv⁻¹ · (pos_target − Φ_rr · pos0)
     * → Δv = v_required − vel0
     */

    double t   = transfer_time;
    double nt  = n * t;
    double snt = std::sin(nt);
    double cnt = std::cos(nt);

    // Φ_rr · pos0
    Vec3 phi_rr_p(
        (4.0 - 3.0 * cnt) * pos0.x,
        6.0 * (snt - nt) * pos0.x + pos0.y,
        cnt * pos0.z
    );

    Vec3 residual = pos_target - phi_rr_p;

    // Φ_rv sub-matrix (x-y block + decoupled z)
    double a11 = snt / n;
    double a12 = 2.0 * (1.0 - cnt) / n;
    double a21 = 2.0 * (cnt - 1.0) / n;
    double a22 = 4.0 * snt / n - 3.0 * t;
    double a33 = snt / n;

    // Invert 2×2 for in-plane, scalar for cross-track
    double det = a11 * a22 - a12 * a21;

    Vec3 v_req;
    if (std::abs(det) > 1e-12) {
        v_req.x = ( a22 * residual.x - a12 * residual.y) / det;
        v_req.y = (-a21 * residual.x + a11 * residual.y) / det;
    } else {
        v_req.x = residual.x / t;
        v_req.y = residual.y / t;
    }

    v_req.z = (std::abs(a33) > 1e-12)
            ? residual.z / a33
            : residual.z / t;

    return v_req - vel0;
}

double OrbitalMechanics::fuel_cost(
    double dv_ms, double mass_kg, double isp_s)
{
    // Tsiolkovsky: Δm = m · (1 − exp(−Δv / (g₀ · Isp)))
    constexpr double g0 = 9.80665;
    double ve = isp_s * g0;
    return mass_kg * (1.0 - std::exp(-dv_ms / ve));
}

bool OrbitalMechanics::is_path_clear(
    const Vec3& from,
    const Vec3& to,
    const Vec3* obstacles,
    int num_obstacles,
    double min_clearance_m)
{
    Vec3   dir  = to - from;
    double len  = dir.length();
    if (len < 1e-6) return true;

    Vec3 d = dir.normalized();

    for (int i = 0; i < num_obstacles; i++) {
        Vec3 toObs = obstacles[i] - from;

        // Project onto path segment
        double proj = toObs.dot(d);
        if (proj < 0 || proj > len) continue;

        Vec3   closest = from + d * proj;
        double dist    = Vec3::distance(closest, obstacles[i]);

        if (dist < min_clearance_m) return false;
    }

    return true;
}
