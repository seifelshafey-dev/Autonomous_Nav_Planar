/**
 * @file stop_zone.cpp
 * @author Seif ElShafey
 * @brief Implementation of the stop-zone test (see stop_zone.hpp).
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/control/stop_zone.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "core/diagnostics/check.hpp"

bool motionBlocked(const RobotParams & robot, const std::vector<Vec2> & points_body,
                   double v, double omega, double reaction_time, const StopZoneParams & params) {
    CHECK(std::isfinite(v), "motionBlocked: v must be finite");
    CHECK(std::isfinite(omega), "motionBlocked: omega must be finite");
    CHECK(std::isfinite(reaction_time) && reaction_time >= 0.0, "motionBlocked: reaction_time must be finite and >= 0");
    CHECK(std::isfinite(params.linear_decel) && params.linear_decel > 0.0, "motionBlocked: linear_decel must be finite and > 0");
    CHECK(std::isfinite(params.angular_decel) && params.angular_decel > 0.0, "motionBlocked: angular_decel must be finite and > 0");
    CHECK(std::isfinite(params.forward_pad) && params.forward_pad >= 0.0, "motionBlocked: forward_pad must be finite and >= 0");
    CHECK(std::isfinite(params.side_pad) && params.side_pad > 0.0, "motionBlocked: side_pad must be finite and > 0");

    if (v == 0.0 && omega == 0.0) return false;

    const double av = std::abs(v);
    const double aw = std::abs(omega);
    const double sv = (v < 0.0) ? -1.0 : 1.0;
    const double sw = (omega < 0.0) ? -1.0 : 1.0;
    const double a_v = params.linear_decel;
    const double a_w = params.angular_decel;
    const double S = av + robot.r_out * aw;
    const double tau_v = av / a_v;
    const double tau_w = aw / a_w;

    const auto hit = [&](double x, double y, double phi) {
        const double c = std::cos(phi), sn = std::sin(phi);
        for (const Vec2 & p : points_body) {
            const double dx = p.x() - x, dy = p.y() - y;
            // R(-phi) * (dx, dy)
            const Vec2 q(c * dx + sn * dy, -sn * dx + c * dy);
            bool inside = true;
            for (const HalfPlane & e : robot.half_planes) {
                if (!(e.normal.dot(q) <= e.offset + params.side_pad)) { inside = false; break; }
            }
            if (inside) return true;
        }
        return false;
    };

    double x = 0.0, y = 0.0, phi = 0.0;
    if (hit(x, y, phi)) return true;

    // Advances the pose through one phase of duration T in m steps; speeds(t) gives (u, w) at local time t.
    const auto phase = [&](double T, int m, const auto & speeds) {
        if (!(T > 0.0)) return false;
        const double h = T / m;
        for (int j = 0; j < m; ++j) {
            const auto [u, w] = speeds((j + 0.5) * h);
            if (std::abs(w * h) < 1e-9) {
                x += u * h * std::cos(phi + w * h / 2.0);
                y += u * h * std::sin(phi + w * h / 2.0);
            } else {
                x += (u / w) * (std::sin(phi + w * h) - std::sin(phi));
                y += (u / w) * (std::cos(phi) - std::cos(phi + w * h));
            }
            phi += w * h;
            if (hit(x, y, phi)) return true;
        }
        return false;
    };
    const auto steps = [&](double T) {
        return std::max(1, static_cast<int>(std::ceil(S * T / params.side_pad)));
    };
    using Speeds = std::pair<double, double>;

    const auto constant = [&](double) { return Speeds{v, omega}; };
    if (phase(reaction_time, steps(reaction_time), constant)) return true;

    const double T2 = std::min(tau_v, tau_w);
    const int m2 = std::max(steps(T2), static_cast<int>(std::ceil(std::sqrt(
        (av * a_w + a_v * aw) * T2 * T2 * T2 / (6.0 * params.side_pad)))));
    if (phase(T2, m2, [&](double t) { return Speeds{v - sv * a_v * t, omega - sw * a_w * t}; })) return true;

    const double T3 = std::abs(tau_v - tau_w);
    if (tau_v > tau_w) {
        if (phase(T3, steps(T3), [&](double t) { return Speeds{v - sv * a_v * (T2 + t), 0.0}; })) return true;
    } else {
        if (phase(T3, steps(T3), [&](double t) { return Speeds{0.0, omega - sw * a_w * (T2 + t)}; })) return true;
    }

    const double T4 = params.forward_pad / S;
    return phase(T4, steps(T4), constant);
}
