/**
 * @file stop_zone_dump.cpp
 * @author Seif ElShafey
 * @brief Evaluates `motionBlocked` for three commands of the shipped robot on a 5 mm grid of single
 *        points in the robot frame and writes to stdout one JSON object. Input for
 *        tools/viz/plot_stop_zone.py. Non-ROS, inspection-only.
 *
 * Usage: stop_zone_dump <robot_params.yaml> <scan_safety_guard.yaml> [reaction_time_s]
 *
 * The reaction time defaults to 0.2 s. The three commands are a straight run at 0.5 m/s, an arc at
 * 0.333333 m/s with 0.555556 rad/s, and a turn on the spot at 1.0 rad/s. The grid covers
 * [-0.6, 0.9] m in x and y in the robot frame.
 *
 * Keys: tr (reaction time, s), lo (lowest grid coordinate, m), step (grid spacing, m), n (points per
 * side), r_in (robot inscribed radius, m), decel ([linear, angular] deceleration, m/s^2 and
 * rad/s^2), pads ([forward, side] padding, m) and cases (one entry per command: v in m/s, w in
 * rad/s, and mask). `mask` is n*n characters, row-major from the lowest y, '1' where a single
 * return at that point would hold the robot.
 *
 * Stderr gets one line per command: v=<m/s> w=<rad/s> front_reach_on_axis=<largest x on the axis
 * y = 0 that is in the zone, m> zone_area=<zone area, m^2>, both at grid resolution. Then five lines,
 * one per distance on the x axis, saying whether a single return there blocks a reverse command and
 * a forward command of 0.25 m/s.
 *
 * @copyright Copyright (c) 2026
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "core/config/config_loader.hpp"
#include "core/control/stop_zone.hpp"

namespace {
constexpr double kDefaultReactionTime = 0.2;  // s
constexpr double kCommands[3][2] = {{0.5, 0.0}, {0.333333, 0.555556}, {0.0, 1.0}};  // {v m/s, w rad/s}
constexpr double kGridLow = -0.6;             // lowest grid coordinate, m
constexpr double kGridHigh = 0.9;             // highest grid coordinate, m
constexpr double kGridStep = 0.005;           // grid spacing, m
constexpr double kAxisTol = 1e-9;             // |y| below this counts as on the axis, m
constexpr double kReturnX[] = {0.295, 0.305, 0.312, 0.32, 0.40};  // single returns on the axis, m
constexpr double kReturnSpeed = 0.25;         // speed of the reverse and forward commands, m/s
}  // namespace

// The stop zone of three commands: JSON on stdout, measurements on stderr.
int main(int argc, char ** argv) {
    if (argc != 3 && argc != 4) {
        std::cerr << "usage: stop_zone_dump <robot_params.yaml> <scan_safety_guard.yaml> [reaction_time_s]\n";
        return 2;
    }
    const RobotParams robot = loadRobotParams(argv[1]);
    const ScanGuardParams g = loadScanGuardParams(argv[2]);
    const StopZoneParams p{g.linear_decel, g.angular_decel, g.forward_pad, g.side_pad};
    const double tr = argc == 4 ? std::stod(argv[3]) : kDefaultReactionTime;
    const double lo = kGridLow, st = kGridStep;
    const int n = static_cast<int>(std::lround((kGridHigh - lo) / st)) + 1;
    std::cout << "{\"tr\":" << tr << ",\"lo\":" << lo << ",\"step\":" << st << ",\"n\":" << n << ",\"r_in\":" << robot.r_in
              << ",\"decel\":[" << p.linear_decel << "," << p.angular_decel << "],\"pads\":[" << p.forward_pad << "," << p.side_pad << "],\"cases\":[";
    for (int c = 0; c < 3; ++c) {
        const double v = kCommands[c][0], w = kCommands[c][1];
        std::cout << (c ? "," : "") << "{\"v\":" << v << ",\"w\":" << w << ",\"mask\":\"";
        double front = 0, area = 0;
        for (int j = 0; j < n; ++j) for (int i = 0; i < n; ++i) {
            const Vec2 q(lo + i * st, lo + j * st);
            const bool b = motionBlocked(robot, {q}, v, w, tr, p);
            std::cout << (b ? '1' : '0');
            if (b) { area += st * st; if (std::abs(q.y()) < kAxisTol) front = std::max(front, q.x()); }
        }
        std::cout << "\"}";
        std::cerr << "v=" << v << " w=" << w << " front_reach_on_axis=" << front << " zone_area=" << area << "\n";
    }
    std::cout << "]}\n";
    for (double x : kReturnX)
        std::cerr << "return at x=" << x << " m ahead of centre: reverse -0.25 m/s blocked="
                  << motionBlocked(robot, {Vec2(x, 0.0)}, -kReturnSpeed, 0.0, tr, p)
                  << " forward 0.25 blocked=" << motionBlocked(robot, {Vec2(x, 0.0)}, kReturnSpeed, 0.0, tr, p) << "\n";
}
