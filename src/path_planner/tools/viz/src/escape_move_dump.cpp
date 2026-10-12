/**
 * @file escape_move_dump.cpp
 * @author Seif ElShafey
 * @brief Runs the escape search for a robot standing 0.03 m into a wall and writes to stdout one JSON
 *        object describing the move it finds. Input for tools/viz/plot_navigation.py. Non-ROS,
 *        inspection-only.
 *
 * Usage: escape_move_dump <robot_params.yaml> <navigator.yaml> <controller.yaml>
 *
 * The map is 200 x 200 free cells of 0.05 m with a wall over cells i = 106..115 (all rows). The start
 * pose is (5.03, 5.03, 0), so the robot body overlaps the wall. `stop_short` is the controller's
 * `goal_pos_tol`: the move is lengthened so that every step pose in that last stretch passes the
 * start check.
 *
 * Keys: h (cell size, m), wall_x (x of the wall face, m), r_in (robot inscribed radius, m),
 * start ([x, y], m), delta (length of one checked step, m), slack (extra clearance asked of the
 * start check, m), stop_short (m), travel (length of the move, m, 0 if no move is found), v (signed
 * speed of the move, m/s, 0 if no move is found) and steps (one entry per step pose, running four
 * poses past the end of the move, generated in reverse from the start, which is the direction the
 * search picks in this scene: [x of the step pose in m, passes the start check, passes it with
 * slack], the two checks as 0 or 1).
 *
 * One line of measurements goes to stderr: travel=<move length, m> steps=<number of steps>
 * max_steps=<step cap> crawl(1)=<the follower's final approach time bound at speed scale 1, s>
 * turn=<final turn time bound, s> crawl(scale)=<approach bound at the escape speed scale, s>
 * stall_timeout=<configured stall_timeout, s>. The approach and turn bounds are recomputed here with
 * the same formula as the navigator's start-up check, not read from the navigator.
 *
 * @copyright Copyright (c) 2026
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "core/planner/escape/escape_search.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/config/config_loader.hpp"
#include "core/control/pure_pursuit.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/motion/kinematic_model.hpp"

namespace {
constexpr double kResolution = 0.05;     // cell size, m
constexpr int kGridCells = 200;          // cells per side
constexpr int kWallFirstColumn = 106;    // first occupied column, inclusive
constexpr int kWallLastColumn = 115;     // last occupied column, inclusive
constexpr double kStartX = 5.03;         // start pose, m
constexpr double kStartY = 5.03;
}  // namespace

// The escape move from a start inside a wall: JSON on stdout, one line of measurements on stderr.
int main(int argc, char ** argv) {
    if (argc != 4) {
        std::cerr << "usage: escape_move_dump <robot_params.yaml> <navigator.yaml> <controller.yaml>\n";
        return 2;
    }
    const RobotParams body = loadRobotParams(argv[1]);
    const DiffDriveParams kin = loadDiffDriveParams(argv[1]);
    const NavigatorParams nav = loadNavigatorParams(argv[2]);
    const ControllerParams cp = loadControllerParams(argv[3]);
    const double h = kResolution;
    const int N = kGridCells;
    GridMetaData meta(N, N, h, Eigen::Vector2f::Zero());
    std::vector<int8_t> cells(static_cast<size_t>(N) * N, 0);
    for (int j = 0; j < N; ++j)
        for (int i = kWallFirstColumn; i <= kWallLastColumn; ++i) cells[static_cast<size_t>(j) * N + i] = 100;
    auto grid = std::make_shared<const OccupancyGrid>(meta, cells);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid, 5.0));
    CollisionChecker checker(grid, edt, body);
    const KinematicModel km = buildKinematicModel(body, kin, h);
    const SE2State start{kStartX, kStartY, 0.0};
    const double stop_short = cp.goal_pos_tol;
    const auto p = findEscape(start, checker, *km.motion, kin, nav.escape, stop_short);
    const double delta = nav.escape.step_fraction * h, slack = nav.escape.slack_cells * h;
    const double travel = p && !p->segments.empty() ? std::abs(p->segments[0].u.v) * p->segments[0].u.dt : 0.0;
    const int K = static_cast<int>(std::lround(travel / delta));
    std::cout << "{\"h\":" << h << ",\"wall_x\":" << kWallFirstColumn * h << ",\"r_in\":" << body.r_in
              << ",\"start\":[" << kStartX << "," << kStartY << "],\"delta\":" << delta
              << ",\"slack\":" << slack << ",\"stop_short\":" << stop_short << ",\"travel\":" << travel
              << ",\"v\":" << (p && !p->segments.empty() ? p->segments[0].u.v : 0.0) << ",\"steps\":[";
    for (int k = 0; k <= K + 4; ++k) {
        const SE2State q{start.x - k * delta, start.y, 0.0};
        std::cout << (k ? "," : "") << "[" << q.x << "," << checker.startClear(q, 0.0) << ","
                  << checker.startClear(q, slack) << "]";
    }
    std::cout << "]}\n";

    const double spacing = body.r_in / 2, L = cp.lookahead_gain * spacing;
    auto crawl = [&](double s) {
        return (L / (s * cp.v_cruise)) * std::max({std::log(L / cp.goal_pos_tol), spacing / L, std::log(3.0)});
    };
    std::cerr << "travel=" << travel << " steps=" << K << " max_steps=" << std::floor(nav.escape.max_travel_factor * body.r_out / delta)
              << " crawl(1)=" << crawl(1.0) << " turn=" << PurePursuit::goalAlignTimeBound(cp.omega_pivot, cp.goal_heading_tol)
              << " crawl(scale)=" << crawl(nav.escape_speed_scale) << " stall_timeout=" << nav.stall_timeout << "\n";
}
