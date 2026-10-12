/**
 * @file primitives_dump.cpp
 * @author Seif ElShafey
 * @brief Reads a robot params YAML (argv[1]) and writes to stdout one JSON object describing the control
 *        set of the robot: each motion's control, cost and path from one start pose, and the states
 *        reached after one, two and three motions. Input for tools/viz/plot_primitives.py. Non-ROS,
 *        inspection-only.
 *
 * Grid resolution is 0.05 m and the start pose is (0.02, 0.03, 0.0); the grid the plot draws assumes a
 * map origin at (0, 0) with that fixed 0.05 m resolution. Keys: h (cell size, m), rmin (m),
 * vmax (m/s), n_headings, start ([x, y, theta]), prims (one entry per motion with v (m/s), w (rad/s),
 * dt (s), cost (m), end (endpoint offset in the body frame) and path (21 poses [x, y, theta] along the
 * motion from the start pose)), levels (three lists of poses reached after one, two and three
 * motions). Poses are in metres and radians and are continuous, never snapped to the grid.
 *
 * @copyright Copyright (c) 2026
 */

#include <cmath>
#include <iostream>
#include <vector>
#include "core/config/config_loader.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/primitives/build_diff_prim_lib.hpp"
// The control set of the robot in the given YAML from one pose, and the states reached after one, two and three motions.
int main(int argc, char ** argv) {
    if (argc < 2) {
        std::cerr << "usage: primitives_dump <robot_params.yaml>\n";
        return 2;
    }
    const RobotParams body = loadRobotParams(argv[1]);
    const DiffDriveParams kin = loadDiffDriveParams(argv[1]);
    const double h = 0.05;
    const KinematicModel km = buildKinematicModel(body, kin, h);
    const auto & prims = km.prim_lib->primitives();
    const auto & space = km.motion->space();
    const SE2State s0{0.02, 0.03, 0.0};
    std::cout << "{\"h\":" << h << ",\"rmin\":" << kin.rmin << ",\"vmax\":" << kin.vmax << ",\"n_headings\":" << kin.n_headings
              << ",\"start\":[" << s0.x << "," << s0.y << "," << s0.theta << "],\"prims\":[";
    for (std::size_t p = 0; p < prims.size(); ++p) {
        const auto & u = prims[p].control;
        std::cout << (p ? "," : "") << "{\"v\":" << u.v << ",\"w\":" << u.omega << ",\"dt\":" << u.dt << ",\"cost\":" << prims[p].cost
                  << ",\"end\":[" << prims[p].endpoint_offset.x << "," << prims[p].endpoint_offset.y << ","
                  << prims[p].endpoint_offset.theta << "],\"path\":[";
        for (int j = 0; j <= 20; ++j) {
            const SE2State s = space.propagate(s0, Control{u.v, u.omega, u.dt * j / 20});
            std::cout << (j ? "," : "") << "[" << s.x << "," << s.y << "," << s.theta << "]";
        }
        std::cout << "]}";
    }
    std::cout << "],\"levels\":[";
    std::vector<SE2State> frontier{s0};
    for (int level = 1; level <= 3; ++level) {
        std::vector<SE2State> next;
        for (const auto & s : frontier)
            for (const auto & pr : prims) next.push_back(space.propagate(s, pr.control));
        std::cout << (level > 1 ? "," : "") << "[";
        for (std::size_t i = 0; i < next.size(); ++i)
            std::cout << (i ? "," : "") << "[" << next[i].x << "," << next[i].y << "," << next[i].theta << "]";
        std::cout << "]";
        frontier = std::move(next);
    }
    std::cout << "]}\n";
}
