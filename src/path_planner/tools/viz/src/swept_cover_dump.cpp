/**
 * @file swept_cover_dump.cpp
 * @author Seif ElShafey
 * @brief Reads a robot params YAML (argv[1]) and writes to stdout one JSON object describing the cached
 *        swept-cell cover of the straight forward primitive for the heading bin that starts at heading 0,
 *        next to the exact cells for a fan of start headings across that bin. Input for
 *        tools/viz/plot_swept_cover.py. Non-ROS, inspection-only.
 *
 * Grid resolution is 0.05 m. Keys: h (cell size, m), delta (bin width, rad), theta_k (bin start heading,
 * rad), v (m/s), dt (s), cover (cached cells), fan (17 entries, each with start and end poses
 * [x, y, theta] and the exact cells), verts (body polygon, robot frame, m). Cell indices [i, j] are
 * relative to the robot's cell. Poses are in metres and radians with the robot cell's lower-left corner
 * at the origin.
 *
 * @copyright Copyright (c) 2026
 */

#include <cmath>
#include <iostream>
#include <numbers>
#include "core/config/config_loader.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/primitives/build_diff_prim_lib.hpp"
// Straight forward motion, one heading bin: the cached cover, and the exact cells for a fan of start headings.
int main(int argc, char ** argv) {
    if (argc < 2) {
        std::cerr << "usage: swept_cover_dump <robot_params.yaml>\n";
        return 2;
    }
    const RobotParams body = loadRobotParams(argv[1]);
    const DiffDriveParams kin = loadDiffDriveParams(argv[1]);
    const double h = 0.05;
    const KinematicModel km = buildKinematicModel(body, kin, h);
    const double delta = 2.0 * std::numbers::pi / kin.n_headings;
    const int k = kin.n_headings / 2;                       // bin starting at heading 0
    const double theta_k = -std::numbers::pi + k * delta;
    for (const auto & pr : km.prim_lib->primitives()) {
        if (!(pr.control.v > 0.0) || pr.control.omega != 0.0) continue;
        std::cout << "{\"h\":" << h << ",\"delta\":" << delta << ",\"theta_k\":" << theta_k << ",\"v\":" << pr.control.v
                  << ",\"dt\":" << pr.control.dt << ",\"cover\":[";
        for (std::size_t i = 0; i < pr.swept_cells[k].size(); ++i)
            std::cout << (i ? "," : "") << "[" << pr.swept_cells[k][i].i << "," << pr.swept_cells[k][i].j << "]";
        std::cout << "],\"fan\":[";
        const int F = 16;
        for (int f = 0; f <= F; ++f) {
            const SE2State start{0.5 * h, 0.5 * h, theta_k + delta * f / F};
            const auto exact = km.motion->computeExactCover(start, pr.control);
            const SE2State end = km.motion->space().propagate(start, pr.control);
            std::cout << (f ? "," : "") << "{\"start\":[" << start.x << "," << start.y << "," << start.theta << "],\"end\":["
                      << end.x << "," << end.y << "," << end.theta << "],\"cells\":[";
            for (std::size_t i = 0; i < exact.cells.size(); ++i)
                std::cout << (i ? "," : "") << "[" << exact.cells[i].i << "," << exact.cells[i].j << "]";
            std::cout << "]}";
        }
        std::cout << "],\"verts\":[";
        for (std::size_t i = 0; i < body.vertices.size(); ++i)
            std::cout << (i ? "," : "") << "[" << body.vertices[i].x() << "," << body.vertices[i].y() << "]";
        std::cout << "]}\n";
        break;
    }
}
