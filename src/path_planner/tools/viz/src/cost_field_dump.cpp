/**
 * @file cost_field_dump.cpp
 * @author Seif ElShafey
 * @brief Reads a scenario YAML (argv[1]) and writes to stdout one JSON object holding the cost of travel
 *        and the obstacle-aware cost-to-goal over the scenario map, plus the planned path. Input for
 *        tools/viz/plot_cost_field.py. Non-ROS, inspection-only.
 *
 * Keys: cols, rows (grid size, cells), h (cell size, m), u (cost added per metre in unknown space),
 * r_in, r_out (robot inscribed and circumscribed radii, m), start and goal ([x, y], m),
 * charge (cost of one metre of travel through each cell, 1 + costmap_weight * cost, -1 for an occupied
 * cell; row-major, row 0 first), field (obstacle-aware cost-to-goal of each cell in metres of travel
 * cost, row-major, -1 where there is no route to the goal) and path (the planned path as [x, y] points
 * in metres, an empty list if no path is found).
 *
 * Three measurements go to stderr: the unknown-space cost and the start estimates, the cost of the
 * planned path, and the worst ratio of the field to the straight-line distance on an open grid
 * (401 x 401 free cells at the scenario's resolution, goal at the centre cell, cells nearer than 1 m
 * skipped, both distances cell centre to cell centre).
 *
 * @copyright Copyright (c) 2026
 */

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"
#include "core/scenario/scenario_loader.hpp"
#include "core/scenario/scenario_planner.hpp"
// The cost of travel and the cost-to-goal field of the scenario, the planned path, and three measurements on stderr.
int main(int argc, char ** argv) {
    if (argc != 2) {
        std::cerr << "usage: cost_field_dump <scenario.yaml>\n";
        return 2;
    }
    const ScenarioSpec spec = loadScenario(argv[1]);
    const RobotParams robot = buildRobotParams(spec.robot_vertices);
    OccupancyGrid occ(spec.meta, spec.occupancy);
    EDT edt = buildEDTGrid(occ, spec.edt_clamp);
    auto snap = std::make_shared<const MapSnapshot>(MapSnapshot{std::move(occ), std::move(edt)});
    const double w = spec.a_star.costmap_weight;
    auto cm = std::make_shared<const CostMap>(buildCostMap(*snap, spec.cost_policy, robot.r_in, w));
    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    ObstacleHeuristic oh(grid, cm, w, 1.0);
    oh.prepare(spec.goal);
    const auto & m = spec.meta;

    BuiltScenario built = buildScenario(spec);
    const auto res = built.planner->plan(spec.start, spec.goal);

    std::cout << "{\"cols\":" << m.cols << ",\"rows\":" << m.rows << ",\"h\":" << m.resolution << ",\"u\":" << cm->unknownCost()
              << ",\"r_in\":" << robot.r_in << ",\"r_out\":" << robot.r_out
              << ",\"start\":[" << spec.start.x << "," << spec.start.y << "],\"goal\":[" << spec.goal.x << "," << spec.goal.y << "],\"charge\":[";
    for (int j = 0; j < m.rows; ++j)
        for (int i = 0; i < m.cols; ++i)
            std::cout << ((i || j) ? "," : "")
                      << (snap->grid_.findCellStatus(GridCoords{i, j}) == CellState::OCCUPIED ? -1.0 : 1.0 + w * cm->costAt(i, j));
    std::cout << "],\"field\":[";
    for (int j = 0; j < m.rows; ++j)
        for (int i = 0; i < m.cols; ++i) {
            const double v = oh.fieldAt(GridCoords{i, j});
            std::cout << ((i || j) ? "," : "") << (v >= ObstacleHeuristic::UNREACHABLE ? -1.0 : v);
        }
    std::cout << "],\"path\":[";
    bool first = true;
    if (res.path)
        for (const auto & seg : res.path->segments)
            for (const auto & q : seg.samples_poses) {
                std::cout << (first ? "" : ",") << "[" << q.x << "," << q.y << "]";
                first = false;
            }
    std::cout << "]}\n";

    std::cerr << "u=" << cm->unknownCost() << " r_in=" << robot.r_in << " h_obst(start)=" << oh(spec.start)
              << " h_euclid(start)=" << PointsWeightedEuclideanSE2(spec.diff_drive.rmin * spec.diff_drive.rmin)(spec.start, spec.goal) << "\n";
    if (res.path) std::cerr << "path cost=" << res.path->total_cost << "\n";

    // open grid: largest ratio of the field to the straight-line distance, cell centre to cell centre
    GridMetaData e = m;
    e.cols = 401;
    e.rows = 401;
    OccupancyGrid eo(e, std::vector<int8_t>(401 * 401, 0));
    EDT ee = buildEDTGrid(eo, spec.edt_clamp);
    auto es = std::make_shared<const MapSnapshot>(MapSnapshot{std::move(eo), std::move(ee)});
    auto ec = std::make_shared<const CostMap>(buildCostMap(*es, spec.cost_policy, robot.r_in, w));
    ObstacleHeuristic eh(std::shared_ptr<const OccupancyGrid>(es, &es->grid_), ec, w, 1.0);
    const GridCoords g{200, 200};
    eh.prepare(SE2State{e.origin.x() + 200.5 * e.resolution, e.origin.y() + 200.5 * e.resolution, 0.0});
    double worst = 0, wang = 0;
    for (int j = 0; j < 401; ++j)
        for (int i = 0; i < 401; ++i) {
            const double d = std::hypot(i - g.i, j - g.j) * e.resolution;
            if (d < 1.0) continue;
            const double r = eh.fieldAt(GridCoords{i, j}) / d;
            if (r > worst) {
                worst = r;
                wang = std::atan2(std::abs(j - g.j), std::abs(i - g.i)) * 180 / M_PI;
            }
        }
    std::cerr << "open-grid worst field/straight-line ratio=" << worst << " at " << wang << " deg\n";
}
