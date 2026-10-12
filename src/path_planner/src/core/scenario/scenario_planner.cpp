/**
 * @file scenario_planner.cpp
 * @author Seif ElShafey
 * @brief Implementation of the ScenarioSpec -> runnable planner build-helper.
 * @version 0.1
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/scenario/scenario_planner.hpp"

#include <utility>

#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/hybrid_astar/build_hybrid_astar.hpp"

BuiltScenario buildScenario(const ScenarioSpec & spec) {
    // Robot geometry: validates convexity / CCW winding / origin containment and derives the
    // half-planes plus the inscribed/circumscribed radii.
    RobotParams robot = buildRobotParams(spec.robot_vertices);

    // Stable kinematic bundle. Resolution comes from spec.meta so the cached primitive library is
    // sized to the same cell as the search keying / EDT / costmap below.
    KinematicModel kin = buildKinematicModel(robot, spec.diff_drive, spec.meta.resolution);

    // Occupancy + EDT share spec.meta; both are non-default-constructible, so the MapSnapshot
    // aggregate is brace-initialized with both members in one shot and held as shared_ptr<const>.
    OccupancyGrid occupancy(spec.meta, spec.occupancy);
    EDT edt = buildEDTGrid(occupancy, spec.edt_clamp);

    auto snapshot = std::make_shared<const MapSnapshot>(
        MapSnapshot{std::move(occupancy), std::move(edt)});

    // Factory wires collision checker, costmap, snapshot-aware motion context, and heuristic, then
    // returns the planner aliasing the snapshot's grid and EDT.
    auto planner = buildHybridAstar(kin, spec.a_star, spec.heuristic, spec.cost_policy, snapshot);

    return BuiltScenario{std::move(snapshot), std::move(planner), std::move(robot)};
}
