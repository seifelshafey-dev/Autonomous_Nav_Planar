/**
 * @file scenario_planner.hpp
 * @author Seif ElShafey
 * @brief Build-helper that turns a resolved ScenarioSpec into a runnable planner bound to its map
 *        snapshot, reusing the production factory chain.
 * @version 0.1
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>

#include "core/scenario/scenario.hpp"
#include "core/robot/robot_params.hpp"
#include "core/map/map_provider.hpp"
#include "core/planner/planner_interface.hpp"

/**
 * @brief A scenario instantiated into the live objects needed to run and inspect one plan.
 *
 * The three members are co-dependent: the planner aliases the snapshot's occupancy grid and EDT
 * (it does not copy them), so the snapshot MUST outlive the planner. Keeping the whole
 * `BuiltScenario` alive guarantees this; do not move the planner out and drop the snapshot.
 */
struct BuiltScenario {
    std::shared_ptr<const MapSnapshot> snapshot;  ///< Occupancy + EDT the planner aliases into; must outlive `planner`.
    std::unique_ptr<Planner> planner;             ///< Hybrid A* planner bound to `snapshot`.
    RobotParams robot;                            ///< Footprint polygon plus r_in / r_out for visualization and assertions.
};

/**
 * @brief Instantiate a runnable planner and its map snapshot from a resolved scenario.
 *
 * Wires the production factory chain (robot geometry, kinematic model, occupancy grid, EDT, and
 * `buildHybridAstar`) around the scenario's inputs. The returned planner is ready to accept
 * `spec.start` / `spec.goal` plan queries.
 *
 * @param spec Fully resolved scenario inputs. Grid resolution is read solely from `spec.meta`
 *             (`spec.meta.resolution`); no other resolution source is consulted, so the primitive
 *             library, closed-set keying, EDT, and costmap all agree on cell size.
 * @return BuiltScenario Owning bundle of snapshot + planner + robot geometry. The caller must keep
 *         `snapshot` alive for the planner's lifetime (holding the whole `BuiltScenario` suffices).
 * @throws std::invalid_argument if `spec.robot_vertices` is not a valid convex CCW polygon
 *         containing the body origin (propagated from `buildRobotParams`).
 */
BuiltScenario buildScenario(const ScenarioSpec & spec);
