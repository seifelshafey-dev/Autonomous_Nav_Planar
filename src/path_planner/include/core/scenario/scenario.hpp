/**
 * @file scenario.hpp
 * @author Seif ElShafey
 * @brief Resolved, in-memory description of a single planning scenario shared by the integration
 *        tests and the visualization tooling.
 * @version 0.1
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/types/state.hpp"
#include "core/types/geometry.hpp"
#include "core/map/grid.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/cost/cost_policy.hpp"

/**
 * @brief Coarse, design-intent properties a scenario is authored to exhibit; consumed only by the
 *        test oracle (the visualization ignores this block).
 *
 * These are loose sanity bounds chosen by the scenario author from the scenario's geometry and
 * intent, NOT readings copied from a planner run. They must never be set to the planner's own
 * dumped output: an oracle derived from the result it is meant to check is self-fulfilling and
 * proves nothing. Leave the optional ceilings unset (`std::nullopt`) when no meaningful bound is
 * known rather than back-filling a value from a previous run.
 *
 * @note Tier-0 universal invariants (path starts at `start`, ends within goal tolerance, every
 *       segment is collision-free, costs are finite and monotonic, etc.) are NOT expressed here;
 *       they hold for every scenario and are asserted directly in the test code.
 */
struct Expectations {
    bool expect_found = false;                   ///< Whether a feasible path is known to exist for this scenario by design intent.
    std::optional<double> cost_max;              ///< Optional loose upper ceiling on total path cost; a sanity bound, not a target. Unset = unbounded.
    std::optional<int> expansions_max;           ///< Optional loose ceiling on the number of expanded nodes (node-pool size). Unset = unbounded.
    std::optional<int> unknown_poses_max;        ///< Optional ceiling (>= 0) on the number of sampled path poses whose grid cell is UNKNOWN or outside the grid. Unset = not checked.
};

/**
 * @brief Fully resolved input for one planning scenario: map geometry, occupancy, robot, planner
 *        configuration, and the start/goal query, plus the test oracle.
 *
 * This is a plain value aggregate. A loader populates it from on-disk assets (YAML + an occupancy
 * image) in a later phase; the integration tests and the visualization both consume an already
 * resolved instance and never re-parse those assets.
 *
 * @note Grid geometry — including resolution — lives solely in `meta` (`meta.resolution`). Every
 *       consumer that needs the cell size (the primitive library sizing swept-cell footprints, the
 *       closed-set grid keying, the EDT, and the costmap) reads it from there, so they all agree on
 *       one resolution.
 */
struct ScenarioSpec {
    std::string name;                            ///< Human-readable identifier; used for test-case naming and visualization labeling.

    GridMetaData meta;                           ///< Sole carrier of grid geometry (cols, rows, resolution, origin). All resolution reads come from here.
    std::vector<int8_t> occupancy;               ///< Row-major occupancy, `meta.rows * meta.cols` cells. Per cell: -1 = UNKNOWN, [0, 49] = FREE, [50, 100] = OCCUPIED (matches `OccupancyGrid`/`GridMetaData::occupied_threshold`).
    double edt_clamp = 5.0;                       ///< [m] EDT clamping distance handed to `buildEDTGrid`; cells farther than this report this value. Default matches `buildEDTGrid`'s default.

    std::vector<Vec2> robot_vertices;            ///< Body-frame robot footprint vertices (m), convex and CCW.
    DiffDriveParams diff_drive;                  ///< Differential-drive kinematic limits plus the primitive-library discretization knobs.

    AStarConfig a_star;                          ///< Hybrid A* search tuning (tolerances, caps, shot, reserves, costmap weight).
    HeuristicConfig heuristic;                   ///< Heuristic weight and obstacle-heuristic selection.
    CostPolicy cost_policy;                       ///< Clearance/unknown costmap penalty rates.

    SE2State start;                              ///< World-frame start pose (x, y, theta).
    SE2State goal;                               ///< World-frame goal pose (x, y, theta).

    Expectations expect;                         ///< Coarse design-intent oracle for the test layer; ignored by visualization.
};
