/**
 * @file config.hpp
 * @author Seif ElShafey
 * @brief Tunable configuration parameters for the hybrid A* planner.
 * @version 0.1
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

/**
 * @brief Tunable parameters for the hybrid A* planner. Defaults are conservative, with
 *        generous reserves. Loaded from YAML via loadAStarConfig().
 *
 * @note Defaults are applied for any field not present (or null) in the YAML source. Override
 *       only the fields you intend to tune.
 */
struct AStarConfig {
    // --- Termination ---
    double  goal_pos_tol       = 0.10;     ///< [m] position threshold for goal acceptance.
    double  goal_heading_tol   = 0.10;     ///< [rad] heading threshold for goal acceptance.
    int32_t max_expansions     = 200000;   ///< Hard cap on expanded nodes; plan() returns `BudgetExhausted` when reached.
    double  max_plan_time_sec  = 5.0;      ///< Wall-clock budget for a single plan() call, measured from its entry; plan() returns `BudgetExhausted` when exceeded. The budget is checked periodically, so a call can run slightly past it.

    // --- Analytic shot ---
    bool    enable_shot        = true;     ///< Whether to attempt a direct connection to the goal (turn, straight, turn) from each expanded node.
    double  shot_max_distance  = 5.0;      ///< [m] only attempt shot if within this radius of the goal.
    bool    shot_allow_unknown = false;    ///< If true the shot treats UNKNOWN cells as traversable
                                           ///< (optimistic); only safe when a faster reactive/replanning
                                           ///< layer guards traversal. Default false = conservative
                                           ///< KNOWN-FREE shot (UNKNOWN blocks it).

    // --- Closed-set dedup ---
    double  closed_cell_size   = 0.15;     ///< [m] Closed-set dedup cell size. The search floors grid
                                           ///< coords to this cell (key only) before packing the closed
                                           ///< key, decoupling pruning granularity from the map
                                           ///< resolution. Larger = fewer expansions, slightly
                                           ///< suboptimal paths. Must be > 0. The planner uses the nearest whole number of map
                                           ///< cells (at least one), so the effective cell is a multiple of the map resolution.
                                           ///< The straight primitive's length (`vmax * atomic_time`)
                                           ///< must exceed sqrt(2) times that effective cell, otherwise
                                           ///< building the planner fails.

    // --- Storage hints ---
    int32_t node_pool_reserve  = 200000;   ///< Initial reservation for the node pool.
    int32_t closed_set_reserve = 200000;   ///< Initial reservation for the closed set.

    // --- Cost policy weighting ---
    double  costmap_weight     = 1.0;      ///< Multiplier on per-cell costmap penalty added to g.
};
