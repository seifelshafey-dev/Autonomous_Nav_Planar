/**
 * @file cost_policy.hpp
 * @author Seif ElShafey
 * @brief Defines an interface for the clearance and unknown penalties in map traversal.
 * @version 0.1
 * @date 2026-05-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

/**
 * @brief Tuning parameters for clearance-aware inflation and the unknown-cell policy.
 *
 * Cells close to obstacles are penalized for safety: within an inflation radius around each obstacle
 * the clearance cost decays exponentially with distance, so a cell is strongly avoided when very close
 * to an obstacle and only gently biased against as clearance grows. Unknown cells carry a separate
 * traversal penalty, set relative to the dearest known free space, that lets the planner explore
 * unmapped space with tunable caution.
 *
 * @note Occupied cells are assigned a high finite cost rather than infinity; in practice this steers
 *       the planner onto free paths. This is a soft cost only — actual collision checking is performed
 *       upstream by the planner.
 */

struct CostPolicy{
    float clearance_radius_; ///< Defines the area around an obstacle that is penalized by a clearance cost. A cell whose distance is >= this has no clearance cost; below it the cost is `max_clearance_cost_ * exp(-clearance_decay_ * d)`. Metres; must exceed the robot's circumscribed radius `r_out`, otherwise planner assembly fails.
    float clearance_decay_; ///< Rate of clearance-penalty falloff with distance from obstacles. The higher, the less affected the far cells. Units 1/m.
    float max_clearance_cost_; ///< Clearance cost at distance 0, i.e. at occupied cells (dimensionless; multiplied by costmap_weight in the search's per-metre charge).
    float unknown_cost_factor_; ///< Cost of a metre of unknown or out-of-map space as a multiple of the dearest metre of known free space the robot can occupy (dimensionless, >= 1). Higher values make the planner less willing to explore; the per-metre cost is derived in buildCostMap().
};