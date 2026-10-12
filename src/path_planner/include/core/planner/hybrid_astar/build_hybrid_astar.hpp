/**
 * @file build_hybrid_astar.hpp
 * @author Seif ElShafey
 * @brief Factory that wires a hybrid A* planner to a SLAM map snapshot.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>

#include "core/planner/planner_interface.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/map/map_provider.hpp"

/**
 * @brief Construct a hybrid A* planner bound to the supplied map snapshot.
 *
 * The factory builds the snapshot-dependent dependencies (collision checker, costmap, world-frame
 * state-space and motion context) and returns the planner as the polymorphic `Planner` interface
 * so callers can swap implementations without rewiring.
 *
 * @param kin_model Stable kinematic bundle (body, primitive library, kinematic params); the
 *                  cached primitive library is reused as-is, and the `motion` field is overridden
 *                  internally with a snapshot-aware MotionContext.
 * @param cfg Hybrid A* tuning parameters.
 * @param heur_cfg Heuristic tuning parameters.
 * @param cost_policy Clearance and unknown-cell cost weighting used to build the costmap.
 * @param snapshot Shared ownership of the SLAM map snapshot; the returned planner aliases its
 *                 grid and EDT, keeping the snapshot alive as long as the planner exists.
 * @return std::unique_ptr<Planner> Owning handle to the planner; `nullptr` is never returned.
 *
 * @throws std::invalid_argument If the kinematic model's cell size differs from the snapshot's
 *         grid resolution, or if the straight primitive length `vmax * atomic_time` [m] is not
 *         greater than `sqrt(2) * closedStride(cfg.closed_cell_size, resolution) * resolution` [m],
 *         i.e. sqrt(2) times `cfg.closed_cell_size` rounded to whole map cells (see
 *         HybridAstar::closedStride); or if `cost_policy.clearance_radius_` [m] is not greater than the
 *         robot's circumscribed radius `r_out`; or if the snapshot's
 *         distance-field limit [m] is not greater than the larger of `r_out + sqrt(2) * resolution`
 *         and the largest primitive sample clearance [m] (raise the map provider's `edt_max_dist`).
 *
 * @note Each call constructs a fresh planner bound to the supplied snapshot. The caller rebuilds
 *       via this factory whenever the snapshot changes; cells / EDT / costmap are not updated in
 *       place.
 */
std::unique_ptr<Planner>
buildHybridAstar(const KinematicModel & kin_model,
                 const AStarConfig & cfg,
                 const HeuristicConfig & heur_cfg,
                 const CostPolicy & cost_policy,
                 std::shared_ptr<const MapSnapshot> snapshot);
