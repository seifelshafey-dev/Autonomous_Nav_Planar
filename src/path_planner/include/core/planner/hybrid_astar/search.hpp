/**
 * @file search.hpp
 * @author Seif ElShafey
 * @brief Hybrid A* planner over differential-drive primitives, with EDT-aware collision
 *        checking and per-cell costmap penalties.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#include "core/planner/planner_interface.hpp"
#include "core/planner/path.hpp"
#include "core/planner/search_observer.hpp"
#include "core/planner/hybrid_astar/node.hpp"
#include "core/planner/hybrid_astar/open_set.hpp"
#include "core/planner/hybrid_astar/closed_set.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/grid.hpp"
#include "core/types/state.hpp"
#include "core/types/state_space_interface.hpp"

/**
 * @brief Counts describing the work done by one HybridAstar::plan() call.
 *
 * The four `fast_*` / `scan_*` counters hold exactly one verdict per candidate motion that reached
 * the collision check. Every other candidate is counted in `duplicate_skipped`, so
 * `fast_accept + fast_reject + scan_clear + scan_blocked + duplicate_skipped == P * (expansions - (a path was returned ? 1 : 0))`,
 * with `P` the number of primitives (the expansion that returns the path generates no candidates).
 * All fields are zero when plan() returns before searching.
 */
struct PlanStats {
    std::int64_t expansions   = 0;  ///< Nodes expanded.
    std::int64_t fast_accept  = 0;  ///< Candidate motions accepted without scanning the swept cells.
    std::int64_t fast_reject  = 0;  ///< Candidate motions rejected without scanning the swept cells.
    std::int64_t scan_clear   = 0;  ///< Candidate motions found clear by the full swept-cell scan.
    std::int64_t scan_blocked = 0;  ///< Candidate motions found blocked by the full swept-cell scan.
    std::int64_t exact_accept = 0;  ///< Exact re-checks performed on nodes whose swept-cell scan was blocked, found clear. `exact_accept + exact_reject <= scan_blocked`.
    std::int64_t duplicate_skipped = 0;  ///< Candidate motions dropped because a state already expanded in the same dedup cell and heading bin was at least as cheap, before any collision check.
    std::int64_t exact_reject = 0;  ///< Exact re-checks performed on nodes whose swept-cell scan was blocked, found blocked. Such a node is not counted in `expansions` and is not reported to an observer.
};

/**
 * @brief Hybrid A* planner for SE(2) state-space with cached differential-drive primitives.
 *
 * Call plan(start, goal) to obtain a PlanResult. plan() resets all internal search state
 * (open set, closed set, node pool, statistics) at entry, so the same HybridAstar instance can be reused
 * across queries — recreate only when the injected dependencies (kinematic model, costmap,
 * grid metadata, etc.) change. All external dependencies are injected via the constructor and
 * held by const shared pointers so the same dependency can be shared with other consumers.
 */
class HybridAstar : public Planner {
public:
    /**
     * @brief Width, in whole map cells, of the cell used to treat two states as duplicates.
     *
     * The nearest whole number of map cells to `closed_cell_size / resolution`, never less
     * than 1. The effective duplicate cell is `closedStride(...) * resolution` metres: it
     * differs from `closed_cell_size` when that is not a multiple of the resolution, and
     * equals the resolution when `closed_cell_size` is smaller than it.
     *
     * @param closed_cell_size Requested duplicate cell size [m]; must be > 0.
     * @param resolution Map resolution [m/cell]; must be > 0.
     * @return Cell width in map cells, >= 1.
     */
    static int32_t closedStride(double closed_cell_size, double resolution) noexcept {
        return std::max<int32_t>(1, static_cast<int32_t>(std::lround(closed_cell_size / resolution)));
    }

    /**
     * @brief Construct a hybrid A* planner with its required dependencies and tuning configuration.
     *
     * @param kin_model Robot motion identity: on-demand rasterization/sampling, cached primitive
     *                  library, and raw kinematic constants (rmin, vmax). Shared with other
     *                  motion-aware consumers.
     * @param checker Collision oracle; queried during expansion and for shot segments.
     * @param space Kinematic state space; provides propagation, distance, and interpolation.
     * @param heuristic Cost-to-goal estimator; evaluated once per child generation.
     * @param costmap Per-cell traversal cost field; added to g during expansion.
     * @param grid_meta World spatial discretization (origin, resolution, dims). Used for
     *                  `worldToGrid` when computing closed-set keys.
     * @param cfg Tunable parameters (tolerances, expansion limits, shot config, reserves).
     */
    HybridAstar(KinematicModel                               kin_model,
                std::shared_ptr<const CollisionChecker>      checker,
                std::shared_ptr<const State_Space<SE2State>> space,
                std::shared_ptr<const Heuristic>             heuristic,
                std::shared_ptr<const CostMap>               costmap,
                GridMetaData                                  grid_meta,
                AStarConfig                                   cfg);

    /**
     * @brief Plan a kinematically feasible, collision-free path from start to goal.
     *
     * @param start Start state in world coordinates.
     * @param goal Goal state in world coordinates; tolerances from AStarConfig determine acceptance.
     * @return PlanResult `Found` with the path on success; otherwise the reason (see `PlanOutcome`)
     *         and an empty `path`.
     */
    PlanResult plan(const SE2State & start, const SE2State & goal) override;

    /**
     * @brief Plan as above, additionally reporting each node expansion to `observer`.
     *
     * Carries the entire search; the two-argument overload delegates here with a null observer, so
     * the search proceeds identically whether or not an observer is supplied. When `observer` is
     * non-null, `observer->onExpand` fires exactly once per node accepted for expansion (after the
     * stale-skip check), in pop order; the reported `id`/`parent_id` are node-pool indices forming
     * the expanded-search tree. A node whose cached swept-cell scan was blocked is re-checked over its
     * exact continuous motion when taken for expansion; if blocked it is dropped and not reported.
     * Every state on a returned path, and the goal connection, has been confirmed collision-free over
     * its exact continuous motion before plan() returns. Observer events are emitted only for
     * confirmed nodes. A state satisfying the goal tolerances is never pruned as a duplicate of an
     * earlier state.
     *
     * @param start Start state in world coordinates.
     * @param goal Goal state in world coordinates; tolerances from AStarConfig determine acceptance.
     * @param observer Sink for expansion events; if null, no events are reported. Held only for the
     *        duration of the call and not retained.
     * @param stop When stop-requested, aborts the search and returns `Stopped`; a
     *        default-constructed token never stops.
     * @return PlanResult `Found` with the path on success. Otherwise, first match wins:
     *         `Stopped` if `stop` is set; `StartBlocked` if `CollisionChecker::startClear(start)`
     *         fails; `GoalBlocked` if `goal` is in collision; `BudgetExhausted` if the time limit
     *         or `max_expansions` is reached; `NoPath` if the search runs out of states.
     *         `path` is empty unless `Found`. `lastPlanStats()` is zero when plan() returns before searching.
     */
    PlanResult plan(const SE2State & start, const SE2State & goal,
                             std::shared_ptr<SearchObserver> observer,
                             std::stop_token stop = {}) override;

    /**
     * @brief Read-only access to the node pool from the most recent plan() call, for inspection and
     *        visualization (every generated node, with its parent_idx forming the search tree).
     *
     * May contain generated nodes whose motion was never confirmed collision-free (never taken for
     * expansion, or rejected when taken); only nodes reported to an observer or on the returned path
     * are confirmed.
     *
     * Not a search API — assertions on planner correctness should target plan()'s returned Path, not
     * this internal pool. Valid until the next plan() call (which clears it).
     *
     * @return const std::vector<Node>& The generated nodes in creation (expansion) order.
     */
    const std::vector<Node> & nodePool() const noexcept { return node_pool_; }

    /**
     * @brief Work counts from the most recent plan() call.
     *
     * All zero before the first call and when plan() returns before searching. Valid until the
     * next plan() call. Reading it does not affect planning. Counts cover the work done up to
     * the point plan() returned, including on failure or cancellation.
     *
     * @return PlanStats Copy of the counts.
     */
    PlanStats lastPlanStats() const noexcept { return stats_; }

private:
    /**
     * @brief Pack the closed-set key for a node, flooring its map-resolution grid coords to the
     *        coarser dedup cell given by `stride`.
     *
     * Key-only coarsening: only the packed `(i, j)` are divided by `stride`. The node's own
     * `grid_coords` stay at map resolution everywhere else (collision swept-cell lookup, costmap
     * line integral, worldToGrid), so feasibility and cost are unaffected — only the pruning
     * granularity coarsens. `stride == 1` reproduces the map-resolution key exactly. Both the
     * popped node and every child MUST use this same derivation or dedup breaks.
     *
     * @param gc Node grid coords at map resolution.
     * @param k Heading bin in `[0, n_headings)`.
     * @param stride Closed-cell width in whole map cells (>= 1).
     */
    ClosedKey keyOf(const GridCoords & gc, int16_t k, int32_t stride) const noexcept {
        return packClosedKey(floorDiv(gc.i, stride), floorDiv(gc.j, stride), k);
    }

    /// Floor division toward negative infinity, so the dedup cell tiles uniformly across the
    /// origin (truncating integer division would fold the cells straddling i=0 or j=0 together).
    static int32_t floorDiv(int32_t v, int32_t stride) noexcept {
        return (v >= 0) ? (v / stride) : -((-v + stride - 1) / stride);
    }

    bool atGoal(const SE2State & current) const noexcept;

    std::optional<Path> shotAtGoal(const SE2State & current) const;

    /**
     * @brief Check a motion over its exact continuous footprint sweep against the collision oracle.
     *
     * @param start Motion start pose, map frame [m, rad].
     * @param u Control applied for `u.dt`.
     * @param reject_unknown Whether UNKNOWN cells block.
     * @return true iff no cell the footprint can touch while executing `u` from `start` blocks.
     */
    bool exactMotionClear(const SE2State & start, const Control & u, bool reject_unknown) const;

    Path reconstructPath(int32_t goal_idx) const;

    KinematicModel kin_model_;
    std::shared_ptr<const CollisionChecker> checker_;
    std::shared_ptr<const State_Space<SE2State>> space_;
    std::shared_ptr<const Heuristic> heuristic_;
    std::shared_ptr<const CostMap> costmap_;
    GridMetaData grid_meta_;
    AStarConfig cfg_;

    OpenSet open_;
    ClosedSet closed_;
    std::vector<Node> node_pool_;

    SE2State goal_;
    PlanStats stats_;
};

inline HybridAstar::HybridAstar(KinematicModel kin_model,
                                std::shared_ptr<const CollisionChecker> checker,
                                std::shared_ptr<const State_Space<SE2State>> space,
                                std::shared_ptr<const Heuristic> heuristic,
                                std::shared_ptr<const CostMap> costmap,
                                GridMetaData grid_meta,
                                AStarConfig cfg)
    : kin_model_(std::move(kin_model)),
      checker_(std::move(checker)),
      space_(std::move(space)),
      heuristic_(std::move(heuristic)),
      costmap_(std::move(costmap)),
      grid_meta_(std::move(grid_meta)),
      cfg_(cfg)
{
    node_pool_.reserve(static_cast<size_t>(cfg_.node_pool_reserve));
    closed_.reserve(static_cast<size_t>(cfg_.closed_set_reserve));
}
