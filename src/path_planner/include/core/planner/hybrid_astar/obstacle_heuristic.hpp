/**
 * @file obstacle_heuristic.hpp
 * @author Seif ElShafey
 * @brief Obstacle-aware cost-to-goal heuristic for the hybrid A* planner.
 * @version 0.1
 * @date 2026-06-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/map/grid.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/types/grid_coords.hpp"
#include "core/types/state.hpp"

/**
 * @brief Cost-to-goal estimate that routes around obstacles and through the costmap.
 *
 * The field is valid for the grid and costmap the object was constructed with.
 *
 * The estimate is the least-cost path from a grid cell to the goal cell over the occupancy grid
 * (obstacles are impassable, unknown cells are passable at their costmap cost, and each step toward the
 * goal costs its length (one cell side, or a cell diagonal) times `1 + costmap_weight * cost` of the
 * cell it leaves, so the queried cell is charged and the goal cell is not). The estimate treats the
 * robot as a point and ignores heading. It is not a lower bound on the path cost: where the costmap is zero an
 * 8-connected distance exceeds the straight-line distance by up to about 8 percent, and positions are
 * measured between cell centres (up to about one cell of error). No bound is stated for a varying
 * costmap. `prepare(goal)` only
 * selects the goal; the cost field is built on demand as cells are queried. The first query of a cell
 * may take up to one pass over the grid; later queries of already covered cells are O(1). Returned
 * values do not depend on the order of queries. Not safe to call concurrently on one object,
 * including `operator()` and `fieldAt`.
 *
 * `operator()(current)` returns, for the cell containing `current`:
 *   - the weighted routed cost-to-goal, when that cell can reach the goal (high when a detour is
 *     forced — so the planner is guided around obstacle pockets rather than into them);
 *   - a large value (`weight * UNREACHABLE`), when the cell is an obstacle or is walled off from the
 *     goal — steering the search away from dead ends;
 *   - 0, when `current` lies outside the grid, or when the goal lies outside the grid or on an occupied
 *     cell (the field cannot be rooted) — so a combining heuristic (e.g. MaxHeuristic) defers to its other
 *     estimate and frontier / off-grid goals stay reachable.
 */
class ObstacleHeuristic : public Heuristic {
public:
    /// Field sentinel for cells with no path to the goal; large and finite so `g + h` never yields NaN/inf.
    static constexpr double UNREACHABLE = 3.4028234663852886e38;

    /**
     * @param grid           Occupancy grid the field is computed over; defines traversability.
     * @param costmap         Per-cell penalty accumulated along the path. Its grid must match `grid`.
     * @param costmap_weight  Penalty multiplier; must equal the search's `AStarConfig::costmap_weight`
     *                        so the estimate is comparable to the cost the search accumulates.
     * @param weight          Heuristic multiplier (1.0 = unscaled).
     */
    ObstacleHeuristic(std::shared_ptr<const OccupancyGrid> grid,
                      std::shared_ptr<const CostMap>       costmap,
                      double                               costmap_weight,
                      double                               weight = 1.0);

    /**
     * @brief Selects the goal grid cell that `operator()` and `fieldAt` measure cost to.
     *
     * Only the cell of `goal.x, goal.y` matters; sub-cell position and heading are ignored. A call whose
     * goal falls in the same grid cell as the previous call returns immediately and keeps the estimates
     * computed so far. A call for a new goal cell discards them; it costs O(grid cells) and no search.
     * Only the most recent goal cell is remembered. Not safe to call concurrently on one object.
     *
     * @param goal  Goal pose, world frame, metres. May lie off-grid or on an occupied cell, in which case
     *              `operator()` returns 0 until a usable goal is prepared.
     */
    void   prepare(const SE2State & goal) const override;
    double operator()(const SE2State & current) const override;

    /// Raw (unweighted) cost-to-goal for a cell; `UNREACHABLE` if it has no path, is off-grid, is an
    /// obstacle, or no usable goal has been prepared. The first query of a cell may take up to one pass
    /// over the grid. Not safe to call concurrently on one object.
    /// @param gc Grid cell to query.
    /// @return Least cost from `gc` to the goal cell, or `UNREACHABLE` in the cases above.
    double fieldAt(const GridCoords & gc) const;

private:
    /// Advances the wave from the goal until `target` (a linear cell index) is settled or the queue is exhausted.
    void settleUntil(std::size_t target) const;

    std::shared_ptr<const OccupancyGrid> grid_;
    std::shared_ptr<const CostMap>       costmap_;
    GridMetaData                         meta_;
    double                               costmap_weight_;
    /// Row-major per-cell edge multiplier baked once at construction: `1 + costmap_weight * costAt(cell)`
    /// for traversable cells (the cost of *entering* the cell, scaled later by the geometric step), or
    /// `BLOCKED` for OCCUPIED cells. Reused across every `prepare`, so the Dijkstra inner loop reads one
    /// contiguous static array instead of re-deriving occupancy + costmap per neighbour.
    std::vector<double>                  edge_mul_;
    std::size_t                          n_buckets_ = 0;  ///< Bucket count of the queue, fixed at construction.
    mutable std::vector<double>          cost_to_goal_;  ///< Row-major field; final only for settled cells.
    mutable std::vector<std::vector<int32_t>> buckets_;  ///< Cyclic bucket queue of cell indices.
    mutable std::vector<char>            settled_;       ///< 1 once a cell's final distance is fixed.
    mutable std::size_t                  b_ = 0;             ///< Absolute index of the bucket being drained.
    mutable std::size_t                  scanned_empty_ = 0; ///< Consecutive empty buckets seen.
    mutable std::size_t                  remaining_ = 0;     ///< Cells not yet settled.
    mutable std::optional<GridCoords>    prepared_cell_;  ///< Goal cell of the last `prepare`; empty before it.
    mutable bool                         active_ = false;  ///< False when the goal is off-grid/occupied
                                                           ///< (field unrooted); then operator() returns 0.

    /// Sentinel in `edge_mul_` for an impassable (OCCUPIED) cell; any negative value is impossible for a
    /// real multiplier (`1 + nonneg`), so a single `< 0` test rejects obstacles in the hot loop.
    static constexpr double BLOCKED = -1.0;
};
