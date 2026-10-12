/**
 * @file obstacle_heuristic.cpp
 * @author Seif ElShafey
 * @brief Implementation of the obstacle-aware cost-to-goal heuristic declared in obstacle_heuristic.hpp.
 * @version 0.1
 * @date 2026-06-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

ObstacleHeuristic::ObstacleHeuristic(std::shared_ptr<const OccupancyGrid> grid,
                                     std::shared_ptr<const CostMap>       costmap,
                                     double                               costmap_weight,
                                     double                               weight)
    : Heuristic(weight),
      grid_(std::move(grid)),
      costmap_(std::move(costmap)),
      meta_(grid_->getMetadata()),
      costmap_weight_(costmap_weight) {
    
    // Bake the per-cell edge multiplier once: occupancy + costmap collapse into a single contiguous
    // array the Dijkstra reads per neighbour, replacing the per-relaxation findCellStatus()/costAt()
    // lookups (and their bounds re-checks) with one cache-friendly fetch. Grid, costmap, and
    // costmap_weight are all fixed for this heuristic's lifetime, so the bake is amortized across every
    // prepare() rather than recomputed per query.
    
    const std::size_t n = static_cast<std::size_t>(meta_.rows) * meta_.cols;
    edge_mul_.resize(n);
    for (int j = 0; j < meta_.rows; ++j) {
        for (int i = 0; i < meta_.cols; ++i) {
            const GridCoords gc{i, j};
            const std::size_t idx = meta_.index(gc);
            edge_mul_[idx] = (grid_->findCellStatus(gc) == CellState::OCCUPIED)
                ? BLOCKED
                : 1.0 + costmap_weight_ * static_cast<double>(costmap_->costAt(i, j));
        }
    }

    // Largest live distance gap = diag * max traversable edge multiplier, derived from the baked
    // array. +2 of slack: one slot for the current bucket plus rounding headroom, guaranteeing the
    // cyclic index of any live distance is distinct from the bucket being drained.
    double max_edge_mul = 1.0;
    for (const double m : edge_mul_) {
        if (m > max_edge_mul) { max_edge_mul = m; }
    }
    const double res = meta_.resolution;
    n_buckets_ = static_cast<std::size_t>(std::ceil((std::sqrt(2.0) * res * max_edge_mul) / res)) + 2;
    buckets_.resize(n_buckets_);
    settled_.assign(n, 0);
}

void ObstacleHeuristic::prepare(const SE2State & goal) const {
    const GridCoords g = meta_.worldToGrid(goal.x, goal.y);
    if (prepared_cell_ && *prepared_cell_ == g) { return; }
    prepared_cell_ = g;

    cost_to_goal_.assign(static_cast<std::size_t>(meta_.rows) * meta_.cols, UNREACHABLE);
    for (auto & bucket : buckets_) { bucket.clear(); }
    settled_.assign(cost_to_goal_.size(), 0);
    b_             = 0;
    scanned_empty_ = 0;
    remaining_     = cost_to_goal_.size();

    // The field can only be rooted at an in-grid, free goal cell. Otherwise it stays inactive: every
    // cell keeps the UNREACHABLE sentinel and operator() returns 0 so the search falls back to the
    // weighted-Euclidean heuristic (a goal outside the grid, or temporarily inside an obstacle, must
    // not prune all expansion). Occupancy is read from the baked `edge_mul_` (BLOCKED == OCCUPIED).
    active_ = meta_.inBounds(g) && edge_mul_[meta_.index(g)] != BLOCKED;
    if (!active_) { return; }

    const std::int32_t g_idx = static_cast<std::int32_t>(meta_.index(g));
    cost_to_goal_[static_cast<std::size_t>(g_idx)] = 0.0;
    buckets_[0].push_back(g_idx);
}

void ObstacleHeuristic::settleUntil(std::size_t target) const {
    if (!active_) { return; }

    const double res  = meta_.resolution;
    const double diag = std::sqrt(2.0) * res;
    const double bucket_w = res;                                   // == min edge cost
    const std::size_t n_buckets = n_buckets_;

    // 8-connected Dijkstra from the goal, run with Dial's algorithm (monotone bucket queue) instead
    // of a binary heap. Per-edge cost into the entered cell mirrors the search's g-model: geometric
    // step length * the entered cell's baked edge multiplier `1 + costmap_weight*costAt` (the costmap
    // line integral). Occupied cells are impassable (the costmap assigns them a finite cost, which
    // alone would let the field tunnel through walls); unknown cells are traversable and carry the
    // costmap's unknown penalty. All occupancy + cost lookups are pre-folded into `edge_mul_`.
    auto bucket_of = [&](double d) -> std::size_t {
        return static_cast<std::size_t>(d / bucket_w);
    };

    const int cols = meta_.cols;
    const int rows = meta_.rows;

    // Neighbour offsets as (di, dj, linear-index delta, is-diagonal). The linear delta lets the hot
    // loop advance the flat index without recomputing j*cols + i; di/dj are kept only for the cheap
    // column-edge wrap guard (reject a neighbour that wrapped across a row boundary).
    struct Nb { int di; int dj; int didx; bool diag; };
    const std::array<Nb, 8> NB = {{
        {  1,  0,        1, false}, { -1,  0,       -1, false},
        {  0,  1,     cols, false}, {  0, -1,    -cols, false},
        {  1,  1,  cols + 1,  true}, {  1, -1, -cols + 1,  true},
        { -1,  1,  cols - 1,  true}, { -1, -1, -cols - 1,  true}}};

    // `remaining_` counts not-yet-settled cells so the scan can stop the instant every reachable cell
    // is fixed. When unreachable cells remain (walled-off pockets, occupied cells), termination falls
    // to the empty-bucket guard: once `n_buckets` consecutive buckets are empty, no live bucket can
    // exist anywhere (a live one would be within `max_edge` of the last drained bucket, i.e. fewer than
    // `n_buckets` slots away). The wave pauses right after `target` is settled; all loop state lives in
    // members, so a later call resumes the identical pop sequence.
    while (!settled_[target] && remaining_ > 0 && scanned_empty_ < n_buckets) {
        std::vector<int32_t> & cur_bucket = buckets_[b_ % n_buckets];
        if (cur_bucket.empty()) { ++b_; ++scanned_empty_; continue; }
        scanned_empty_ = 0;

        // Drain this bucket. Relaxations may append to it (a same-bucket distance equal to d is
        // impossible since every edge >= one bucket, so the index only grows past b), so we pop by
        // index until empty rather than range-iterate.
        while (!cur_bucket.empty()) {
            const std::int32_t cidx = cur_bucket.back();
            cur_bucket.pop_back();
            const std::size_t cz = static_cast<std::size_t>(cidx);
            if (settled_[cz]) { continue; }       // stale: already fixed at a smaller distance.
            settled_[cz] = 1;
            --remaining_;
            const double d  = cost_to_goal_[cz];
            const int    ci = cidx % cols;
            const int    cj = cidx / cols;

            for (const Nb & nb : NB) {
                const int ni = ci + nb.di;
                const int nj = cj + nb.dj;
                if (ni < 0 || ni >= cols || nj < 0 || nj >= rows) { continue; }  // off-grid / row wrap
                const std::size_t nidx = static_cast<std::size_t>(cidx + nb.didx);
                if (settled_[nidx]) { continue; }
                const double mul = edge_mul_[nidx];
                if (mul < 0.0) { continue; }  // BLOCKED (OCCUPIED): impassable.
                const double step = nb.diag ? diag : res;
                const double nd   = d + step * mul;
                if (nd < cost_to_goal_[nidx]) {
                    cost_to_goal_[nidx] = nd;
                    buckets_[bucket_of(nd) % n_buckets].push_back(static_cast<std::int32_t>(nidx));
                }
            }
            if (cz == target) { return; }
        }
        ++b_;
    }
}

double ObstacleHeuristic::operator()(const SE2State & current) const {
    if (!active_) { return 0.0; }  // field unrooted (off-grid/occupied goal): defer to other heuristics.

    const GridCoords c = meta_.worldToGrid(current.x, current.y);
    if (!meta_.inBounds(c)) { return 0.0; }  // off-grid frontier: traversable-unknown, defer.

    const std::size_t idx = meta_.index(c);
    if (edge_mul_[idx] == BLOCKED) { return weight_ * UNREACHABLE; }  // obstacle: never relaxed.
    settleUntil(idx);
    // Not settled once the wave ends means no route; never expose a tentative value.
    const double v = settled_[idx] ? cost_to_goal_[idx] : UNREACHABLE;
    // In-grid but no route to goal (obstacle or walled-off pocket): steer the search away.
    return (v >= UNREACHABLE) ? weight_ * UNREACHABLE : weight_ * v;
}

double ObstacleHeuristic::fieldAt(const GridCoords & gc) const {
    if (!meta_.inBounds(gc)) { return UNREACHABLE; }
    if (!active_) { return UNREACHABLE; }
    const std::size_t idx = meta_.index(gc);
    if (edge_mul_[idx] == BLOCKED) { return UNREACHABLE; }
    settleUntil(idx);
    return settled_[idx] ? cost_to_goal_[idx] : UNREACHABLE;
}
