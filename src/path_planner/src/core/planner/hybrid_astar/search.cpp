/**
 * @file search.cpp
 * @author Seif ElShafey
 * @brief Implementation of the hybrid A* planner declared in search.hpp.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/planner/hybrid_astar/search.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/planner/hybrid_astar/closed_set.hpp"
#include "core/planner/hybrid_astar/open_set.hpp"
#include "core/planner/primitives/control_primitives.hpp"
#include "core/planner/search_observer.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/types/grid_coords.hpp"
#include "core/types/state.hpp"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <stop_token>
#include <cstdio>
#include <optional>
#include <cmath>
#include <utility>
#include <vector>

bool HybridAstar::exactMotionClear(const SE2State & start, const Control & u, bool reject_unknown) const {
    return this->kin_model_.motion->visitExactCover(start, u, [&](const GridCoords & c){
        return this->checker_->checkCell(c, reject_unknown);
    });
}

 bool HybridAstar::atGoal(const SE2State & a) const noexcept{
    const double dx = goal_.x - a.x;
    const double dy = goal_.y - a.y;
    const double dth = wrapToPi(goal_.theta - a.theta);

    const double position_tol = cfg_.goal_pos_tol * cfg_.goal_pos_tol;
    return  (dx*dx+ dy*dy) <= position_tol 
            && std::abs(dth) <=cfg_.goal_heading_tol;
 }

std::optional<Path> HybridAstar::shotAtGoal(const SE2State & current) const {
    const double dx = goal_.x - current.x;
    const double dy = goal_.y - current.y;
    const double d  = std::sqrt(dx * dx + dy * dy);

    if (d > cfg_.shot_max_distance) {return std::nullopt;}

    const double alpha    = std::atan2(dy, dx);
    const double dtheta_1 = wrapToPi(alpha - current.theta);
    const double dtheta_2 = wrapToPi(goal_.theta - alpha);

    constexpr double DIST_EPS  = 1e-9;    // [m]   — both below the 1e-6 reconnect contract
    constexpr double ANGLE_EPS = 1e-9;    // [rad]

    const double rmin       = this->kin_model_.kin.rmin;
    const double vmax       = this->kin_model_.kin.vmax;
    const double pivot_rate = vmax / rmin;   // max |omega| during a pivot [rad/s]
    const auto control_cost = makeControlCost(this->kin_model_.kin);

    struct Seg { SE2State start; Control u; SE2State end; };
    Seg segs[3];
    int n_segs = 0;

    if (std::abs(dtheta_1) > ANGLE_EPS) {
        const double  omega = (dtheta_1 > 0.0) ? +pivot_rate : -pivot_rate;
        segs[n_segs++] = Seg{current, Control{ .v = 0.0, .omega = omega, .dt = std::abs(dtheta_1) / pivot_rate },
                             SE2State{ current.x, current.y, alpha }};
    }
    if (d > DIST_EPS) {
        segs[n_segs++] = Seg{SE2State{ current.x, current.y, alpha },
                             Control{ .v = vmax, .omega = 0.0, .dt = d / vmax },
                             SE2State{ goal_.x, goal_.y, alpha }};
    }
    if (std::abs(dtheta_2) > ANGLE_EPS) {
        const double  omega = (dtheta_2 > 0.0) ? +pivot_rate : -pivot_rate;
        segs[n_segs++] = Seg{SE2State{ goal_.x, goal_.y, alpha },
                             Control{ .v = 0.0, .omega = omega, .dt = std::abs(dtheta_2) / pivot_rate },
                             SE2State{ goal_.x, goal_.y, goal_.theta }};
    }

    // UNKNOWN blocks unless shot_allow_unknown. Check the cheapest segments first; the result is the conjunction.
    const bool reject_unknown = !this->cfg_.shot_allow_unknown;
    const double r_out = this->kin_model_.motion->body().r_out;
    // Cheap pre-test on the translate centre cells, before any exact scan. A centre cell that is
    // COLLISION by fastAcceptReject lies inside the inscribed disc, hence in that pose's footprint
    // raster; a centre cell failing checkCell is in the raster too. Both would fail the exact check.
    std::vector<SE2State> translate_samples;
    if (d > DIST_EPS) {
        for (int k = 0; k < n_segs; ++k) {
            if (segs[k].u.v == 0.0) {continue;}
            translate_samples = this->kin_model_.motion->sampleTrajectory(segs[k].start, segs[k].u);
            for (const auto & p : translate_samples) {
                const GridCoords c = this->grid_meta_.worldToGrid(p.x, p.y);
                if (this->checker_->fastAcceptReject(c) == Validity::COLLISION) {return std::nullopt;}
                if (!this->checker_->checkCell(c, reject_unknown))     {return std::nullopt;}
            }
        }
    }
    int order[3] = {0, 1, 2};
    auto reach = [&](int k){
        const Control & u = segs[k].u;
        return (std::abs(u.v) + r_out * std::abs(u.omega)) * u.dt;
    };
    std::stable_sort(order, order + n_segs, [&](int a, int b){ return reach(a) < reach(b); });
    for (int n = 0; n < n_segs; ++n) {
        const Seg & s = segs[order[n]];
        if (!this->exactMotionClear(s.start, s.u, reject_unknown)) {return std::nullopt;}
    }

    Path path;
    path.total_cost = 0.0;
    path.node_points.push_back(current);
    for (int n = 0; n < n_segs; ++n) {
        const Seg & s = segs[n];
        PathSegment seg{};
        seg.u = s.u;
        seg.samples_poses = (s.u.v != 0.0) ? std::move(translate_samples)
                                           : this->kin_model_.motion->sampleTrajectory(s.start, s.u);

        double cm_penalty = 0.0;
        for (const auto & sample : seg.samples_poses) {
            const GridCoords c = this->grid_meta_.worldToGrid(sample.x, sample.y);
            cm_penalty += this->costmap_->costAt(c.i, c.j);
        }
        // Same line-integral measure as the expansion loop: mean per-cell cost x travelled distance.
        const std::size_t n_samples = seg.samples_poses.size();
        const double cm_mean = (n_samples > 0) ? cm_penalty / static_cast<double>(n_samples) : 0.0;
        const double arc_len = std::abs(s.u.v) * s.u.dt;
        seg.cost = control_cost(s.u) + cfg_.costmap_weight * cm_mean * arc_len;

        path.total_cost += seg.cost;
        path.segments.push_back(std::move(seg));
        path.node_points.push_back(s.end);
    }
    return path;
}

Path HybridAstar::reconstructPath(int32_t goal_idx) const {
    // Walk parent pointers backward from goal; reversing gives start->goal in execution order.
    std::vector<int32_t> chain;
    chain.reserve(500);

    for (int32_t idx = goal_idx; idx != -1; idx = node_pool_[idx].parent_idx) {
        chain.push_back(idx);
    }
    std::reverse(chain.begin(), chain.end());

    Path path;
    path.total_cost = 0.0;

    if (chain.empty()) {return path;}

    path.node_points.reserve(chain.size());
    path.segments.reserve(chain.size() - 1);
    path.node_points.push_back(node_pool_[chain.front()].state);

    const auto & prims = this->kin_model_.prim_lib->primitives();
    for (std::size_t i = 1; i < chain.size(); ++i) {
        const Node & parent = node_pool_[chain[i - 1]];
        const Node & child  = node_pool_[chain[i]];
        const Primitive & prim = prims[child.prim_idx];

        PathSegment seg{};
        seg.u = prim.control;
        // Samples regenerated from the continuous parent state.
        seg.samples_poses = this->kin_model_.motion->sampleTrajectory(parent.state, prim.control);
        seg.cost = child.g_cost - parent.g_cost;

        path.total_cost += seg.cost;
        path.segments.push_back(std::move(seg));
        path.node_points.push_back(child.state);
    }

    return path;
}

PlanResult HybridAstar::plan(const SE2State & start, const SE2State & goal){
    return this->plan(start, goal, {});
}

PlanResult HybridAstar::plan(const SE2State & start, const SE2State & goal,
                                      std::shared_ptr<SearchObserver> observer,
                                      std::stop_token stop){
    this->node_pool_.clear();
    this->closed_.clear();
    this->open_ = OpenSet();
    this->stats_ = PlanStats{};

    // Wall-clock budget. ROS lives one layer up: the caller converts whatever clock/budget
    // semantics it uses (action timeout, parameter, etc.) into seconds and sets
    // AStarConfig::max_plan_time_sec. Inside the planner we use std::chrono::steady_clock so the
    // library has no ROS dependency and is unit-testable in isolation.
    //
    // The deadline is anchored at plan() entry, BEFORE heuristic prepare(), so max_plan_time_sec
    // bounds the whole call. Heuristic prep can itself be heavy (e.g. ObstacleHeuristic runs a
    // full-grid Dijkstra over the whole map) — anchoring after prep would leave that cost
    // unbounded and let plan() overshoot the budget by the entire prep time. We cannot interrupt
    // prepare() (it is the heuristic owner's contract), but we can refuse to start the search once
    // it has already consumed the budget.
    using clock_t = std::chrono::steady_clock;
    const auto deadline = clock_t::now()
        + std::chrono::duration_cast<clock_t::duration>(
            std::chrono::duration<double>(cfg_.max_plan_time_sec));

    if (stop.stop_requested())                {return {PlanOutcome::Stopped, std::nullopt};}
    if (!this->checker_->startClear(start))   {return {PlanOutcome::StartBlocked, std::nullopt};}
    if (!this->checker_->checkState(goal))    {return {PlanOutcome::GoalBlocked, std::nullopt};}

    this->goal_ = goal;
    this->heuristic_->prepare(goal);

    // If heuristic prep alone blew the budget, do not enter the search loop.
    if (clock_t::now() > deadline) {return {PlanOutcome::BudgetExhausted, std::nullopt};}
    if (stop.stop_requested())     {return {PlanOutcome::Stopped, std::nullopt};}

    Node start_node{.state = start,
                    .g_cost = 0,
                    .h_cost = (*heuristic_)(start),
                    .parent_idx = -1, .prim_idx = -1,
                    .heading_bin = this->kin_model_.prim_lib->headingBin(start.theta),
                    .grid_coords = this->grid_meta_.worldToGrid(start.x, start.y)
                    };

    node_pool_.push_back(start_node);
    open_.push(OpenEntry{.idx = 0, .f = start_node.h_cost, .h = start_node.h_cost});

    // Check the clock every TIME_CHECK_PERIOD loop iterations instead of every iteration; now() is
    // ~20ns but adds up. With power-of-two period we use a cheap bitmask test.
    constexpr int32_t TIME_CHECK_PERIOD = 1024;
    static_assert((TIME_CHECK_PERIOD & (TIME_CHECK_PERIOD - 1)) == 0,
                  "TIME_CHECK_PERIOD must be a power of two for bitmask test");

    int32_t expansions = 0;
    uint32_t iterations = 0;

    // Closed-set dedup cell, in whole map cells. The closed key floors grid coords to this stride
    // (key only) so the pruning granularity is decoupled from the map resolution: at 0.03 m cells,
    // states a few cm apart would otherwise hash to distinct keys and barely prune.
    // grid_coords themselves stay at map resolution for collision/costmap/heuristic.
    const int32_t closed_stride = closedStride(cfg_.closed_cell_size, grid_meta_.resolution);

    while(!open_.empty() && expansions < this->cfg_.max_expansions){
        if ((iterations++ & (TIME_CHECK_PERIOD - 1)) == 0) {
            if (stop.stop_requested())   {return {PlanOutcome::Stopped, std::nullopt};}
            if (clock_t::now() > deadline) {return {PlanOutcome::BudgetExhausted, std::nullopt};}
        }
        const OpenEntry top = open_.top(); open_.pop();
        const Node curr = node_pool_[top.idx]; // Copied by value since node_pool might mutate.
        const ClosedKey curr_key = keyOf(curr.grid_coords, curr.heading_bin, closed_stride);

        // Stale skip -> key exists in closed set AND cost is higher than previously found.
        // A goal-satisfying node is never stale: it is always expanded, and the search returns on it.
        const bool curr_at_goal = atGoal(curr.state);
        auto it = closed_.find(curr_key);
        if(!curr_at_goal && it != closed_.end() && it->second.g_cost <= curr.g_cost){continue;}

        // A node whose swept-cell scan was blocked is confirmed over its exact motion only now, when
        // taken for expansion; a rejected node leaves no trace in the closed set or the counters.
        if(top.pending_exact){
            const auto & prim = this->kin_model_.prim_lib->primitives()[curr.prim_idx];
            if(!exactMotionClear(node_pool_[curr.parent_idx].state, prim.control, false)){
                ++stats_.exact_reject;
                continue;
            }
            ++stats_.exact_accept;
        }
        if(!curr_at_goal || it == closed_.end() || it->second.g_cost > curr.g_cost){
            closed_[curr_key] = ClosedEntry{curr.g_cost, top.idx};
        }

        ++stats_.expansions;

        // Report this node now that it is accepted for expansion (not stale). One event per pop;
        // id/parent_id are node-pool indices, so the stream reconstructs the expanded search tree.
        // Guarded so the null-observer path carries zero overhead and stays byte-identical.
        if(observer){
            observer->onExpand(SearchExpansion{
                .id        = top.idx,
                .parent_id = curr.parent_idx,
                .state     = curr.state,
                .g         = curr.g_cost,
                .f         = curr.g_cost + curr.h_cost});
        }

        if(curr_at_goal) {return {PlanOutcome::Found, reconstructPath(top.idx)};}

        if(cfg_.enable_shot){
            if(auto shot = shotAtGoal(curr.state)){
                Path full = reconstructPath(top.idx);
                
                // Non-const ref so std::move actually moves; const-ref would silently fall back to copy.
                for(auto & seg : shot->segments){full.segments.push_back(std::move(seg));}

                for(std::size_t i = 1; i < shot->node_points.size(); ++i){
                    full.node_points.push_back(shot->node_points[i]);
                }

                full.total_cost += shot->total_cost;
                return {PlanOutcome::Found, std::move(full)};
            }
        }

        int16_t k = curr.heading_bin;
        const std::vector<Primitive> & prims = this->kin_model_.prim_lib->primitives();
        for(std::size_t i = 0; i < prims.size(); ++i){
            SE2State new_state = this->space_->propagate(curr.state, prims[i].control);
            GridCoords new_grid_coords = this->grid_meta_.worldToGrid(new_state.x, new_state.y);
            int16_t new_heading_bin = this->kin_model_.prim_lib->headingBin(new_state.theta);

            double cm_cost = 0.0;
            for(const auto & sc:prims[i].sample_cells[k]){
                const GridCoords world_c = sc + curr.grid_coords;
                cm_cost += this->costmap_->costAt(world_c.i, world_c.j);
            }
            // Costmap penalty as a line integral over the primitive: mean per-cell cost x travelled
            // distance (|v|*dt). This keeps the penalty invariant to trajectory-sampling density, so
            // per-cell costs are expressed per metre and read directly against the path-distance metric.
            // A pure pivot (v=0) travels no distance and accrues no clearance penalty by this measure.
            const std::size_t n_samples = prims[i].sample_cells[k].size();
            const double cm_mean = (n_samples > 0) ? cm_cost / static_cast<double>(n_samples) : 0.0;
            const double arc_len = std::abs(prims[i].control.v) * prims[i].control.dt;

            double new_g_cost = curr.g_cost + this->cfg_.costmap_weight * cm_mean * arc_len + prims[i].cost;
            ClosedKey new_key = keyOf(new_grid_coords, new_heading_bin, closed_stride);
            // Drop a candidate dominated by an already-expanded state in the same dedup cell and
            // heading bin before paying for the collision check. A goal-satisfying child is never dropped.
            if(!atGoal(new_state)){
                auto it = closed_.find(new_key);
                if(it != closed_.end() && it->second.g_cost <= new_g_cost){
                    ++stats_.duplicate_skipped;
                    continue;
                }
            }

            const TrajectoryVerdict verdict = this->checker_->checkTrajectory(
                prims[i].swept_cells[k],
                prims[i].sample_cells[k],
                prims[i].sample_clearance,
                curr.grid_coords);
            switch(verdict){
                case TrajectoryVerdict::FAST_ACCEPT:  ++stats_.fast_accept;  break;
                case TrajectoryVerdict::FAST_REJECT:  ++stats_.fast_reject;  break;
                case TrajectoryVerdict::SCAN_CLEAR:   ++stats_.scan_clear;   break;
                case TrajectoryVerdict::SCAN_BLOCKED: ++stats_.scan_blocked; break;
            }
            if(verdict == TrajectoryVerdict::FAST_REJECT) {continue;}
            // A blocked scan is not proof of collision (the cache is a cover): the node is pushed
            // unconfirmed and re-checked over its exact motion if it is ever taken for expansion.
            const bool pending = verdict == TrajectoryVerdict::SCAN_BLOCKED;

            double new_h_cost = (*heuristic_)(new_state);
            const int32_t new_idx = static_cast<int32_t>(node_pool_.size());

            Node new_node{new_state, new_g_cost, new_h_cost, top.idx,
                static_cast<int16_t>(i), new_heading_bin, new_grid_coords};

            node_pool_.push_back(new_node);
            open_.push(OpenEntry{
                .idx = new_idx,
                .f   = new_g_cost + new_h_cost,
                .h   = new_h_cost,
                .pending_exact = pending});
        }
        expansions++;
    }

    return {open_.empty() ? PlanOutcome::NoPath : PlanOutcome::BudgetExhausted, std::nullopt};
}

