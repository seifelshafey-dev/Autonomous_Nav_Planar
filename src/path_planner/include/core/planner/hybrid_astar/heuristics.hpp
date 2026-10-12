/**
 * @file heuristics.hpp
 * @author Seif ElShafey
 * @brief Polymorphic cost-to-goal estimator for the hybrid A* planner.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "core/motion/distance_functions.hpp"
#include "core/types/state.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

/**
 * @brief Configuration knob for the hybrid A* heuristic.
 *
 * @note `weight` multiplies the raw estimate: 1.0 leaves it unscaled, larger values make the search
 *       greedier. The planner guarantees no optimality or suboptimality bound for any weight.
 */
struct HeuristicConfig {
    double weight = 1.0;   ///< Multiplier applied to the raw heuristic distance.
    bool   use_obstacle_heuristic = true;  ///< true -> max(obstacle-aware Dijkstra cost-to-goal,
                                           ///< weighted-Euclidean); false -> weighted-Euclidean only.
};

/**
 * @brief Cost-to-goal estimator interface.
 *
 * The interface is split into two phases: a one-shot `prepare(goal)` that binds the heuristic to a
 * planning query's goal, followed by many cheap `operator()(current)` calls during the search loop.
 *
 * - **Closed-form** heuristics (e.g. weighted Euclidean over SE(2)) implement `prepare` as just
 *   storing the goal, and do the math in `operator()`.
 *
 * The heuristic owns its own weight: `operator()` returns the already-weighted estimate so the
 * planner can plug `f = g + h` directly with no per-call multiplication. weight = 1 (default) leaves
 * the estimate unscaled; a larger weight makes the search greedier. No optimality bound is guaranteed.
 */
class Heuristic {
public:
    /**
     * @param weight Multiplier applied to the raw estimate (dimensionless; 1.0 = unscaled).
     */
    explicit Heuristic(double weight = 1.0) : weight_(weight) {}
    virtual ~Heuristic() = default;

    /**
     * @brief Bind this heuristic to a planning query's goal. Called once by `plan()` before the
     *        search loop and never again during expansion.
     *
     * @param goal World-frame target pose for the upcoming search.
     */
    virtual void prepare(const SE2State & goal) const = 0;

    /**
     * @brief Estimate weighted cost-to-goal from `current` to the goal bound by the last `prepare`
     *        call. Called many times per plan; must be cheap.
     *
     * @param current Candidate state in world coordinates.
     * @return double `weight * h_raw(current)` where `h_raw` is the underlying raw estimate.
     */
    virtual double operator()(const SE2State & current) const = 0;

    double weight() const noexcept { return weight_; } ///< The heuristic weighting factor applied inside `operator()`. @return The weight (dimensionless).

protected:
    double weight_; ///< Multiplier baked into the returned estimate; subclasses apply on the raw distance.
};

/**
 * @brief Closed-form heuristic backed by an injected `SE2Distance` function (e.g. weighted
 *        Euclidean). `prepare` just stores the goal; `operator()` evaluates the distance
 *        and applies the heuristic weight.
 */
class DistanceHeuristic : public Heuristic {
public:
    /**
     * @param dist   Distance functor evaluated between the query state and the goal; must not be null.
     * @param weight Multiplier applied to the distance (dimensionless; 1.0 = unscaled).
     */
    explicit DistanceHeuristic(std::shared_ptr<const SE2Distance> dist, double weight = 1.0)
        : Heuristic(weight), dist_(std::move(dist)) {}

    void prepare(const SE2State & goal) const override { goal_ = goal; }

    double operator()(const SE2State & current) const override {
        return weight_ * (*dist_)(current, goal_);
    }

private:
    std::shared_ptr<const SE2Distance> dist_;
    mutable SE2State goal_{};  ///< Goal cached by the last `prepare` call.
};

/**
 * @brief Cost-to-goal estimator that returns the maximum estimate over a set of child heuristics.
 *        `prepare` binds the goal on every child; `operator()` returns the largest child estimate.
 */
class MaxHeuristic : public Heuristic {
public:
    /**
     * @param children Heuristics to combine. Each must already carry its own weight; the composite
     *                 returns their per-call maximum.
     */
    explicit MaxHeuristic(std::vector<std::shared_ptr<const Heuristic>> children)
        : Heuristic(1.0), children_(std::move(children)) {}

    void prepare(const SE2State & goal) const override {
        for (const auto & c : children_) { c->prepare(goal); }
    }

    double operator()(const SE2State & current) const override {
        double best = 0.0;
        for (const auto & c : children_) { best = std::max(best, (*c)(current)); }
        return best;
    }

private:
    std::vector<std::shared_ptr<const Heuristic>> children_;
};
