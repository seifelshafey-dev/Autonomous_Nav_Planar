/**
 * @file planner_interface.hpp
 * @author Seif ElShafey
 * @brief Abstract interface for any motion planner producing an SE(2) path between two poses.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <memory>
#include <optional>
#include <stop_token>
#include "core/planner/path.hpp"
#include "core/planner/search_observer.hpp"

/// Why a call to `Planner::plan` returned.
enum class PlanOutcome {
    Found,            ///< A path was found.
    StartBlocked,     ///< The start pose fails the planner's start clearance check.
    GoalBlocked,      ///< The goal pose is in collision.
    NoPath,           ///< The search ran out of states without reaching the goal.
    BudgetExhausted,  ///< The expansion cap or the time limit was reached.
    Stopped           ///< The stop token was set. A planner that ignores the token never returns this.
};

/**
 * @brief Result of `Planner::plan`: the outcome and, on success, the path.
 *
 * `path` is set if and only if `outcome == PlanOutcome::Found`. A zero-length plan (the start
 * already satisfies the goal) is `Found`.
 */
struct PlanResult {
    PlanOutcome outcome;         ///< Why the call returned.
    std::optional<Path> path;    ///< The path; set iff `outcome == PlanOutcome::Found`.
};

/**
 * @brief Polymorphic motion-planner interface. Concrete planners (hybrid A*, RRT*, etc.) plug into
 *        the same calling contract so callers can swap implementations without changing wiring.
 */
class Planner{
    public:
        virtual ~Planner() = default;

        /**
         * @brief Plan a kinematically feasible, collision-free path from `start` to `goal`.
         *
         * @param start Start pose in world coordinates.
         * @param goal Goal pose in world coordinates.
         * @return PlanResult With `outcome == Found` and the path on success; otherwise the
         *         reason no path was returned and an empty `path`.
         */
        virtual PlanResult plan(const SE2State& start, const SE2State& goal) = 0;

        /**
         * @brief Plan as above, additionally reporting each node expansion to `observer`.
         *
         * Identical contract to the two-argument `plan`: same inputs, same return semantics. The only
         * difference is that, while searching, the planner calls `observer->onExpand` once per expanded
         * node in expansion order — letting a caller (e.g. a visualizer) capture the true search tree.
         *
         * @param start Start pose in world coordinates.
         * @param goal Goal pose in world coordinates.
         * @param observer Sink for expansion events; if null, no events are reported. Held only for
         *        the duration of the call.
         * @param stop When stop-requested, the planner aborts the search and returns `Stopped`. A default-constructed token never stops.
         * @return PlanResult As the two-argument `plan`.
         *
         * @note The default implementation ignores the observer and delegates to the two-argument
         *       `plan`. Planners that can report search progress override this to emit events.
         */
        virtual PlanResult plan(const SE2State& start, const SE2State& goal,
                                         std::shared_ptr<SearchObserver> observer,
                                         std::stop_token stop = {}) {
            (void)observer;
            (void)stop;
            return plan(start, goal);
        }
};
