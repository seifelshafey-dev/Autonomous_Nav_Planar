/**
 * @file navigation_mode.hpp
 * @brief ROS-free navigation mode logic: decides when to plan, follow, escape, stop or give up.
 */

#pragma once

#include <cstdint>
#include <variant>
#include <vector>

#include "core/planner/planner_interface.hpp"

/** @brief Navigation state. */
enum class NavState {
    Idle,       ///< No goal.
    Planning,   ///< A plan request to the current goal is outstanding. A request with no reply within
                ///< `NavigationConfig::plan_reply_timeout` is abandoned: the robot is stopped and
                ///< navigation is `Blocked` with `BlockedReason::NoPathToGoal`.
    Following,  ///< A path to the goal is being driven.
    Clearing,   ///< An escape path out of a blocked start is being driven.
    Blocked     ///< Navigation gave up for now; see `BlockedReason`.
};

/** @brief Why navigation is in `NavState::Blocked`; `None` in every other state. */
enum class BlockedReason {
    None,
    NoPathToGoal,  ///< The planner found no path to the goal.
    NoEscape,      ///< The robot could not be moved out of its blocked start.
    Stalled        ///< A path exists but the robot made no progress along it (held by the safety guard, or stuck).
};

/** @brief Kind of path being published. */
enum class PlanKind {
    Goal,   ///< Path to the goal.
    Escape  ///< Short path out of a blocked start.
};

/**
 * @brief Tuning for `NavigationMode`. The constructor of `NavigationMode` rejects out-of-range values.
 */
struct NavigationConfig {
    double stop_distance;  ///< Distance along the driven path (m, > 0). A path found invalid is stopped when the blockage is nearer than this.
    double stall_timeout;  ///< Time without cursor progress (s, > 0) after which a driven path is abandoned.
    double blocked_retry_period;  ///< Minimum time between plan requests while blocked (s, > 0): a blocked goal is retried every this period, and at once on a map change after it. `Blocked(Stalled)` is not retried.
    int    max_clearing_attempts;  ///< Escape moves (>= 1) the navigator may drive per goal between two found plans; a search that finds no escape does not count. Once used up the navigator stays `Blocked(NoEscape)`, keeps re-planning on `blocked_retry_period`, and moves again only if a plan is found or a new goal arrives.
    int    max_stall_replans;  ///< Replans (count, >= 0) allowed per goal after a stall; the next stall then ends in `Blocked` with `BlockedReason::Stalled`. 0 makes the first stall final. Only a new goal or arrival resets the count; movement along the path restarts the stall timer but does not reset the count.
    double plan_reply_timeout;  ///< Time (s, > 0) after a plan request with no reply after which the request is abandoned: the robot is stopped and navigation is `Blocked` with `BlockedReason::NoPathToGoal`.
};

/** @brief A new goal was set. `stamp` is monotonic seconds, as for every event. */
struct GoalReceived {
    double stamp;  ///< Event time (s, monotonic).
};

/** @brief The planner answered a plan request. */
struct PlanReply {
    double stamp;  ///< Event time (s, monotonic).
    uint32_t request_id;  ///< Id of the `RequestPlan` being answered.
    PlanOutcome outcome;  ///< Result of the plan.
};

/** @brief The escape search finished. */
struct EscapeResult {
    double stamp;  ///< Event time (s, monotonic).
    bool found;  ///< True when an escape path exists.
};

/** @brief The driven path was re-checked, after a map change or periodically while a replan is in flight. */
struct MapChanged {
    double stamp;  ///< Event time (s, monotonic).
    bool path_valid;  ///< Whether the remaining part of the driven path is clear; true when nothing is driven.
    double blocked_distance;  ///< Distance along the path to the blockage (m, >= 0); read only when `path_valid` is false.
};

/** @brief Progress reported by the path follower. */
struct Progress {
    double stamp;  ///< Event time (s, monotonic).
    uint32_t plan_id;  ///< Id of the published plan being followed.
    uint32_t cursor;  ///< Index of the follower's current path point.
    bool arrived;  ///< True when the end of that plan was reached.
};

/** @brief Periodic time signal. */
struct Tick {
    double stamp;  ///< Event time (s, monotonic).
};

/** @brief Any input to `NavigationMode::step`. */
using NavEvent = std::variant<GoalReceived, PlanReply, EscapeResult, MapChanged, Progress, Tick>;

/** @brief Ask the planner for a path to the current goal; the reply carries `request_id`. */
struct RequestPlan {
    uint32_t request_id;  ///< Id the reply must carry.
};
/** @brief Cancel the running plan request. */
struct CancelPlan {};
/** @brief Run the escape search from the current pose and answer with `EscapeResult`. */
struct RunEscape {};
/** @brief Publish the path that came with the event just handled, under `plan_id`. */
struct PublishPlan {
    uint32_t plan_id;  ///< Id the follower reports progress under.
    PlanKind kind;  ///< Whether the path leads to the goal or out of a blocked start.
};
/** @brief Publish the empty plan so the robot stops. */
struct PublishStop {};
/** @brief The state or reason changed. */
struct Report {
    NavState state;  ///< New navigation state.
    BlockedReason reason;  ///< New blocked reason; `None` unless `state` is `Blocked`.
};

/** @brief Any output of `NavigationMode::step`. */
using NavAction = std::variant<RequestPlan, CancelPlan, RunEscape, PublishPlan, PublishStop, Report>;

/**
 * @brief Event-driven navigation state machine. Holds no clock, threads or I/O: the caller feeds
 *        events with monotonic timestamps and carries out the returned actions in order.
 *
 *        A driven path with no cursor progress for `stall_timeout` is replanned up to `max_stall_replans`
 *        times per goal; after that navigation is `Blocked(Stalled)` and only a new goal leaves it.
 *
 *        A start-blocked reply triggers an escape search; at most `max_clearing_attempts` escape moves are driven
 *        per goal between found plans (a search that finds none does not count). After that navigation is
 *        `Blocked(NoEscape)` until a plan is found or a new goal arrives.
 *
 *        A plan request with no reply within `plan_reply_timeout` is abandoned: the robot is stopped and
 *        navigation is `Blocked(NoPathToGoal)`, retried every `blocked_retry_period` like any other blocked goal.
 */
class NavigationMode {
public:
    /**
     * @brief Construct in `Idle`. Aborts via CHECK (throws std::invalid_argument) if any config field is out of range.
     * @param cfg Tuning; see `NavigationConfig` for the valid range of each field.
     */
    explicit NavigationMode(const NavigationConfig& cfg);

    /**
     * @brief Handle one event.
     * @param ev Event; its `stamp` (s) must not decrease between calls.
     * @return Actions to carry out, in order; empty when the event changes nothing.
     */
    std::vector<NavAction> step(const NavEvent& ev);

    /** @brief Current state.
     *  @return The navigation state. */
    NavState state() const { return state_; }
    /** @brief Current blocked reason; `None` unless the state is `Blocked`.
     *  @return The reason. */
    BlockedReason reason() const { return reason_; }
    /** @brief Id of the published plan being driven.
     *  @return The plan id; 0 when nothing is driven. */
    uint32_t drivenPlanId() const { return driven_plan_id_; }
    /** @brief Id of the outstanding plan request.
     *  @return The request id; 0 when none. */
    uint32_t pendingRequestId() const { return pending_request_id_; }

private:
    void request(double stamp, std::vector<NavAction>& out);
    void stop(std::vector<NavAction>& out);
    void publish(PlanKind kind, double stamp, std::vector<NavAction>& out);
    void go(NavState s, BlockedReason r, std::vector<NavAction>& out);
    bool stalled(double stamp) const { return stamp - last_progress_stamp_ >= cfg_.stall_timeout; }
    bool cursor(const Progress& p);

    NavigationConfig cfg_;
    NavState state_ = NavState::Idle;
    BlockedReason reason_ = BlockedReason::None;
    bool has_goal_ = false;
    uint32_t goal_seq_ = 0;
    uint32_t driven_goal_seq_ = 0;
    uint32_t next_request_id_ = 1;
    uint32_t pending_request_id_ = 0;
    uint32_t next_plan_id_ = 1;
    uint32_t driven_plan_id_ = 0;
    uint32_t last_cursor_ = 0;
    double last_progress_stamp_ = 0.0;
    double last_request_stamp_ = 0.0;
    int clearing_attempts_ = 0;
    int stall_replans_ = 0;
};
