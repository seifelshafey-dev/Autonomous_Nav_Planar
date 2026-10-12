/**
 * @file planner_node.hpp
 * @author Seif ElShafey
 * @brief ROS2 lifecycle node that owns hybrid A* configs and serves plan requests against the
 *        latest SLAM map snapshot.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "bondcpp/bond.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "path_planner_msgs/action/plan_path.hpp"
#include "path_planner_msgs/msg/plan_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ros/planner_server/search_marker_observer.hpp"

#include "core/map/map_provider.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/path.hpp"
#include "core/planner/planner_interface.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"

/**
 * @brief Hybrid A* lifecycle node: serves path requests against the newest map snapshot.
 *
 * Action server `plan_path` (`path_planner_msgs/action/PlanPath`, relative to the node namespace):
 * plans from the robot's current pose to the goal pose. The goal pose must be in `planning_frame`;
 * a goal in any other frame, including an empty frame id, is rejected with a warning. The result
 * outcome is one of `FOUND` (the result carries the trajectory), `START_BLOCKED`, `GOAL_BLOCKED`,
 * `NO_PATH`, `BUDGET_EXHAUSTED` or `STOPPED`. The goal is aborted with `STOPPED` when no search
 * could be attempted (no robot pose or no map yet) and when a newer request supersedes it; it
 * is canceled with `STOPPED` when the client cancels. While a search runs, feedback with
 * scalar progress is published. A request is planned on the newest map snapshot at the time it is
 * accepted; a map update during the search does not affect it. The goal succeeds only with
 * `FOUND`; every other outcome ends it aborted (canceled on a client cancel). Snapshots whose grid resolution differs from the planner's
 * kinematic model are rejected with a logged error and the previous map keeps being used.
 *
 * Publishers:
 * - `search_markers` (`visualization_msgs/MarkerArray`, depth 10): while a search runs, about
 *   every 0.066 s, the states expanded since the previous publish as one point marker in
 *   `planning_frame`. Nothing is published when no search is running.
 *
 * TF: looks up `planning_frame` <- `robot_frame` (latest available) as the start pose of each
 * request.
 *
 * Parameters (all strings; read at configure):
 * - `planning_frame`: frame of goals, paths and markers (default "map").
 * - `robot_frame`: frame of the robot base (default "base_link").
 * - `robot_params_path`, `diff_drive_params_path`, `astar_config_path`, `heuristic_config_path`,
 *   `cost_policy_path`: configuration files (default "", must be set to valid files).
 *
 * Lifecycle: configure loads the files; activate creates the interfaces; deactivate and cleanup
 * remove them.
 *
 * @note The map provider must be bound once with `bindMapProvider` before the first activation
 *       and stays bound across cleanup; `on_activate` throws `std::invalid_argument` if it is not, so the activate transition fails.
 */
class PlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;  ///< Result of a lifecycle transition callback.
    using State = rclcpp_lifecycle::State;  ///< Lifecycle state passed to a transition callback.

    using PlanPath   = path_planner_msgs::action::PlanPath;  ///< The `plan_path` action type.
    using GoalHandle = rclcpp_action::ServerGoalHandle<PlanPath>;  ///< Server-side handle of one `plan_path` goal.

    /**
     * @brief Create the node (named "planner_node") and declare its parameters.
     * @param opts Node options, including parameter overrides.
     */
    explicit PlannerNode(const rclcpp::NodeOptions & opts);

    /**
     * @brief Read the parameters and load the configuration files they name.
     * @return `SUCCESS`.
     * @throws YAML::Exception if a file is missing or malformed, `std::invalid_argument` if a value
     *         is out of range; a lifecycle transition reports either as a failed configuration.
     */
    CallbackReturn on_configure(const State &) override;

    /**
     * @brief Create the action server, the `search_markers` publisher, the TF listener and the
     *        heartbeat bond.
     * @return `SUCCESS`.
     * @throws std::invalid_argument if no map provider was bound with `bindMapProvider`; a lifecycle
     *         transition reports this as a failed activation.
     */
    CallbackReturn on_activate(const State &) override;

    /**
     * @brief Stop any running search, wait for it to return, and remove the interfaces created
     *        on activation.
     * @return `SUCCESS`.
     */
    CallbackReturn on_deactivate(const State &) override;

    /**
     * @brief Stop any running search, wait for it to return, and remove the interfaces created
     *        on activation. The motion model is also discarded, so a map of another resolution
     *        is accepted afterwards.
     * @return `SUCCESS`.
     */
    CallbackReturn on_cleanup(const State &) override;

    /**
     * @brief Bind the upstream map provider before activation.
     * @param provider Source of map snapshots; must be non-null. Bind before the node is activated.
     */
    void bindMapProvider(std::shared_ptr<IMapProvider> provider);

private:
    /// @brief Rebuilds the planner if the latest snapshot differs from the bound one.
    void refreshPlannerIfStale();

    /// @brief Timer callback (0.2 s): builds the motion model once the first map is available,
    ///        then cancels its timer. Runs on the executor thread.
    void warmUp();

    rclcpp_action::GoalResponse   handleGoal(const rclcpp_action::GoalUUID & uuid,
                                             std::shared_ptr<const PlanPath::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void                          handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

    /// @brief Worker body: resolves TF start, runs plan(), resolves the goal handle.
    void                          execute(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Timer callback (~15 Hz): drains new expansions from the active observer into an
     *        incremental MarkerArray and publishes throttled feedback to the active goal handle.
     *
     * Runs on the executor thread. Reads `observer_` and `active_goal_` via atomic load; safe to
     * call when no plan is in progress (both are null).
     */
    void publishProgress();

    // Loaded once in on_configure (kinematic model deferred until first snapshot):
    RobotParams      robot_params_;
    DiffDriveParams  kin_params_;
    AStarConfig      cfg_;
    HeuristicConfig  heur_cfg_;
    CostPolicy       cost_policy_;

    // Built lazily on first snapshot (resolution sourced from snapshot):
    KinematicModel kin_model_;
    bool           kin_model_built_ = false;

    // Upstream (bound once via bindMapProvider; kept across cleanup):
    std::shared_ptr<IMapProvider> map_provider_;

    // Per-snapshot (rebuilt lazily inside refreshPlannerIfStale):
    std::shared_ptr<const MapSnapshot> last_snap_;
    std::unique_ptr<Planner>           planner_;

    // Action server (created in on_activate, reset in on_deactivate/on_cleanup):
    rclcpp_action::Server<PlanPath>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::jthread    worker_;        ///< Runs one plan() at a time off the executor thread.
    std::stop_source stop_source_;  ///< Per-goal cancellation; request_stop() aborts the running plan.
                                    // single-writer: only handleAccepted, after joining the prior worker.
    /// Distinguishes a client-initiated cancel from a supersede/preempt as the reason the worker's
    /// stop token flipped. Set true ONLY by handleCancel (executor thread), alongside request_stop();
    /// cleared by handleAccepted before spawning each worker (a preempt leaves it false). The worker
    /// (execute) only reads it, so it resolves a canceled goal with canceled() and a preempted goal
    /// with abort(). Single-writer on the executor thread; the worker is read-only.
    std::atomic<bool> cancel_requested_{false};
    std::string planning_frame_ = "map";
    std::string robot_frame_    = "base_link";

    // Visualization / feedback (created in on_activate, reset in on_deactivate/on_cleanup):
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr warmup_timer_;  ///< Created in on_activate; cancels itself once the model is built.

    // Shared across executor thread (publishProgress) and worker jthread (execute).
    // Must be atomic<shared_ptr> to avoid a data race on the pointer slot + control block.
    std::atomic<std::shared_ptr<SearchMarkerObserver>> observer_;   ///< Active during plan; null otherwise.
    std::atomic<std::shared_ptr<GoalHandle>>           active_goal_; ///< Active goal handle for feedback; null otherwise.

    // Touched only by the timer (executor thread) — plain member, no atomics needed.
    std::size_t published_markers_ = 0;

    /// Heartbeat the lifecycle manager monitors after activation; the manager fails
    /// activation if its bond_timeout elapses before this bond opens. Created in
    /// on_activate, broken in on_deactivate/on_cleanup.
    std::unique_ptr<bond::Bond> bond_;
};
