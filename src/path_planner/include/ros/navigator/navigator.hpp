/**
 * @file navigator.hpp
 * @brief ROS 2 node that turns goal poses into driven paths: it requests plans, publishes them,
 *        watches the map for blockages and clears a blocked start.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_planner_msgs/action/plan_path.hpp"
#include "path_planner_msgs/msg/plan_progress.hpp"
#include "path_planner_msgs/msg/plan_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "core/collision_checker/collision_checker.hpp"
#include "core/config/config_loader.hpp"
#include "core/map/map_provider.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/navigation/forward_clock.hpp"
#include "core/navigation/navigation_checks.hpp"
#include "core/navigation/navigation_mode.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/state.hpp"

/**
 * @brief Owns the navigation goal and sequences planning, driving and clearing a blocked start.
 *
 * A goal pose arrives, the navigator asks the planner for a path, publishes it for the follower,
 * and keeps checking the part of the path still ahead of the robot against every new map. When
 * that path becomes blocked it stops the robot (if the blockage is close) and plans again. When
 * the planner reports the robot's start pose blocked, it stops the robot, waits for it to come to
 * rest, and publishes a short straight move that takes it to a clear pose before planning again.
 * Progress of the published plan is read from the follower. The navigator plans nothing itself.
 * Until the first map is available the navigator stays idle and only holds the newest goal.
 *
 * Outcomes a user can observe:
 * - A path the planner returns that is blocked on the newest map is not driven; the robot is
 *   stopped and the goal is treated as not plannable.
 * - A goal that could not be planned ends in `Blocked`; it is retried every `blocked_retry_period`
 *   until it succeeds or a new goal arrives.
 * - A plan request with no reply within the planner's search budget plus `stall_timeout` is
 *   abandoned and the robot is stopped.
 * - After `max_stall_replans` replans for one goal because the robot made no progress, navigation
 *   ends in `Blocked` with reason `Stalled`; only a new goal leaves that state.
 *
 * Topics and actions (names are relative to the node namespace):
 * - Subscribes `goal_pose` (`geometry_msgs/PoseStamped`): the navigation goal, in `planning_frame`
 *   (default "map"); a goal in any other frame, including an empty frame id, is ignored with a warning.
 *   A new message replaces the current goal. A goal received before the first map is available is
 *   held, not planned; when the first map arrives the newest held goal is planned once, and earlier
 *   held goals are discarded.
 * - Subscribes `plan_progress` (`path_planner_msgs/PlanProgress`): the follower's report on the
 *   plan it is following.
 * - Publishes `plan` (`path_planner_msgs/PlanTrajectory`, transient-local, depth 1): the plan to
 *   follow. A plan with no poses and `plan_id` 0 tells the follower to stop. Plans toward the goal
 *   carry `speed_scale` 1.0; the clearing move carries `escape_speed_scale`.
 * - Publishes `plan_path` (`nav_msgs/Path`, transient-local, depth 1): the path of whatever was
 *   last published on `plan`, for visualisation.
 * - Action client `plan_path` (`path_planner_msgs/action/PlanPath`): requests a path from the
 *   planner server for the current goal from the robot's current pose.
 *
 * TF: reads `planning_frame` <- `robot_frame` for the robot pose.
 *
 * Parameters (strings; the file paths are required and must be non-empty):
 * - `planning_frame`: frame of goals and paths (default "map").
 * - `robot_frame`: frame of the robot base (default "base_link").
 * - `robot_params_path`: robot geometry and kinematics file.
 * - `diff_drive_params_path`: file holding the differential-drive kinematic section.
 * - `astar_config_path`: planner configuration file (its time budget sets the stopping distance).
 * - `controller_params_path`: follower configuration file.
 * - `navigator_params_path`: navigator configuration file; see `NavigatorParams`.
 *
 * Everything else the node needs (stopping distance, clearing move length, rest tolerances) is
 * derived from those files and from the resolution of the map.
 *
 * @note The constructor throws a YAML exception if a file is missing or malformed, and
 *       `std::invalid_argument` if a file path is empty, a value in a file is out of range,
 *       the follower's cruise speed exceeds the differential-drive maximum speed, or
 *       `stall_timeout` does not exceed the time the follower needs to finish a plan at the
 *       speed of a goal plan and at the speed of the clearing move.
 * @note All callbacks share one mutually exclusive callback group, so the node is safe to run on a
 *       multi-threaded executor.
 */
class Navigator : public rclcpp::Node {
public:
    using PlanPath   = path_planner_msgs::action::PlanPath;  ///< The `plan_path` action type.
    using GoalHandle = rclcpp_action::ClientGoalHandle<PlanPath>;  ///< Client-side handle of one plan request.

    /**
     * @brief Load the configuration and create the interfaces.
     * @param opts Node options; parameters are read as listed in the class description.
     */
    explicit Navigator(const rclcpp::NodeOptions & opts);

    /**
     * @brief Set the source of map snapshots.
     * @param provider Map provider; must be non-null. Bind it before the node is spun.
     */
    void bindMapProvider(std::shared_ptr<IMapProvider> provider);

private:
    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onProgress(const path_planner_msgs::msg::PlanProgress::SharedPtr msg);
    void onTick();

    void dispatch(NavEvent ev);
    void act(const RequestPlan & a);
    void act(const CancelPlan &);
    void act(const RunEscape &);
    void act(const PublishPlan & a);
    void act(const PublishStop &);
    void act(const Report & a);

    void publishTrajectory(const path_planner_msgs::msg::PlanTrajectory & traj);
    void checkEscapeWait(double stamp);
    void runEscape(const SE2State & pose, double stamp);

    // Event time in seconds, counted from the first event. Follows the node clock (so it honors
    // use_sim_time) but never decreases when that clock steps backward. Only differences are meaningful.
    double stamp() { return event_clock_.advance(now().seconds()); }
    bool ensureKinModel(const MapSnapshot & snap);
    std::optional<SE2State> robotPose() const;
    CollisionChecker makeChecker(const std::shared_ptr<const MapSnapshot> & snap) const;
    RemainingPathCheck checkDriven(const std::shared_ptr<const MapSnapshot> & snap, const SE2State & pose);

    // Configuration (fixed at construction).
    std::string     planning_frame_;
    std::string     robot_frame_;
    RobotParams     robot_params_;
    DiffDriveParams kin_params_;
    NavigatorParams nav_params_;
    double          stop_short_ = 0.0;   ///< m; follower goal position tolerance.

    std::optional<NavigationMode> mode_;
    std::shared_ptr<IMapProvider> map_provider_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<path_planner_msgs::msg::PlanProgress>::SharedPtr progress_sub_;
    rclcpp::Publisher<path_planner_msgs::msg::PlanTrajectory>::SharedPtr plan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
    rclcpp_action::Client<PlanPath>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Event loop.
    ForwardClock event_clock_;   // makes event stamps non-decreasing
    std::deque<NavEvent> queue_;
    bool draining_ = false;

    // Navigation state.
    std::optional<geometry_msgs::msg::PoseStamped> goal_;
    GoalHandle::SharedPtr request_handle_;   ///< Accepted, unfinished plan request; null if none.
    path_planner_msgs::msg::PlanTrajectory pending_goal_traj_;
    path_planner_msgs::msg::PlanTrajectory escape_traj_;
    std::vector<SE2State> driven_samples_;   ///< Goal path being driven; empty otherwise.
    std::size_t cursor_ = 0;                 ///< Robot's position on `driven_samples_`.
    std::shared_ptr<const MapSnapshot> last_snap_;
    KinematicModel kin_model_;               ///< Built on the first snapshot; `motion` null before. Goals are held while null.
    std::optional<path_planner_msgs::msg::PlanProgress> last_progress_;

    // Escape wait.
    bool   escape_active_ = false;
    double escape_start_  = 0.0;
    std::optional<SE2State> escape_prev_;
    double rest_pos_tol_ = 0.0;   ///< m; set with the kinematic model.
    double rest_ang_tol_ = 0.0;   ///< rad; set with the kinematic model.
};
