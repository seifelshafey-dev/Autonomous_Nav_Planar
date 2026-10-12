/**
 * @file controller_node.hpp
 * @author Seif ElShafey
 * @brief ROS2 adapter node that drives a differential-drive base along a published plan using a
 *        pure-pursuit control law.
 * @version 0.1
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "path_planner_msgs/msg/plan_progress.hpp"
#include "path_planner_msgs/msg/plan_trajectory.hpp"

#include "core/control/pure_pursuit.hpp"

/**
 * @brief Thin ROS adapter that follows a published `PlanTrajectory` with a pure-pursuit law.
 *
 * Subscribes to `plan` (`path_planner_msgs/PlanTrajectory`, transient-local so the last plan is
 * latched), and on a fixed-rate timer looks up the robot pose from TF, asks the
 * `PurePursuit` law for the next control, and publishes it as a `geometry_msgs/Twist` on
 * `cmd_vel`. The node owns no control math — all tracking geometry lives in the ROS-free
 * `PurePursuit` core, constructed from gains loaded via `loadControllerParams`.
 *
 * Publishes `plan_progress` (`path_planner_msgs/PlanProgress`, depth 10) at 5 Hz and whenever the
 * followed plan id, the tracked pose index or the arrived flag changes: `plan_id` is the id of the
 * plan being followed (0 when idle), `cursor` the index of the tracked pose, `size` the number of
 * poses (0 when idle) and `arrived` whether the final pose is reached within the goal tolerances.
 * Each plan's `speed_scale` multiplies the cruise speed and pivot rate; a plan whose `speed_scale`
 * is outside (0, 1] is rejected and treated as a stop.
 *
 * The follower degrades safely to a zero `Twist` whenever there is no active plan, TF is
 * unavailable, or the plan has been completed.
 *
 * The node tracks in the smooth `odom` frame, not the plan (`map`) frame: it transforms the whole
 * incoming plan map->odom once per `plan` message and then steps pursuit against odom->base_link. This
 * keeps the 20 Hz control loop immune to SLAM's map->odom corrections (which would otherwise jump
 * the tracked pose every map update and weave the steering).
 *
 * ROS parameters:
 * - `ctrl_rate` (double, Hz, default 20.0): control-loop / publish rate.
 * - `planning_frame` (string, default "map"): fixed frame the incoming plan is expressed in.
 * - `odom_frame` (string, default "odom"): smooth tracking frame; the plan is transformed into it
 *   once per `plan` message, and the per-tick robot pose is looked up in it.
 * - `robot_frame` (string, default "base_link"): robot body frame looked up against the odom frame.
 * - `controller_params_path` (string): path to the `controller.yaml` gains file (required).
 * - `robot_params_path` (string): path to `robot_params.yaml`; supplies r_in (footprint) and rmin
 *   (kinematics) used to derive the absolute lookahead L = lookahead_gain * (r_in / 2). Required.
 * - `use_sim_time` (bool, standard): use the `/clock` time source when true.
 *
 * The `cmd_vel` topic is named plainly by the node; a launch remap wires it onto the concrete
 * drive sink (e.g. `diff_drive_controller`'s `~/cmd_vel_unstamped`), keeping the node
 * deployment-agnostic.
 */
class ControllerNode : public rclcpp::Node {
public:
    /**
     * @brief Construct the node: declare params, load gains, and wire the subscription, timer,
     *        publisher, and TF listener.
     * @param opts Node options forwarded to rclcpp::Node.
     */
    explicit ControllerNode(const rclcpp::NodeOptions & opts);

private:
    /**
     * @brief `/plan` callback: transform the plan planning_frame->odom_frame once, convert it to
     *        core `TrajectoryPoint`s, and (re)build the follower on the odom-frame trajectory.
     *        An empty plan or a failed planning_frame->odom_frame TF lookup clears the active
     *        follower and commands a zero `Twist`.
     */
    void onPlan(const path_planner_msgs::msg::PlanTrajectory::SharedPtr msg);

    /**
     * @brief Timer callback at `ctrl_rate`: look up the odom_frame->robot_frame pose and publish
     *        the next control (tracking runs entirely in the odom frame).
     *
     * Publishes a zero `Twist` when there is no active plan, when the TF lookup fails, or when the
     * follower has reported completion.
     */
    void onTimer();

    /// @brief Publish a zero-velocity `Twist` on `/cmd_vel` (safe-stop).
    void publishStop();

    /// @brief Publish the current `PlanProgress`; with `only_on_change` skip it if no field changed.
    void publishProgress(bool only_on_change);

    PurePursuitParams params_;      ///< Follower gains loaded from `controller_params_path`.
    double            dt_;          ///< Control period (s) = 1 / ctrl_rate.
    std::string       planning_frame_;
    std::string       odom_frame_;
    std::string       robot_frame_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<path_planner_msgs::msg::PlanTrajectory>::SharedPtr plan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                 cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                            timer_;
    rclcpp::Publisher<path_planner_msgs::msg::PlanProgress>::SharedPtr      progress_pub_;
    rclcpp::TimerBase::SharedPtr                                            progress_timer_;

    /// Active follower; null until a non-empty plan arrives, reset on each new plan.
    std::optional<PurePursuit> pursuit_;

    uint32_t plan_id_   = 0;  ///< plan_id of the followed plan; 0 when idle.
    uint32_t plan_size_ = 0;  ///< Number of poses in the followed plan; 0 when idle.

    /// Last published progress, to detect changes.
    uint32_t last_pub_id_ = 0, last_pub_cursor_ = 0;
    bool     last_pub_arrived_ = false;
};
