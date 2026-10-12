/**
 * @file controller_node.cpp
 * @author Seif ElShafey
 * @brief Implementation of the pure-pursuit follower ROS adapter (see controller_node.hpp).
 * @version 0.1
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ros/controller/controller_node.hpp"

#include <chrono>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "core/config/config_loader.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/diagnostics/check.hpp"

using namespace std::placeholders;

namespace {

/// Map a PlanTrajectory `int8` mode value to the core `MotionMode` enum.
MotionMode toMotionMode(int8_t mode) {
    using PlanTrajectory = path_planner_msgs::msg::PlanTrajectory;
    switch (mode) {
        case PlanTrajectory::REV:   return MotionMode::Reverse;
        case PlanTrajectory::PIVOT: return MotionMode::Pivot;
        case PlanTrajectory::FWD:
        default:                    return MotionMode::Forward;
    }
}

}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & opts)
    : rclcpp::Node("controller_node", opts) {
    const double ctrl_rate = declare_parameter<double>("ctrl_rate", 20.0);
    CHECK(ctrl_rate > 0.0, "ControllerNode: ctrl_rate must be positive");
    dt_ = 1.0 / ctrl_rate;

    planning_frame_ = declare_parameter<std::string>("planning_frame", "map");
    odom_frame_     = declare_parameter<std::string>("odom_frame", "odom");
    robot_frame_    = declare_parameter<std::string>("robot_frame", "base_link");

    const auto params_path = declare_parameter<std::string>("controller_params_path", "");
    const ControllerParams cp = loadControllerParams(params_path);

    // The absolute carrot lookahead is derived from the footprint so it always tracks the followed
    // sample spacing (r_in/2). Reading r_in (footprint) and rmin (kinematics) from the same robot file.
    const auto robot_path = declare_parameter<std::string>("robot_params_path", "");
    const RobotParams     rp  = loadRobotParams(robot_path);
    const DiffDriveParams dd  = loadDiffDriveParams(robot_path);
    const double sample_spacing = rp.r_in / 2.0;
    const double L = cp.lookahead_gain * sample_spacing;
    CHECK(L > sample_spacing && L < 2.0 * dd.rmin,
          "ControllerNode: derived lookahead L must satisfy r_in/2 < L < 2*rmin "
          "(adjust controller.lookahead_gain)");

    CHECK(cp.v_cruise <= dd.vmax,
          "ControllerNode: controller v_cruise (" + std::to_string(cp.v_cruise) +
          " m/s) must not exceed diff-drive vmax (" + std::to_string(dd.vmax) +
          " m/s), the speed the planner's motions are built at");

    params_ = PurePursuitParams{
        L, cp.v_cruise, cp.a_lat_max,
        cp.omega_pivot, cp.goal_pos_tol, cp.goal_heading_tol};

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    progress_pub_ = create_publisher<path_planner_msgs::msg::PlanProgress>("plan_progress", 10);

    // Transient-local matches the navigator's latched /plan publisher, so a late-joining controller
    // still receives the last plan.
    plan_sub_ = create_subscription<path_planner_msgs::msg::PlanTrajectory>(
        "plan", rclcpp::QoS(1).transient_local(),
        std::bind(&ControllerNode::onPlan, this, _1));

    // Node-clock timer (not create_wall_timer) so the control loop honors use_sim_time: system
    // clock under mock/CI, /clock under sim. A wall-clock loop under sim time would skew the
    // command/TF timeline by ~v*dt and smear the SLAM map.
    timer_ = rclcpp::create_timer(this, get_clock(),
                                  rclcpp::Duration::from_seconds(dt_),
                                  std::bind(&ControllerNode::onTimer, this));
    progress_timer_ = rclcpp::create_timer(this, get_clock(),
                                           rclcpp::Duration::from_seconds(0.2),
                                           [this]() { publishProgress(false); });
}

void ControllerNode::onPlan(const path_planner_msgs::msg::PlanTrajectory::SharedPtr msg) {
    // Empty plan (poses.empty()) => stop and clear the active follower.
    if (msg->path.poses.empty()) {
        pursuit_.reset();
        plan_id_ = plan_size_ = 0;
        publishStop();
        publishProgress(true);
        return;
    }

    if (!(msg->speed_scale > 0.0 && msg->speed_scale <= 1.0)) {
        RCLCPP_ERROR(get_logger(), "plan %u has speed_scale %g outside (0, 1]; treating it as a stop.",
                     msg->plan_id, msg->speed_scale);
        pursuit_.reset();
        plan_id_ = plan_size_ = 0;
        publishStop();
        publishProgress(true);
        return;
    }

    // Consumer-side index-alignment CHECK (mirrors the producer in pathToTrajectory).
    CHECK(msg->mode.size() == msg->path.poses.size() &&
          msg->speed.size() == msg->path.poses.size(),
          "ControllerNode: received PlanTrajectory with misaligned mode/speed/poses");

    // Transform the plan planning_frame_->odom_frame_ (map->odom) ONCE, here, and track in odom.
    // Tracking in the smooth odom frame keeps SLAM's 20 Hz map->odom corrections from perturbing
    // the control loop: the corrections move map->odom, not odom->base_link, so a plan pinned to
    // odom no longer jumps under the follower. The transform is refreshed on every /plan
    // (each replan), so the odom-frame trajectory stays consistent with the latest map->odom.
    // Use the plan's own stamp when the buffer has that transform, else the latest (TimePointZero).
    geometry_msgs::msg::TransformStamped plan_to_odom;
    try {
        const rclcpp::Time stamp(msg->path.header.stamp);
        if (tf_buffer_->canTransform(odom_frame_, planning_frame_, stamp)) {
            plan_to_odom = tf_buffer_->lookupTransform(odom_frame_, planning_frame_, stamp);
        } else {
            plan_to_odom = tf_buffer_->lookupTransform(odom_frame_, planning_frame_,
                                                       tf2::TimePointZero);
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup %s->%s failed: %s; commanding stop.",
                             planning_frame_.c_str(), odom_frame_.c_str(), e.what());
        pursuit_.reset();
        plan_id_ = plan_size_ = 0;
        publishStop();
        publishProgress(true);
        return;
    }

    std::vector<TrajectoryPoint> traj;
    traj.reserve(msg->path.poses.size());
    for (std::size_t i = 0; i < msg->path.poses.size(); ++i) {
        // Reproject each plan pose (position + yaw) into the odom frame. mode[]/speed[] are
        // frame-independent, so they copy straight across.
        geometry_msgs::msg::Pose p_odom;
        tf2::doTransform(msg->path.poses[i].pose, p_odom, plan_to_odom);
        TrajectoryPoint tp;
        tp.pose  = SE2State{p_odom.position.x, p_odom.position.y, tf2::getYaw(p_odom.orientation)};
        tp.mode  = toMotionMode(msg->mode[i]);
        tp.speed = msg->speed[i];
        traj.push_back(tp);
    }

    // Rebuild the follower on each new plan (construction resets the progress cursor).
    pursuit_.emplace(params_, std::move(traj), dt_, msg->speed_scale);
    plan_id_   = msg->plan_id;
    plan_size_ = static_cast<uint32_t>(msg->path.poses.size());
    publishProgress(true);
}

void ControllerNode::onTimer() {
    if (!pursuit_) { publishStop(); return; }

    // Robot pose in the (smooth) odom frame as tracking ground truth: latest available
    // (TimePointZero, no extrapolation). The odom->base_link transform is smooth — unlike
    // map->base_link it is not perturbed by SLAM's map->odom corrections — so stepping pursuit
    // against it keeps the 20 Hz loop jump-free. On failure, safe-stop rather than
    // commanding against a stale pose.
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(odom_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup %s->%s failed: %s; commanding stop.",
                             odom_frame_.c_str(), robot_frame_.c_str(), e.what());
        publishStop();
        return;
    }

    const SE2State pose{tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf2::getYaw(tf.transform.rotation)};

    const Control u = pursuit_->step(pose);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = u.v;
    cmd.angular.z = u.omega;
    cmd_pub_->publish(cmd);

    // On completion, hold a zero command until a new plan arrives.
    if (pursuit_->done()) publishStop();
    publishProgress(true);
}

void ControllerNode::publishProgress(bool only_on_change) {
    path_planner_msgs::msg::PlanProgress p;
    p.header.stamp    = now();
    p.header.frame_id = planning_frame_;
    p.plan_id = pursuit_ ? plan_id_ : 0;
    p.cursor  = pursuit_ ? static_cast<uint32_t>(pursuit_->cursor()) : 0;
    p.size    = pursuit_ ? plan_size_ : 0;
    p.arrived = pursuit_ && pursuit_->done();
    if (only_on_change && p.plan_id == last_pub_id_ && p.cursor == last_pub_cursor_ &&
        p.arrived == last_pub_arrived_) {
        return;
    }
    last_pub_id_ = p.plan_id;
    last_pub_cursor_ = p.cursor;
    last_pub_arrived_ = p.arrived;
    progress_pub_->publish(p);
}

void ControllerNode::publishStop() {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
}
