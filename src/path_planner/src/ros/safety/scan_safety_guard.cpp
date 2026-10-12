/**
 * @file scan_safety_guard.cpp
 * @author Seif ElShafey
 * @brief Implementation of the reactive /scan safety filter (see scan_safety_guard.hpp).
 * @version 0.1
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ros/safety/scan_safety_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "core/control/stop_zone.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::placeholders;

ScanSafetyGuard::ScanSafetyGuard(const rclcpp::NodeOptions & opts)
    : rclcpp::Node("scan_safety_guard", opts),
      last_scan_stamp_(0, 0, RCL_ROS_TIME) {
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

    // Reuse the existing footprint geometry file (no new geometry surface) and the guard tuning.
    const auto robot_path = declare_parameter<std::string>("robot_params_path", "");
    robot_ = loadRobotParams(robot_path);

    const auto guard_path = declare_parameter<std::string>("guard_params_path", "");
    params_ = loadScanGuardParams(guard_path);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Best-effort sensor QoS matches the lidar driver / slam_toolbox scan stream and stays
    // compatible with a reliable test publisher.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&ScanSafetyGuard::onScan, this, _1));

    // The command stream drives the gate (and the debounce cadence): purely event-driven, no timer.
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 10,
        std::bind(&ScanSafetyGuard::onCmdVelRaw, this, _1));
}

void ScanSafetyGuard::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Resolve the (static, URDF-fixed) scan-frame->robot-frame transform ONCE per scan message, not
    // once per point. Prefer the scan's own stamp; fall back to the latest available transform.
    const std::string scan_frame = msg->header.frame_id;
    geometry_msgs::msg::TransformStamped tf;
    try {
        const rclcpp::Time stamp(msg->header.stamp);
        if (tf_buffer_->canTransform(robot_frame_, scan_frame, stamp,
                                     rclcpp::Duration::from_seconds(params_.tf_lookup_timeout))) {
            tf = tf_buffer_->lookupTransform(robot_frame_, scan_frame, stamp);
        } else {
            tf = tf_buffer_->lookupTransform(robot_frame_, scan_frame, tf2::TimePointZero);
        }
    } catch (const std::exception & e) {
        // A single missed lookup on an otherwise-static transform is almost certainly transient
        // (executor contention). Leave the previous cache authoritative; sustained absence is
        // caught by stale_scan_timeout in onCmdVelRaw, not by immediate-tripping here.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup %s->%s failed: %s; keeping previous scan cache.",
                             scan_frame.c_str(), robot_frame_.c_str(), e.what());
        return;
    }

    // Planar projection of the rigid transform (ignore z / roll / pitch): a horizontal lidar's
    // mount is an SE(2) offset in the body plane, matching the codebase's SE(2) convention.
    const double yaw = tf2::getYaw(tf.transform.rotation);
    const Rot2 rot(yaw);
    const Vec2 trans(tf.transform.translation.x, tf.transform.translation.y);

    std::vector<Vec2> pts;
    pts.reserve(msg->ranges.size());
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
        const float range = msg->ranges[i];
        if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) continue;
        const double bearing = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        const Vec2 p_scan(range * std::cos(bearing), range * std::sin(bearing));
        pts.push_back(rot * p_scan + trans);
    }

    const rclcpp::Time stamp(msg->header.stamp);
    cached_points_   = std::move(pts);
    // Store the sensor stamp and the arrival time; the scan age is the larger of the two elapsed times.
    last_scan_stamp_        = stamp;
    last_scan_receive_time_ = get_clock()->now();
    scan_ever_received_     = true;
}

void ScanSafetyGuard::onCmdVelRaw(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const rclcpp::Time now = get_clock()->now();
    if (cmd_ever_received_) cmd_period_ = (now - last_cmd_time_).seconds();
    last_cmd_time_     = now;
    cmd_ever_received_ = true;

    // Fail-safe (bypasses debounce): a guard that has gone blind must not pass commands through.
    if (!scan_ever_received_ || scanAge(now) > params_.stale_scan_timeout) {
        publishStop();
        stopped_          = true;
        violation_streak_ = 0;
        clear_streak_     = 0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "scan stale or never received; fail-safe stop.");
        return;
    }

    const bool raw_violation = computeViolation(*msg, now);
    if (raw_violation) {
        violation_streak_++;
        clear_streak_ = 0;
        if (!stopped_ && violation_streak_ >= params_.stop_count) {
            stopped_ = true;
            double nearest = std::numeric_limits<double>::infinity();
            for (const Vec2 & p : cached_points_) nearest = std::min(nearest, p.norm());
            RCLCPP_WARN(get_logger(),
                        "holding the robot: scan return in the stop zone "
                        "(nearest %.2f m from the robot centre)", nearest);
        }
    } else {
        clear_streak_++;
        violation_streak_ = 0;
        if (stopped_ && clear_streak_ >= params_.clear_count) {
            stopped_ = false;
            RCLCPP_INFO(get_logger(), "released the hold");
        }
    }

    const bool finite = std::isfinite(msg->linear.x) && std::isfinite(msg->angular.z);
    if (stopped_ || !finite) publishStop();
    else                     cmd_pub_->publish(*msg);
}

double ScanSafetyGuard::scanAge(const rclcpp::Time & now) const {
    // The arrival time comes from this node's clock, so `now` earlier than it means the clock stepped
    // back; the age is unknown and counts as stale.
    if (now < last_scan_receive_time_) return std::numeric_limits<double>::infinity();
    return std::max({(now - last_scan_stamp_).seconds(), (now - last_scan_receive_time_).seconds(), 0.0});
}

double ScanSafetyGuard::reactionTime(const rclcpp::Time & now) const {
    const double scan_age = std::clamp(scanAge(now), 0.0, params_.stale_scan_timeout);
    const auto term = [&](double p) {
        return (p > 0.0 && p <= params_.stale_scan_timeout) ? p : params_.stale_scan_timeout;
    };
    return scan_age + params_.stop_count * term(cmd_period_);
}

bool ScanSafetyGuard::computeViolation(const geometry_msgs::msg::Twist & cmd,
                                      const rclcpp::Time & now) const {
    const double v = cmd.linear.x;
    const double w = cmd.angular.z;
    if (!std::isfinite(v) || !std::isfinite(w)) return true;  // fail-safe; motionBlocked would throw
    return motionBlocked(robot_, cached_points_, v, w, reactionTime(now),
                         StopZoneParams{params_.linear_decel, params_.angular_decel,
                                        params_.forward_pad, params_.side_pad});
}

void ScanSafetyGuard::publishStop() {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
}
