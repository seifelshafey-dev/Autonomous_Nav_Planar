/**
 * @file scan_safety_guard.hpp
 * @author Seif ElShafey
 * @brief ROS2 reactive safety filter interposed between the controller's commanded velocity and the
 *        drive sink, safe-stopping the base on an imminent in-footprint scan return.
 * @version 0.1
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "core/config/config_loader.hpp"
#include "core/robot/robot_params.hpp"

/**
 * @brief Reactive safety filter between the controller's commanded velocity and the drive sink.
 *
 * Subscribes to `scan` and, for each command on `cmd_vel_in`, checks the scan returns (taken in the
 * robot frame) against the area the robot's footprint would cover while stopping from that command:
 * the reaction distance plus the braking distance plus `forward_pad` along the motion, and
 * `side_pad` sideways; for a turn, the swept rotation likewise. A command with a return in that area
 * is a blocked command; a zero command is never blocked and counts as clear. While the hold is engaged the command is replaced by a zero Twist on
 * `cmd_vel`; otherwise it is passed through unchanged, including a blocked command before the hold engages. The
 * reaction time is the age of the newest scan, taken as the larger of the age by its stamp and the time since it
 * arrived, plus `stop_count` measured command periods, so it
 * follows the actual rates and latency. A stop is held until `clear_count` consecutive clear commands, and engaged after
 * `stop_count` consecutive blocked ones. If no scan arrived within `stale_scan_timeout`, the
 * output is zero at once and the hold is set. A command with a non-finite linear or angular velocity is replaced by a zero
 * Twist at once and counts as a blocked command. Needs no `map` or `odom` frame: the check uses only the scan and the footprint in
 * the robot frame, so it is independent of localization.
 *
 * ROS parameters:
 * - `robot_frame` (string, default "base_link"): body frame the scan is transformed into.
 * - `robot_params_path` (string, required): path to `robot_params.yaml` (footprint geometry).
 * - `guard_params_path` (string, required): path to `scan_safety_guard.yaml` (deceleration limits,
 *   pads, debounce counts, timeouts).
 * - `use_sim_time` (bool, standard): use the `/clock` time source when true.
 *
 * Topics (relative names; a launch remap wires them onto the real pipeline):
 * - subscribes `scan` (sensor_msgs/LaserScan)
 * - subscribes `cmd_vel_in` (geometry_msgs/Twist) -- the controller's intended command
 * - publishes `cmd_vel` (geometry_msgs/Twist) -- the gated command
 */
class ScanSafetyGuard : public rclcpp::Node {
public:
    /**
     * @brief Construct the node: declare params, load geometry + guard tuning, and wire the scan /
     *        command subscriptions, the gated-command publisher, and the TF listener.
     * @param opts Node options forwarded to rclcpp::Node.
     */
    explicit ScanSafetyGuard(const rclcpp::NodeOptions & opts);

private:
    /**
     * @brief `scan` callback: transform the valid returns once into the robot frame and cache them.
     *        Never publishes. A failed TF lookup leaves the previous cache authoritative until it
     *        ages out via `stale_scan_timeout` in onCmdVelRaw.
     */
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief `cmd_vel_in` callback and the only publish path: fail-safe on a stale/absent scan,
     *        else gate `msg` against the cached scan + debounce state, and publish the result
     *        (unchanged or zero) on `cmd_vel`. A command whose linear or angular velocity is not
     *        finite is always replaced by a zero Twist and counts as a blocked command.
     */
    void onCmdVelRaw(const geometry_msgs::msg::Twist::SharedPtr msg);

    /// @brief True if a cached scan return lies in the stop zone of the commanded (v, omega), or the
    ///        command is not finite.
    /// @param cmd Commanded velocity.
    /// @param now Current node-clock time, used to age the cached scan.
    bool computeViolation(const geometry_msgs::msg::Twist & cmd, const rclcpp::Time & now) const;

    /// @brief Age of the newest scan at `now` (s): the larger of the age by its stamp and the time
    ///        since it arrived, never negative. Returns infinity when the node clock is earlier than the
    ///        arrival time (clock stepped back), which the guard treats as a stale scan.
    /// @param now Current node-clock time.
    double scanAge(const rclcpp::Time & now) const;

    /// @brief Age of the newest scan at `now` (see scanAge; clamped to [0, stale_scan_timeout]) plus
    ///        `stop_count` command periods (s); a command period not yet measured or outside
    ///        (0, stale_scan_timeout] counts as `stale_scan_timeout`.
    double reactionTime(const rclcpp::Time & now) const;

    /// @brief Publish a zero-velocity `Twist` on `cmd_vel` (safe-stop).
    void publishStop();

    RobotParams     robot_;         ///< Footprint geometry loaded from `robot_params_path`.
    ScanGuardParams params_;        ///< Guard tuning loaded from `guard_params_path`.
    std::string     robot_frame_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;

    std::vector<Vec2> cached_points_;      ///< Latest scan, valid returns only, in robot_frame_.
    rclcpp::Time      last_scan_stamp_;
    rclcpp::Time      last_scan_receive_time_{0, 0, RCL_ROS_TIME};  ///< Node-clock time the newest scan arrived.
    bool             scan_ever_received_ = false;

    rclcpp::Time      last_cmd_time_{0, 0, RCL_ROS_TIME};  ///< Node-clock receive time of the last command.
    bool              cmd_ever_received_ = false;
    double            cmd_period_ = 0.0;   ///< Receive-time gap of the last two commands (s); 0 = unmeasured.

    int  violation_streak_ = 0;
    int  clear_streak_     = 0;
    bool stopped_          = false;        ///< Committed gate state.
};
