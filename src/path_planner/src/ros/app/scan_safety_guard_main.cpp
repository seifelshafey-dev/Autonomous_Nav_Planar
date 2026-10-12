/**
 * @file scan_safety_guard_main.cpp
 * @brief Standalone executable for the ScanSafetyGuard reactive /scan safety filter.
 *
 * Runs a single ScanSafetyGuard on the default single-threaded executor. The `scan` and
 * `cmd_vel_in` subscriptions are serialized on this one thread, so the node's members need no
 * atomics or mutexes.
 */
#include "rclcpp/rclcpp.hpp"
#include "ros/safety/scan_safety_guard.hpp"

/**
 * @brief Run a ScanSafetyGuard until shutdown.
 * @param argc Argument count, forwarded to ROS.
 * @param argv Arguments, forwarded to ROS.
 * @return 0 on a normal shutdown.
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanSafetyGuard>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
