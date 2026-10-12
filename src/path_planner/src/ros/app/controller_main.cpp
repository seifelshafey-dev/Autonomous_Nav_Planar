/**
 * @file controller_main.cpp
 * @brief Standalone executable for the ControllerNode pure-pursuit follower.
 *
 * Runs a single ControllerNode on the default single-threaded executor. The `/plan`
 * subscription and the control timer are serialized on this one thread, so the node's
 * members need no atomics or mutexes.
 */
#include "rclcpp/rclcpp.hpp"
#include "ros/controller/controller_node.hpp"

/**
 * @brief Run a ControllerNode until shutdown.
 * @param argc Argument count, forwarded to ROS.
 * @param argv Arguments, forwarded to ROS.
 * @return 0 on a normal shutdown.
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
