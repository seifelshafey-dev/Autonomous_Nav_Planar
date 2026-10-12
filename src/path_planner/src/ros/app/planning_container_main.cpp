/**
 * @file planning_container_main.cpp
 * @brief Single-process container that co-locates the map provider, the planner node and the navigator,
 *        handing the planner and the navigator a direct in-process handle to the provider (no map topic
 *        between them).
 */
#include <atomic>
#include <csignal>
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros/map_provider/map_provider_server.hpp"
#include "ros/navigator/navigator.hpp"
#include "ros/planner_server/planner_node.hpp"

namespace {

/// Executor to unblock on SIGINT/SIGTERM. Set once before the handler is armed.
/// Raw pointer: the executor outlives the handler (owned by `main`'s local).
std::atomic<rclcpp::Executor *> g_exec{nullptr};

/// Signal handler installed in place of rclcpp's default. It only cancels the
/// executor so `spin()` returns; it deliberately does NOT shut the context down,
/// so the deactivate -> cleanup transitions in `main` still run against a valid
/// context. `Executor::cancel()` trips the interrupt guard condition, which is
/// the intended way to unblock a spin from another thread.
void handleSignal(int /*signum*/) {
    if (auto * exec = g_exec.load()) {
        exec->cancel();
    }
}

/// Drive a lifecycle node down to the unconfigured state, invoking its
/// `on_deactivate`/`on_cleanup` callbacks so their wall timers, bonds, and
/// publishers are released while the rclcpp context is still valid. A no-op for
/// any primary state the node has already reached (e.g. the lifecycle manager
/// already tore it down before this container caught the signal).
void shutdownLifecycleNode(rclcpp_lifecycle::LifecycleNode & node) {
    if (node.get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        node.deactivate();
    }
    if (node.get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        node.cleanup();
    }
}

}  // namespace

/**
 * @brief Run the map provider, planner node and navigator in one process until SIGINT or SIGTERM.
 * @param argc Argument count, forwarded to ROS.
 * @param argv Arguments, forwarded to ROS.
 * @return 0 on a normal shutdown.
 */
int main(int argc, char ** argv) {
    // Suppress rclcpp's built-in signal handler: its default action invalidates
    // the context immediately, which would kill the nodes' timers/bonds before we
    // can walk them through their own shutdown path and abort during teardown.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    auto exec     = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::NodeOptions opts;
    auto provider = std::make_shared<ROSMapProvider>(opts);
    auto server   = std::make_shared<PlannerNode>(opts);
    auto navigator = std::make_shared<Navigator>(opts);

    server->bindMapProvider(provider);
    navigator->bindMapProvider(provider);

    exec->add_node(provider->get_node_base_interface());
    exec->add_node(server->get_node_base_interface());
    exec->add_node(navigator);

    // Arm our own SIGINT/SIGTERM handler now that the executor exists.
    g_exec.store(exec.get());
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    exec->spin();

    // spin() returned because our handler cancelled it; the context is still
    // valid. Detach the nodes, then run each node's own deactivate -> cleanup path
    // to release its wall timers, bond heartbeat timer, and publishers before the
    // context is destroyed. Skipping this aborts on teardown when a live timer
    // handle touches the invalidated context.
    g_exec.store(nullptr);
    exec->remove_node(navigator);
    exec->remove_node(server->get_node_base_interface());
    exec->remove_node(provider->get_node_base_interface());
    shutdownLifecycleNode(*server);
    shutdownLifecycleNode(*provider);

    rclcpp::shutdown();
    return 0;
}
