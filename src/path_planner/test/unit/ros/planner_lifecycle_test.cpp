#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <thread>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "ros/map_provider/map_provider_server.hpp"
#include "ros/planner_server/planner_node.hpp"

/*
Regression test for a lifecycle non-idempotency bug: on_configure used to declare its ROS
parameters directly, and on_cleanup never undeclared them, so a second on_configure (as issued
by a lifecycle_manager respawn: configure -> ... -> cleanup -> configure) threw "parameter '...'
has already been declared" and left the node permanently stuck. Parameters are now declared once
in each node's constructor (persisting for the node's lifetime); on_configure only reads them via
get_parameter, so repeated configure -> cleanup -> configure cycles succeed. Both ROS-layer
lifecycle nodes are covered here.
*/

namespace {

std::string writeTemp(const std::string & name, const std::string & content) {
    const std::string path = std::string(testing::TempDir()) + name;
    std::ofstream out(path);
    out << content;
    return path;
}

rclcpp::NodeOptions plannerOptions() {
    const std::string cfg = writeTemp("planner_lifecycle_cfg.yml",
        "vertices: [[0.3,0.3],[-0.3,0.3],[-0.3,-0.3],[0.3,-0.3]]\n"
        "diff_drive:\n  rmin: 0.3\n  vmax: 3.0\n  atomic_time: 1.0\n  n_headings: 8\n"
        "a_star:\n  goal_pos_tol: 0.1\n  goal_heading_tol: 0.1\n"
        "heuristic:\n  weight: 1.0\n"
        "cost_policy:\n  clearance_radius: 0.5\n  clearance_decay: 15.0\n"
        "  max_clearance_cost: 30.0\n  unknown_cost_factor: 1.0\n");

    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        rclcpp::Parameter("robot_params_path", cfg),
        rclcpp::Parameter("diff_drive_params_path", cfg),
        rclcpp::Parameter("astar_config_path", cfg),
        rclcpp::Parameter("heuristic_config_path", cfg),
        rclcpp::Parameter("cost_policy_path", cfg),
    });
    return opts;
}

// Provider that never has a map; enough for activation.
struct EmptyProvider : IMapProvider {
    std::shared_ptr<const MapSnapshot> snapshot() const override { return nullptr; }
};

}  // namespace

TEST(ROSMapProviderLifecycle, configureCleanupConfigureSucceeds) {
    rclcpp::NodeOptions opts;
    ROSMapProvider node(opts);
    rclcpp_lifecycle::State state;

    ASSERT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node.on_cleanup(state), CallbackReturn::SUCCESS);
    // The regression: this second declare_parameter (inside on_configure, pre-fix) threw.
    EXPECT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
}

// edt_max_dist must be positive: 0, negative and NaN fail configure; 5.0 is the control.
TEST(ROSMapProviderLifecycle, nonPositiveEdtMaxDistFailsConfigure) {
    rclcpp_lifecycle::State state;
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        rclcpp::NodeOptions opts;
        opts.parameter_overrides({rclcpp::Parameter("edt_max_dist", bad)});
        ROSMapProvider node(opts);
        EXPECT_EQ(node.on_configure(state), CallbackReturn::FAILURE) << "edt_max_dist=" << bad;
    }
    rclcpp::NodeOptions ok;
    ok.parameter_overrides({rclcpp::Parameter("edt_max_dist", 5.0)});
    ROSMapProvider node(ok);
    EXPECT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
}

namespace {
using OG = nav_msgs::msg::OccupancyGrid;

OG makeMap(uint32_t w, uint32_t h, float res, size_t n, int8_t first = 0) {
    OG m;
    m.header.frame_id = "map";
    m.info.width = w;
    m.info.height = h;
    m.info.resolution = res;
    m.info.origin.orientation.w = 1.0;
    m.data.assign(n, 0);
    if (!m.data.empty()) m.data[0] = first;
    return m;
}
}  // namespace

// Time given to the provider's own subscription to take a message the probe already received
// in the same process.
constexpr std::chrono::milliseconds kProviderSettle{200};

// Malformed maps are ignored (snapshot pointer unchanged). The final good map proves the node
// still works after the bad ones and that the stored map signature was not disturbed; it does
// not prove each bad map was processed. Delivery of each bad map to the provider is made likely
// by the probe plus the 200 ms spin, not guaranteed.
TEST(ROSMapProviderLifecycle, malformedMapsAreIgnored) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("edt_max_dist", 5.0)});
    auto node = std::make_shared<ROSMapProvider>(opts);
    rclcpp_lifecycle::State state;
    ASSERT_EQ(node->on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node->on_activate(state), CallbackReturn::SUCCESS);

    auto pub_node = std::make_shared<rclcpp::Node>("malformed_map_pub");
    auto pub = pub_node->create_publisher<OG>("/map", rclcpp::QoS(1).transient_local());
    int seen = 0;  // probe subscription: counts deliveries so each bad map is known to have arrived
    auto probe = pub_node->create_subscription<OG>("/map", rclcpp::QoS(20),
        [&seen](OG::ConstSharedPtr) { ++seen; });
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_node(pub_node);

    auto spinUntil = [&](const std::function<bool()> & done) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!done() && std::chrono::steady_clock::now() < deadline)
            exec.spin_some(std::chrono::milliseconds(20));
        return done();
    };

    pub->publish(makeMap(4, 3, 0.05f, 12));
    ASSERT_TRUE(spinUntil([&] { return node->snapshot() != nullptr; }));
    const auto good = node->snapshot();

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    auto withOrigin = [](double x, double y) {
        OG m = makeMap(4, 3, 0.05f, 12);
        m.info.origin.position.x = x;
        m.info.origin.position.y = y;
        return m;
    };
    const std::vector<OG> bad = {
        makeMap(0, 3, 0.05f, 0), makeMap(4, 0, 0.05f, 0), makeMap(4, 3, 0.0f, 12),
        makeMap(4, 3, nan, 12), makeMap(4, 3, 0.05f, 11),
        makeMap(4, 3, inf, 12),                                                       // (f) resolution +inf
        withOrigin(std::numeric_limits<double>::quiet_NaN(), 0.0),                    // (g) origin x NaN
        withOrigin(0.0, std::numeric_limits<double>::infinity()),                     // (h) origin y +inf
        withOrigin(1e300, 0.0)};                                                      // (i) finite double, inf as float
    for (const auto & m : bad) {
        const int before = seen;
        pub->publish(m);
        ASSERT_TRUE(spinUntil([&] { return seen > before; }));
        const auto until = std::chrono::steady_clock::now() + kProviderSettle;
        while (std::chrono::steady_clock::now() < until) {
            exec.spin_some(std::chrono::milliseconds(5));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        EXPECT_EQ(node->snapshot(), good);
    }

    pub->publish(makeMap(4, 3, 0.05f, 12, 100));
    ASSERT_TRUE(spinUntil([&] { return node->snapshot() != good; }));
    const auto next = node->snapshot();
    ASSERT_NE(next, nullptr);
    EXPECT_EQ(next->grid_.rows(), 3);
    EXPECT_EQ(next->grid_.cols(), 4);

    exec.remove_node(pub_node);
    exec.remove_node(node->get_node_base_interface());
    node->on_deactivate(state);
}

TEST(PlannerNodeLifecycle, configureCleanupConfigureSucceeds) {
    PlannerNode node(plannerOptions());
    rclcpp_lifecycle::State state;

    ASSERT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node.on_cleanup(state), CallbackReturn::SUCCESS);
    // The regression: this second configure (7 declare_parameter calls, pre-fix) threw.
    EXPECT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
}

// A provider bound once before the first configure stays bound across cleanup and re-configure.
TEST(PlannerNodeLifecycle, configureCleanupConfigureActivateSucceeds) {
    PlannerNode node(plannerOptions());
    node.bindMapProvider(std::make_shared<EmptyProvider>());
    rclcpp_lifecycle::State state;

    ASSERT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node.on_cleanup(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node.on_configure(state), CallbackReturn::SUCCESS);
    EXPECT_EQ(node.on_activate(state), CallbackReturn::SUCCESS);
    EXPECT_EQ(node.on_deactivate(state), CallbackReturn::SUCCESS);
}

// A goal whose frame is not the planning frame ("map") is rejected: the client gets a null handle.
TEST(PlannerNodeLifecycle, goalInWrongFrameIsRejected) {
    using PlanPath = path_planner_msgs::action::PlanPath;
    auto node = std::make_shared<PlannerNode>(plannerOptions());
    node->bindMapProvider(std::make_shared<EmptyProvider>());
    rclcpp_lifecycle::State state;
    ASSERT_EQ(node->on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(node->on_activate(state), CallbackReturn::SUCCESS);

    auto client_node = std::make_shared<rclcpp::Node>("wrong_frame_client");
    auto client = rclcpp_action::create_client<PlanPath>(client_node, "plan_path");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_node(client_node);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));

    PlanPath::Goal goal;
    goal.goal.header.frame_id = "odom";
    goal.goal.pose.orientation.w = 1.0;
    auto fut = client->async_send_goal(goal);
    ASSERT_EQ(exec.spin_until_future_complete(fut, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(fut.get(), nullptr);

    // Positive control: the same goal in the planning frame is accepted.
    goal.goal.header.frame_id = "map";
    auto fut_ok = client->async_send_goal(goal);
    ASSERT_EQ(exec.spin_until_future_complete(fut_ok, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    auto handle = fut_ok.get();
    EXPECT_NE(handle, nullptr);
    if (handle) client->async_cancel_goal(handle);
    exec.spin_some();

    exec.remove_node(client_node);
    exec.remove_node(node->get_node_base_interface());
    node->on_deactivate(state);
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Node construction needs an initialized global context; declare_parameter (now in each
    // node's constructor) runs before on_configure is ever called.
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
