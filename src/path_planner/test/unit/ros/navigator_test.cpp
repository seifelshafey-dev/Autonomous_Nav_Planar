#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "path_planner_msgs/action/plan_path.hpp"
#include "path_planner_msgs/msg/plan_progress.hpp"
#include "path_planner_msgs/msg/plan_trajectory.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "core/config/config_loader.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/map/map_provider.hpp"
#include "ros/navigator/navigator.hpp"

/*
In-process test of the Navigator node against a scripted plan_path action server. One executor
spins the navigator, the fake planner and a probe node; the test thread publishes goals and
progress and reads back the published plans. Each TEST builds fresh nodes in its own namespace so
graph entries left by a previous test cannot satisfy this test's readiness waits.

Timing: the navigator ticks every TICK = 0.1 s. A driven plan without progress reports is declared
stalled after STALL_S = 5.0 s, so every case finishes well inside that.
*/

namespace {

using namespace std::chrono_literals;
using path_planner_msgs::msg::PlanProgress;
using path_planner_msgs::msg::PlanTrajectory;
using PlanPath = path_planner_msgs::action::PlanPath;
using Cells    = std::vector<int8_t>;

constexpr int    N        = 200;     // grid is N x N cells, origin (0, 0): a 10 m square
constexpr double H        = 0.05;    // grid resolution (m)
constexpr double TICK     = 0.1;     // navigator tick period written to the test config (s)
constexpr double STALL_S  = 5.0;     // stall_timeout written to the test config (s); must exceed the shipped follower's crawl + turn bound (4.47 s)
constexpr double ROBOT_X  = 5.03;    // robot pose: footprint x, y in [4.73, 5.33]
constexpr double ROBOT_Y  = 5.03;
constexpr double STEP     = 0.1;     // spacing of the poses in a scripted path (m)
constexpr double WAIT_S   = 10.0;    // generous bound for every waitFor

template <class Pred>
bool waitFor(Pred pred, double timeout_s = WAIT_S) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

// Yaw of a planar pose (rotation about z only).
double yawOf(const geometry_msgs::msg::Quaternion & q) { return 2.0 * std::atan2(q.z, q.w); }

// "Nothing happens" waits only: n navigator ticks.
void waitTicks(int n) { std::this_thread::sleep_for(std::chrono::duration<double>(n * TICK)); }

std::string shippedConfig(const char * dir, const char * file) {
    // <path_planner>/test/unit/ros/navigator_test.cpp -> up 4 dirs to <path_planner>.
    const std::filesystem::path root =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
    return (root / "config" / dir / file).string();
}

// ---- maps ------------------------------------------------------------------------------------

// Cell (i, j) = (column, row) is at cells[j * N + i]; its x range is [i*H, (i+1)*H).
void fillRect(Cells & c, int i0, int i1, int j0, int j1, int8_t val) {  // inclusive
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) c[static_cast<size_t>(j) * N + i] = val;
}

std::shared_ptr<const MapSnapshot> makeSnapshot(int8_t fill, const std::function<void(Cells &)> & paint) {
    GridMetaData meta(N, N, H, Eigen::Vector2f::Zero());
    Cells cells(static_cast<size_t>(N) * N, fill);
    if (paint) paint(cells);
    OccupancyGrid grid(meta, cells);
    EDT edt = buildEDTGrid(grid, 5.0);
    return std::make_shared<const MapSnapshot>(MapSnapshot{std::move(grid), std::move(edt)});
}

std::shared_ptr<const MapSnapshot> freeMap() { return makeSnapshot(0, nullptr); }

// Wall ahead: cells i = 106..115 (x from 5.30), every row. Its face is 0.03 m inside the start
// footprint front (5.33), so the start is blocked and backing out clears it.
std::shared_ptr<const MapSnapshot> wallAheadMap() {
    return makeSnapshot(0, [](Cells & c) { fillRect(c, 106, 115, 0, N - 1, 100); });
}

// Closed box: every cell occupied except i, j = 94..106 (x, y in [4.70, 5.35]); no way out.
std::shared_ptr<const MapSnapshot> closedBoxMap() {
    return makeSnapshot(100, [](Cells & c) { fillRect(c, 94, 106, 94, 106, 0); });
}

// Wall across the whole map at cells i = 120..123 (x in [6.00, 6.20]): 1.0 m ahead of the robot
// centre, across a path running along +x.
std::shared_ptr<const MapSnapshot> blockedAheadMap() {
    return makeSnapshot(0, [](Cells & c) { fillRect(c, 120, 123, 0, N - 1, 100); });
}

// ---- fakes -----------------------------------------------------------------------------------

class FakeMapProvider : public IMapProvider {
public:
    explicit FakeMapProvider(std::shared_ptr<const MapSnapshot> s) : snap_(std::move(s)) {}
    std::shared_ptr<const MapSnapshot> snapshot() const override {
        std::lock_guard<std::mutex> lk(m_);
        return snap_;
    }
    void set(std::shared_ptr<const MapSnapshot> s) {
        std::lock_guard<std::mutex> lk(m_);
        snap_ = std::move(s);
    }
private:
    mutable std::mutex m_;
    std::shared_ptr<const MapSnapshot> snap_;
};

// Straight path in the map frame from (x0, y0) to (x1, y1), STEP apart, all FWD at 0.5 m/s.
PlanTrajectory lineTraj(double x0, double y0, double x1, double y1) {
    PlanTrajectory t;
    t.header.frame_id      = "map";
    t.path.header.frame_id = "map";
    const double len = std::hypot(x1 - x0, y1 - y0);
    const int    n   = std::max(1, static_cast<int>(std::ceil(len / STEP)));
    const double yaw = std::atan2(y1 - y0, x1 - x0);
    for (int i = 0; i <= n; ++i) {
        const double f = static_cast<double>(i) / n;
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id    = "map";
        ps.pose.position.x    = x0 + f * (x1 - x0);
        ps.pose.position.y    = y0 + f * (y1 - y0);
        ps.pose.orientation.z = std::sin(yaw / 2.0);
        ps.pose.orientation.w = std::cos(yaw / 2.0);
        t.path.poses.push_back(ps);
        t.mode.push_back(PlanTrajectory::FWD);
        t.speed.push_back(0.5);
    }
    t.plan_id     = 0;
    t.speed_scale = 1.0;
    return t;
}

struct Reply {
    uint8_t        outcome = PlanPath::Result::FOUND;
    PlanTrajectory trajectory;   // used for FOUND only
};

// Scripted plan_path server. For each accepted goal it replies after `delay` with the outcome the
// script gives for (goal, index of the goal in arrival order). A goal still unanswered when a newer
// one is accepted is ended with STOPPED (aborted, or canceled if the client asked), like the real
// planner node.
class FakePlanServer : public rclcpp::Node {
public:
    using Script = std::function<Reply(const geometry_msgs::msg::PoseStamped &, size_t)>;
    using GoalHandle = rclcpp_action::ServerGoalHandle<PlanPath>;

    explicit FakePlanServer(const std::string & ns) : rclcpp::Node("fake_planner", ns) {
        server_ = rclcpp_action::create_server<PlanPath>(
            this, "plan_path",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const PlanPath::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](std::shared_ptr<GoalHandle> h) { onAccepted(h); });
        timer_ = create_wall_timer(10ms, [this] { onTimer(); });
    }

    void configure(Script script, double delay_s) {
        std::lock_guard<std::mutex> lk(m_);
        script_ = std::move(script);
        delay_s_ = delay_s;
    }
    size_t goalCount() const {
        std::lock_guard<std::mutex> lk(m_);
        return goals_.size();
    }
    geometry_msgs::msg::PoseStamped goal(size_t i) const {
        std::lock_guard<std::mutex> lk(m_);
        return goals_.at(i);
    }

private:
    struct Pending {
        std::shared_ptr<GoalHandle> handle;
        size_t index;
        std::chrono::steady_clock::time_point deadline;
    };

    void onAccepted(std::shared_ptr<GoalHandle> h) {
        // ACCEPT_AND_EXECUTE: the handle is already EXECUTING here; execute() would be an invalid transition.
        std::lock_guard<std::mutex> lk(m_);
        pending_.push_back({h, goals_.size(),
                            std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                   std::chrono::duration<double>(delay_s_))});
        goals_.push_back(h->get_goal()->goal);
    }

    void onTimer() {
        std::vector<Pending> keep;
        std::vector<Pending> work;
        Script script;
        size_t total;
        {
            std::lock_guard<std::mutex> lk(m_);
            work.swap(pending_);
            script = script_;
            total = goals_.size();
        }
        const auto now = std::chrono::steady_clock::now();
        for (auto & p : work) {
            if (!p.handle->is_executing() && !p.handle->is_canceling()) continue;   // already resolved
            const bool superseded = p.index + 1 < total;
            if (superseded || p.handle->is_canceling()) {
                auto r = std::make_shared<PlanPath::Result>();
                r->outcome = PlanPath::Result::STOPPED;
                if (p.handle->is_canceling()) p.handle->canceled(r);
                else p.handle->abort(r);
            } else if (now >= p.deadline) {
                const Reply rep = script(p.handle->get_goal()->goal, p.index);
                auto r = std::make_shared<PlanPath::Result>();
                r->outcome = rep.outcome;
                if (rep.outcome == PlanPath::Result::FOUND) {
                    r->trajectory = rep.trajectory;
                    p.handle->succeed(r);
                } else {
                    p.handle->abort(r);
                }
            } else {
                keep.push_back(p);
            }
        }
        std::lock_guard<std::mutex> lk(m_);
        pending_.insert(pending_.begin(), keep.begin(), keep.end());
    }

    mutable std::mutex m_;
    Script script_;
    double delay_s_ = 0.0;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    std::vector<Pending> pending_;
    rclcpp_action::Server<PlanPath>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Test-side node: publishes goal_pose and plan_progress, records every plan message in order.
class Probe : public rclcpp::Node {
public:
    explicit Probe(const std::string & ns) : rclcpp::Node("navigator_probe", ns), tf_buffer_(get_clock()) {
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        progress_pub_ = create_publisher<PlanProgress>("plan_progress", 10);
        // Depth 10 on this side (the publisher keeps 1): a burst of plans must not be dropped here.
        plan_sub_ = create_subscription<PlanTrajectory>(
            "plan", rclcpp::QoS(10).transient_local().reliable(), [this](PlanTrajectory::SharedPtr m) {
                std::lock_guard<std::mutex> lk(m_);
                plans_.push_back(*m);
            });
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this, false);
    }

    void publishGoal(double x, double y, const std::string & frame = "map") {
        geometry_msgs::msg::PoseStamped g;
        g.header.frame_id    = frame;
        g.pose.position.x    = x;
        g.pose.position.y    = y;
        g.pose.orientation.w = 1.0;
        goal_pub_->publish(g);
    }
    void publishArrived(uint32_t plan_id, uint32_t size) {
        PlanProgress p;
        p.plan_id = plan_id;
        p.cursor  = size - 1;
        p.size    = size;
        p.arrived = true;
        progress_pub_->publish(p);
    }
    std::vector<PlanTrajectory> plans() const {
        std::lock_guard<std::mutex> lk(m_);
        return plans_;
    }
    size_t planCount() const {
        std::lock_guard<std::mutex> lk(m_);
        return plans_.size();
    }
    // Plans carrying a real plan_id (not the stop plan).
    std::vector<PlanTrajectory> realPlans() const {
        std::vector<PlanTrajectory> out;
        for (auto & p : plans()) if (p.plan_id != 0) out.push_back(p);
        return out;
    }
    bool wired(const std::string & ns) {
        const auto svcs = get_service_names_and_types();
        for (const char * s : {"send_goal", "cancel_goal", "get_result"})
            if (!svcs.count(ns + "/plan_path/_action/" + s)) return false;
        return goal_pub_->get_subscription_count() >= 1 && progress_pub_->get_subscription_count() >= 1 &&
               count_publishers(ns + "/plan_path/_action/status") >= 1 &&
               tf_buffer_.canTransform("map", "base_link", tf2::TimePointZero);
    }

private:
    mutable std::mutex m_;
    std::vector<PlanTrajectory> plans_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<PlanProgress>::SharedPtr progress_pub_;
    rclcpp::Subscription<PlanTrajectory>::SharedPtr plan_sub_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

// ---- fixture ---------------------------------------------------------------------------------

class NavigatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        static std::atomic<int> counter{0};
        ns_ = "/nav_t" + std::to_string(counter++);

        // tick 0.1 s, stall STALL_S = 5.0 s (> 4.47 s, the shipped follower's final approach plus
        // final turn at speed scale 1: 1.35 s + 3.12 s, checked by the Navigator constructor), retry 0.5 s.
        nav_yaml_ = std::string(testing::TempDir()) + "navigator_test" + std::to_string(counter) + ".yaml";
        std::ofstream(nav_yaml_) <<
            "navigator:\n"
            "  tick_period_sec: 0.1\n"
            "  stall_timeout: " + std::to_string(STALL_S) + "\n"
            "  blocked_retry_period: 0.5\n"
            "  max_clearing_attempts: 2\n"
            "  max_stall_replans: 2\n"
            "  escape_speed_scale: 0.5\n"
            "  escape:\n"
            "    step_fraction: 0.4\n"
            "    slack_cells: 1\n"
            "    max_travel_factor: 5.0\n";

        const std::string robot = shippedConfig("robot", "robot_params.yaml");
        rclcpp::NodeOptions opts;
        opts.arguments({"--ros-args", "-r", "__ns:=" + ns_});
        opts.parameter_overrides({
            rclcpp::Parameter("robot_params_path", robot),
            rclcpp::Parameter("diff_drive_params_path", robot),
            rclcpp::Parameter("astar_config_path", shippedConfig("planner", "planner.yaml")),
            rclcpp::Parameter("controller_params_path", shippedConfig("controller", "controller.yaml")),
            rclcpp::Parameter("navigator_params_path", nav_yaml_),
        });
        if (sim_time_) opts.append_parameter_override("use_sim_time", true);
        provider_ = std::make_shared<FakeMapProvider>(freeMap());
        nav_      = std::make_shared<Navigator>(opts);
        nav_->bindMapProvider(provider_);
        fake_  = std::make_shared<FakePlanServer>(ns_);
        probe_ = std::make_shared<Probe>(ns_);
        tf_    = std::make_shared<tf2_ros::StaticTransformBroadcaster>(probe_);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp            = probe_->now();
        t.header.frame_id         = "map";
        t.child_frame_id          = "base_link";
        t.transform.translation.x = ROBOT_X;
        t.transform.translation.y = ROBOT_Y;
        t.transform.rotation.w    = 1.0;
        tf_->sendTransform(t);
    }

    void TearDown() override {
        if (exec_) {
            exec_->cancel();
            if (spin_.joinable()) spin_.join();
        }
        tf_.reset(); probe_.reset(); fake_.reset(); nav_.reset(); exec_.reset();
        std::filesystem::remove(nav_yaml_);
    }

    // Spin all nodes and wait until goals, progress, the action and the robot pose are all wired.
    void start() {
        exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        exec_->add_node(nav_);
        exec_->add_node(fake_);
        exec_->add_node(probe_);
        spin_ = std::thread([this] { exec_->spin(); });
        ASSERT_TRUE(waitFor([&] { return probe_->wired(ns_); })) << "navigator / fake planner not wired";
    }

    void script(FakePlanServer::Script s, double delay_s = 0.0) { fake_->configure(std::move(s), delay_s); }

    // FOUND along a straight path from the robot to the goal.
    static FakePlanServer::Script foundStraight() {
        return [](const geometry_msgs::msg::PoseStamped & g, size_t) {
            Reply r;
            r.outcome    = PlanPath::Result::FOUND;
            r.trajectory = lineTraj(ROBOT_X, ROBOT_Y, g.pose.position.x, g.pose.position.y);
            return r;
        };
    }
    static FakePlanServer::Script fixed(uint8_t outcome) {
        return [outcome](const geometry_msgs::msg::PoseStamped &, size_t) {
            Reply r;
            r.outcome = outcome;
            return r;
        };
    }

    // Goal 2.0 m ahead of the robot along +x; the straight path to it is 21 poses (2.0 / 0.1 + 1).
    static constexpr double GOAL_X = ROBOT_X + 2.0;

    bool sim_time_ = false;   // set by a derived fixture's constructor: the navigator runs on /clock
    std::string ns_, nav_yaml_;
    std::shared_ptr<FakeMapProvider> provider_;
    std::shared_ptr<Navigator> nav_;
    std::shared_ptr<FakePlanServer> fake_;
    std::shared_ptr<Probe> probe_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    std::thread spin_;
};

void expectSamePoses(const PlanTrajectory & got, const PlanTrajectory & want) {
    ASSERT_EQ(got.path.poses.size(), want.path.poses.size());
    for (size_t i = 0; i < got.path.poses.size(); ++i) {
        EXPECT_EQ(got.path.poses[i].pose.position.x, want.path.poses[i].pose.position.x) << i;
        EXPECT_EQ(got.path.poses[i].pose.position.y, want.path.poses[i].pose.position.y) << i;
        EXPECT_EQ(got.path.poses[i].pose.orientation.z, want.path.poses[i].pose.orientation.z) << i;
        EXPECT_EQ(got.path.poses[i].pose.orientation.w, want.path.poses[i].pose.orientation.w) << i;
    }
}

}  // namespace

// 1. A found path is published once, under a real plan id at full speed, and the planner was asked
// exactly once for the goal that was published.
TEST_F(NavigatorTest, foundPathIsPublishedOnce) {
    script(foundStraight());
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(5);   // nothing else may follow; 5 ticks = 0.5 s, inside the stall timeout
    const auto plans = probe_->plans();
    ASSERT_EQ(plans.size(), 1u);
    EXPECT_NE(plans[0].plan_id, 0u);
    EXPECT_EQ(plans[0].speed_scale, 1.0);
    expectSamePoses(plans[0], lineTraj(ROBOT_X, ROBOT_Y, GOAL_X, ROBOT_Y));
    EXPECT_EQ(plans[0].path.poses.size(), 21u);

    ASSERT_EQ(fake_->goalCount(), 1u);
    EXPECT_EQ(fake_->goal(0).pose.position.x, GOAL_X);
    EXPECT_EQ(fake_->goal(0).pose.position.y, ROBOT_Y);
    EXPECT_EQ(fake_->goal(0).pose.orientation.w, 1.0);
}

// 1b. A goal stamped in a frame other than "map" is ignored; the same goal in "map" is planned once.
TEST_F(NavigatorTest, goalInWrongFrameIsIgnored) {
    script(foundStraight());
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y, "odom");
    waitTicks(5);
    EXPECT_EQ(fake_->goalCount(), 0u);
    EXPECT_EQ(probe_->planCount(), 0u);

    probe_->publishGoal(GOAL_X, ROBOT_Y);
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(5);
    EXPECT_EQ(fake_->goalCount(), 1u);
    EXPECT_EQ(probe_->realPlans().size(), 1u);
}

// 2. NO_PATH ends in the stop plan (id 0, no poses); no real plan is ever published.
TEST_F(NavigatorTest, noPathPublishesStopPlanOnly) {
    script(fixed(PlanPath::Result::NO_PATH));
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return probe_->planCount() >= 1; }));
    waitTicks(5);
    const auto plans = probe_->plans();
    ASSERT_GE(plans.size(), 1u);
    EXPECT_EQ(plans[0].plan_id, 0u);
    EXPECT_TRUE(plans[0].path.poses.empty());
    EXPECT_TRUE(probe_->realPlans().empty());
}

// 3. START_BLOCKED: stop, then (follower idle, robot at rest for two ticks) a straight REV escape
// plan at escape speed. Travel = distance to the slack margin + stop_short: the wall face is 0.03 m
// inside the footprint front, the start needs (sqrt2/2 + 1/2)*H + slack H = 0.0604 + 0.05 beyond the
// front, minus stop_short (goal_pos_tol 0.10) => 0.22 m, as in escape_search_test. Arrival of the
// escape plan makes the navigator plan again.
TEST_F(NavigatorTest, blockedStartBacksOutThenReplans) {
    provider_->set(wallAheadMap());
    script([](const geometry_msgs::msg::PoseStamped &, size_t idx) {
        Reply r;
        r.outcome = idx == 0 ? PlanPath::Result::START_BLOCKED : PlanPath::Result::NO_PATH;
        return r;
    });
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return probe_->planCount() >= 2; }));
    const auto plans = probe_->plans();
    EXPECT_EQ(plans[0].plan_id, 0u);
    EXPECT_TRUE(plans[0].path.poses.empty());

    const PlanTrajectory & esc = plans[1];
    EXPECT_NE(esc.plan_id, 0u);
    EXPECT_EQ(esc.speed_scale, 0.5);
    ASSERT_GE(esc.path.poses.size(), 2u);
    ASSERT_EQ(esc.mode.size(), esc.path.poses.size());
    for (const auto m : esc.mode) EXPECT_EQ(m, PlanTrajectory::REV);
    const auto & first = esc.path.poses.front().pose;
    const auto & last  = esc.path.poses.back().pose;
    EXPECT_NEAR(first.position.x, ROBOT_X, 1e-6);
    EXPECT_NEAR(first.position.y, ROBOT_Y, 1e-6);
    EXPECT_NEAR(last.position.x, ROBOT_X - 0.22, 1e-6);
    EXPECT_NEAR(last.position.y, ROBOT_Y, 1e-6);
    EXPECT_NEAR(yawOf(first.orientation), 0.0, 1e-6);
    EXPECT_NEAR(yawOf(last.orientation), 0.0, 1e-6);
    EXPECT_EQ(fake_->goalCount(), 1u);

    probe_->publishArrived(esc.plan_id, static_cast<uint32_t>(esc.path.poses.size()));
    ASSERT_TRUE(waitFor([&] { return fake_->goalCount() >= 2; }));
}

// 4. START_BLOCKED with no way out: the stop plan only; no escape plan is ever published. The
// blocked goal is retried on the timer, so further requests (each ending the same way) are expected.
TEST_F(NavigatorTest, blockedStartWithoutEscapePublishesNoMove) {
    provider_->set(closedBoxMap());
    script(fixed(PlanPath::Result::START_BLOCKED));
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return probe_->planCount() >= 1; }));
    EXPECT_EQ(probe_->plans()[0].plan_id, 0u);
    waitTicks(15);   // escape needs two ticks; 15 ticks = 1.5 s, inside the stall timeout
    EXPECT_TRUE(probe_->realPlans().empty());
    EXPECT_GE(fake_->goalCount(), 2u);
}

// 5. A reply for a superseded request is ignored: only the newer goal's path is published.
TEST_F(NavigatorTest, supersededRequestReplyIsIgnored) {
    script(foundStraight(), 0.5);
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);                       // goal A
    ASSERT_TRUE(waitFor([&] { return fake_->goalCount() >= 1; }));   // inside the 0.5 s delay
    probe_->publishGoal(ROBOT_X, ROBOT_Y + 2.0);                // goal B, a different pose

    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(8);   // 0.8 s: A's own deadline has long passed; a stale publish would show by now
    const auto real = probe_->realPlans();
    ASSERT_EQ(real.size(), 1u);
    expectSamePoses(real[0], lineTraj(ROBOT_X, ROBOT_Y, ROBOT_X, ROBOT_Y + 2.0));
    EXPECT_EQ(fake_->goalCount(), 2u);
}

// 6. Map changes while a request is outstanding do not trigger another request.
TEST_F(NavigatorTest, mapChangesWhilePlanningDoNotReplan) {
    script(foundStraight(), 1.0);
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);
    ASSERT_TRUE(waitFor([&] { return fake_->goalCount() >= 1; }));

    for (int i = 0; i < 10; ++i) {   // paced swaps over 0.5 s, inside the 1.0 s reply delay
        provider_->set(freeMap());
        std::this_thread::sleep_for(50ms);
    }
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(5);
    EXPECT_EQ(fake_->goalCount(), 1u);
    EXPECT_EQ(probe_->realPlans().size(), 1u);
}

// 7. An adopted path that a new map blocks 1.0 m ahead (inside the stopping distance
// v_cruise * max_plan_time_sec) is stopped at once and re-requested.
TEST_F(NavigatorTest, pathBlockedNearbyStopsAndReplans) {
    const double stop_distance =
        loadControllerParams(shippedConfig("controller", "controller.yaml")).v_cruise *
        (loadAStarConfig(shippedConfig("planner", "planner.yaml")).max_plan_time_sec + TICK);
    ASSERT_LT(1.0, stop_distance);

    script([](const geometry_msgs::msg::PoseStamped & g, size_t idx) {
        Reply r;
        if (idx == 0) {
            r.outcome    = PlanPath::Result::FOUND;
            r.trajectory = lineTraj(ROBOT_X, ROBOT_Y, g.pose.position.x, g.pose.position.y);
        } else {
            r.outcome = PlanPath::Result::NO_PATH;   // the blocked map admits no path
        }
        return r;
    });
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));

    provider_->set(blockedAheadMap());
    ASSERT_TRUE(waitFor([&] { return probe_->planCount() >= 2; }));
    const auto plans = probe_->plans();
    EXPECT_NE(plans[0].plan_id, 0u);
    EXPECT_EQ(plans[1].plan_id, 0u);
    EXPECT_TRUE(plans[1].path.poses.empty());
    ASSERT_TRUE(waitFor([&] { return fake_->goalCount() >= 2; }));
}

// 8. After the follower reports arrival the goal is cleared: a later map change (here one that
// would block the old path) sends no request.
TEST_F(NavigatorTest, arrivalClearsGoal) {
    script(foundStraight());
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    const auto plan = probe_->realPlans()[0];

    probe_->publishArrived(plan.plan_id, static_cast<uint32_t>(plan.path.poses.size()));
    // The navigator publishes nothing on arrival, so there is nothing to wait on: let the progress
    // message be handled (3 ticks) before the map changes.
    waitTicks(3);
    provider_->set(blockedAheadMap());
    waitTicks(10);
    EXPECT_EQ(fake_->goalCount(), 1u);
    EXPECT_EQ(probe_->planCount(), 1u);
}

// 9. Goals received before the first map snapshot are held: no request, no publication. When the
// first snapshot appears, only the newest held goal is planned, exactly once.
TEST_F(NavigatorTest, goalBeforeFirstMapIsHeldThenPlannedOnce) {
    provider_->set(nullptr);
    script(foundStraight());
    start();
    probe_->publishGoal(GOAL_X - 1.0, ROBOT_Y);   // superseded by the next goal
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    waitTicks(5);   // 0.5 s: both goals delivered, nothing may happen without a map
    EXPECT_EQ(fake_->goalCount(), 0u);
    EXPECT_EQ(probe_->planCount(), 0u);

    provider_->set(freeMap());
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(5);   // nothing else may follow
    ASSERT_EQ(fake_->goalCount(), 1u);
    EXPECT_EQ(fake_->goal(0).pose.position.x, GOAL_X);
    const auto plans = probe_->plans();
    ASSERT_EQ(plans.size(), 1u);
    EXPECT_NE(plans[0].plan_id, 0u);
}

// 10. A found path that the navigator's own newest map blocks is never driven: stop plan, and the
// goal is retried on the timer (blocked_retry_period 0.5 s) while the wall stays. Once the map is
// free, exactly one real plan appears.
TEST_F(NavigatorTest, foundPathBlockedOnOwnMapIsNotDriven) {
    provider_->set(blockedAheadMap());
    script(foundStraight());
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return probe_->planCount() >= 1; }));
    const auto plans = probe_->plans();
    EXPECT_EQ(plans[0].plan_id, 0u);
    EXPECT_TRUE(plans[0].path.poses.empty());

    waitTicks(15);   // 1.5 s with the wall in place: at least one timer retry
    EXPECT_TRUE(probe_->realPlans().empty());
    EXPECT_GE(fake_->goalCount(), 2u);

    provider_->set(freeMap());
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));
    waitTicks(5);
    EXPECT_EQ(probe_->realPlans().size(), 1u);
}

// 10b. A blocked goal is retried on the timer with no map change at all.
TEST_F(NavigatorTest, blockedGoalIsRetriedWithoutAMapChange) {
    script([](const geometry_msgs::msg::PoseStamped & g, size_t idx) {
        Reply r;
        if (idx == 0) {
            r.outcome = PlanPath::Result::NO_PATH;
        } else {
            r.outcome    = PlanPath::Result::FOUND;
            r.trajectory = lineTraj(ROBOT_X, ROBOT_Y, g.pose.position.x, g.pose.position.y);
        }
        return r;
    });
    start();
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }, WAIT_S));
    EXPECT_GE(fake_->goalCount(), 2u);
}

// 11. While a replan is outstanding, a blockage farther than stop_distance does not stop the robot;
// once the robot is within stop_distance of it, the stop plan is published.
TEST_F(NavigatorTest, farBlockageStopsOnceTheRobotGetsNear) {
    const double stop_distance =
        loadControllerParams(shippedConfig("controller", "controller.yaml")).v_cruise *
        (loadAStarConfig(shippedConfig("planner", "planner.yaml")).max_plan_time_sec + TICK);   // 2.55 m
    constexpr double FAR_GOAL_X = 9.5;   // 4.47 m path along +x
    // Wall x in [8.4, 8.6): footprint (front 0.30 m ahead of the pose) first touches it with the
    // pose at x = 8.1, i.e. 3.07 m from the robot, beyond stop_distance.
    ASSERT_GT(8.4 - 0.30 - ROBOT_X, stop_distance);
    ASSERT_LT(8.4 - 0.30 - (ROBOT_X + 1.0), stop_distance);

    script(foundStraight());
    start();
    probe_->publishGoal(FAR_GOAL_X, ROBOT_Y);
    ASSERT_TRUE(waitFor([&] { return !probe_->realPlans().empty(); }));

    fake_->configure(foundStraight(), 20.0);   // the next request stays unanswered
    provider_->set(makeSnapshot(0, [](Cells & c) { fillRect(c, 168, 171, 0, N - 1, 100); }));
    ASSERT_TRUE(waitFor([&] { return fake_->goalCount() >= 2; }));
    waitTicks(5);
    EXPECT_EQ(probe_->planCount(), 1u);   // no stop plan yet

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = probe_->now();
    t.header.frame_id         = "map";
    t.child_frame_id          = "base_link";
    t.transform.translation.x = ROBOT_X + 1.0;
    t.transform.translation.y = ROBOT_Y;
    t.transform.rotation.w    = 1.0;
    tf_->sendTransform(t);
    waitTicks(10);
    ASSERT_GE(probe_->planCount(), 2u);
    EXPECT_EQ(probe_->plans()[1].plan_id, 0u);
    EXPECT_TRUE(probe_->plans()[1].path.poses.empty());
}

// 12. Navigator on simulated time; the clock steps back by 300 s while a blocked goal waits for its
// retry. The retry is due blocked_retry_period (R = 0.5 s) after the first request, measured on the
// forward-only event clock, so it must come about R of forward sim time after the step, not ~300 s.
//
// The clock is held still until request 1 and the stop plan have arrived, so T1 (the sim time at
// the request) is the navigator's own request stamp. The forward clock adds time only between
// consecutive stamps, so it loses the part of the tick interval that straddles the step. The retry
// fires on the first tick where forward time since the request is at least R. Forward time before
// the step is exactly R/2, so after the step the retry needs at least R/2 more forward time; it
// arrives at most one lost interval (<= P) plus one tick (P) later, plus delivery latency.
//   Lower bound: forward time since T1 at detection (forward_before + A) >= R, because the
//     navigator's forward time is at most the test's and must reach R before the retry.
//   Upper bound: A <= R + 2P + 0.5 s.
// The test publishes on the absolute topic /clock, so a simulator or another sim-time node on the
// same ROS domain would receive these values; do not run it alongside one (the suite runs serially).
struct NavigatorSimTime : NavigatorTest {
    NavigatorSimTime() { sim_time_ = true; }
};

TEST_F(NavigatorSimTime, backwardClockStepDoesNotDelayRetry) {
    constexpr double WALL_S = 20.0;
    constexpr int64_t NS = 1'000'000'000;
    const int64_t R = std::llround(0.5 * NS), P = std::llround(TICK * NS);
    constexpr int64_t DT = 50'000'000, JUMP = 300 * NS;
    script(fixed(PlanPath::Result::NO_PATH));
    start();

    auto clock_pub = probe_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    int64_t sim = 1000 * NS;
    // Publishes one clock sample, then lets the executor process it (sim runs about 5x wall).
    auto publish = [&](int64_t t) {
        sim = t;
        rosgraph_msgs::msg::Clock c;
        c.clock.sec     = static_cast<int32_t>(t / NS);
        c.clock.nanosec = static_cast<uint32_t>(t % NS);
        clock_pub->publish(c);
        std::this_thread::sleep_for(10ms);
    };
    auto expired = [](std::chrono::steady_clock::time_point end) { return std::chrono::steady_clock::now() > end; };
    const auto wall_end = std::chrono::steady_clock::now() + std::chrono::seconds(static_cast<int>(WALL_S));

    // Stage 1: navigator picks up the clock.
    while (nav_->now().seconds() < 999.0) {
        ASSERT_FALSE(expired(wall_end)) << "stage 1 timed out: navigator never took /clock";
        publish(sim);
    }
    probe_->publishGoal(GOAL_X, ROBOT_Y);

    // Stage 2: clock held until the first request and the stop plan have arrived.
    while (fake_->goalCount() < 1 || probe_->planCount() < 1) {
        ASSERT_FALSE(expired(wall_end)) << "stage 2 timed out: no first plan request or stop plan";
        publish(sim);
    }
    const int64_t T1 = sim;

    // Stage 3: forward to T1 + R/2 (below R), then step back by JUMP.
    while (sim - T1 < R / 2) {
        ASSERT_FALSE(expired(wall_end)) << "stage 3 timed out";
        publish(sim + DT);
    }
    ASSERT_EQ(fake_->goalCount(), 1u) << "retry fired before blocked_retry_period";
    const int64_t forward_before = sim - T1;
    publish(sim - JUMP);
    const int64_t t_after = sim;

    // Stage 4: advance after the step until the second request.
    const int64_t limit = R + 2 * P + NS / 2;
    while (fake_->goalCount() < 2) {
        ASSERT_FALSE(expired(wall_end)) << "stage 4 timed out (wall): no second plan request";
        ASSERT_LE(sim - t_after, limit)
            << "no second request within " << static_cast<double>(limit) / NS
            << " s of sim time after the backward step";
        publish(sim + DT);
    }
    const int64_t A = sim - t_after;
    std::cout << "[ measured ] second request " << static_cast<double>(A) / NS << " s sim after the step ("
              << static_cast<double>(A) / P << " ticks of P = " << static_cast<double>(P) / NS
              << " s); forward sim since T1 = " << static_cast<double>(forward_before + A) / NS << "\n";
    EXPECT_LE(A, limit);
    EXPECT_GE(forward_before + A, R) << "retry fired too early";
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
