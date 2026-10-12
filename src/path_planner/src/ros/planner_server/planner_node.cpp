/**
 * @file planner_node.cpp
 * @author Seif ElShafey
 * @brief Implementation of the hybrid A* ROS2 lifecycle node skeleton.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ros/planner_server/planner_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/config/config_loader.hpp"
#include "core/diagnostics/check.hpp"
#include "core/planner/hybrid_astar/build_hybrid_astar.hpp"
#include "ros/planner_server/plan_conversions.hpp"

using namespace std::placeholders;

PlannerNode::PlannerNode(const rclcpp::NodeOptions & opts)
    : LifecycleNode("planner_node", opts) {
    // Declared once here (not in on_configure) so configure -> cleanup -> configure is
    // idempotent: rclcpp throws if the same parameter is declared twice, and on_cleanup
    // does not undeclare. Parameters persist for the node's lifetime; on_configure only
    // reads them via get_parameter.
    declare_parameter<std::string>("robot_params_path", "");
    declare_parameter<std::string>("diff_drive_params_path", "");
    declare_parameter<std::string>("astar_config_path", "");
    declare_parameter<std::string>("heuristic_config_path", "");
    declare_parameter<std::string>("cost_policy_path", "");
    declare_parameter<std::string>("planning_frame", "map");
    declare_parameter<std::string>("robot_frame", "base_link");
}

PlannerNode::CallbackReturn PlannerNode::on_configure(const State &) {
    const auto robot_path = get_parameter("robot_params_path").as_string();
    const auto dd_path    = get_parameter("diff_drive_params_path").as_string();
    const auto astar_path = get_parameter("astar_config_path").as_string();
    const auto heur_path  = get_parameter("heuristic_config_path").as_string();
    const auto cost_path  = get_parameter("cost_policy_path").as_string();

    robot_params_ = loadRobotParams(robot_path);
    kin_params_   = loadDiffDriveParams(dd_path);
    cfg_          = loadAStarConfig(astar_path);
    heur_cfg_     = loadHeuristicConfig(heur_path);
    cost_policy_  = loadCostPolicy(cost_path);

    planning_frame_ = get_parameter("planning_frame").as_string();
    robot_frame_    = get_parameter("robot_frame").as_string();

    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn PlannerNode::on_activate(const State &) {
    CHECK(map_provider_ != nullptr,
          "PlannerNode: map provider must be bound before activation (call bindMapProvider)");

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("search_markers", 10);
    markers_pub_->on_activate();

    // Node-clock timer (not create_wall_timer) so the viz loop honors use_sim_time: system clock
    // under mock/CI, /clock under sim.
    viz_timer_ = rclcpp::create_timer(this, get_clock(),
                                      rclcpp::Duration::from_seconds(0.066),
                                      std::bind(&PlannerNode::publishProgress, this));

    // Builds the motion model as soon as the first map exists. Default (mutually exclusive)
    // callback group, same as handleAccepted, so the two never run concurrently.
    warmup_timer_ = rclcpp::create_timer(this, get_clock(),
                                         rclcpp::Duration::from_seconds(0.2),
                                         std::bind(&PlannerNode::warmUp, this));

    action_server_ = rclcpp_action::create_server<PlanPath>(
        get_node_base_interface(), get_node_clock_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "plan_path",
        std::bind(&PlannerNode::handleGoal,     this, _1, _2),
        std::bind(&PlannerNode::handleCancel,   this, _1),
        std::bind(&PlannerNode::handleAccepted, this, _1));

    bond_ = std::make_unique<bond::Bond>(
        "bond", get_name(),
        get_node_base_interface(), get_node_logging_interface(),
        get_node_parameters_interface(), get_node_timers_interface(),
        get_node_topics_interface());
    bond_->setHeartbeatPeriod(1.0);   // nav2_util default (DEFAULT_HEARTBEAT_PERIOD)
    bond_->setHeartbeatTimeout(4.0);  // nav2_util default (DEFAULT_HEARTBEAT_TIMEOUT)
    bond_->start();

    return CallbackReturn::SUCCESS;
}

void PlannerNode::bindMapProvider(std::shared_ptr<IMapProvider> provider) {
    map_provider_ = std::move(provider);
}

PlannerNode::CallbackReturn PlannerNode::on_deactivate(const State &) {
    if (bond_) { bond_->breakBond(); bond_.reset(); }
    if (worker_.joinable()) { stop_source_.request_stop(); worker_.join(); }
    action_server_.reset();
    warmup_timer_.reset();
    viz_timer_.reset();
    markers_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    planner_.reset();
    last_snap_.reset();
    observer_.store(nullptr);
    active_goal_.store(nullptr);
    published_markers_ = 0;
    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn PlannerNode::on_cleanup(const State &) {
    if (bond_) { bond_->breakBond(); bond_.reset(); }
    if (worker_.joinable()) { stop_source_.request_stop(); worker_.join(); }
    action_server_.reset();
    warmup_timer_.reset();
    viz_timer_.reset();
    markers_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    planner_.reset();
    last_snap_.reset();
    observer_.store(nullptr);
    active_goal_.store(nullptr);
    published_markers_ = 0;
    kin_model_built_ = false;
    return CallbackReturn::SUCCESS;
}

void PlannerNode::refreshPlannerIfStale() {
    if (!map_provider_) return;
    auto current = map_provider_->snapshot();
    if (!current) return;

    const double snap_res = current->grid_.getMetadata().resolution;

    if (!kin_model_built_) {
        kin_model_ = buildKinematicModel(robot_params_, kin_params_, snap_res);
        kin_model_built_ = true;
    } else if (snap_res != kin_model_.motion->cellSize()) {
        RCLCPP_ERROR(get_logger(),
                     "Snapshot resolution %.4f m disagrees with kinematic model resolution "
                     "%.4f m; rejecting snapshot.",
                     snap_res, kin_model_.motion->cellSize());
        return;
    }

    if (current != last_snap_) {
        last_snap_ = current;
        planner_   = buildHybridAstar(kin_model_, cfg_, heur_cfg_, cost_policy_, last_snap_);
    }
}

void PlannerNode::warmUp() {
    if (kin_model_built_) { warmup_timer_->cancel(); return; }
    // A worker that has not been joined may still read planner_; leave the build to the next
    // plan request (handleAccepted joins first).
    if (worker_.joinable()) return;
    if (!map_provider_ || !map_provider_->snapshot()) return;
    const auto t0 = std::chrono::steady_clock::now();
    refreshPlannerIfStale();
    if (!kin_model_built_) return;
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(get_logger(), "motion model built in %.0f ms on the first map", ms);
    warmup_timer_->cancel();
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse
PlannerNode::handleGoal(const rclcpp_action::GoalUUID &,
                        std::shared_ptr<const PlanPath::Goal> goal) {
    if (goal->goal.header.frame_id != planning_frame_) {
        RCLCPP_WARN(get_logger(), "goal rejected: frame '%s' is not the planning frame '%s'",
                    goal->goal.header.frame_id.c_str(), planning_frame_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
PlannerNode::handleCancel(const std::shared_ptr<GoalHandle>) {
    // Mark the stop REASON as a client cancel (vs a supersede/preempt) before flipping the token, so
    // the worker resolves this goal with canceled() rather than abort(). Returning ACCEPT here is
    // what drives the handle's EXECUTING -> CANCELING transition inside rclcpp_action; the worker
    // gates on is_canceling() so it never calls canceled() before that transition lands.
    cancel_requested_.store(true);
    stop_source_.request_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    // Single active goal: preempt any running search before starting the new one. This is a
    // supersede, NOT a client cancel, so cancel_requested_ stays false and the preempted worker (if
    // any) resolves its handle with abort() (valid EXECUTING -> ABORTED), not canceled().
    if (worker_.joinable()) { stop_source_.request_stop(); worker_.join(); }
    stop_source_ = std::stop_source{};
    // Fresh token for the new worker: clear the cancel reason so a stop during this goal is only
    // treated as a cancel if handleCancel sets it again. Written here on the executor thread after
    // the prior worker is joined, so the new worker sees a settled value.
    cancel_requested_.store(false);
    // Refresh the planner here, on the executor thread, so that kin_model_/planner_/last_snap_
    // are only written by executor callbacks (handleAccepted) — never by the
    // worker thread. The prior worker has already been joined above, so planner_ is stable for
    // the lifetime of the new worker (next handleAccepted joins before touching it again).
    refreshPlannerIfStale();
    worker_ = std::jthread([this, goal_handle]() { execute(goal_handle); });
}

void PlannerNode::execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<PlanPath::Result>();
    // Aborts before the search runs (no TF, no map) report STOPPED: no plan was attempted.
    result->outcome = PlanPath::Result::STOPPED;

    // Capture the stop token once: both the plan() call and the post-plan stop_requested() check
    // must use the same token snapshot so they agree on cancellation state.
    const std::stop_token token = stop_source_.get_token();

    // 1) Resolve the start pose from TF (planning_frame_ -> robot_frame_).
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(planning_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "TF lookup %s->%s failed: %s",
                     planning_frame_.c_str(), robot_frame_.c_str(), e.what());
        goal_handle->abort(result);
        return;
    }
    const SE2State start{tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf2::getYaw(tf.transform.rotation)};

    // 2) Goal pose from the action goal (frame checked in handleGoal).
    // The search budget is the planner's AStarConfig.max_plan_time_sec.
    const auto & gp = goal_handle->get_goal()->goal.pose;
    const SE2State goal_state{gp.position.x, gp.position.y, tf2::getYaw(gp.orientation)};

    // 3) planner_ was set by handleAccepted (executor thread) before this worker was spawned;
    //    it is stable for the lifetime of this worker (next handleAccepted joins before mutating).
    if (!planner_) {
        RCLCPP_ERROR(get_logger(), "PlannerNode: no map snapshot available");
        goal_handle->abort(result);
        return;
    }

    // Set up the live observer and active goal handle before calling plan().
    // Both are stored as atomic<shared_ptr> so the executor-thread timer can read them safely.
    auto obs = std::make_shared<SearchMarkerObserver>(
        static_cast<std::size_t>(cfg_.max_expansions));
    observer_.store(obs);
    active_goal_.store(goal_handle);

    const PlanResult plan_result = planner_->plan(start, goal_state, obs, token);
    const std::optional<Path> & path = plan_result.path;
    switch (plan_result.outcome) {
        case PlanOutcome::Found:           result->outcome = PlanPath::Result::FOUND; break;
        case PlanOutcome::StartBlocked:    result->outcome = PlanPath::Result::START_BLOCKED; break;
        case PlanOutcome::GoalBlocked:     result->outcome = PlanPath::Result::GOAL_BLOCKED; break;
        case PlanOutcome::NoPath:          result->outcome = PlanPath::Result::NO_PATH; break;
        case PlanOutcome::BudgetExhausted: result->outcome = PlanPath::Result::BUDGET_EXHAUSTED; break;
        case PlanOutcome::Stopped:         result->outcome = PlanPath::Result::STOPPED; break;
    }

    // Clear observer/goal before resolving — publishProgress will see null and stop early.
    observer_.store(nullptr);
    active_goal_.store(nullptr);

    // 4) Resolve the goal handle. If the plan was stopped, resolve by the REASON the token flipped,
    //    from the handle's ACTUAL state — never race the token against rclcpp_action's goal state
    //    machine (which only permits CANCELED from CANCELING; EXECUTING -> CANCELED aborts).
    if (token.stop_requested()) {
        result->outcome = PlanPath::Result::STOPPED;
        if (cancel_requested_.load()) {
            // Client cancel: handleCancel returned ACCEPT, so the handle's
            // EXECUTING -> CANCELING transition is imminent (driven by rclcpp_action). Wait for it
            // to land before finalizing; canceled() is only legal once is_canceling() is true.
            constexpr auto poll = std::chrono::milliseconds(2);
            constexpr auto budget = std::chrono::seconds(2);
            const auto deadline = std::chrono::steady_clock::now() + budget;
            while (!goal_handle->is_canceling() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(poll);
            }
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
            } else {
                // Transition never landed (should not happen after an ACCEPTed cancel): fall back to
                // the always-legal EXECUTING -> ABORTED rather than risk an illegal CANCELED.
                RCLCPP_WARN(get_logger(),
                            "cancel accepted but goal never entered CANCELING; aborting instead");
                goal_handle->abort(result);
            }
        } else {
            // Preempt/supersede (a new goal arrived): the goal is being superseded, not canceled.
            // EXECUTING -> ABORTED is the valid terminal transition here.
            goal_handle->abort(result);
        }
        return;
    }
    if (path.has_value()) {
        std_msgs::msg::Header header;
        header.stamp    = now();
        header.frame_id = planning_frame_;
        result->trajectory = pathToTrajectory(*path, header);
        result->expansions = static_cast<uint32_t>(obs->writeCount());
        result->cost       = path->total_cost;
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }
}

void PlannerNode::publishProgress() {
    // Load atomic handles — safe cross-thread read (worker thread writes via .store()).
    auto obs = observer_.load();
    auto gh  = active_goal_.load();
    if (!obs || !gh) { published_markers_ = 0; return; }

    const std::size_t count = obs->writeCount();

    // Incremental markers: one POINTS marker holding only the new expansions since last publish.
    // Each slice gets a distinct id so RViz ADD calls accumulate across timer firings.
    if (count > published_markers_) {
        visualization_msgs::msg::Marker pts;
        pts.header.frame_id = planning_frame_;
        pts.header.stamp    = now();
        pts.ns              = "search";
        pts.id              = static_cast<int>(published_markers_);
        pts.type            = visualization_msgs::msg::Marker::POINTS;
        pts.action          = visualization_msgs::msg::Marker::ADD;
        pts.scale.x = 0.03; pts.scale.y = 0.03;
        pts.color.g = 1.0;  pts.color.a = 1.0;

        for (std::size_t i = published_markers_; i < count; ++i) {
            geometry_msgs::msg::Point p;
            p.x = obs->at(i).state.x;
            p.y = obs->at(i).state.y;
            pts.points.push_back(p);
        }

        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(pts);
        markers_pub_->publish(arr);
        published_markers_ = count;
    }

    // Throttled feedback (scalar progress) — only publish once at least one expansion exists.
    if (count > 0) {
        auto fb        = std::make_shared<PlanPath::Feedback>();
        fb->expansions = static_cast<uint32_t>(count);
        fb->best_f     = obs->bestF();
        gh->publish_feedback(fb);
    }
}
