#include "ros/navigator/navigator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <variant>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "core/control/pure_pursuit.hpp"
#include "core/diagnostics/check.hpp"
#include "core/planner/escape/escape_search.hpp"
#include "ros/planner_server/plan_conversions.hpp"

using namespace std::placeholders;
using path_planner_msgs::msg::PlanProgress;
using path_planner_msgs::msg::PlanTrajectory;

namespace {

const char * stateName(NavState s) {
    switch (s) {
        case NavState::Idle:      return "Idle";
        case NavState::Planning:  return "Planning";
        case NavState::Following: return "Following";
        case NavState::Clearing:  return "Clearing";
        case NavState::Blocked:   return "Blocked";
    }
    return "?";
}

const char * reasonName(BlockedReason r) {
    switch (r) {
        case BlockedReason::None:         return "None";
        case BlockedReason::NoPathToGoal: return "NoPathToGoal";
        case BlockedReason::NoEscape:     return "NoEscape";
        case BlockedReason::Stalled:      return "Stalled";
    }
    return "?";
}

PlanOutcome toOutcome(uint8_t code) {
    using R = Navigator::PlanPath::Result;
    switch (code) {
        case R::FOUND:            return PlanOutcome::Found;
        case R::START_BLOCKED:    return PlanOutcome::StartBlocked;
        case R::GOAL_BLOCKED:     return PlanOutcome::GoalBlocked;
        case R::NO_PATH:          return PlanOutcome::NoPath;
        case R::BUDGET_EXHAUSTED: return PlanOutcome::BudgetExhausted;
        default:                  return PlanOutcome::Stopped;
    }
}

std::string requiredPath(rclcpp::Node & node, const char * name) {
    node.declare_parameter<std::string>(name, "");
    const auto value = node.get_parameter(name).as_string();
    CHECK(!value.empty(), std::string("Navigator: parameter '") + name + "' must be a non-empty path");
    return value;
}

}  // namespace

Navigator::Navigator(const rclcpp::NodeOptions & opts) : rclcpp::Node("navigator", opts) {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "map");
    robot_frame_    = declare_parameter<std::string>("robot_frame", "base_link");

    robot_params_ = loadRobotParams(requiredPath(*this, "robot_params_path"));
    kin_params_   = loadDiffDriveParams(requiredPath(*this, "diff_drive_params_path"));
    const AStarConfig      astar = loadAStarConfig(requiredPath(*this, "astar_config_path"));
    const ControllerParams cp    = loadControllerParams(requiredPath(*this, "controller_params_path"));
    nav_params_ = loadNavigatorParams(requiredPath(*this, "navigator_params_path"));

    CHECK(cp.v_cruise <= kin_params_.vmax,
          "Navigator: controller v_cruise (" + std::to_string(cp.v_cruise) +
          " m/s) must not exceed diff-drive vmax (" + std::to_string(kin_params_.vmax) +
          " m/s), the speed the planner's motions are built at");

    stop_short_ = cp.goal_pos_tol;

    // Time the cursor can stand still during the final approach: the longer of creeping the last
    // lookahead into the tolerance and crossing the last full sample interval.
    // A stall must not be declared before the follower has crept the last lookahead L onto the goal
    // at speed fraction s and, at full speed scale, turned up to half a turn onto the goal heading.
    const double spacing = robot_params_.r_in / 2.0;
    const double L       = cp.lookahead_gain * spacing;
    const auto crawl = [&](double s) {
        return (L / (s * cp.v_cruise)) * std::max({std::log(L / cp.goal_pos_tol), spacing / L, std::log(3.0)});
    };
    {
        const double c   = crawl(1.0);
        const double t   = PurePursuit::goalAlignTimeBound(cp.omega_pivot, cp.goal_heading_tol);
        CHECK(nav_params_.stall_timeout > c + t,
              "Navigator: stall_timeout (" + std::to_string(nav_params_.stall_timeout) +
              " s) must exceed the follower's final approach time plus its final turn (" +
              std::to_string(c + t) + " s = " + std::to_string(c) + " + " + std::to_string(t) +
              ") at speed scale 1");
    }
    {
        const double s = nav_params_.escape_speed_scale;
        const double c = crawl(s);
        CHECK(nav_params_.stall_timeout > c,
              "Navigator: stall_timeout (" + std::to_string(nav_params_.stall_timeout) +
              " s) must exceed the follower's final approach time (" + std::to_string(c) +
              " s) at speed scale " + std::to_string(s));
    }

    // Distance covered while a replan runs plus one tick of travel between two path checks; a
    // blocked path nearer than this is stopped at once.
    const double stop_distance = cp.v_cruise * (astar.max_plan_time_sec + nav_params_.tick_period_sec);
    NavigationConfig nc{};
    nc.stop_distance         = stop_distance;
    nc.stall_timeout         = nav_params_.stall_timeout;
    nc.blocked_retry_period  = nav_params_.blocked_retry_period;
    nc.max_clearing_attempts = nav_params_.max_clearing_attempts;
    nc.max_stall_replans     = nav_params_.max_stall_replans;
    // The planner's search budget plus the no-progress tolerance.
    nc.plan_reply_timeout    = astar.max_plan_time_sec + nav_params_.stall_timeout;
    mode_.emplace(nc);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = rclcpp_action::create_client<PlanPath>(this, "plan_path", cb_group_);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&Navigator::onGoalPose, this, _1), sub_opts);
    progress_sub_ = create_subscription<PlanProgress>(
        "plan_progress", 10, std::bind(&Navigator::onProgress, this, _1), sub_opts);

    plan_pub_      = create_publisher<PlanTrajectory>("plan", rclcpp::QoS(1).transient_local());
    plan_path_pub_ = create_publisher<nav_msgs::msg::Path>("plan_path", rclcpp::QoS(1).transient_local());

    // Node-clock timer (not create_wall_timer) so the loop honors use_sim_time.
    timer_ = rclcpp::create_timer(this, get_clock(),
                                  rclcpp::Duration::from_seconds(nav_params_.tick_period_sec),
                                  std::bind(&Navigator::onTick, this), cb_group_);
}

void Navigator::bindMapProvider(std::shared_ptr<IMapProvider> provider) {
    map_provider_ = std::move(provider);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool Navigator::ensureKinModel(const MapSnapshot & snap) {
    const double h = snap.grid_.getMetadata().resolution;
    if (!kin_model_.motion) {
        kin_model_ = buildKinematicModel(robot_params_, kin_params_, h);
        const double step = nav_params_.escape.step_fraction * h;
        rest_pos_tol_ = step / 4.0;
        rest_ang_tol_ = step / (4.0 * robot_params_.r_out);
        return true;
    }
    if (h != kin_model_.motion->cellSize()) {
        RCLCPP_ERROR(get_logger(),
                     "Snapshot resolution %.4f m disagrees with kinematic model resolution "
                     "%.4f m; ignoring snapshot.",
                     h, kin_model_.motion->cellSize());
        return false;
    }
    return true;
}

std::optional<SE2State> Navigator::robotPose() const {
    try {
        const auto tf = tf_buffer_->lookupTransform(planning_frame_, robot_frame_, tf2::TimePointZero);
        return SE2State{tf.transform.translation.x, tf.transform.translation.y,
                        tf2::getYaw(tf.transform.rotation)};
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

CollisionChecker Navigator::makeChecker(const std::shared_ptr<const MapSnapshot> & snap) const {
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(snap, &snap->edt_);
    return CollisionChecker(grid_ptr, edt_ptr, kin_model_.motion->body());
}

RemainingPathCheck Navigator::checkDriven(const std::shared_ptr<const MapSnapshot> & snap,
                                          const SE2State & pose) {
    const CollisionChecker checker = makeChecker(snap);
    const RemainingPathCheck r =
        checkRemainingPath(driven_samples_, pose.x, pose.y, cursor_, checker);
    cursor_ = r.cursor;
    return r;
}

void Navigator::publishTrajectory(const PlanTrajectory & traj) {
    plan_pub_->publish(traj);
    plan_path_pub_->publish(traj.path);
}

// ---------------------------------------------------------------------------
// Event sources
// ---------------------------------------------------------------------------

void Navigator::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg->header.frame_id != planning_frame_) {
        RCLCPP_WARN(get_logger(), "goal ignored: frame '%s' is not the planning frame '%s'",
                    msg->header.frame_id.c_str(), planning_frame_.c_str());
        return;
    }
    goal_ = *msg;
    if (!kin_model_.motion) {
        RCLCPP_INFO(get_logger(), "goal held until the first map arrives");
        return;
    }
    dispatch(GoalReceived{stamp()});
}

void Navigator::onProgress(const PlanProgress::SharedPtr msg) {
    last_progress_ = *msg;
    if (msg->plan_id != 0 && msg->plan_id == mode_->drivenPlanId()) {
        dispatch(Progress{stamp(), msg->plan_id, msg->cursor, msg->arrived});
    }
}

void Navigator::onTick() {
    // 1) New map: re-check the driven path.
    const auto snap = map_provider_ ? map_provider_->snapshot() : nullptr;

    // The mode stays Idle until the motion model exists. The model build can take seconds, and the
    // stall and escape timers must not span it, so the held goal is dispatched only after it.
    if (!kin_model_.motion) {
        if (!snap) return;
        if (!ensureKinModel(*snap)) return;
        if (goal_) dispatch(GoalReceived{stamp()});
    }

    if (snap && snap != last_snap_ && ensureKinModel(*snap)) {
        if (driven_samples_.empty()) {
            last_snap_ = snap;
            dispatch(MapChanged{stamp(), true, 0.0});
        } else if (const auto pose = robotPose()) {
            const RemainingPathCheck r = checkDriven(snap, *pose);
            last_snap_ = snap;
            if (!r.valid) {
                RCLCPP_INFO(get_logger(), "driven path blocked %.2f m ahead by a map update",
                            r.blocked_distance);
            }
            dispatch(MapChanged{stamp(), r.valid, r.blocked_distance});
        } else {
            // last_snap_ stays unchanged: the check is retried on the next tick.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s->%s unavailable; path check deferred",
                                 planning_frame_.c_str(), robot_frame_.c_str());
        }
    }

    // While a replan runs, keep re-checking the driven path as the robot advances.
    else if (last_snap_ && mode_->state() == NavState::Planning && mode_->drivenPlanId() != 0 &&
             !driven_samples_.empty()) {
        if (const auto pose = robotPose()) {
            const RemainingPathCheck r = checkDriven(last_snap_, *pose);
            if (!r.valid) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "driven path blocked %.2f m ahead while replanning", r.blocked_distance);
                dispatch(MapChanged{stamp(), false, r.blocked_distance});
            }
        }
    }

    // 2) Time signal for the stall and retry rules.
    const double t = stamp();
    dispatch(Tick{t});

    // 3) Escape wait.
    checkEscapeWait(t);
}

void Navigator::checkEscapeWait(double t) {
    if (!escape_active_) return;
    if (mode_->state() != NavState::Clearing) { escape_active_ = false; return; }   // superseded by a new goal

    if (t - escape_start_ >= nav_params_.stall_timeout) {
        escape_active_ = false;
        dispatch(EscapeResult{t, false});
        return;
    }
    const auto pose = robotPose();
    if (!pose) return;   // keep waiting
    const bool follower_idle = !last_progress_ || last_progress_->plan_id == 0;
    if (follower_idle && escape_prev_ && atRest(*escape_prev_, *pose, rest_pos_tol_, rest_ang_tol_)) {
        escape_active_ = false;
        runEscape(*pose, t);
    } else {
        escape_prev_ = pose;
    }
}

void Navigator::runEscape(const SE2State & pose, double t) {
    if (!last_snap_ || !kin_model_.motion) {
        dispatch(EscapeResult{t, false});
        return;
    }
    const CollisionChecker checker = makeChecker(last_snap_);
    const auto p = findEscape(pose, checker, *kin_model_.motion, kin_params_, nav_params_.escape, stop_short_);
    if (!p) {
        dispatch(EscapeResult{t, false});
        return;
    }
    std_msgs::msg::Header header;
    header.stamp    = now();
    header.frame_id = planning_frame_;
    escape_traj_ = pathToTrajectory(*p, header);
    dispatch(EscapeResult{t, true});
}

// ---------------------------------------------------------------------------
// Event loop and actions
// ---------------------------------------------------------------------------

void Navigator::dispatch(NavEvent ev) {
    queue_.push_back(std::move(ev));
    if (draining_) return;   // the running loop picks it up; step() is never re-entered
    draining_ = true;
    struct Reset { bool & f; ~Reset() { f = false; } } reset{draining_};
    while (!queue_.empty()) {
        const NavEvent e = std::move(queue_.front());
        queue_.pop_front();
        const bool was_following = mode_->state() == NavState::Following;
        const bool is_tick       = std::holds_alternative<Tick>(e);
        for (const NavAction & a : mode_->step(e)) {
            // A Tick while following that asks for a plan can only be the stall rule.
            if (was_following && is_tick && std::holds_alternative<RequestPlan>(a)) {
                RCLCPP_INFO(get_logger(), "no progress along the path for %.1f s; replanning",
                            nav_params_.stall_timeout);
            }
            std::visit([this](const auto & x) { act(x); }, a);
        }
        if (mode_->drivenPlanId() == 0) driven_samples_.clear();   // nothing is being driven
    }
}

void Navigator::act(const RequestPlan & a) {
    CHECK(kin_model_.motion != nullptr, "Navigator: plan requested before the motion model exists");
    const uint32_t id = a.request_id;
    if (!goal_ || !client_->action_server_is_ready()) {
        RCLCPP_WARN(get_logger(), "plan request %u not sent: %s", id,
                    goal_ ? "plan_path action server not ready" : "no goal");
        dispatch(PlanReply{stamp(), id, PlanOutcome::BudgetExhausted});   // "no plan now"; retried every blocked_retry_period
        return;
    }

    PlanPath::Goal goal;
    goal.goal = *goal_;

    rclcpp_action::Client<PlanPath>::SendGoalOptions opts;
    opts.goal_response_callback = [this, id](GoalHandle::SharedPtr handle) {
        if (!handle) {
            dispatch(PlanReply{stamp(), id, PlanOutcome::BudgetExhausted});
        } else if (id == mode_->pendingRequestId()) {
            request_handle_ = handle;
        }
    };
    opts.result_callback = [this, id](const GoalHandle::WrappedResult & result) {
        if (request_handle_ && request_handle_->get_goal_id() == result.goal_id) request_handle_.reset();
        // The planner aborts with STOPPED when it could not attempt a search (no TF or no map): "no
        // plan now". A canceled goal is STOPPED for real. A superseded goal also arrives as
        // ABORTED + STOPPED but carries a stale request id, which the mode ignores.
        PlanOutcome outcome =
            result.result ? toOutcome(result.result->outcome) : PlanOutcome::Stopped;
        if (outcome == PlanOutcome::Stopped && result.code == rclcpp_action::ResultCode::ABORTED) {
            outcome = PlanOutcome::BudgetExhausted;
        }
        if (outcome == PlanOutcome::Found && id == mode_->pendingRequestId()) {
            // Do not adopt a path the newest map already blocks.
            std::vector<SE2State> samples;
            for (const auto & ps : result.result->trajectory.path.poses) {
                samples.push_back(SE2State{ps.pose.position.x, ps.pose.position.y,
                                           tf2::getYaw(ps.pose.orientation)});
            }
            const auto snap = map_provider_ ? map_provider_->snapshot() : nullptr;
            bool rejected = false;
            if (snap && !samples.empty() && ensureKinModel(*snap)) {
                const RemainingPathCheck r =
                    checkRemainingPath(samples, samples.front().x, samples.front().y, 0, makeChecker(snap));
                if (!r.valid) {
                    RCLCPP_WARN(get_logger(), "planned path rejected: blocked %.2f m along the path on the newest map",
                                r.blocked_distance);
                    rejected = true;
                }
            }
            if (rejected) outcome = PlanOutcome::BudgetExhausted;
            else pending_goal_traj_ = result.result->trajectory;
        }
        dispatch(PlanReply{stamp(), id, outcome});
    };
    client_->async_send_goal(goal, opts);
}

void Navigator::act(const CancelPlan &) {
    // No handle yet: the planner node supersedes a running search on the next goal, and the mode
    // ignores a reply with a stale request id.
    if (request_handle_) {
        client_->async_cancel_goal(request_handle_);
        request_handle_.reset();
    }
}

void Navigator::act(const RunEscape &) {
    escape_active_ = true;
    escape_start_  = stamp();
    escape_prev_.reset();
}

void Navigator::act(const PublishPlan & a) {
    PlanTrajectory traj;
    if (a.kind == PlanKind::Goal) {
        traj = std::move(pending_goal_traj_);
        pending_goal_traj_ = PlanTrajectory{};
        traj.speed_scale = 1.0;
    } else {
        traj = std::move(escape_traj_);
        escape_traj_ = PlanTrajectory{};
        traj.speed_scale = nav_params_.escape_speed_scale;
    }
    traj.plan_id = a.plan_id;
    publishTrajectory(traj);

    driven_samples_.clear();
    cursor_ = 0;

    // A plan with no poses (a zero-length plan, or an escape from a pose that already has clearance)
    // is complete at once: the follower has nothing to report progress on, so report arrival here.
    if (traj.path.poses.empty()) {
        dispatch(Progress{stamp(), a.plan_id, 0, true});
        return;
    }
    if (a.kind == PlanKind::Escape) return;   // the escape path is not path-checked

    for (const auto & ps : traj.path.poses) {
        driven_samples_.push_back(SE2State{ps.pose.position.x, ps.pose.position.y,
                                           tf2::getYaw(ps.pose.orientation)});
    }
}

void Navigator::act(const PublishStop &) {
    PlanTrajectory empty;
    empty.plan_id              = 0;
    empty.speed_scale          = 1.0;
    empty.header.frame_id      = planning_frame_;
    empty.header.stamp         = now();
    empty.path.header          = empty.header;
    publishTrajectory(empty);
    driven_samples_.clear();
}

void Navigator::act(const Report & a) {
    RCLCPP_INFO(get_logger(), "navigation state %s, reason %s", stateName(a.state), reasonName(a.reason));
}
