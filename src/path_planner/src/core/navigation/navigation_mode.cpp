#include "core/navigation/navigation_mode.hpp"

#include "core/diagnostics/check.hpp"

NavigationMode::NavigationMode(const NavigationConfig& cfg) : cfg_(cfg) {
    CHECK(cfg.stop_distance > 0.0, "NavigationConfig.stop_distance must be > 0");
    CHECK(cfg.stall_timeout > 0.0, "NavigationConfig.stall_timeout must be > 0");
    CHECK(cfg.blocked_retry_period > 0.0, "NavigationConfig.blocked_retry_period must be > 0");
    CHECK(cfg.max_clearing_attempts >= 1, "NavigationConfig.max_clearing_attempts must be >= 1");
    CHECK(cfg.max_stall_replans >= 0, "NavigationConfig.max_stall_replans must be >= 0");
    CHECK(cfg.plan_reply_timeout > 0.0, "NavigationConfig.plan_reply_timeout must be > 0");
}

void NavigationMode::request(double stamp, std::vector<NavAction>& out) {
    pending_request_id_ = next_request_id_++;
    last_request_stamp_ = stamp;
    out.push_back(RequestPlan{pending_request_id_});
}

void NavigationMode::stop(std::vector<NavAction>& out) {
    out.push_back(PublishStop{});
    driven_plan_id_ = 0;
    driven_goal_seq_ = 0;
}

void NavigationMode::publish(PlanKind kind, double stamp, std::vector<NavAction>& out) {
    driven_plan_id_ = next_plan_id_++;
    last_cursor_ = 0;
    last_progress_stamp_ = stamp;
    out.push_back(PublishPlan{driven_plan_id_, kind});
}

void NavigationMode::go(NavState s, BlockedReason r, std::vector<NavAction>& out) {
    if (s == state_ && r == reason_) return;
    state_ = s;
    reason_ = r;
    out.push_back(Report{s, r});
}

bool NavigationMode::cursor(const Progress& p) {
    if (p.cursor == last_cursor_) return false;
    last_cursor_ = p.cursor;
    last_progress_stamp_ = p.stamp;
    return true;
}

std::vector<NavAction> NavigationMode::step(const NavEvent& ev) {
    std::vector<NavAction> out;
    out.reserve(4);

    if (const auto* e = std::get_if<GoalReceived>(&ev)) {
        has_goal_ = true;
        ++goal_seq_;
        clearing_attempts_ = 0;
        stall_replans_ = 0;
        if (pending_request_id_ != 0) out.push_back(CancelPlan{});
        request(e->stamp, out);
        go(NavState::Planning, BlockedReason::None, out);
        return out;
    }

    switch (state_) {
    case NavState::Idle:
        break;

    case NavState::Planning:
        if (const auto* e = std::get_if<PlanReply>(&ev)) {
            if (e->request_id != pending_request_id_) break;
            pending_request_id_ = 0;
            switch (e->outcome) {
            case PlanOutcome::Found:
                clearing_attempts_ = 0;
                publish(PlanKind::Goal, e->stamp, out);
                driven_goal_seq_ = goal_seq_;
                go(NavState::Following, BlockedReason::None, out);
                break;
            case PlanOutcome::StartBlocked:
                if (clearing_attempts_ >= cfg_.max_clearing_attempts) {
                    stop(out);
                    go(NavState::Blocked, BlockedReason::NoEscape, out);
                } else {
                    stop(out);
                    out.push_back(RunEscape{});
                    go(NavState::Clearing, BlockedReason::None, out);
                }
                break;
            case PlanOutcome::Stopped:
                stop(out);
                has_goal_ = false;
                go(NavState::Idle, BlockedReason::None, out);
                break;
            default:  // GoalBlocked, NoPath, BudgetExhausted
                stop(out);
                go(NavState::Blocked, BlockedReason::NoPathToGoal, out);
                break;
            }
        } else if (const auto* m = std::get_if<MapChanged>(&ev)) {
            if (!m->path_valid && driven_plan_id_ != 0 && m->blocked_distance < cfg_.stop_distance)
                stop(out);
        } else if (const auto* p = std::get_if<Progress>(&ev)) {
            if (driven_plan_id_ != 0 && p->plan_id == driven_plan_id_ && p->arrived) {
                if (driven_goal_seq_ == goal_seq_) {
                    out.push_back(CancelPlan{});
                    pending_request_id_ = 0;
                    driven_plan_id_ = 0;
                    stall_replans_ = 0;
                    has_goal_ = false;
                    go(NavState::Idle, BlockedReason::None, out);
                } else {
                    driven_plan_id_ = 0;
                }
            }
        } else if (const auto* t = std::get_if<Tick>(&ev)) {
            if (pending_request_id_ != 0 && t->stamp - last_request_stamp_ >= cfg_.plan_reply_timeout) {
                out.push_back(CancelPlan{});
                pending_request_id_ = 0;  // a late reply now carries a stale id and is ignored
                stop(out);
                go(NavState::Blocked, BlockedReason::NoPathToGoal, out);
            }
        }
        break;

    case NavState::Following:
        if (const auto* p = std::get_if<Progress>(&ev)) {
            if (p->plan_id != driven_plan_id_) break;
            if (p->arrived) {
                driven_plan_id_ = 0;
                stall_replans_ = 0;
                has_goal_ = false;
                go(NavState::Idle, BlockedReason::None, out);
            } else {
                cursor(*p);
            }
        } else if (const auto* m = std::get_if<MapChanged>(&ev)) {
            if (m->path_valid) break;
            if (m->blocked_distance < cfg_.stop_distance) stop(out);
            request(m->stamp, out);
            go(NavState::Planning, BlockedReason::None, out);
        } else if (const auto* t = std::get_if<Tick>(&ev)) {
            if (!stalled(t->stamp)) break;
            stop(out);
            if (stall_replans_ >= cfg_.max_stall_replans) {
                go(NavState::Blocked, BlockedReason::Stalled, out);
                break;
            }
            ++stall_replans_;
            request(t->stamp, out);
            go(NavState::Planning, BlockedReason::None, out);
        }
        break;

    case NavState::Clearing:
        if (const auto* e = std::get_if<EscapeResult>(&ev)) {
            if (e->found) {
                ++clearing_attempts_;
                publish(PlanKind::Escape, e->stamp, out);
            } else go(NavState::Blocked, BlockedReason::NoEscape, out);
        } else if (const auto* p = std::get_if<Progress>(&ev)) {
            if (driven_plan_id_ == 0 || p->plan_id != driven_plan_id_) break;
            if (p->arrived) {
                driven_plan_id_ = 0;
                request(p->stamp, out);
                go(NavState::Planning, BlockedReason::None, out);
            } else {
                cursor(*p);
            }
        } else if (const auto* t = std::get_if<Tick>(&ev)) {
            if (driven_plan_id_ != 0 && stalled(t->stamp)) {
                stop(out);
                go(NavState::Blocked, BlockedReason::NoEscape, out);
            }
        }
        break;

    case NavState::Blocked:
        {
            double stamp = -1.0;
            if (const auto* m = std::get_if<MapChanged>(&ev)) stamp = m->stamp;
            else if (const auto* t = std::get_if<Tick>(&ev)) stamp = t->stamp;
            else break;
            if (reason_ != BlockedReason::Stalled && has_goal_ && stamp - last_request_stamp_ >= cfg_.blocked_retry_period) {
                request(stamp, out);
                go(NavState::Planning, BlockedReason::None, out);
            }
        }
        break;
    }
    return out;
}
