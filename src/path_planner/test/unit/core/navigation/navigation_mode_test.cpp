#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "core/navigation/forward_clock.hpp"
#include "core/navigation/navigation_mode.hpp"

namespace {

std::string str(const NavAction& a) {
    struct V {
        std::string operator()(const RequestPlan& x) const { return "Req" + std::to_string(x.request_id); }
        std::string operator()(const CancelPlan&) const { return "Cancel"; }
        std::string operator()(const RunEscape&) const { return "Escape"; }
        std::string operator()(const PublishPlan& x) const {
            return std::string(x.kind == PlanKind::Goal ? "PubG" : "PubE") + std::to_string(x.plan_id);
        }
        std::string operator()(const PublishStop&) const { return "Stop"; }
        std::string operator()(const Report& x) const {
            return "Rep" + std::to_string(int(x.state)) + "/" + std::to_string(int(x.reason));
        }
    };
    return std::visit(V{}, a);
}

std::string seq(const std::vector<NavAction>& v) {
    std::string s;
    for (const auto& a : v) s += (s.empty() ? "" : ",") + str(a);
    return s;
}

std::string rep(NavState s, BlockedReason r = BlockedReason::None) {
    return "Rep" + std::to_string(int(s)) + "/" + std::to_string(int(r));
}

constexpr auto NoR = BlockedReason::None;

class Nav : public ::testing::Test {
protected:
    NavigationMode n{NavigationConfig{2.0, 3.0, 2.0, 2, 2, 8.0}};
    std::string go(double t) { return seq(n.step(GoalReceived{t})); }
    std::string reply(double t, uint32_t id, PlanOutcome o) { return seq(n.step(PlanReply{t, id, o})); }
    std::string map(double t, bool ok, double d = 0.0) { return seq(n.step(MapChanged{t, ok, d})); }
    std::string prog(double t, uint32_t id, uint32_t c, bool arr = false) { return seq(n.step(Progress{t, id, c, arr})); }
    std::string tick(double t) { return seq(n.step(Tick{t})); }
    std::string esc(double t, bool f) { return seq(n.step(EscapeResult{t, f})); }
    // Idle -> Planning(req 1) -> Following(plan 1) at t=0.
    void following() {
        go(0);
        reply(0, n.pendingRequestId(), PlanOutcome::Found);
        ASSERT_EQ(n.state(), NavState::Following);
    }
};

TEST_F(Nav, Rule1GoalFromIdle) {
    EXPECT_EQ(go(1), "Req1," + rep(NavState::Planning));
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(n.pendingRequestId(), 1u);
}

TEST_F(Nav, Rule1GoalDuringPlanningCancelsAndSupersedes) {
    go(0);
    EXPECT_EQ(go(1), "Cancel,Req2");
    EXPECT_EQ(n.pendingRequestId(), 2u);
    EXPECT_EQ(reply(1, 1, PlanOutcome::Found), "");
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(n.drivenPlanId(), 0u);
    EXPECT_EQ(reply(1, 2, PlanOutcome::Found), "PubG1," + rep(NavState::Following));
}

TEST_F(Nav, Rule1GoalLeavesDrivenPlanRunning) {
    following();
    EXPECT_EQ(go(1), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.drivenPlanId(), 1u);
}

TEST_F(Nav, Rule2IgnoredReplies) {
    go(0);
    EXPECT_EQ(reply(0, 7, PlanOutcome::Found), "");
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(n.pendingRequestId(), 1u);
    NavigationMode m({2.0, 3.0, 2.0, 2, 0, 8.0});
    EXPECT_TRUE(m.step(PlanReply{0, 1, PlanOutcome::Found}).empty());  // Idle
}

TEST_F(Nav, Rule3FoundIdsIncrease) {
    go(0);
    EXPECT_EQ(reply(1, 1, PlanOutcome::Found), "PubG1," + rep(NavState::Following));
    EXPECT_EQ(n.pendingRequestId(), 0u);
    EXPECT_EQ(n.drivenPlanId(), 1u);
    go(2);
    EXPECT_EQ(n.pendingRequestId(), 2u);
    EXPECT_EQ(reply(2, 2, PlanOutcome::Found), "PubG2," + rep(NavState::Following));
    go(3);
    EXPECT_EQ(n.pendingRequestId(), 3u);
    EXPECT_EQ(reply(3, 3, PlanOutcome::Found), "PubG3," + rep(NavState::Following));
}

TEST_F(Nav, Rule4StartBlockedRunsEscape) {
    go(0);
    EXPECT_EQ(reply(0, 1, PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing));
    EXPECT_EQ(n.state(), NavState::Clearing);
    EXPECT_EQ(n.pendingRequestId(), 0u);
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule4ClearingLoopGivesUpOnThirdStartBlocked) {
    go(0);
    uint32_t id = 1;
    for (int i = 0; i < 2; ++i) {
        EXPECT_EQ(reply(0, id, PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing)) << i;
        EXPECT_EQ(esc(0, true), "PubE" + std::to_string(i + 1));
        EXPECT_EQ(prog(0, i + 1, 0, true), "Req" + std::to_string(id + 1) + "," + rep(NavState::Planning));
        ++id;
    }
    EXPECT_EQ(reply(0, id, PlanOutcome::StartBlocked), "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
    EXPECT_EQ(n.reason(), BlockedReason::NoEscape);
}

TEST_F(Nav, FoundResetsClearingCount) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    esc(0, true);
    prog(0, 1, 0, true);  // Planning, req 2
    reply(0, 2, PlanOutcome::StartBlocked);
    esc(0, true);
    prog(0, 2, 0, true);  // Planning, req 3
    EXPECT_EQ(reply(0, 3, PlanOutcome::Found), "PubG3," + rep(NavState::Following));
    // new episode: two more escapes allowed
    map(1, false, 0.5);   // Planning, req 4
    EXPECT_EQ(reply(1, 4, PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing));
}

TEST_F(Nav, Rule5NoPathOutcomes) {
    for (auto o : {PlanOutcome::GoalBlocked, PlanOutcome::NoPath, PlanOutcome::BudgetExhausted}) {
        NavigationMode m({2.0, 3.0, 2.0, 2, 0, 8.0});
        m.step(GoalReceived{0});
        auto a = m.step(PlanReply{0, 1, o});
        EXPECT_EQ(seq(a), "Stop," + rep(NavState::Blocked, BlockedReason::NoPathToGoal));
        EXPECT_EQ(m.pendingRequestId(), 0u);
    }
}

TEST_F(Nav, Rule6StoppedGoesIdle) {
    go(0);
    EXPECT_EQ(reply(0, 1, PlanOutcome::Stopped), "Stop," + rep(NavState::Idle));
    EXPECT_EQ(n.pendingRequestId(), 0u);
    EXPECT_EQ(map(100, false, 0.0), "");  // no goal: nothing
}

TEST_F(Nav, Rule7PlanningMapChanged) {
    following();
    go(1);  // Planning with plan 1 still driven
    EXPECT_EQ(map(1, false, 1.9), "Stop");
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(n.drivenPlanId(), 0u);
    EXPECT_EQ(map(1, false, 0.1), "");  // nothing driven now
}

TEST_F(Nav, Rule7FarBlockageAndValidIgnored) {
    following();
    go(1);
    EXPECT_EQ(map(1, false, 2.0), "");
    EXPECT_EQ(map(1, true), "");
    EXPECT_EQ(n.drivenPlanId(), 1u);
}

TEST_F(Nav, TenFarMapChangesWhilePlanningNoActions) {
    go(0);
    for (int i = 0; i < 10; ++i) EXPECT_EQ(map(i, false, 5.0), "");
    EXPECT_EQ(n.state(), NavState::Planning);
}

TEST_F(Nav, Rule8ArrivalDuringReplanForSameGoal) {
    following();
    map(1, false, 5.0);  // Following -> Planning, plan 1 still driven (far)
    ASSERT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(prog(2, 1, 9, true), "Cancel," + rep(NavState::Idle));
    EXPECT_EQ(n.pendingRequestId(), 0u);
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule8ArrivalAtOlderGoalKeepsPlanning) {
    following();
    go(1);  // new goal, plan 1 belongs to the old goal
    EXPECT_EQ(prog(2, 1, 9, true), "");
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(n.pendingRequestId(), 2u);
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule9Arrived) {
    following();
    EXPECT_EQ(prog(1, 1, 5, true), rep(NavState::Idle));
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule9OtherPlanIdIgnoredEverywhere) {
    following();
    EXPECT_EQ(prog(1, 9, 5, true), "");
    EXPECT_EQ(n.state(), NavState::Following);
}

TEST_F(Nav, Rule10MapChanged) {
    following();
    EXPECT_EQ(map(1, true), "");
    EXPECT_EQ(map(1, false, 1.9), "Stop,Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule10AtStopDistanceNoStop) {
    following();
    EXPECT_EQ(map(1, false, 2.0), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.drivenPlanId(), 1u);
}

TEST_F(Nav, Rule11Stall) {
    following();
    EXPECT_EQ(tick(2.999), "");
    EXPECT_EQ(tick(3.0), "Stop,Req2," + rep(NavState::Planning));
}

TEST_F(Nav, StallCursorChangeResetsTimer) {
    following();
    EXPECT_EQ(prog(2.0, 1, 1), "");
    EXPECT_EQ(tick(4.999), "");
    EXPECT_EQ(tick(5.0), "Stop,Req2," + rep(NavState::Planning));
}

TEST_F(Nav, StallSameCursorDoesNotReset) {
    following();
    prog(1.0, 1, 0);
    prog(2.0, 1, 0);
    EXPECT_EQ(tick(3.0), "Stop,Req2," + rep(NavState::Planning));
}

TEST_F(Nav, Rule12EscapeFoundAndNot) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    EXPECT_EQ(esc(1, true), "PubE1");
    EXPECT_EQ(n.state(), NavState::Clearing);
    EXPECT_EQ(n.drivenPlanId(), 1u);

    NavigationMode m({2.0, 3.0, 2.0, 2, 0, 8.0});
    m.step(GoalReceived{0});
    m.step(PlanReply{0, 1, PlanOutcome::StartBlocked});
    EXPECT_EQ(seq(m.step(EscapeResult{1, false})), rep(NavState::Blocked, BlockedReason::NoEscape));

    NavigationMode k({2.0, 3.0, 2.0, 2, 0, 8.0});  // Idle ignores EscapeResult
    EXPECT_TRUE(k.step(EscapeResult{0, true}).empty());
}

TEST_F(Nav, Rule13EscapeArrivedReplans) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    esc(0, true);
    EXPECT_EQ(prog(1, 1, 3), "");
    EXPECT_EQ(prog(1, 1, 4, true), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST_F(Nav, Rule14ClearingStall) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    EXPECT_EQ(tick(100), "");  // nothing driven yet
    esc(10, true);
    EXPECT_EQ(tick(12.999), "");
    EXPECT_EQ(tick(13.0), "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
}

TEST_F(Nav, Rule15BlockedRetry) {
    go(0);
    reply(0, 1, PlanOutcome::NoPath);
    EXPECT_EQ(map(1.999, true), "");
    EXPECT_EQ(map(2.0, true), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.reason(), NoR);
}

TEST_F(Nav, Rule15RetryKeepsClearingCount) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    esc(0, true); prog(0, 1, 0, true);
    reply(0, 2, PlanOutcome::StartBlocked);
    esc(0, true); prog(0, 2, 0, true);
    reply(0, 3, PlanOutcome::StartBlocked);  // Blocked(NoEscape)
    ASSERT_EQ(n.state(), NavState::Blocked);
    EXPECT_EQ(map(5, false, 0.0), "Req4," + rep(NavState::Planning));
    EXPECT_EQ(reply(5, 4, PlanOutcome::StartBlocked), "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
}

// Every plan reply is StartBlocked and every escape succeeds. Returns the RunEscape count until Blocked(NoEscape),
// then drives `retries` timed retries (period 2.0 s) and checks each one asks for a plan and is refused without escape.
int exhaustClearing(NavigationMode& m, int retries, double& t) {
    int escapes = 0;
    m.step(GoalReceived{t});
    while (m.state() != NavState::Blocked) {
        const std::string a = seq(m.step(PlanReply{t, m.pendingRequestId(), PlanOutcome::StartBlocked}));
        if (a.find("Escape") == std::string::npos) break;
        ++escapes;
        m.step(EscapeResult{t, true});
        m.step(Progress{t, m.drivenPlanId(), 0, true});
    }
    EXPECT_EQ(m.state(), NavState::Blocked);
    EXPECT_EQ(m.reason(), BlockedReason::NoEscape);
    int requests = 0;
    for (int i = 0; i < retries; ++i) {
        t += 2.0;
        const std::string a = seq(m.step(Tick{t}));
        EXPECT_EQ(a, "Req" + std::to_string(m.pendingRequestId()) + "," + rep(NavState::Planning)) << i;
        requests += a.rfind("Req", 0) == 0;
        EXPECT_EQ(seq(m.step(PlanReply{t, m.pendingRequestId(), PlanOutcome::StartBlocked})),
                  "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape)) << i;
    }
    EXPECT_EQ(requests, retries);
    return escapes;
}

TEST_F(Nav, TimedRetriesNeverGrantMoreEscapesThanTheLimit) {
    double t = 0;
    EXPECT_EQ(exhaustClearing(n, 10, t), 2);
}

TEST(NavClearingLimit, SingleAttemptLimit) {
    NavigationMode m{NavigationConfig{2.0, 3.0, 2.0, 1, 2, 8.0}};
    double t = 0;
    EXPECT_EQ(exhaustClearing(m, 10, t), 1);
}

TEST_F(Nav, FoundAfterExhaustedClearingRestoresEscapes) {
    double t = 0;
    exhaustClearing(n, 10, t);
    t += 2.0;
    n.step(Tick{t});
    EXPECT_EQ(reply(t, n.pendingRequestId(), PlanOutcome::Found).rfind("PubG", 0), 0u);
    ASSERT_EQ(n.state(), NavState::Following);
    EXPECT_EQ(map(t + 1, false, 5.0).rfind("Req", 0), 0u);
    ASSERT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(reply(t + 1, n.pendingRequestId(), PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing));
}

TEST_F(Nav, NewGoalAfterExhaustedClearingRestoresEscapes) {
    double t = 0;
    exhaustClearing(n, 10, t);
    t += 1.0;
    EXPECT_EQ(go(t).rfind("Req", 0), 0u);
    ASSERT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(reply(t, n.pendingRequestId(), PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing));
}

int countOf(const std::string& s, const std::string& needle) {
    int c = 0;
    for (auto p = s.find(needle); p != std::string::npos; p = s.find(needle, p + 1)) ++c;
    return c;
}

TEST_F(Nav, EscapesNotFoundAreNeverCounted) {
    std::string all;
    double t = 0;
    all += go(t);
    const std::string blocked = rep(NavState::Blocked, BlockedReason::NoEscape);
    for (int i = 0; i <= 10; ++i) {
        if (i > 0) {
            t += 2.0;
            const std::string a = tick(t);
            EXPECT_EQ(a, "Req" + std::to_string(n.pendingRequestId()) + "," + rep(NavState::Planning)) << i;
            all += a;
        }
        const std::string r = reply(t, n.pendingRequestId(), PlanOutcome::StartBlocked);
        EXPECT_EQ(r, "Stop,Escape," + rep(NavState::Clearing)) << i;
        all += r;
        EXPECT_EQ(esc(t, false), blocked) << i;
    }
    EXPECT_EQ(countOf(all, "Escape"), 11);
    EXPECT_EQ(countOf(all, "PubE"), 0);
}

TEST_F(Nav, ExactlyLimitEscapePlansArePublishedAfterNotFoundCycles) {
    double t = 0;
    go(t);
    for (int i = 0; i < 5; ++i) {
        reply(t, n.pendingRequestId(), PlanOutcome::StartBlocked);
        esc(t, false);
        t += 2.0;
        tick(t);
    }
    int published = 0;
    for (int i = 0; i < 2; ++i) {
        EXPECT_EQ(reply(t, n.pendingRequestId(), PlanOutcome::StartBlocked), "Stop,Escape," + rep(NavState::Clearing));
        published += countOf(esc(t, true), "PubE");
        prog(t, n.drivenPlanId(), 0, true);
    }
    EXPECT_EQ(published, 2);
    ASSERT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(reply(t, n.pendingRequestId(), PlanOutcome::StartBlocked),
              "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
}

TEST(NavClearingLimit, StalledPublishedEscapeCountsAsAnAttempt) {
    NavigationMode m{NavigationConfig{2.0, 3.0, 2.0, 1, 2, 8.0}};
    m.step(GoalReceived{0});
    m.step(PlanReply{0, m.pendingRequestId(), PlanOutcome::StartBlocked});
    m.step(EscapeResult{0, true});
    EXPECT_EQ(seq(m.step(Tick{3.0})), "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
    const std::string retry = seq(m.step(Tick{5.0}));
    EXPECT_EQ(retry, "Req" + std::to_string(m.pendingRequestId()) + "," + rep(NavState::Planning));
    EXPECT_EQ(seq(m.step(PlanReply{5.0, m.pendingRequestId(), PlanOutcome::StartBlocked})),
              "Stop," + rep(NavState::Blocked, BlockedReason::NoEscape));
}

TEST_F(Nav, BlockedGoalIsRetriedOnTheTimer) {
    go(0);
    reply(0.1, 1, PlanOutcome::NoPath);
    ASSERT_EQ(n.reason(), BlockedReason::NoPathToGoal);
    EXPECT_EQ(tick(1.9), "");
    EXPECT_EQ(tick(2.0), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.state(), NavState::Planning);
}

TEST_F(Nav, NoEscapeBlockedGoalIsRetriedOnTheTimer) {
    go(0);
    reply(0, 1, PlanOutcome::StartBlocked);
    esc(0, true); prog(0, 1, 0, true);
    reply(0, 2, PlanOutcome::StartBlocked);
    esc(0, true); prog(0, 2, 0, true);
    reply(0, 3, PlanOutcome::StartBlocked);  // Blocked(NoEscape), request 3 stamped 0
    ASSERT_EQ(n.reason(), BlockedReason::NoEscape);
    EXPECT_EQ(tick(1.9), "");
    EXPECT_EQ(tick(2.0), "Req4," + rep(NavState::Planning));
}

TEST_F(Nav, Rule16IdleIgnoresEverythingElse) {
    EXPECT_EQ(map(0, false, 0), "");
    EXPECT_EQ(tick(100), "");
    EXPECT_EQ(prog(0, 1, 0, true), "");
    EXPECT_EQ(esc(0, true), "");
    EXPECT_EQ(reply(0, 1, PlanOutcome::Found), "");
    EXPECT_EQ(n.state(), NavState::Idle);
}

// ---- stall give-up (max_stall_replans) ----------------------------------------------------

class NavStall : public ::testing::Test {
protected:
    explicit NavStall(int max_stall_replans = 2) : n{NavigationConfig{2.0, 3.0, 2.0, 2, max_stall_replans, 8.0}} {}
    NavigationMode n;
    std::string tick(double t) { return seq(n.step(Tick{t})); }
    void follow(double t) {
        n.step(GoalReceived{t});
        n.step(PlanReply{t, n.pendingRequestId(), PlanOutcome::Found});
        ASSERT_EQ(n.state(), NavState::Following);
    }
    // A stall tick at t that must replan, answered with a found path at t (progress timer restarts).
    void stallAndReplan(double t) {
        const std::string a = tick(t);
        EXPECT_EQ(a.rfind("Stop,Req", 0), 0u) << a;
        EXPECT_EQ(n.state(), NavState::Planning);
        n.step(PlanReply{t, n.pendingRequestId(), PlanOutcome::Found});
        ASSERT_EQ(n.state(), NavState::Following);
    }
};

class NavStallZero : public NavStall {
protected:
    NavStallZero() : NavStall(0) {}
};

TEST_F(NavStall, TwoReplansThenBlockedStalled) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    EXPECT_EQ(tick(9), "Stop," + rep(NavState::Blocked, BlockedReason::Stalled));
    EXPECT_EQ(n.state(), NavState::Blocked);
    EXPECT_EQ(n.reason(), BlockedReason::Stalled);
}

TEST_F(NavStallZero, FirstStallIsFinal) {
    follow(0);
    EXPECT_EQ(tick(3), "Stop," + rep(NavState::Blocked, BlockedReason::Stalled));
}

TEST_F(NavStall, StalledIgnoresMapChangedAndNewGoalLeaves) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    tick(9);
    ASSERT_EQ(n.reason(), BlockedReason::Stalled);
    EXPECT_EQ(seq(n.step(MapChanged{20, false, 0.0})), "");
    EXPECT_EQ(seq(n.step(MapChanged{40, true, 0.0})), "");
    EXPECT_EQ(n.state(), NavState::Blocked);
    EXPECT_EQ(n.reason(), BlockedReason::Stalled);
    EXPECT_EQ(seq(n.step(GoalReceived{50})).rfind("Req", 0), 0u);
    EXPECT_EQ(n.state(), NavState::Planning);
}

TEST_F(NavStall, StalledIsNotRetriedOnTheTimer) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    tick(9);
    ASSERT_EQ(n.reason(), BlockedReason::Stalled);
    EXPECT_EQ(tick(20), "");
    EXPECT_EQ(tick(100), "");
    EXPECT_EQ(n.state(), NavState::Blocked);
    EXPECT_EQ(n.reason(), BlockedReason::Stalled);
}

// After two replans, a new goal must restore the full allowance: two more stalls replan.
TEST_F(NavStall, NewGoalResetsCount) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    n.step(GoalReceived{6});
    n.step(PlanReply{6, n.pendingRequestId(), PlanOutcome::Found});
    ASSERT_EQ(n.state(), NavState::Following);
    stallAndReplan(9);
    stallAndReplan(12);
    EXPECT_EQ(tick(15), "Stop," + rep(NavState::Blocked, BlockedReason::Stalled));
}

// Cursor movement restarts the stall timer but not the per-goal count.
TEST_F(NavStall, CursorChangeDoesNotResetCount) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    EXPECT_EQ(seq(n.step(Progress{7, n.drivenPlanId(), 1, false})), "");
    EXPECT_EQ(tick(9), "");  // movement at 7 restarted the stall timer: 7 + 3 = 10 not reached
    EXPECT_EQ(tick(10), "Stop," + rep(NavState::Blocked, BlockedReason::Stalled));
}

TEST_F(NavStall, ArrivalResetsCount) {
    follow(0);
    stallAndReplan(3);
    stallAndReplan(6);
    EXPECT_EQ(seq(n.step(Progress{7, n.drivenPlanId(), 5, true})), rep(NavState::Idle));
    follow(7);
    stallAndReplan(10);
    stallAndReplan(13);
    EXPECT_EQ(tick(16), "Stop," + rep(NavState::Blocked, BlockedReason::Stalled));
}

// ---- plan reply timeout (plan_reply_timeout = 8.0) ----------------------------------------

TEST_F(Nav, PlanningTimesOutWithoutReply) {
    go(0);
    EXPECT_EQ(tick(7.9), "");
    EXPECT_EQ(n.state(), NavState::Planning);
    EXPECT_EQ(tick(8.0), "Cancel,Stop," + rep(NavState::Blocked, BlockedReason::NoPathToGoal));
    EXPECT_EQ(n.state(), NavState::Blocked);
    EXPECT_EQ(n.reason(), BlockedReason::NoPathToGoal);
}

TEST_F(Nav, LateReplyAfterTimeoutIsIgnored) {
    go(0);
    const uint32_t id = n.pendingRequestId();
    tick(8.0);
    EXPECT_EQ(reply(9, id, PlanOutcome::Found), "");
    EXPECT_EQ(n.state(), NavState::Blocked);
}

TEST_F(Nav, TimedOutGoalIsRetriedOnMapChange) {
    go(0);
    tick(8.0);
    EXPECT_EQ(map(10.5, true, 0.0), "Req2," + rep(NavState::Planning));
    EXPECT_EQ(n.state(), NavState::Planning);
}

TEST_F(Nav, TimeoutWhileDrivingStopsTheRobot) {
    following();
    EXPECT_EQ(map(1, false, 2.0), "Req2," + rep(NavState::Planning));
    ASSERT_EQ(n.drivenPlanId(), 1u);
    EXPECT_EQ(tick(8.9), "");
    EXPECT_EQ(tick(9.0), "Cancel,Stop," + rep(NavState::Blocked, BlockedReason::NoPathToGoal));
    EXPECT_EQ(n.drivenPlanId(), 0u);
}

TEST(NavigationConfig, RejectsOutOfRange) {
    const NavigationConfig ok{2.0, 3.0, 2.0, 2, 2, 8.0};
    EXPECT_NO_THROW(NavigationMode{ok});
    for (int f = 0; f < 6; ++f) {
        for (double bad : {0.0, -1.0}) {
            if (f == 4 && bad == 0.0) continue;  // 0 is a valid max_stall_replans
            auto c = ok;
            if (f == 0) c.stop_distance = bad;
            if (f == 1) c.stall_timeout = bad;
            if (f == 2) c.blocked_retry_period = bad;
            if (f == 3) c.max_clearing_attempts = static_cast<int>(bad);
            if (f == 4) c.max_stall_replans = static_cast<int>(bad);
            if (f == 5) c.plan_reply_timeout = bad;
            EXPECT_THROW(NavigationMode{c}, std::invalid_argument) << f << " " << bad;
        }
    }
}

}  // namespace

// ---- raw clock that steps backward, fed through ForwardClock ----

namespace {

enum class Scenario { Blocked, Following, Planning };

// Raw clock starts at 1000 and ticks every 0.5 s of real time; at real time 1.0 s it steps by `step`.
// Returns the real time (s) of the first Tick whose actions contain `needle`, or -1 if none within 30 s.
double firstFiringTick(Scenario sc, double step, const std::string& needle) {
    NavigationMode m{NavigationConfig{1.0, 5.0, 2.0, 2, 4, 10.0}};
    ForwardClock clock;
    auto raw = [&](double real) { return 1000.0 + real + (real >= 1.0 ? step : 0.0); };
    const double t0 = clock.advance(raw(0.0));
    m.step(GoalReceived{t0});
    if (sc != Scenario::Planning) {
        m.step(PlanReply{clock.advance(raw(0.0)), m.pendingRequestId(),
                         sc == Scenario::Blocked ? PlanOutcome::NoPath : PlanOutcome::Found});
    }
    for (int k = 1; k <= 60; ++k) {
        const double real = 0.5 * k;
        const std::string a = seq(m.step(Tick{clock.advance(raw(real))}));
        if (a.find(needle) != std::string::npos) return real;
    }
    return -1.0;
}

}  // namespace

TEST(NavClockStep, blockedRetryIsOneTickLateAfterBackwardStep) {
    EXPECT_EQ(firstFiringTick(Scenario::Blocked, -300.0, "Req"), 2.5);
}

TEST(NavClockStep, stallTimeoutIsOneTickLateAfterBackwardStep) {
    EXPECT_EQ(firstFiringTick(Scenario::Following, -300.0, "Stop"), 5.5);
}

TEST(NavClockStep, planReplyTimeoutIsOneTickLateAfterBackwardStep) {
    EXPECT_EQ(firstFiringTick(Scenario::Planning, -300.0, "Stop"), 10.5);
}

TEST(NavClockStep, controlWithoutStepFiresOnTime) {
    EXPECT_EQ(firstFiringTick(Scenario::Blocked, 0.0, "Req"), 2.0);
    EXPECT_EQ(firstFiringTick(Scenario::Following, 0.0, "Stop"), 5.0);
    EXPECT_EQ(firstFiringTick(Scenario::Planning, 0.0, "Stop"), 10.0);
}
