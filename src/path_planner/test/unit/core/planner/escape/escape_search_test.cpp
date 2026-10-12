#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/escape/escape_search.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/motion_context.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/math/wrap_angle.hpp"

namespace {

constexpr double H = 0.05;          // grid resolution (m)
constexpr int N = 200;              // map is N x N cells, origin (0, 0), 10 m square
constexpr double DELTA = 0.4 * H;   // escape step length for the default config

// Start used by every map below: footprint spans x in [4.73, 5.33], y in [4.73, 5.33].
const SE2State START{5.03, 5.03, 0.0};

DiffDriveParams kin() {
    DiffDriveParams k;
    k.rmin = 0.3; k.vmax = 0.5; k.atomic_time = 0.6; k.n_headings = 16;
    return k;
}

// 0 free, 100 occupied, -1 unknown. Cell (i, j) = (column, row) is at data[j * N + i]; its x range is
// [i*H, (i+1)*H).
struct Env {
    std::shared_ptr<const OccupancyGrid> grid;
    std::optional<CollisionChecker> checker;
    std::optional<MotionContext> motion;
    DiffDriveParams k = kin();
};

Env makeEnv(int8_t fill, const std::function<void(std::vector<int8_t> &)> & paint,
            const Eigen::Vector2f & origin = Eigen::Vector2f::Zero(),
            const Eigen::Vector2f & motion_origin = Eigen::Vector2f::Zero()) {
    GridMetaData meta(N, N, H, origin);
    GridMetaData motion_meta(N, N, H, motion_origin);
    std::vector<int8_t> cells(static_cast<size_t>(N) * N, fill);
    paint(cells);
    Env e;
    e.grid = std::make_shared<const OccupancyGrid>(meta, cells);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*e.grid, 5.0));
    auto body = buildRobotParams({{0.3, 0.3}, {-0.3, 0.3}, {-0.3, -0.3}, {0.3, -0.3}});
    e.checker.emplace(e.grid, edt, body);
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(e.k.rmin * e.k.rmin);
    auto space = std::make_shared<const DiffDriveStateSpace>(motion_meta, dist);
    e.motion.emplace(body, space, motion_meta);
    return e;
}

void fillRect(std::vector<int8_t> & c, int i0, int i1, int j0, int j1, int8_t val) {  // inclusive
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) c[static_cast<size_t>(j) * N + i] = val;
}

// Wall ahead: cells i = 106..115 (x from 5.30), every row. Its face is 0.03 m inside the start
// footprint front (5.33). Other cells hold `fill`.
Env wallAhead(int8_t fill = 0) {
    return makeEnv(fill, [](auto & c) { fillRect(c, 106, 115, 0, N - 1, 100); });
}

// Slot: wall ahead (i >= 106), side walls i = 74..105 for j <= 93 (y < 4.70) and j >= 107 (y >= 5.35).
// The footprint (y in [4.73, 5.33]) clears both sides by 0.03 / 0.02. Rear open.
Env slot(int top_from = 107) {
    return makeEnv(0, [top_from](auto & c) {
        fillRect(c, 106, N - 1, 0, N - 1, 100);
        fillRect(c, 74, 105, 0, 93, 100);
        fillRect(c, 74, 105, top_from, N - 1, 100);
    });
}

// Corridor: walls j <= 93 (y < 4.70) and j >= 109 (y >= 5.45), every column.
Env corridor() {
    return makeEnv(0, [](auto & c) {
        fillRect(c, 0, N - 1, 0, 93, 100);
        fillRect(c, 0, N - 1, 109, N - 1, 100);
    });
}

// Tight corridor: wall ahead as in wallAhead, plus side walls over every column for rows j <= 92 and
// j >= 108. The nearest side-wall cell centres are y = 4.625 and y = 5.425, 0.105 m and 0.095 m from the
// footprint edges (y in [4.73, 5.33]). The plain start margin is (sqrt(2)/2 + 1/2)*H = 0.0604 and with one
// cell of slack 0.1104, so the plain start check can pass here while the slack never can.
Env tightCorridor() {
    return makeEnv(0, [](auto & c) {
        fillRect(c, 106, 115, 0, N - 1, 100);
        fillRect(c, 0, N - 1, 0, 92, 100);
        fillRect(c, 0, N - 1, 108, N - 1, 100);
    });
}

// Closed box: every cell occupied except i, j = 94..106 (x, y in [4.70, 5.35]).
Env closedBox() {
    return makeEnv(100, [](auto & c) { fillRect(c, 94, 106, 94, 106, 0); });
}

double travel(const Path & p) { return std::abs(p.segments[0].u.v) * p.segments[0].u.dt; }

double slackM(const EscapeConfig & c) { return c.slack_cells * H; }

Path expectPath(const Env & e, const SE2State & s, const EscapeConfig & cfg = {}, double stop_short = 0.0) {
    auto p = findEscape(s, *e.checker, *e.motion, e.k, cfg, stop_short);
    EXPECT_TRUE(p.has_value());
    if (!p) return Path{};
    EXPECT_EQ(p->segments.size(), 1u);
    return *p;
}

}  // namespace

// 1. Clear start: no motion, even when it lacks the slack margin.
TEST(EscapeSearch, clearStartReturnsEmptyPath) {
    // Wall cells i >= 108 (x >= 5.40): 0.07 m from the front edge, between the plain margin
    // (0.0604) and margin plus slack (0.1104).
    Env near_wall = makeEnv(0, [](auto & c) { fillRect(c, 108, N - 1, 0, N - 1, 100); });
    ASSERT_TRUE(near_wall.checker->startClear(START, 0.0));
    ASSERT_FALSE(near_wall.checker->startClear(START, H));
    Env open = makeEnv(0, [](auto &) {});
    for (const Env * e : {&near_wall, &open}) {
        auto p = findEscape(START, *e->checker, *e->motion, e->k, EscapeConfig{}, 0.0);
        ASSERT_TRUE(p.has_value());
        EXPECT_EQ(p->segments.size(), 0u);
        ASSERT_EQ(p->node_points.size(), 1u);
        EXPECT_EQ(p->node_points[0].x, START.x);
        EXPECT_EQ(p->node_points[0].y, START.y);
        EXPECT_EQ(p->node_points[0].theta, START.theta);
        EXPECT_EQ(p->total_cost, 0.0);
    }
}

// 2. Wall ahead: one reverse segment, first step count that reaches the slack margin.
TEST(EscapeSearch, wallAheadBacksOutToSlackMargin) {
    Env e = wallAhead();
    const Path p = expectPath(e, START);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_EQ(p.segments[0].u.v, -0.5);
    EXPECT_EQ(p.segments[0].u.omega, 0.0);
    EXPECT_NEAR(travel(p), 0.12, 1e-6);
    const SE2State end = p.node_points.back();
    EXPECT_TRUE(e.checker->startClear(end, H));
    EXPECT_TRUE(e.checker->checkState(end));
    EXPECT_FALSE(e.checker->startClear({START.x - 0.10, START.y, 0.0}, H));
}

// 3. slack_cells = 0 stops at the plain margin.
TEST(EscapeSearch, zeroSlackStopsAtPlainMargin) {
    Env e = wallAhead();
    EscapeConfig cfg;
    cfg.slack_cells = 0;
    const Path p = expectPath(e, START, cfg);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_NEAR(travel(p), 0.08, 1e-6);
    EXPECT_TRUE(e.checker->startClear(p.node_points.back(), 0.0));
    EXPECT_FALSE(e.checker->startClear({START.x - 0.06, START.y, 0.0}, 0.0));
}

// 4. Slot: the side walls hold at every step, rear open; backs out of the whole slot.
TEST(EscapeSearch, slotBacksOutOfWholeSlot) {
    Env e = slot();
    const Path p = expectPath(e, START);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_NEAR(travel(p), 1.72, 1e-6);
    EXPECT_TRUE(e.checker->startClear(p.node_points.back(), H));
}

// 5. Slot whose top wall overlaps the footprint: backing out would touch new top-wall cells.
TEST(EscapeSearch, slotWithOverlapReturnsNullopt) {
    Env e = slot(106);
    EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.0).has_value());
}

// 6. Corridor narrower than the pose requires: no straight move clears it.
TEST(EscapeSearch, corridorReturnsNullopt) {
    Env e = corridor();
    ASSERT_FALSE(e.checker->startClear(START, 0.0));
    EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.0).has_value());
}

// 6b. Tight corridor: the slack never fits, so the first move that passes the plain start check is taken.
TEST(EscapeSearch, tightCorridorTakesFirstPassingMove) {
    Env e = tightCorridor();
    const Path p = expectPath(e, START);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_EQ(p.segments[0].u.v, -0.5);
    EXPECT_NEAR(travel(p), 0.08, 1e-6);
    const SE2State end = p.node_points.back();
    EXPECT_TRUE(e.checker->startClear(end, 0.0));
    EXPECT_FALSE(e.checker->startClear(end, H));
    EXPECT_FALSE(e.checker->startClear({START.x - 0.06, START.y, 0.0}, 0.0));
}

// 7. Closed box.
TEST(EscapeSearch, closedBoxReturnsNullopt) {
    Env e = closedBox();
    EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.0).has_value());
}

// 8. Only forward is free: wall cells i = 84..94 (x < 4.75), 0.02 m inside the rear edge (4.73).
// Driving forward k steps puts the rear edge at 4.73 + 0.02k. The nearest wall cell centre is
// 4.725 (cell i = 94). startClear at slack 0.05 needs the centre more than
// (sqrt(2)/2 + 1/2)*0.05 + 0.05 = 0.110355 behind the rear edge:
//   4.73 + 0.02k - 4.725 > 0.110355  =>  0.02k > 0.105355  =>  k > 5.27  =>  k = 6, travel 0.12 m.
TEST(EscapeSearch, forwardWhenOnlyForwardIsFree) {
    Env e = makeEnv(0, [](auto & c) { fillRect(c, 84, 94, 0, N - 1, 100); });
    const Path p = expectPath(e, START);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_EQ(p.segments[0].u.v, 0.5);
    EXPECT_EQ(p.segments[0].u.omega, 0.0);
    EXPECT_NEAR(travel(p), 0.12, 1e-6);
    EXPECT_TRUE(e.checker->startClear(p.node_points.back(), H));
}

// 9. Travel cap: floor(0.14 * r_out / 0.02) = 2 steps, too few to clear.
TEST(EscapeSearch, travelCapGivesUp) {
    Env e = wallAhead();
    EscapeConfig cfg;
    cfg.max_travel_factor = 0.14;
    EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, cfg, 0.0).has_value());
}

// 10. Path invariants.
TEST(EscapeSearch, pathInvariants) {
    for (Env e : {wallAhead(), slot()}) {
        const Path p = expectPath(e, START);
        ASSERT_EQ(p.segments.size(), 1u);
        ASSERT_EQ(p.node_points.size(), p.segments.size() + 1);
        const Control u = p.segments[0].u;
        SE2State next = e.motion->space().propagate(START, u);
        next.theta = wrapToPi(next.theta);
        EXPECT_NEAR(p.node_points[1].x, next.x, 1e-12);
        EXPECT_NEAR(p.node_points[1].y, next.y, 1e-12);
        EXPECT_NEAR(p.node_points[1].theta, next.theta, 1e-12);
        EXPECT_EQ(p.segments[0].cost, makeControlCost(e.k)(u));
        EXPECT_EQ(p.total_cost, p.segments[0].cost);
        EXPECT_NEAR(p.node_points[1].theta, wrapToPi(START.theta), 1e-12);
    }
}

// 11. Angled start: value, one reverse segment, clear end. Travel is printed, not pinned.
TEST(EscapeSearch, angledStart) {
    Env e = wallAhead();
    const SE2State s{5.03, 5.03, 0.2};
    const Path p = expectPath(e, s);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_LT(p.segments[0].u.v, 0.0);
    EXPECT_TRUE(e.checker->startClear(p.node_points.back(), 0.0));
    RecordProperty("travel", std::to_string(travel(p)));
    std::cout << "angled travel = " << travel(p) << " m\n";
}

// 12. Config validation.
TEST(EscapeSearch, rejectsBadConfig) {
    Env e = wallAhead();
    auto run = [&](auto mutate) {
        EscapeConfig c;
        mutate(c);
        return findEscape(START, *e.checker, *e.motion, e.k, c, 0.0);
    };
    constexpr double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(run([](auto & c) { c.step_fraction = 0.0; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.step_fraction = 1.0; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.step_fraction = -0.1; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.slack_cells = -1; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.max_travel_factor = 0.0; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.max_travel_factor = -1.0; }), std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.max_travel_factor = std::numeric_limits<double>::quiet_NaN(); }),
                 std::invalid_argument);
    EXPECT_THROW(run([](auto & c) { c.max_travel_factor = inf; }), std::invalid_argument);
}

// 13. Same inputs, same path.
TEST(EscapeSearch, deterministic) {
    Env e = slot();
    auto a = findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.0);
    auto b = findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.0);
    ASSERT_TRUE(a && b);
    ASSERT_EQ(a->node_points.size(), b->node_points.size());
    for (size_t i = 0; i < a->node_points.size(); ++i) {
        EXPECT_EQ(a->node_points[i].x, b->node_points[i].x);
        EXPECT_EQ(a->node_points[i].y, b->node_points[i].y);
        EXPECT_EQ(a->node_points[i].theta, b->node_points[i].theta);
    }
    ASSERT_EQ(a->segments.size(), b->segments.size());
    for (size_t i = 0; i < a->segments.size(); ++i) {
        EXPECT_EQ(a->segments[i].u.v, b->segments[i].u.v);
        EXPECT_EQ(a->segments[i].u.omega, b->segments[i].u.omega);
        EXPECT_EQ(a->segments[i].u.dt, b->segments[i].u.dt);
    }
}

// 14. Random starts: end clear, heading constant, and no step touches an occupied cell that the
// previous step did not. The contact set is an independent brute force: every cell containing a
// sample of the footprint (5 mm body-frame grid over the square, boundary included).
namespace {

using Cell = std::pair<int, int>;

std::set<Cell> occupiedContacts(const SE2State & q, const std::function<bool(int, int)> & occupied) {
    std::set<Cell> out;
    const double c = std::cos(q.theta), s = std::sin(q.theta);
    constexpr int M = 120;  // 0.6 m / 5 mm
    for (int a = 0; a <= M; ++a)
        for (int b = 0; b <= M; ++b) {
            const double bx = -0.3 + 0.005 * a, by = -0.3 + 0.005 * b;
            const double x = q.x + c * bx - s * by, y = q.y + s * bx + c * by;
            const int i = static_cast<int>(std::floor(x / H)), j = static_cast<int>(std::floor(y / H));
            if (i >= 0 && i < N && j >= 0 && j < N && occupied(i, j)) out.insert({i, j});
        }
    return out;
}

void runProperty(const char * name, const Env & e, const std::function<bool(int, int)> & occupied,
                 double stop_short = 0.0) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> pos(4.93, 5.13), ang(-std::numbers::pi, std::numbers::pi);
    int returned = 0;
    for (int n = 0; n < 300; ++n) {
        const SE2State s{pos(rng), pos(rng), ang(rng)};
        auto p = findEscape(s, *e.checker, *e.motion, e.k, EscapeConfig{}, stop_short);
        if (!p || p->segments.empty()) continue;
        ++returned;
        SCOPED_TRACE(testing::Message() << name << " case " << n << " start (" << s.x << ", " << s.y << ", " << s.theta << ")");
        ASSERT_EQ(p->segments.size(), 1u);
        const SE2State end = p->node_points.back();
        EXPECT_TRUE(e.checker->startClear(end, 0.0));
        EXPECT_NEAR(end.theta, wrapToPi(s.theta), 1e-12);
        const Control u = p->segments[0].u;
        const int steps = static_cast<int>(std::lround(travel(*p) / DELTA));
        ASSERT_NEAR(steps * DELTA, travel(*p), 1e-6);
        const double dir = u.v < 0 ? -1.0 : 1.0;
        std::set<Cell> prev = occupiedContacts(s, occupied);
        for (int k = 1; k <= steps; ++k) {
            const SE2State q{s.x + dir * k * DELTA * std::cos(s.theta), s.y + dir * k * DELTA * std::sin(s.theta), s.theta};
            const std::set<Cell> cur = occupiedContacts(q, occupied);
            for (const Cell & c : cur)
                ASSERT_TRUE(prev.count(c)) << "step " << k << " touches new occupied cell (" << c.first << ", " << c.second << ")";
            prev = cur;
            if (stop_short > 0.0 && (steps - k) * DELTA <= stop_short + 1e-9)
                EXPECT_TRUE(e.checker->startClear(q, 0.0)) << "step " << k << " within stop_short of the end";
        }
    }
    std::cout << name << ": " << returned << " of 300 cases returned a path\n";
    testing::Test::RecordProperty(name, std::to_string(returned));
}

}  // namespace

TEST(EscapeSearchProperty, noNewContactWallAhead) {
    Env e = wallAhead();
    runProperty("wall_ahead", e, [](int i, int) { return i >= 106 && i <= 115; });
}

TEST(EscapeSearchProperty, noNewContactCorridor) {
    Env e = corridor();
    runProperty("corridor", e, [](int, int j) { return j <= 93 || j >= 109; });
}

TEST(EscapeSearchProperty, noNewContactWallAheadStopShort) {
    Env e = wallAhead();
    runProperty("wall_ahead_stop_short", e, [](int i, int) { return i >= 106 && i <= 115; }, 0.10);
}

TEST(EscapeSearchProperty, noNewContactCorridorStopShort) {
    Env e = corridor();
    runProperty("corridor_stop_short", e, [](int, int j) { return j <= 93 || j >= 109; }, 0.10);
}

// 15. stop_short: the path is longer by about stop_short and every pose within stop_short of the end is
// clear. Expected travels: wall ahead 0.12 + 0.10 = 0.22 (0.03 -> 2 extra steps, 0.16); zero slack
// 0.08 + 0.10 = 0.18; slot 1.72 + 0.10 = 1.82.
TEST(EscapeSearchStopShort, wallAheadTravel) {
    Env e = wallAhead();
    EXPECT_NEAR(travel(expectPath(e, START, {}, 0.10)), 0.22, 1e-6);
    EXPECT_NEAR(travel(expectPath(e, START, {}, 0.03)), 0.16, 1e-6);
}

TEST(EscapeSearchStopShort, zeroSlackTravel) {
    Env e = wallAhead();
    EscapeConfig cfg;
    cfg.slack_cells = 0;
    EXPECT_NEAR(travel(expectPath(e, START, cfg, 0.10)), 0.18, 1e-6);
}

TEST(EscapeSearchStopShort, tightCorridorTravel) {
    Env e = tightCorridor();
    const Path p = expectPath(e, START, {}, 0.10);
    ASSERT_EQ(p.segments.size(), 1u);
    EXPECT_LT(p.segments[0].u.v, 0.0);
    EXPECT_NEAR(travel(p), 0.18, 1e-6);
}

TEST(EscapeSearchStopShort, slotTravel) {
    Env e = slot();
    EXPECT_NEAR(travel(expectPath(e, START, {}, 0.10)), 1.82, 1e-6);
}

TEST(EscapeSearchStopShort, unreachableReturnsNullopt) {
    for (Env e : {corridor(), closedBox(), slot(106)})
        EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, EscapeConfig{}, 0.10).has_value());
}

// floor(0.40 * 0.424264 / 0.02) = floor(8.49) = 8 steps; the move needs 11.
TEST(EscapeSearchStopShort, travelCapCountsExtraSteps) {
    Env e = wallAhead();
    EscapeConfig cfg;
    cfg.max_travel_factor = 0.40;
    ASSERT_EQ(static_cast<int>(std::floor(cfg.max_travel_factor * 0.3 * std::sqrt(2.0) / DELTA)), 8);
    EXPECT_FALSE(findEscape(START, *e.checker, *e.motion, e.k, cfg, 0.10).has_value());
}

TEST(EscapeSearchStopShort, poseNearEndStaysClear) {
    for (Env e : {wallAhead(), slot()}) {
        const Path p = expectPath(e, START, {}, 0.10);
        const double L = travel(p);
        for (int j = 0; j <= 5; ++j) {
            const SE2State q{START.x - (L - j * DELTA), START.y, START.theta};
            EXPECT_TRUE(e.checker->startClear(q, 0.0)) << "L=" << L << " j=" << j;
        }
        const SE2State q{START.x - (L - 0.10), START.y, START.theta};
        EXPECT_TRUE(e.checker->startClear(q, 0.05)) << "L=" << L;
    }
}

TEST(EscapeSearchStopShort, rejectsBadStopShort) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Env blocked = wallAhead();
    Env open = makeEnv(0, [](auto &) {});
    for (const Env * e : {&blocked, &open})
        for (double bad : {-0.01, nan, inf})
            EXPECT_THROW(findEscape(START, *e->checker, *e->motion, e->k, EscapeConfig{}, bad), std::invalid_argument)
                << "stop_short " << bad;
}

// ---- Maps whose origin is not (0, 0) -------------------------------------------------------
// The map content in cell indices is the same as the origin-(0, 0) cases; the grid origin and the
// start move together by T (exactly representable in float).

namespace {

const Eigen::Vector2f T_SHIFT(-1.5f, -4.0f);

Env shiftedWall(const Eigen::Vector2f & origin, bool decoy, const SE2State & start) {
    return makeEnv(0, [&](auto & c) {
        fillRect(c, 106, 115, 0, N - 1, 100);
        if (decoy) {
            // 12 x 12 occupied block centred on the cell an origin-ignoring lookup of `start` lands on.
            const int ci = static_cast<int>(std::floor(start.x / H));
            const int cj = static_cast<int>(std::floor(start.y / H));
            fillRect(c, ci - 6, ci + 5, cj - 6, cj + 5, 100);
        }
    }, origin);
}

void expectSameEscape(const Path & a, const Path & b, const Eigen::Vector2f & t) {
    ASSERT_EQ(a.segments.size(), b.segments.size());
    ASSERT_EQ(a.segments.size(), 1u);
    EXPECT_NEAR(a.segments[0].u.v, b.segments[0].u.v, 1e-9);
    EXPECT_NEAR(a.segments[0].u.omega, b.segments[0].u.omega, 1e-9);
    EXPECT_NEAR(a.segments[0].u.dt, b.segments[0].u.dt, 1e-9);
    const SE2State ea = a.node_points.back(), eb = b.node_points.back();
    EXPECT_NEAR(ea.x, eb.x + t.x(), 1e-9);
    EXPECT_NEAR(ea.y, eb.y + t.y(), 1e-9);
    EXPECT_NEAR(ea.theta, eb.theta, 1e-9);
}

SE2State shifted(const SE2State & s, const Eigen::Vector2f & t) { return {s.x + t.x(), s.y + t.y(), s.theta}; }

}  // namespace

// T1a. Case 2 on a shifted grid: same direction, duration and end pose (shifted).
TEST(EscapeSearchShiftedOrigin, wallAheadSameAsOrigin) {
    Env base = wallAhead();
    const Path pb = expectPath(base, START);
    const SE2State s = shifted(START, T_SHIFT);
    Env e = shiftedWall(T_SHIFT, false, s);
    const Path p = expectPath(e, s);
    expectSameEscape(p, pb, T_SHIFT);
}

// T1b. Case 3 (slack_cells = 0) on a shifted grid.
TEST(EscapeSearchShiftedOrigin, zeroSlackSameAsOrigin) {
    EscapeConfig cfg;
    cfg.slack_cells = 0;
    Env base = wallAhead();
    const Path pb = expectPath(base, START, cfg);
    const SE2State s = shifted(START, T_SHIFT);
    Env e = shiftedWall(T_SHIFT, false, s);
    const Path p = expectPath(e, s, cfg);
    expectSameEscape(p, pb, T_SHIFT);
}

// T2. A block of occupied cells exactly where an origin-ignoring lookup lands must change nothing.
TEST(EscapeSearchShiftedOrigin, decoyWhereOriginIgnoredLookupLands) {
    Env base = wallAhead();
    const Path pb = expectPath(base, START);
    const SE2State s = shifted(START, T_SHIFT);
    const int ci = static_cast<int>(std::floor(s.x / H));
    const int cj = static_cast<int>(std::floor(s.y / H));
    ASSERT_GE(ci - 6, 0); ASSERT_LT(ci + 5, N);
    ASSERT_GE(cj - 6, 0); ASSERT_LT(cj + 5, N);
    // Block centre in world coordinates (cells are indexed from the grid origin).
    const double bx = T_SHIFT.x() + (ci) * H, by = T_SHIFT.y() + cj * H;
    EXPECT_GT(std::hypot(bx - s.x, by - s.y), 3.0);
    Env e = shiftedWall(T_SHIFT, true, s);
    const Path p = expectPath(e, s);
    expectSameEscape(p, pb, T_SHIFT);
}

// T3. The motion context's grid metadata has another origin than the checker's grid: no effect.
TEST(EscapeSearchShiftedOrigin, motionContextOriginIsIgnored) {
    Env base = wallAhead();
    const Path pb = expectPath(base, START);
    Env e = makeEnv(0, [](auto & c) { fillRect(c, 106, 115, 0, N - 1, 100); },
                    Eigen::Vector2f::Zero(), Eigen::Vector2f(3.0f, -2.0f));
    const Path p = expectPath(e, START);
    expectSameEscape(p, pb, Eigen::Vector2f::Zero());
}

