#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

#include "core/navigation/navigation_checks.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/robot/robot_params.hpp"

namespace {

constexpr double H = 0.05;       // grid resolution (m)
constexpr int N = 200;           // map is N x N cells, origin (0, 0), 10 m square
constexpr double SPACING = 0.125;
constexpr double X0 = 2.0, Y0 = 5.0;
constexpr std::size_t COUNT = 11;

double sx(std::size_t i) { return X0 + SPACING * static_cast<double>(i); }  // exact in binary

// Samples along +x. A 0.6 m square at theta = 0 spans 0.6 m in x, so an obstacle touching one sample
// would touch its neighbours (0.125 m away) too. Sample `blocked` is therefore turned to pi/4, whose
// corner reaches 0.424 m above the centre, and one occupied cell sits at y in [5.35, 5.40) under that
// corner. All other samples keep theta = 0 and clear that cell (their top edge is 5.30).
struct Fixture {
    std::shared_ptr<const OccupancyGrid> grid;
    std::optional<CollisionChecker> checker;
    std::vector<SE2State> samples;
};

Fixture makeFixture(std::size_t blocked) {
    GridMetaData meta(N, N, H, Eigen::Vector2f::Zero());
    std::vector<int8_t> cells(static_cast<size_t>(N) * N, 0);
    const int col = static_cast<int>(std::floor(sx(blocked) / H));
    cells[static_cast<size_t>(107) * N + col] = 100;
    Fixture f;
    f.grid = std::make_shared<const OccupancyGrid>(meta, cells);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*f.grid, 5.0));
    auto body = buildRobotParams({{0.3, 0.3}, {-0.3, 0.3}, {-0.3, -0.3}, {0.3, -0.3}});
    f.checker.emplace(f.grid, edt, body);
    for (std::size_t i = 0; i < COUNT; ++i)
        f.samples.push_back({sx(i), Y0, i == blocked ? std::numbers::pi / 4 : 0.0});
    return f;
}

void expectOnlyBlocked(const Fixture & f, std::size_t blocked) {
    for (std::size_t i = 0; i < f.samples.size(); ++i)
        ASSERT_EQ(f.checker->checkState(f.samples[i]), i != blocked) << "sample " << i;
}

}  // namespace

TEST(CheckRemainingPath, blockedAheadReportsDistanceAlongPath) {
    Fixture f = makeFixture(7);
    expectOnlyBlocked(f, 7);
    auto r = checkRemainingPath(f.samples, sx(2), Y0, 0, *f.checker);
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.blocked_distance, 0.625);  // 5 * 0.125
    EXPECT_EQ(r.cursor, 2u);
}

TEST(CheckRemainingPath, pastTheBlockIsValid) {
    Fixture f = makeFixture(7);
    expectOnlyBlocked(f, 7);
    auto r = checkRemainingPath(f.samples, sx(9), Y0, 0, *f.checker);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.blocked_distance, 0.0);
    EXPECT_EQ(r.cursor, 9u);
}

TEST(CheckRemainingPath, cursorPrevPinsTheCursor) {
    Fixture f = makeFixture(7);
    expectOnlyBlocked(f, 7);
    auto r = checkRemainingPath(f.samples, sx(2), Y0, 5, *f.checker);
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.blocked_distance, 0.25);
    EXPECT_EQ(r.cursor, 5u);
}

TEST(CheckRemainingPath, robotOnBlockedSample) {
    Fixture f = makeFixture(7);
    expectOnlyBlocked(f, 7);
    auto r = checkRemainingPath(f.samples, sx(7), Y0, 0, *f.checker);
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.blocked_distance, 0.0);
    EXPECT_EQ(r.cursor, 7u);
}

TEST(CheckRemainingPath, tieTakesLowerIndex) {
    Fixture f = makeFixture(7);
    auto r = checkRemainingPath(f.samples, sx(8) + SPACING / 2, Y0, 0, *f.checker);
    EXPECT_EQ(r.cursor, 8u);
    EXPECT_TRUE(r.valid);
}

TEST(CheckRemainingPath, samplesBeforeCursorAreNotChecked) {
    Fixture f = makeFixture(0);
    expectOnlyBlocked(f, 0);
    auto r = checkRemainingPath(f.samples, sx(1), Y0, 1, *f.checker);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.cursor, 1u);
    // Same robot with cursor_prev 0: the cursor is 1 and sample 0 is still not checked.
    EXPECT_TRUE(checkRemainingPath(f.samples, sx(1), Y0, 0, *f.checker).valid);
    // Robot at sample 0 with cursor_prev 0: blocked cursor sample.
    EXPECT_FALSE(checkRemainingPath(f.samples, sx(0), Y0, 0, *f.checker).valid);
}

// Random robot positions and cursor_prev: the cursor never decreases, and validity is exactly
// "cursor is past the blocked sample".
TEST(CheckRemainingPath, cursorNeverDecreasesProperty) {
    Fixture f = makeFixture(7);
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> px(1.5, 4.0), py(4.5, 5.5);
    std::uniform_int_distribution<std::size_t> prev(0, COUNT - 1);
    for (int n = 0; n < 1000; ++n) {
        const std::size_t cp = prev(rng);
        const auto r = checkRemainingPath(f.samples, px(rng), py(rng), cp, *f.checker);
        ASSERT_GE(r.cursor, cp);
        ASSERT_LT(r.cursor, COUNT);
        ASSERT_EQ(r.valid, r.cursor > 7) << "cursor " << r.cursor;
        if (r.valid) ASSERT_EQ(r.blocked_distance, 0.0);
    }
}

TEST(CheckRemainingPath, emptyPath) {
    Fixture f = makeFixture(7);
    for (std::size_t cp : {0u, 3u, 100u}) {
        auto r = checkRemainingPath({}, 1.0, 1.0, cp, *f.checker);
        EXPECT_TRUE(r.valid);
        EXPECT_EQ(r.blocked_distance, 0.0);
        EXPECT_EQ(r.cursor, 0u);
    }
}

TEST(CheckRemainingPath, cursorPrevOutOfRangeThrows) {
    Fixture f = makeFixture(7);
    EXPECT_THROW(checkRemainingPath(f.samples, sx(2), Y0, f.samples.size(), *f.checker), std::invalid_argument);
}

TEST(AtRest, positionToleranceInclusive) {
    // hypot(0.003, 0.004) is not guaranteed to equal the double 0.005, so the tolerance is the computed value.
    const double tol = std::hypot(0.003, 0.004);
    EXPECT_TRUE(atRest({0, 0, 0}, {0.003, 0.004, 0}, tol, 0.0118));
    EXPECT_FALSE(atRest({0, 0, 0}, {0.004, 0.004, 0}, tol, 0.0118));
}

TEST(AtRest, headingWrapsAcrossPi) {
    EXPECT_TRUE(atRest({0, 0, 3.14}, {0, 0, -3.14}, 0.005, 0.0118));  // wrapped difference 0.00319
    EXPECT_FALSE(atRest({0, 0, 0.0}, {0, 0, 0.02}, 0.005, 0.0118));
}

TEST(AtRest, rejectsBadTolerance) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {-0.001, nan, inf}) {
        EXPECT_THROW(atRest({0, 0, 0}, {0, 0, 0}, bad, 0.01), std::invalid_argument) << bad;
        EXPECT_THROW(atRest({0, 0, 0}, {0, 0, 0}, 0.01, bad), std::invalid_argument) << bad;
    }
}
