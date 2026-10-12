#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/control/stop_zone.hpp"
#include "core/robot/robot_params.hpp"

/*
motionBlocked example tests. Robot: square, vertices (+-0.3, +-0.3), r_out = 0.424264.
Params: a = 1.5, alpha = 2.5, p_f = 0.02, p_s = 0.01, reaction time 0.2 s. Expected values are the
acceptance table of the stop-zone design (points in the body frame, metres).
*/

namespace {

const RobotParams & robot() {
    static const RobotParams r = buildRobotParams({Vec2(0.3, 0.3), Vec2(-0.3, 0.3), Vec2(-0.3, -0.3), Vec2(0.3, -0.3)});
    return r;
}

const StopZoneParams kP{1.5, 2.5, 0.02, 0.01};
constexpr double kTr = 0.2;

bool blocked(double px, double py, double v, double w, double tr = kTr, StopZoneParams p = kP) {
    return motionBlocked(robot(), {Vec2(px, py)}, v, w, tr, p);
}

}  // namespace

TEST(StopZone, forwardStraight) {
    EXPECT_TRUE(blocked(0.50, 0.0, 0.5, 0.0));
    EXPECT_FALSE(blocked(0.52, 0.0, 0.5, 0.0));
    EXPECT_TRUE(blocked(0.20, 0.305, 0.5, 0.0));
    EXPECT_FALSE(blocked(0.20, 0.32, 0.5, 0.0));
    EXPECT_FALSE(blocked(-0.35, 0.0, 0.5, 0.0));
}

TEST(StopZone, slowerForwardShorterZone) {
    EXPECT_TRUE(blocked(0.39, 0.0, 0.25, 0.0));
    EXPECT_FALSE(blocked(0.41, 0.0, 0.25, 0.0));
}

TEST(StopZone, reverse) {
    EXPECT_TRUE(blocked(-0.50, 0.0, -0.5, 0.0));
    EXPECT_FALSE(blocked(0.35, 0.0, -0.5, 0.0));
}

TEST(StopZone, pivot) {
    EXPECT_TRUE(blocked(0.33, 0.0, 0.0, 1.0));
    EXPECT_FALSE(blocked(0.36, 0.0, 0.0, 1.0));
    EXPECT_FALSE(blocked(0.33, 0.0, 0.0, 0.3));
}

TEST(StopZone, nearStraightArcStaysShort) {
    EXPECT_TRUE(blocked(0.50, 0.0, 0.5, 0.016));
    EXPECT_FALSE(blocked(0.53, 0.0, 0.5, 0.016));
    EXPECT_FALSE(blocked(1.0, 0.0, 0.5, 0.016));
    EXPECT_TRUE(blocked(0.39, 0.0, 0.25, 0.01));
    EXPECT_FALSE(blocked(0.42, 0.0, 0.25, 0.01));
    EXPECT_FALSE(blocked(0.60, 0.0, 0.25, 0.01));
}

TEST(StopZone, nearPivotWithTinyLinearRate) {
    EXPECT_TRUE(blocked(0.33, 0.0, 0.001, 1.0));
    EXPECT_FALSE(blocked(0.36, 0.0, 0.001, 1.0));
}

TEST(StopZone, zeroCommandIsClear) {
    EXPECT_FALSE(blocked(0.0, 0.0, 0.0, 0.0));
    EXPECT_FALSE(blocked(0.31, 0.0, 0.0, 0.0));
}

TEST(StopZone, emptyPointsIsClear) {
    EXPECT_FALSE(motionBlocked(robot(), {}, 0.5, 0.0, kTr, kP));
}

TEST(StopZone, straightBoundaryInclusive) {
    const double front = 0.3 + (0.5 * 0.2 + 0.25 / 3.0) + 0.02 + 0.01;
    EXPECT_TRUE(blocked(front - 1e-6, 0.0, 0.5, 0.0));
    EXPECT_FALSE(blocked(front + 1e-6, 0.0, 0.5, 0.0));
}

// Front limit = 0.30 + v t_r + v^2/(2a) + forward_pad + side_pad, probed 2 mm either side.
TEST(StopZone, zeroReactionTimeFrontLimit) {
    EXPECT_TRUE(blocked(0.41333 - 0.002, 0.0, 0.5, 0.0, 0.0));
    EXPECT_FALSE(blocked(0.41333 + 0.002, 0.0, 0.5, 0.0, 0.0));
}

TEST(StopZone, zeroForwardPadFrontLimit) {
    const StopZoneParams p{1.5, 2.5, 0.0, 0.01};
    EXPECT_TRUE(blocked(0.49333 - 0.002, 0.0, 0.5, 0.0, kTr, p));
    EXPECT_FALSE(blocked(0.49333 + 0.002, 0.0, 0.5, 0.0, kTr, p));
}

TEST(StopZone, nonFinitePointNeverBlocks) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(blocked(nan, 0.0, 0.5, 0.0));
}

TEST(StopZone, rejectsBadCommandOrReactionTime) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(blocked(0.5, 0, nan, 0.0), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, inf), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, -0.1), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, nan), std::invalid_argument);
}

TEST(StopZone, rejectsBadParams) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto with = [](double StopZoneParams::*f, double val) { StopZoneParams p = kP; p.*f = val; return p; };
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, kTr, with(&StopZoneParams::linear_decel, 0.0)), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, kTr, with(&StopZoneParams::angular_decel, -1.0)), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, kTr, with(&StopZoneParams::forward_pad, -0.01)), std::invalid_argument);
    EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, kTr, with(&StopZoneParams::side_pad, 0.0)), std::invalid_argument);
    for (auto f : {&StopZoneParams::linear_decel, &StopZoneParams::angular_decel,
                   &StopZoneParams::forward_pad, &StopZoneParams::side_pad})
        EXPECT_THROW(blocked(0.5, 0, 0.5, 0.0, kTr, with(f, nan)), std::invalid_argument);
}

TEST(StopZone, validatesEvenWhenStationary) {
    EXPECT_THROW(blocked(0.0, 0.0, 0.0, 0.0, kTr, StopZoneParams{0.0, 2.5, 0.02, 0.01}), std::invalid_argument);
}
