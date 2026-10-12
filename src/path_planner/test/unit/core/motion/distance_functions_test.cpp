#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

#include "core/motion/distance_functions.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/robot/diff_drive_parameters.hpp"

/*
distance_functions contract:

PointsWeightedEuclideanSE2(w)(a, b) = sqrt(dx^2 + dy^2 + w * wrapToPi(b.theta - a.theta)^2)
    A weighted SE(2) metric: Euclidean on (x, y) combined with the geodesic angular distance,
    scaled by w. w is the squared rotational weight (canonically rmin^2 in this metric's units).

VelWeightedEuclideanSE2(w)(Control{v, omega, dt}) = sqrt(v^2 + omega^2 * w^2) * dt
    Weighted arc length of one control application; w is the rotational radius weight (canonically rmin).

makeControlCost(kin) = VelWeightedEuclideanSE2{kin.rmin}

Preconditions are caller-owned: w > 0 (derived from diff-drive rmin) and states/controls are finite,
so no input-validation tests. Expected values below are computed by hand.
*/

namespace {
constexpr double kPi = std::numbers::pi;
}

// ---- PointsWeightedEuclideanSE2 ----

TEST(PointsWeightedEuclidean, identicalPosesAreZeroDistance){
    PointsWeightedEuclideanSE2 d(4.0);
    EXPECT_DOUBLE_EQ(d(SE2State{1.0, 2.0, 0.5}, SE2State{1.0, 2.0, 0.5}), 0.0);
}

TEST(PointsWeightedEuclidean, pureTranslationIsPlainEuclidean){
    // Equal headings -> angular term vanishes regardless of weight; 3-4-5 triangle.
    PointsWeightedEuclideanSE2 d(7.5);
    EXPECT_DOUBLE_EQ(d(SE2State{0.0, 0.0, 0.3}, SE2State{3.0, 4.0, 0.3}), 5.0);
}

TEST(PointsWeightedEuclidean, pureRotationIsWeightedAngle){
    // No translation -> distance = sqrt(w) * |dtheta|. w=4, dtheta=pi/2 -> 2 * pi/2 = pi.
    PointsWeightedEuclideanSE2 d(4.0);
    EXPECT_NEAR(d(SE2State{1.0, 1.0, 0.0}, SE2State{1.0, 1.0, kPi / 2.0}), kPi, 1e-9);
}

TEST(PointsWeightedEuclidean, combinedTranslationAndRotation){
    // dx=3, dy=4, dtheta=pi/2, w=4 -> sqrt(25 + 4 * (pi/2)^2) = sqrt(25 + pi^2).
    PointsWeightedEuclideanSE2 d(4.0);
    EXPECT_NEAR(d(SE2State{1.0, 1.0, 0.0}, SE2State{4.0, 5.0, kPi / 2.0}),
                std::sqrt(25.0 + kPi * kPi), 1e-9);
}

TEST(PointsWeightedEuclidean, angularDeltaIsWrappedToShortestArc){
    // theta -3.0 -> 3.0 is a 6.0 raw delta, but the shortest arc is 6.0 - 2*pi ~ -0.2832.
    // Both headings are valid (within [-pi, pi]). Distance must use the wrapped delta, not 6.0.
    PointsWeightedEuclideanSE2 d(1.0);
    double wrapped = 6.0 - 2.0 * kPi;
    EXPECT_NEAR(d(SE2State{0.0, 0.0, -3.0}, SE2State{0.0, 0.0, 3.0}), std::abs(wrapped), 1e-9);
}

// ---- VelWeightedEuclideanSE2 ----

TEST(VelWeightedEuclidean, pureTranslationControl){
    // omega = 0 -> cost = |v| * dt, independent of weight.
    VelWeightedEuclideanSE2 c(2.0);
    EXPECT_DOUBLE_EQ(c(Control{3.0, 0.0, 1.0}), 3.0);
}

TEST(VelWeightedEuclidean, pureRotationControlUsesWeight){
    // v = 0 -> cost = |omega| * w * dt. w=2, omega=2 -> sqrt(4*4) = 4.
    VelWeightedEuclideanSE2 c(2.0);
    EXPECT_DOUBLE_EQ(c(Control{0.0, 2.0, 1.0}), 4.0);
}

TEST(VelWeightedEuclidean, combinedControlIsWeightedArcLength){
    // w=1, v=3, omega=4 -> sqrt(9 + 16) = 5.
    VelWeightedEuclideanSE2 c(1.0);
    EXPECT_DOUBLE_EQ(c(Control{3.0, 4.0, 1.0}), 5.0);
}

TEST(VelWeightedEuclidean, costScalesLinearlyWithDt){
    VelWeightedEuclideanSE2 c(1.0);
    EXPECT_DOUBLE_EQ(c(Control{3.0, 4.0, 2.0}), 10.0); // 5 * dt(=2)
}

// ---- makeControlCost ----

TEST(MakeControlCost, setsRotationalWeightToRmin){
    DiffDriveParams kin;
    kin.rmin = 2.0;
    VelWeightedEuclideanSE2 c = makeControlCost(kin);
    // v=0, omega=3, w=rmin=2 -> sqrt(9 * 4) = 6.
    EXPECT_DOUBLE_EQ(c(Control{0.0, 3.0, 1.0}), 6.0);
}
