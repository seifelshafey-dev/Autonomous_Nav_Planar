#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <numbers>
#include <Eigen/Core>

#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/map/grid.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"

/*
DiffDriveStateSpace tests.

propagate: exact unicycle integration. Straight-line when omega == 0 (forward by v*dt along the
heading); otherwise an arc of signed radius R = v/omega swept by omega*dt, with the body-frame
displacement rotated into the world by the start heading. Expected endpoints are hand-computed.

inBounds: strictly-interior AABB membership; bounds = [origin, origin + (cols, rows)*resolution].
A state on an edge is out of bounds.

distance/interpolate are covered elsewhere (distance delegates to the injected functor, exercised by
distance_functions and kinematic_model tests).
*/

namespace {
constexpr double kPi = std::numbers::pi;

DiffDriveStateSpace makeSpace(const GridMetaData & grid){
    DiffDriveParams kin;
    kin.rmin = 2.0; kin.vmax = 1.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    return DiffDriveStateSpace(grid, dist);
}

// A 10x10 m workspace anchored at the origin: bounds [0,10] x [0,10].
DiffDriveStateSpace unitSpace(){ return makeSpace(GridMetaData{10, 10, 1.0, Eigen::Vector2f::Zero()}); }
}

// ---- propagate ----

TEST(DiffDrivePropagate, zeroControlLeavesStateUnchanged){
    DiffDriveStateSpace s = unitSpace();
    SE2State a{0.5, -0.3, 0.4};
    SE2State r = s.propagate(a, Control{0.0, 0.0, 1.0});
    EXPECT_DOUBLE_EQ(r.x, 0.5);
    EXPECT_DOUBLE_EQ(r.y, -0.3);
    EXPECT_DOUBLE_EQ(r.theta, 0.4);
}

TEST(DiffDrivePropagate, straightLineAlongPositiveX){
    DiffDriveStateSpace s = unitSpace();
    SE2State r = s.propagate(SE2State{0.0, 0.0, 0.0}, Control{2.0, 0.0, 1.0});
    EXPECT_DOUBLE_EQ(r.x, 2.0);
    EXPECT_DOUBLE_EQ(r.y, 0.0);
    EXPECT_DOUBLE_EQ(r.theta, 0.0);
}

TEST(DiffDrivePropagate, straightLineFacingPositiveYMovesAlongY){
    // Heading pi/2: a forward body displacement of 2 maps to world +y.
    DiffDriveStateSpace s = unitSpace();
    SE2State r = s.propagate(SE2State{0.0, 0.0, kPi / 2.0}, Control{2.0, 0.0, 1.0});
    EXPECT_NEAR(r.x, 0.0, 1e-9);
    EXPECT_NEAR(r.y, 2.0, 1e-9);
    EXPECT_NEAR(r.theta, kPi / 2.0, 1e-9);
}

TEST(DiffDrivePropagate, pureRotationInPlace){
    // v = 0 -> R = 0 -> no translation; heading advances by omega*dt.
    DiffDriveStateSpace s = unitSpace();
    SE2State r = s.propagate(SE2State{1.0, 2.0, 0.0}, Control{0.0, kPi / 2.0, 1.0});
    EXPECT_NEAR(r.x, 1.0, 1e-9);
    EXPECT_NEAR(r.y, 2.0, 1e-9);
    EXPECT_NEAR(r.theta, kPi / 2.0, 1e-9);
}

TEST(DiffDrivePropagate, quarterCircleLeftTurn){
    // R = (pi/2)/(pi/2) = 1, swept pi/2 CCW from origin facing +x -> (1, 1) facing +y.
    DiffDriveStateSpace s = unitSpace();
    SE2State r = s.propagate(SE2State{0.0, 0.0, 0.0}, Control{kPi / 2.0, kPi / 2.0, 1.0});
    EXPECT_NEAR(r.x, 1.0, 1e-9);
    EXPECT_NEAR(r.y, 1.0, 1e-9);
    EXPECT_NEAR(r.theta, kPi / 2.0, 1e-9);
}

TEST(DiffDrivePropagate, quarterCircleRightTurnMirrorsLeft){
    // Negative omega: R = -1, swept -pi/2 -> (1, -1) facing -pi/2.
    DiffDriveStateSpace s = unitSpace();
    SE2State r = s.propagate(SE2State{0.0, 0.0, 0.0}, Control{kPi / 2.0, -kPi / 2.0, 1.0});
    EXPECT_NEAR(r.x, 1.0, 1e-9);
    EXPECT_NEAR(r.y, -1.0, 1e-9);
    EXPECT_NEAR(r.theta, -kPi / 2.0, 1e-9);
}

// ---- inBounds (strictly interior) ----

TEST(DiffDriveInBounds, interiorStateIsInBounds){
    DiffDriveStateSpace s = unitSpace();
    EXPECT_TRUE(s.inBounds(SE2State{5.0, 5.0, 0.0}));
    EXPECT_TRUE(s.inBounds(SE2State{0.001, 9.999, 3.0}));   // heading does not matter
}

TEST(DiffDriveInBounds, statesOutsideEachSideAreRejected){
    DiffDriveStateSpace s = unitSpace();
    EXPECT_FALSE(s.inBounds(SE2State{-0.1, 5.0, 0.0}));
    EXPECT_FALSE(s.inBounds(SE2State{10.1, 5.0, 0.0}));
    EXPECT_FALSE(s.inBounds(SE2State{5.0, -0.1, 0.0}));
    EXPECT_FALSE(s.inBounds(SE2State{5.0, 10.1, 0.0}));
}

TEST(DiffDriveInBounds, edgesAreOutOfBounds){
    DiffDriveStateSpace s = unitSpace();
    EXPECT_FALSE(s.inBounds(SE2State{0.0, 5.0, 0.0}));    // left edge
    EXPECT_FALSE(s.inBounds(SE2State{10.0, 5.0, 0.0}));   // right edge
    EXPECT_FALSE(s.inBounds(SE2State{5.0, 0.0, 0.0}));    // bottom edge
    EXPECT_FALSE(s.inBounds(SE2State{5.0, 10.0, 0.0}));   // top edge
    EXPECT_FALSE(s.inBounds(SE2State{0.0, 0.0, 0.0}));    // corner
}

TEST(DiffDriveInBounds, boundsAreOffsetByOrigin){
    // origin (2, 3), 4x5 cells at res 1.0 -> bounds (2,6) x (3,8), exclusive.
    DiffDriveStateSpace s = makeSpace(GridMetaData{4, 5, 1.0, Eigen::Vector2f(2.0f, 3.0f)});
    EXPECT_TRUE(s.inBounds(SE2State{4.0, 5.0, 0.0}));
    EXPECT_FALSE(s.inBounds(SE2State{1.5, 5.0, 0.0}));   // left of origin
    EXPECT_FALSE(s.inBounds(SE2State{2.0, 5.0, 0.0}));   // on the origin (left edge)
    EXPECT_FALSE(s.inBounds(SE2State{6.0, 5.0, 0.0}));   // right edge
    EXPECT_FALSE(s.inBounds(SE2State{4.0, 8.0, 0.0}));   // top edge
}

// ---- interpolate (linear position, shortest-arc heading) ----

TEST(DiffDriveInterpolate, headingTakesShortArcThroughNegativePi){
    // -3.0 to +3.0: the short arc (~0.283 rad) crosses -pi, not the 6 rad long way through 0.
    DiffDriveStateSpace s = unitSpace();
    SE2State m = s.interpolate(SE2State{0.0, 0.0, -3.0}, SE2State{0.0, 0.0, 3.0}, 0.5);
    EXPECT_NEAR(m.theta, -kPi, 1e-9);
    EXPECT_DOUBLE_EQ(m.x, 0.0);
    EXPECT_DOUBLE_EQ(m.y, 0.0);
}

TEST(DiffDriveInterpolate, headingTakesShortArcThroughPositivePi){
    // Reverse direction: +3.0 to -3.0 crosses +pi.
    DiffDriveStateSpace s = unitSpace();
    SE2State m = s.interpolate(SE2State{0.0, 0.0, 3.0}, SE2State{0.0, 0.0, -3.0}, 0.5);
    EXPECT_NEAR(m.theta, kPi, 1e-9);
}
