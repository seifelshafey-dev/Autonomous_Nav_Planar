#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <memory>
#include <Eigen/Core>

#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/map/grid.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/math/wrap_angle.hpp"

/*
Property tests for DiffDriveStateSpace::propagate. These check structural properties of the rigid-body
kinematics rather than re-deriving the sin/cos endpoint formula:

1. roundTrip: applying {v,omega,dt} then {-v,-omega,dt} returns to the start (the reverse control
   retraces the arc). Holds exactly by the SE(2) group structure.
2. translationEquivariance: the body-frame displacement depends only on heading and control, so
   translating the start translates the result by the same vector.
3. rotationEquivariance: rotating the start heading by phi rotates the world displacement by phi and
   adds phi to the result heading.
*/

namespace {
DiffDriveStateSpace makeSpace(){
    DiffDriveParams kin;
    kin.rmin = 2.0; kin.vmax = 1.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    return DiffDriveStateSpace(GridMetaData{1, 1, 1.0, Eigen::Vector2f::Zero()}, dist);
}
double coord(){ return *rc::gen::inRange(-100, 101) / 10.0; }
double theta(){ return *rc::gen::inRange(-31415, 31416) / 10000.0; }
Control ctrl(){
    return Control{*rc::gen::inRange(-30, 31) / 10.0,   // v     in [-3.0, 3.0]
                   *rc::gen::inRange(-30, 31) / 10.0,    // omega in [-3.0, 3.0] (includes 0)
                   *rc::gen::inRange(1, 21) / 10.0};     // dt    in (0, 2.0]
}
}

RC_GTEST_PROP(DiffDrivePropagateProp, reverseControlReturnsToStart, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    Control u = ctrl();

    SE2State fwd  = s.propagate(a, u);
    SE2State back = s.propagate(fwd, Control{-u.v, -u.omega, u.dt});

    RC_ASSERT(std::abs(back.x - a.x) < 1e-9);
    RC_ASSERT(std::abs(back.y - a.y) < 1e-9);
    RC_ASSERT(std::abs(back.theta - a.theta) < 1e-9);
}

RC_GTEST_PROP(DiffDrivePropagateProp, translationEquivariance, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    Control u = ctrl();
    double dx = coord(), dy = coord();

    SE2State r1 = s.propagate(a, u);
    SE2State r2 = s.propagate(SE2State{a.x + dx, a.y + dy, a.theta}, u);

    RC_ASSERT(std::abs(r2.x - (r1.x + dx)) < 1e-9);
    RC_ASSERT(std::abs(r2.y - (r1.y + dy)) < 1e-9);
    RC_ASSERT(std::abs(r2.theta - r1.theta) < 1e-9);
}

RC_GTEST_PROP(DiffDrivePropagateProp, rotationEquivariance, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    Control u = ctrl();
    double phi = theta();

    SE2State r1 = s.propagate(a, u);
    SE2State r2 = s.propagate(SE2State{a.x, a.y, a.theta + phi}, u);

    // Displacement from the (shared) start position rotates by phi.
    Vec2 d1(r1.x - a.x, r1.y - a.y);
    Vec2 d2(r2.x - a.x, r2.y - a.y);
    Vec2 rotated = Rot2(phi) * d1;

    RC_ASSERT((d2 - rotated).norm() < 1e-9);
    RC_ASSERT(std::abs(r2.theta - (r1.theta + phi)) < 1e-9);
}

/*
interpolate properties: position is a linear blend; heading follows the shortest arc. Endpoints and
the shortest-arc semantics are checked independently of the closed-form expression. Angle comparisons
go through wrapToPi so the +-pi seam (where -pi and +pi denote the same heading) is not a false miss.
*/

RC_GTEST_PROP(InterpolateProp, endpointsRecoverInputs, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    SE2State b{coord(), coord(), theta()};

    SE2State at0 = s.interpolate(a, b, 0.0);
    SE2State at1 = s.interpolate(a, b, 1.0);

    RC_ASSERT(at0.x == a.x);
    RC_ASSERT(at0.y == a.y);
    RC_ASSERT(std::abs(wrapToPi(at0.theta - a.theta)) < 1e-9);
    RC_ASSERT(at1.x == b.x);
    RC_ASSERT(at1.y == b.y);
    RC_ASSERT(std::abs(wrapToPi(at1.theta - b.theta)) < 1e-9);
}

RC_GTEST_PROP(InterpolateProp, headingAdvancesByFractionOfShortestArc, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    SE2State b{coord(), coord(), theta()};
    double t = *rc::gen::inRange(0, 1001) / 1000.0;

    SE2State m = s.interpolate(a, b, t);
    // The heading offset from a is exactly fraction t of the shortest arc a->b (compared mod 2pi).
    double offset    = wrapToPi(m.theta - a.theta);
    double shortArc  = wrapToPi(b.theta - a.theta);
    RC_ASSERT(std::abs(wrapToPi(offset - t * shortArc)) < 1e-9);
}

RC_GTEST_PROP(InterpolateProp, symmetricUnderEndpointSwap, ()){
    DiffDriveStateSpace s = makeSpace();
    SE2State a{coord(), coord(), theta()};
    SE2State b{coord(), coord(), theta()};
    double t = *rc::gen::inRange(0, 1001) / 1000.0;

    SE2State ab = s.interpolate(a, b, t);
    SE2State ba = s.interpolate(b, a, 1.0 - t);

    RC_ASSERT(std::abs(ab.x - ba.x) < 1e-9);
    RC_ASSERT(std::abs(ab.y - ba.y) < 1e-9);
    RC_ASSERT(std::abs(wrapToPi(ab.theta - ba.theta)) < 1e-9);
}
