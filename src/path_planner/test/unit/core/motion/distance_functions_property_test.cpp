#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>

#include "core/motion/distance_functions.hpp"
#include "core/types/state.hpp"

/*
Property-based tests for PointsWeightedEuclideanSE2 (RapidCheck). These verify the SE(2)-metric
axioms over random valid poses and positive weights -- independent of the closed-form expression.

1. symmetry: d(a,b) == d(b,a). Exact: dx/dy negate-then-square is bit-exact, and wrapToPi is odd so
   the squared angular term is identical -- the argument to sqrt is bit-identical either direction.
2. triangleInequality: d(a,c) <= d(a,b) + d(b,c). Holds because d is the l2-combination of the
   Euclidean metric on (x,y) and the geodesic metric on the heading circle.
3. identityAndNonNegativity: d(a,a) == 0 and d(a,b) >= 0 for all poses.

The control-cost functors and makeControlCost are exercised by the deterministic suite; their metric
axioms do not apply (a single Control input, not a pose pair).
*/

namespace {
double coord(){ return *rc::gen::inRange(-100, 101) / 10.0; }        // -10.0 .. 10.0
double theta(){ return *rc::gen::inRange(-31415, 31416) / 10000.0; } // ~ -pi .. pi
double posWeight(){ return *rc::gen::inRange(1, 1001) / 100.0; }     // 0.01 .. 10.0
SE2State pose(){ return SE2State{coord(), coord(), theta()}; }
}

RC_GTEST_PROP(PointsWeightedProp, symmetry, ()){
    PointsWeightedEuclideanSE2 d(posWeight());
    SE2State a = pose(), b = pose();
    RC_ASSERT(d(a, b) == d(b, a));
}

RC_GTEST_PROP(PointsWeightedProp, triangleInequality, ()){
    PointsWeightedEuclideanSE2 d(posWeight());
    SE2State a = pose(), b = pose(), c = pose();
    RC_ASSERT(d(a, c) <= d(a, b) + d(b, c) + 1e-9);
}

RC_GTEST_PROP(PointsWeightedProp, identityAndNonNegativity, ()){
    PointsWeightedEuclideanSE2 d(posWeight());
    SE2State a = pose(), b = pose();
    RC_ASSERT(d(a, a) == 0.0);
    RC_ASSERT(d(a, b) >= 0.0);
}
