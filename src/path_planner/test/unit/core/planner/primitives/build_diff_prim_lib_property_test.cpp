#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cstdint>
#include <map>
#include <memory>
#include <numbers>
#include <utility>
#include <Eigen/Core>

#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"

/*
Properties for headingBin, varying n_headings in [1, 24] so nothing is hardwired to 8:

1. alwaysInRange: for any finite theta (swept across +-4pi), 0 <= headingBin(theta) < n_headings.
   Catches a seam off-by-one and a missing internal wrap (large theta would otherwise escape range).

2. invertsBinAnchoring: for k in [0,n) and an interior offset frac in [0.01,0.99], the angle
   theta = -pi + (k+frac)*bin_size maps back to bin k. headingBin must invert the same anchoring
   (theta_k = -pi + k*bin_size) that buildPrimitive uses, or cached cells get looked up in the wrong
   bin. Oracle = interval membership, independent of the floor expression. Interior frac keeps the
   sample off the float-fragile bin edges (seams are covered by property 1 + deterministic tests).

3. is2PiPeriodic: headingBin(theta) == headingBin(theta +- m*2pi). Pins the internal wrapToPi; the
   interior margin dwarfs the float error introduced by the 2pi shifts.

headingBin depends only on n_headings, but is a member of a fully-built library; libraries are memoized
by n so at most 24 are ever constructed across all cases.
*/

namespace {

DiffDriveParams makeKinN(int n){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.4; kin.n_headings = n;
    return kin;
}

DiffDrivePrimLib makeLib(const DiffDriveParams & kin){
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    auto space = std::make_shared<const DiffDriveStateSpace>(
        GridMetaData{100, 100, 0.1, Eigen::Vector2f::Zero()}, dist);
    RobotParams body = buildRobotParams(
        {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
    MotionContext motion{std::move(body), std::move(space),
                         GridMetaData{100, 100, 0.1, Eigen::Vector2f::Zero()}};
    return DiffDrivePrimLib{std::move(motion), kin};
}

const DiffDrivePrimLib & libForN(int n){
    static std::map<int, DiffDrivePrimLib> cache;
    auto it = cache.find(n);
    if (it == cache.end()) it = cache.emplace(n, makeLib(makeKinN(n))).first;
    return it->second;
}

}  // namespace

RC_GTEST_PROP(HeadingBinProp, alwaysInRange, ()){
    const int n = *rc::gen::inRange(1, 25);
    const double theta = *rc::gen::inRange(-12566, 12567) / 1000.0;  // ~ +-4pi
    int16_t b = libForN(n).headingBin(theta);
    RC_ASSERT(b >= 0);
    RC_ASSERT(b < n);
}

RC_GTEST_PROP(HeadingBinProp, invertsBinAnchoring, ()){
    const int n = *rc::gen::inRange(1, 25);
    const int k = *rc::gen::inRange(0, n);
    const double frac = *rc::gen::inRange(1, 100) / 100.0;  // 0.01 .. 0.99 (interior)
    const double bin_size = (2.0 * std::numbers::pi) / n;
    const double theta = -std::numbers::pi + (k + frac) * bin_size;
    RC_ASSERT(libForN(n).headingBin(theta) == static_cast<int16_t>(k));
}

RC_GTEST_PROP(HeadingBinProp, is2PiPeriodic, ()){
    const int n = *rc::gen::inRange(1, 25);
    const int k = *rc::gen::inRange(0, n);
    const double frac = *rc::gen::inRange(1, 100) / 100.0;
    const double bin_size = (2.0 * std::numbers::pi) / n;
    const double theta = -std::numbers::pi + (k + frac) * bin_size;
    const int m = *rc::gen::inRange(-3, 4);  // -3 .. 3
    const double two_pi = 2.0 * std::numbers::pi;

    int16_t base = libForN(n).headingBin(theta);
    RC_ASSERT(libForN(n).headingBin(theta + m * two_pi) == base);
}
