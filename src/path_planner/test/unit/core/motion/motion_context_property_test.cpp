#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Core>

#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/map/grid.hpp"

/*
Property test for MotionContext::sampleTrajectory against the REAL differential-drive kinematics --
the one place real-collaborator integration earns its keep, because the soundness contract (footprints
of consecutive samples overlap) is only meaningful with the true motion model.

Contract: the trajectory is sampled densely enough that between consecutive poses the translation is
at most r_in/2 and the heading change is at most r_in/(2*r_out). With those bounds, each pose's
inflated footprint overlaps its neighbour's, so no traversed cell can be skipped.
*/

namespace {
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}

// Body-frame MotionContext built directly (no primitive library -> cheap to construct per case).
MotionContext bodyFrameContext(){
    DiffDriveParams kin;
    kin.rmin = 2.0; kin.vmax = 1.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    GridMetaData meta{1, 1, 0.1, Eigen::Vector2f::Zero()};
    auto space = std::make_shared<const DiffDriveStateSpace>(meta, dist);
    return MotionContext(buildRobotParams(ccwRect()), space, meta);
}
}

RC_GTEST_PROP(MotionContextProp, consecutiveSamplesRespectOverlapBounds, ()){
    MotionContext mc = bodyFrameContext();
    const RobotParams & body = mc.body();
    const double trans_bound = body.r_in / 2.0;
    const double rot_bound   = body.r_in / (2.0 * body.r_out);

    SE2State start{*rc::gen::inRange(-50, 51) / 10.0,
                   *rc::gen::inRange(-50, 51) / 10.0,
                   *rc::gen::inRange(-31415, 31416) / 10000.0};
    Control u{*rc::gen::inRange(-20, 21) / 10.0,   // v   in [-2.0, 2.0]
              *rc::gen::inRange(-30, 31) / 10.0,   // omega in [-3.0, 3.0]
              *rc::gen::inRange(1, 21) / 10.0};    // dt  in (0, 2.0]

    std::vector<SE2State> poses = mc.sampleTrajectory(start, u);
    RC_ASSERT(poses.size() >= 2u);

    for (std::size_t i = 1; i < poses.size(); ++i){
        double dx = poses[i].x - poses[i - 1].x;
        double dy = poses[i].y - poses[i - 1].y;
        double chord = std::sqrt(dx * dx + dy * dy);
        double dtheta = std::abs(wrapToPi(poses[i].theta - poses[i - 1].theta));
        RC_ASSERT(chord  <= trans_bound + 1e-9);
        RC_ASSERT(dtheta <= rot_bound   + 1e-9);
    }
}
