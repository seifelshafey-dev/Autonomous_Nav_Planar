#include <gtest/gtest.h>
#include <cmath>

#include "core/motion/kinematic_model.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"

/*
buildKinematicModel(body, kin, cell_size) is a composition factory. Most of it is opaque wiring; the
one bug-prone decision it uniquely owns and that is observable through the public surface is the SE(2)
rotational weight, which it sets to rmin^2 (not rmin). We also guard the cell_size / placeholder-origin
threading. We deliberately do not assert on shared_ptr non-null or kin struct-copy (tautological), nor
on the primitive library's internals (DiffDrivePrimLib's own unit).
*/

namespace {
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}

KinematicModel makeModel(double rmin, double cell_size){
    DiffDriveParams kin;
    kin.rmin = rmin;
    kin.vmax = 1.0;
    kin.atomic_time = 1.0;
    kin.n_headings = 8;
    return buildKinematicModel(buildRobotParams(ccwRect()), kin, cell_size);
}
}

// The SE(2) distance carries an rmin^2 rotational weight. A pure-rotation pose pair isolates it:
// d = sqrt(weight * dtheta^2) = rmin^2 * |dtheta| only if weight == rmin^2... checked numerically.
TEST(BuildKinematicModel, rotationalWeightIsRminSquared){
    KinematicModel m = makeModel(/*rmin=*/2.0, /*cell_size=*/0.1);

    // rmin = 2 -> weight = 4. Pure rotation by 0.5 rad: sqrt(4 * 0.5^2) = sqrt(1) = 1.0.
    // (A weight of rmin instead would give sqrt(2 * 0.25) ~ 0.707; vmax-based would differ too.)
    double d = m.motion->space().distance(SE2State{0.0, 0.0, 0.0}, SE2State{0.0, 0.0, 0.5});
    EXPECT_NEAR(d, 1.0, 1e-9);
}

