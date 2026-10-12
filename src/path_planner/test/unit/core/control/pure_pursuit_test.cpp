#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

#include "core/control/pure_pursuit.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/map/grid.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/math/wrap_angle.hpp"

/*
PurePursuit example-based tests.

The law is closed-form and deterministic; its only state is a monotonic progress cursor. These tests
pin the sign/magnitude conventions of a single `step` (and short step sequences):

  - straight on-track    -> y_c ~= 0 => kappa ~= 0 => omega ~= 0
  - lateral offset       -> carrot left => omega > 0; carrot right => omega < 0, for BOTH drive modes
  - pivot pose           -> v = 0, omega turns the short way, terminates within goal_heading_tol
  - curvature slowdown   -> tight arc => |v| clamped so v^2*|kappa| <= a_lat_max
  - progress monotonicity-> cursor never regresses; goal termination latches done()
  - goal                 -> within goal_pos_tol then heading-align => zero Control + done()

Construction-precondition CHECKs (positive gains, non-empty trajectory, dt > 0) throw
std::invalid_argument (see core/diagnostics/check.hpp) and are covered here as boundary tests.
*/

namespace {

constexpr double kPi = std::numbers::pi;

// Nominal gains matching the config/controller/controller.yaml seeds. Individual tests tweak
// fields where a smaller lookahead / larger a_lat_max sharpens the behaviour under test.
PurePursuitParams nominalParams() {
    return PurePursuitParams{
        /*lookahead=*/0.75,
        /*v_cruise=*/2.0,
        /*a_lat_max=*/2.0,
        /*omega_pivot=*/1.0,
        /*goal_pos_tol=*/0.10,
        /*goal_heading_tol=*/0.10};
}

// A dense straight trajectory along +x at y = 0, all Forward, spaced 0.1 m (below lookahead).
std::vector<TrajectoryPoint> straightForward(double length = 5.0, double spacing = 0.1) {
    std::vector<TrajectoryPoint> traj;
    for (double x = 0.0; x <= length + 1e-9; x += spacing) {
        traj.push_back(TrajectoryPoint{SE2State{x, 0.0, 0.0}, MotionMode::Forward, 1.0});
    }
    return traj;
}

// Same straight geometry but marked Reverse; the reference heading still points along +x, so a robot
// facing -x (theta = pi) drives backward down the line.
std::vector<TrajectoryPoint> straightReverse(double length = 5.0, double spacing = 0.1) {
    std::vector<TrajectoryPoint> traj;
    for (double x = 0.0; x <= length + 1e-9; x += spacing) {
        traj.push_back(TrajectoryPoint{SE2State{x, 0.0, 0.0}, MotionMode::Reverse, -1.0});
    }
    return traj;
}

}  // namespace

// ---- construction preconditions (CHECK -> std::invalid_argument) -------------------------------

TEST(PurePursuitCtor, rejectsNonPositiveGains) {
    const auto traj = straightForward();
    for (int field = 0; field < 6; ++field) {
        PurePursuitParams p = nominalParams();
        double * fields[] = {&p.lookahead, &p.v_cruise, &p.a_lat_max,
                             &p.omega_pivot, &p.goal_pos_tol, &p.goal_heading_tol};
        *fields[field] = 0.0;
        EXPECT_THROW(PurePursuit(p, traj, 0.05), std::invalid_argument)
            << "field index " << field << " == 0 should be rejected";
    }
}

TEST(PurePursuitCtor, rejectsNonPositiveDt) {
    EXPECT_THROW(PurePursuit(nominalParams(), straightForward(), 0.0), std::invalid_argument);
    EXPECT_THROW(PurePursuit(nominalParams(), straightForward(), -0.05), std::invalid_argument);
}

TEST(PurePursuitCtor, rejectsEmptyTrajectory) {
    EXPECT_THROW(PurePursuit(nominalParams(), std::vector<TrajectoryPoint>{}, 0.05),
                 std::invalid_argument);
}

// ---- straight, on-track: y_c ~= 0 => kappa ~= 0 => omega ~= 0 ----------------------------------

TEST(PurePursuitStraight, onTrackForwardCommandsNoTurn) {
    PurePursuit pp(nominalParams(), straightForward(), 0.05);
    const Control u = pp.step(SE2State{1.0, 0.0, 0.0});  // perfectly on the line, facing +x
    EXPECT_GT(u.v, 0.0);                                  // drives forward
    EXPECT_NEAR(u.omega, 0.0, 1e-9);                      // no steering
}

TEST(PurePursuitStraight, onTrackReverseCommandsNoTurn) {
    PurePursuit pp(nominalParams(), straightReverse(), 0.05);
    // On the line, facing -x (pursuit heading = theta+pi = +x, aligned with the trajectory forward).
    const Control u = pp.step(SE2State{1.0, 0.0, kPi});
    EXPECT_LT(u.v, 0.0);                                  // drives in reverse
    EXPECT_NEAR(u.omega, 0.0, 1e-9);                      // no steering
}

// ---- lateral offset: sign of omega for both drive modes ----------------------------------------
// Body-frame "carrot left" means the carrot has positive y in the robot's travel frame, which the
// pursuit law must correct with a positive (CCW) omega, and vice versa. For Reverse the law commands
// omega = kappa*speed (positive magnitude, not kappa*v), so applying it via propagate steers the
// backing robot onto the path -- verified against propagate in the property test.

TEST(PurePursuitLateral, forwardCarrotLeftTurnsPositive) {
    PurePursuit pp(nominalParams(), straightForward(), 0.05);
    // Robot below the line (y < 0) facing +x: the line (carrot) is to its left => omega > 0.
    const Control u = pp.step(SE2State{1.0, -0.3, 0.0});
    EXPECT_GT(u.omega, 0.0);
}

TEST(PurePursuitLateral, forwardCarrotRightTurnsNegative) {
    PurePursuit pp(nominalParams(), straightForward(), 0.05);
    // Robot above the line (y > 0) facing +x: the carrot is to its right => omega < 0.
    const Control u = pp.step(SE2State{1.0, 0.3, 0.0});
    EXPECT_LT(u.omega, 0.0);
}

// Reverse sign convention. The follower drives in the PURSUIT frame psi = theta+pi, so for a robot
// facing -x (theta = pi) the pursuit heading is +x. A robot ABOVE the line (world y > 0) has the carrot
// (on the line, y_c < 0 in the pursuit frame) to the RIGHT of the pursuit direction, so kappa < 0 and,
// since omega = kappa*speed with speed > 0, omega < 0. What makes this correct is NOT the Cartesian
// label but that feeding {v<0, omega} to propagate curves the base back onto the line: we assert both
// the sign the geometry dictates AND that one propagate step reduces |y| (the reverse-sign guard, also
// exercised exhaustively by the property test).

TEST(PurePursuitLateral, reverseAboveLineCurvesBackOntoIt) {
    const auto space = []{
        DiffDriveParams kin; kin.rmin = 0.3; kin.vmax = 3.0; kin.atomic_time = 1.0; kin.n_headings = 8;
        auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
        return DiffDriveStateSpace(
            GridMetaData{1000, 1000, 1.0, Eigen::Vector2f(-500.0f, -500.0f)}, dist);
    }();
    PurePursuit pp(nominalParams(), straightReverse(), 0.05);
    const SE2State pose{1.0, 0.3, kPi};   // above the line, facing -x (backs along +x)
    const Control u = pp.step(pose);
    EXPECT_LT(u.v, 0.0);                   // drives in reverse
    EXPECT_LT(u.omega, 0.0);              // pursuit-frame carrot is on the right
    const SE2State next = space.propagate(pose, u);
    EXPECT_LT(std::abs(next.y), std::abs(pose.y));  // one step pulls the base toward the line
    EXPECT_GT(next.x, pose.x);                        // and it does move in the reverse (+x) direction
}

TEST(PurePursuitLateral, reverseBelowLineCurvesBackOntoIt) {
    const auto space = []{
        DiffDriveParams kin; kin.rmin = 0.3; kin.vmax = 3.0; kin.atomic_time = 1.0; kin.n_headings = 8;
        auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
        return DiffDriveStateSpace(
            GridMetaData{1000, 1000, 1.0, Eigen::Vector2f(-500.0f, -500.0f)}, dist);
    }();
    PurePursuit pp(nominalParams(), straightReverse(), 0.05);
    const SE2State pose{1.0, -0.3, kPi};  // below the line, facing -x
    const Control u = pp.step(pose);
    EXPECT_LT(u.v, 0.0);
    EXPECT_GT(u.omega, 0.0);             // mirror: pursuit-frame carrot on the left
    const SE2State next = space.propagate(pose, u);
    EXPECT_LT(std::abs(next.y), std::abs(pose.y));
    EXPECT_GT(next.x, pose.x);
}

// ---- pivot: v = 0, short-way rotation, terminates within goal_heading_tol ----------------------

TEST(PurePursuitPivot, commandsZeroSpeedAndShortWayRotation) {
    // Single pivot point far enough from the robot's position that the goal test does not pre-empt it
    // (goal_pos_tol = 0.10; place the robot 1 m away). Target heading +pi/2, robot at 0 => turn CCW.
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{1.0, 0.0, kPi / 2.0}, MotionMode::Pivot, 0.0}};
    PurePursuit pp(nominalParams(), traj, 0.05);
    const Control u = pp.step(SE2State{0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(u.v, 0.0);
    EXPECT_GT(u.omega, 0.0);                        // short way to +pi/2 is CCW
    EXPECT_DOUBLE_EQ(std::abs(u.omega), 1.0);       // omega_pivot magnitude
}

TEST(PurePursuitPivot, rotatesClockwiseForNegativeError) {
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{1.0, 0.0, -kPi / 2.0}, MotionMode::Pivot, 0.0}};
    PurePursuit pp(nominalParams(), traj, 0.05);
    const Control u = pp.step(SE2State{0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(u.v, 0.0);
    EXPECT_LT(u.omega, 0.0);                        // short way to -pi/2 is CW
}

TEST(PurePursuitPivot, picksShortWayAcrossThePiSeam) {
    // Robot at theta = 3.0, target at -3.0: the short arc (~0.283 rad) crosses +pi, i.e. positive omega,
    // not the long way back through 0.
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{1.0, 0.0, -3.0}, MotionMode::Pivot, 0.0}};
    PurePursuit pp(nominalParams(), traj, 0.05);
    const Control u = pp.step(SE2State{0.0, 0.0, 3.0});
    EXPECT_DOUBLE_EQ(u.v, 0.0);
    EXPECT_GT(u.omega, 0.0);
}

TEST(PurePursuitPivot, terminatesWithinHeadingTolerance) {
    // Drive a pivot to completion. Place the pivot pose at the robot's position so, once aligned, the
    // goal test (which is on the LAST pose) declares done. Iterate step->rotate until done or a cap.
    PurePursuitParams params = nominalParams();
    const double dt = 0.05;
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{0.0, 0.0, kPi / 2.0}, MotionMode::Pivot, 0.0}};
    PurePursuit pp(params, traj, dt);

    SE2State pose{0.0, 0.0, 0.0};
    int iters = 0;
    const int kMaxIters = 1000;
    for (; iters < kMaxIters && !pp.done(); ++iters) {
        const Control u = pp.step(pose);
        // Integrate the pivot: pure rotation, position unchanged.
        pose.theta = wrapToPi(pose.theta + u.omega * u.dt);
    }
    EXPECT_TRUE(pp.done());
    EXPECT_LT(iters, kMaxIters);
    EXPECT_LT(std::abs(wrapToPi(kPi / 2.0 - pose.theta)), params.goal_heading_tol);
}

// ---- curvature slowdown: v^2 * |kappa| <= a_lat_max on a tight arc -----------------------------

TEST(PurePursuitCurvature, slowsSoLateralAccelBoundHolds) {
    // A short lookahead + a sharply offset carrot forces a tight kappa; the speed must be clamped so
    // v^2 * |kappa| <= a_lat_max. Use a small lookahead and a carrot offset comparable to it.
    PurePursuitParams params = nominalParams();
    params.lookahead = 0.5;
    params.a_lat_max = 1.0;
    params.v_cruise  = 3.0;   // high ceiling so the curvature clamp (not the ceiling) binds

    // Dense arc-like trajectory curving hard left away from the robot so the carrot has large |y_c|.
    std::vector<TrajectoryPoint> traj;
    for (int k = 0; k <= 40; ++k) {
        const double s = 0.05 * k;               // arc length
        const double r = 0.5;                    // tight radius => high curvature
        traj.push_back(TrajectoryPoint{
            SE2State{r * std::sin(s / r), r * (1.0 - std::cos(s / r)), s / r},
            MotionMode::Forward, 1.0});
    }
    PurePursuit pp(params, traj, 0.05);

    // Step from the start of the arc, facing +x.
    const Control u = pp.step(SE2State{0.0, 0.0, 0.0});
    // Recover the curvature the law used from the commanded (v, omega): kappa = omega / v.
    ASSERT_GT(std::abs(u.v), 1e-6);
    const double kappa = u.omega / u.v;
    EXPECT_GT(std::abs(kappa), 1e-3) << "test setup must produce a genuinely curved arc";
    // The lateral-accel invariant, with a small slack for the ramp/ceiling interaction.
    EXPECT_LE(u.v * u.v * std::abs(kappa), params.a_lat_max + 1e-6);
}

TEST(PurePursuitCurvature, straightPathRunsAtCruiseCeiling) {
    // On a straight path with no endpoint ramp in effect (far from the goal), |v| hits v_cruise.
    PurePursuitParams params = nominalParams();
    PurePursuit pp(params, straightForward(/*length=*/10.0), 0.05);
    const Control u = pp.step(SE2State{1.0, 0.0, 0.0});
    EXPECT_NEAR(u.v, params.v_cruise, 1e-9);
}

// ---- progress monotonicity + goal termination --------------------------------------------------

TEST(PurePursuitProgress, cursorNeverRegressesAcrossSteps) {
    // Drive the follower along the straight path via propagate-free integration of its own commands,
    // sampling poses that march forward; the follower must terminate at the goal and never re-track.
    // We assert progress indirectly: once we are near the end, done() latches, and it does not un-latch.
    PurePursuit pp(nominalParams(), straightForward(/*length=*/2.0), 0.05);

    // Feed a monotonically increasing sequence of on-track poses toward the goal (2.0, 0, 0).
    bool ever_done = false;
    for (double x = 0.0; x <= 2.0 + 1e-9; x += 0.1) {
        const Control u = pp.step(SE2State{x, 0.0, 0.0});
        if (pp.done()) {
            ever_done = true;
            EXPECT_DOUBLE_EQ(u.v, 0.0);
            EXPECT_DOUBLE_EQ(u.omega, 0.0);
        }
    }
    EXPECT_TRUE(ever_done);
    // done() stays latched: a further step past the goal still reports done + zero control.
    const Control after = pp.step(SE2State{2.0, 0.0, 0.0});
    EXPECT_TRUE(pp.done());
    EXPECT_DOUBLE_EQ(after.v, 0.0);
    EXPECT_DOUBLE_EQ(after.omega, 0.0);
}

TEST(PurePursuitGoal, withinPosTolAlignsHeadingThenStops) {
    // Robot sitting within goal_pos_tol of the goal but misaligned in heading => the law pivots
    // (v = 0, omega != 0) toward the goal heading, and does not yet report done.
    std::vector<TrajectoryPoint> traj = straightForward(/*length=*/1.0);
    // Goal heading is 0 (all points face +x). Put the robot at the goal but facing +pi/2.
    PurePursuit pp(nominalParams(), traj, 0.05);
    const SE2State goal = traj.back().pose;

    const Control aligning = pp.step(SE2State{goal.x, goal.y, kPi / 2.0});
    EXPECT_FALSE(pp.done());
    EXPECT_DOUBLE_EQ(aligning.v, 0.0);
    EXPECT_LT(aligning.omega, 0.0);  // short way from +pi/2 to 0 is CW

    // Now already aligned within tolerance => zero control + done.
    const Control stopped = pp.step(SE2State{goal.x, goal.y, 0.0});
    EXPECT_TRUE(pp.done());
    EXPECT_DOUBLE_EQ(stopped.v, 0.0);
    EXPECT_DOUBLE_EQ(stopped.omega, 0.0);
}

TEST(PurePursuitGoal, resetRewindsDoneAndCursor) {
    PurePursuit pp(nominalParams(), straightForward(/*length=*/1.0), 0.05);
    const SE2State goal = straightForward(/*length=*/1.0).back().pose;
    (void)pp.step(SE2State{goal.x, goal.y, 0.0});  // reach goal already aligned
    ASSERT_TRUE(pp.done());

    pp.reset();
    EXPECT_FALSE(pp.done());
    // After reset, stepping from the start commands forward motion again (cursor rewound).
    const Control u = pp.step(SE2State{0.0, 0.0, 0.0});
    EXPECT_GT(u.v, 0.0);
}

// ---- Regressions: co-located leading pivots + the cursor+2 guard -------------------------------
// Every hybrid A* plan begins with in-place PIVOT poses (heading alignment) stacked on the exact
// start position, followed by Forward poses that also start at that position before translating.
// advanceCursor()'s position-nearest scan alone can never move off those co-located leading poses
// (their position distance to the base stays 0), so on its own it would pivot forever at the origin
// and never translate the base -- the full-pipeline deadlock. advanceCursor()'s objective-based
// Phase 1 skip prevents that: retire leading pivots once heading has converged, and retire co-located
// leading drive points, BUT only while a point remains after the one advanced onto (the
// `cursor_ + 2 < trajectory_.size()` guard) so the cursor never lands on the final pose and zeroes
// the endpoint deceleration ramp. These two tests pin both halves of that contract. Iterate the real
// kinematic model (step -> propagate, wrapping theta as the property test does) so a regression that
// reintroduces the stall loops forever at the origin and fails via the step cap.

namespace {

// Wide-open workspace so propagate is never gated by bounds; matches the property-test builder.
DiffDriveStateSpace makeIntegrationSpace() {
    DiffDriveParams kin;
    kin.rmin = 0.3; kin.vmax = 3.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    return DiffDriveStateSpace(
        GridMetaData{1000, 1000, 1.0, Eigen::Vector2f(-500.0f, -500.0f)}, dist);
}

}  // namespace

// Co-located leading pivots: the plan opens with two in-place Pivot poses at (0,0,theta0) and two
// Forward poses that also start at (0,0,theta0) before the path begins translating toward a
// reachable goal. Driving the real kinematic model, the follower must retire the leading pivots
// (heading already converged) and the co-located Forward poses, START TRANSLATING, and settle at the
// final pose. Without the leading-pivot skip this would loop forever at the origin (advanceCursor
// never leaving the stacked poses); it must converge in a few tens of steps.
TEST(PurePursuitDefectB, colocatedLeadingPivotsDoNotDeadlock) {
    const DiffDriveStateSpace space = makeIntegrationSpace();
    const double dt = 0.05;
    const double theta0 = 0.0;  // start heading already aligned with the path tangent (+x)

    // [Pivot@(0,0), Pivot@(0,0), Forward@(0,0), Forward@(0,0), then a dense translating +x run].
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{0.0, 0.0, theta0}, MotionMode::Pivot,   0.0},
        TrajectoryPoint{SE2State{0.0, 0.0, theta0}, MotionMode::Pivot,   0.0},
        TrajectoryPoint{SE2State{0.0, 0.0, theta0}, MotionMode::Forward, 1.0},
        TrajectoryPoint{SE2State{0.0, 0.0, theta0}, MotionMode::Forward, 1.0},
    };
    for (double x = 0.1; x <= 2.0 + 1e-9; x += 0.1) {
        traj.push_back(TrajectoryPoint{SE2State{x, 0.0, theta0}, MotionMode::Forward, 1.0});
    }
    const SE2State goal = traj.back().pose;

    PurePursuit pp(nominalParams(), traj, dt);
    SE2State pose{0.0, 0.0, theta0};
    int iters = 0;
    const int kMaxIters = 2000;
    for (; iters < kMaxIters && !pp.done(); ++iters) {
        const Control u = pp.step(pose);
        pose = space.propagate(pose, u);
        pose.theta = wrapToPi(pose.theta);
    }

    EXPECT_TRUE(pp.done()) << "follower deadlocked on co-located leading poses (never latched done)";
    EXPECT_LT(iters, kMaxIters) << "follower never converged within the step cap (stall regression)";
    // The base must have actually left the origin and reached the final pose within goal tolerance.
    EXPECT_GT(pose.x, 0.5) << "base never translated off the co-located leading poses";
    const double dist_to_goal = std::hypot(pose.x - goal.x, pose.y - goal.y);
    EXPECT_LT(dist_to_goal, nominalParams().goal_pos_tol)
        << "base settled " << dist_to_goal << " m from the goal (did not reach it)";
}

// The cursor+2 guard boundary: a dense straight path whose PENULTIMATE point coincides EXACTLY with
// an integrated base position must still terminate. If the co-located skip lacked the
// `cursor_ + 2 < trajectory_.size()` guard, arriving on the penultimate point would advance the
// cursor onto the FINAL pose, zeroing the endpoint deceleration ramp (remaining path distance -> 0)
// and stalling the base ~0.1 m short of the goal -- it would reach within a spacing of the goal but
// never inside goal_pos_tol. This drives the real kinematic model and asserts the base actually
// settles inside goal_pos_tol, so dropping the guard (cursor jumps to the final pose) fails here.
TEST(PurePursuitDefectB, colocatedPenultimatePointStillTerminates) {
    const DiffDriveStateSpace space = makeIntegrationSpace();
    const double dt = 0.05;

    // Dense straight path (0.1 m spacing) to (2.0, 0, 0). The penultimate point is (1.9, 0, 0);
    // driving from (0,0,0) the integrated base position passes exactly through it (on-axis motion),
    // which is precisely the co-located case the guard must NOT let jump the cursor to the goal.
    const auto traj = straightForward(/*length=*/2.0, /*spacing=*/0.1);
    const SE2State goal = traj.back().pose;

    PurePursuit pp(nominalParams(), traj, dt);
    SE2State pose{0.0, 0.0, 0.0};
    int iters = 0;
    const int kMaxIters = 2000;
    double best = std::numeric_limits<double>::infinity();
    for (; iters < kMaxIters && !pp.done(); ++iters) {
        const Control u = pp.step(pose);
        pose = space.propagate(pose, u);
        pose.theta = wrapToPi(pose.theta);
        best = std::min(best, std::hypot(pose.x - goal.x, pose.y - goal.y));
    }

    EXPECT_TRUE(pp.done()) << "follower stalled short of the goal (endpoint ramp zeroed by the "
                              "cursor jumping onto the final pose)";
    EXPECT_LT(iters, kMaxIters);
    const double dist_to_goal = std::hypot(pose.x - goal.x, pose.y - goal.y);
    EXPECT_LT(dist_to_goal, nominalParams().goal_pos_tol)
        << "base settled " << dist_to_goal << " m from the goal; closest approach " << best
        << " m -- guard regression stalls the base ~0.1 m short";
}

// ---- corner-cut: two complementary guards (smooth-arc-exact + discontinuity corner bound) ---------
// Deriving the lookahead from the footprint sample spacing, L = k*(r_in/2) with k=3 => L = 0.45,
// bounds the pursuit corner-cut. Two facts drive the guards below:
//
//   (a) On a *smooth* constant-curvature arc pure pursuit is an EXACT circle-tracker. For a base on a
//       circle of radius R with tangent heading pursuing ANY on-circle carrot, the commanded curvature
//       is kappa = 2*y_c/|carrot|^2 = 1/R exactly (independent of L; y_c = |carrot|^2/(2R) is the chord
//       sagitta), and DiffDriveStateSpace::propagate integrates that arc exactly. So the executed cut is
//       identically 0 for EVERY lookahead -- the chord-sagitta e_geom = R - sqrt(R^2-(L/2)^2) NEVER
//       manifests on a single arc. `smoothArcTrackedExactly` pins this (and that the follower actually
//       completes the arc -- a spiral-off or a stall would fail).
//
//   (b) The corner-cut arises ONLY at path DISCONTINUITIES between primitives (a heading step at an
//       in-place pivot, or a curvature step at an arc<->arc / arc<->straight junction). There the cut
//       grows ~linearly with L and with the junction turn-angle theta, bounded by the corner sagitta
//       e_corner(L, theta) = (L/2)*sin(theta/2). `cornerCutBoundedAndShrinksWithLookahead` pins both
//       the L-lever (cut at L=0.45 is strictly below cut at L=0.75) and the theta-based envelope on a
//       representative real corner (a drive -> in-place pivot -> drive 90-degree reorientation).
//
// The control set's tightest DRIVE arc is R = 0.5*rmin = 0.15 m, and because L (0.45) is 3x that
// radius, junctions leaving the tightest arc are the true worst case (~0.22 m cut at L=0.45).

namespace {

// Wide-open diff-drive space (propagate only; the closed loop never touches the bounds).
DiffDriveStateSpace makeArcSpace() {
    DiffDriveParams kin; kin.rmin = 0.3; kin.vmax = 3.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    return DiffDriveStateSpace(
        GridMetaData{1000, 1000, 1.0, Eigen::Vector2f(-500.0f, -500.0f)}, dist);
}

// Left arc of radius R centred at (0, R), swept angle `sweep` (rad), sampled at <= `spacing` m,
// all Forward, headings tangent. Pose on the arc: (R*sin a, R*(1-cos a), a).
std::vector<TrajectoryPoint> tightArc(double R, double sweep, double spacing) {
    std::vector<TrajectoryPoint> traj;
    const int n = std::max(2, static_cast<int>(std::ceil(R * sweep / spacing)));
    for (int i = 0; i <= n; ++i) {
        const double a = sweep * i / n;
        traj.push_back(TrajectoryPoint{
            SE2State{R * std::sin(a), R * (1.0 - std::cos(a)), a}, MotionMode::Forward, 1.0});
    }
    return traj;
}

// Exact perpendicular cut for a circular arc centred at (0, R): |dist(pose, centre) - R|.
double arcCut(const SE2State & p, double R) {
    return std::abs(std::hypot(p.x - 0.0, p.y - R) - R);
}

}  // namespace

TEST(PurePursuitCornerCut, smoothArcTrackedExactly) {
    constexpr double R = 0.3;            // a representative drive arc (== rmin)
    constexpr double L = 0.45;           // k=3 * (r_in/2 = 0.15)
    constexpr double spacing = 0.15;     // worst-case followed-sample spacing (r_in/2)

    // e_geom = R - sqrt(R^2 - (L/2)^2); L/2 = 0.225 < R -> well-defined. This is the conservative
    // chord-sagitta envelope; the executed cut on a smooth arc stays well below it (see the note above).
    const double e_geom = R - std::sqrt(R * R - (L / 2.0) * (L / 2.0));   // ~= 0.10157
    const double tol    = 0.03;          // envelope headroom (loop discretisation + arc/chord slack)

    PurePursuitParams pp_params{L, 2.0, 2.0, 1.0, 0.10, 0.10};
    const auto traj = tightArc(R, kPi, spacing);     // half turn about (0, R)
    const SE2State goal = traj.back().pose;
    const DiffDriveStateSpace space = makeArcSpace();
    const double dt = 0.02;
    PurePursuit pp(pp_params, traj, dt);

    SE2State pose{0.0, 0.0, 0.0};        // start on the arc at the origin, tangent heading
    double max_cut = 0.0;
    const int kMaxSteps = 6000;
    int steps = 0;
    for (; steps < kMaxSteps && !pp.done(); ++steps) {
        pose = space.propagate(pose, pp.step(pose));
        pose.theta = wrapToPi(pose.theta);
        max_cut = std::max(max_cut, arcCut(pose, R));
    }

    EXPECT_LT(max_cut, e_geom + tol);    // executed path stays within the geometric budget
    // The bound above only has teeth if the follower actually traverses the arc: confirm it tracked
    // the arc to completion and settled on the endpoint.
    EXPECT_TRUE(pp.done()) << "follower did not complete the arc";
    EXPECT_LT(steps, kMaxSteps);
    EXPECT_LT(std::hypot(pose.x - goal.x, pose.y - goal.y), pp_params.goal_pos_tol)
        << "follower settled off the arc endpoint";
}

namespace {

// Perpendicular distance from `p` to the corner path: the two arms of a symmetric V meeting at the
// origin. The robot travels IN along heading -theta/2 (so the incoming arm's reference points lie in
// the -u_in direction from the vertex) and OUT along heading +theta/2 (the outgoing arm along +u_out).
// Distance to the nearer arm == the inside cut.
double cornerCut(const SE2State & p, double theta) {
    const double s = std::sin(theta / 2.0), c = std::cos(theta / 2.0);
    const Eigen::Vector2d q(p.x, p.y);
    const Eigen::Vector2d arm_in(-c, s), arm_out(c, s);   // arms emanate from the vertex (origin)
    auto rayDist = [&](const Eigen::Vector2d & u) {
        const double t = q.dot(u);                 // projection onto the arm
        if (t <= 0.0) return q.norm();             // nearest point is the vertex
        return (q - t * u).norm();                 // perpendicular distance to the arm
    };
    return std::min(rayDist(arm_in), rayDist(arm_out));
}

// A representative real-plan heading corner: incoming Forward drive along heading -theta/2 into the
// vertex at the origin, an in-place Pivot cluster reorienting to +theta/2 (as hybrid A* plans do at a
// sharp turn), then outgoing Forward drive. All poses spaced <= r_in/2 = 0.15 m; the pivot cluster is
// stepped finely in heading. Returns the max inside cut over the driven closed loop for lookahead L.
double runCorner(double theta, double L) {
    constexpr double len = 1.5, spacing = 0.15;
    const double thin = -theta / 2.0, thout = theta / 2.0;
    const Eigen::Vector2d uin(std::cos(thin), std::sin(thin));
    const Eigen::Vector2d uout(std::cos(thout), std::sin(thout));

    std::vector<TrajectoryPoint> traj;
    const int n = std::max(2, static_cast<int>(std::ceil(len / spacing)));
    for (int i = n; i >= 1; --i) {                 // incoming: -len*uin ... origin
        const Eigen::Vector2d p = -(len * i / n) * uin;
        traj.push_back(TrajectoryPoint{SE2State{p.x(), p.y(), thin}, MotionMode::Forward, 1.0});
    }
    const int m = std::max(2, static_cast<int>(std::ceil(std::abs(theta) / 0.25)));
    for (int i = 0; i <= m; ++i) {                 // in-place pivot cluster at the vertex
        const double th = thin + (thout - thin) * i / m;
        traj.push_back(TrajectoryPoint{SE2State{0.0, 0.0, th}, MotionMode::Pivot, 0.0});
    }
    for (int i = 1; i <= n; ++i) {                 // outgoing: origin ... len*uout
        const Eigen::Vector2d p = (len * i / n) * uout;
        traj.push_back(TrajectoryPoint{SE2State{p.x(), p.y(), thout}, MotionMode::Forward, 1.0});
    }

    PurePursuitParams pp_params{L, 2.0, 2.0, 1.0, 0.10, 0.10};
    const DiffDriveStateSpace space = makeArcSpace();
    PurePursuit pp(pp_params, traj, 0.02);
    SE2State pose = traj.front().pose;
    double max_cut = 0.0;
    const int kMaxSteps = 8000;
    for (int step = 0; step < kMaxSteps && !pp.done(); ++step) {
        pose = space.propagate(pose, pp.step(pose));
        pose.theta = wrapToPi(pose.theta);
        max_cut = std::max(max_cut, cornerCut(pose, theta));
    }
    return max_cut;
}

}  // namespace

// The corner-cut regression with teeth. On a heading discontinuity the pursuit cut is real and
// L-dependent, so this asserts BOTH halves of the guard:
//   1. the derived lookahead reduces the cut: cut(L=0.45) is strictly below cut(L=0.75) -- this is the
//      root-cause lever (a larger L=0.75 cuts the e2e corner by ~0.41 m);
//   2. the cut at L=0.45 stays within the corner-sagitta envelope e_corner = (L/2)*sin(t/2).
// Measured on this 90-degree drive->pivot->drive corner: cut(0.45) ~= 0.116 m, cut(0.75) ~= 0.197 m,
// envelope (L/2)*sin(45) = 0.159 m -> the tol below is non-load-bearing (the bound alone clears the
// measured cut by ~37%). A regression that widened the cut (e.g. reinstating the large lookahead, or a
// carrot-selection bug) trips assertion 1; one that grossly over-cut trips assertion 2.
TEST(PurePursuitCornerCut, cornerCutBoundedAndShrinksWithLookahead) {
    constexpr double theta = kPi / 2.0;   // 90-degree reorientation -- a canonical sharp real corner
    const double cut_45 = runCorner(theta, 0.45);
    const double cut_75 = runCorner(theta, 0.75);

    // 1. The lookahead-derivation lever: a shorter L cuts the corner less.
    EXPECT_LT(cut_45, cut_75)
        << "reducing the lookahead did not reduce the corner-cut (root-cause lever broken): "
        << "cut(0.45)=" << cut_45 << "  cut(0.75)=" << cut_75;

    // 2. The theta-based envelope: e_corner(L, theta) = (L/2) * sin(theta/2), the corner sagitta.
    const double e_corner = (0.45 / 2.0) * std::sin(theta / 2.0);   // ~= 0.159 m at 90 degrees
    const double tol      = 0.03;         // non-load-bearing: the bound already clears the ~0.116 m cut
    EXPECT_LT(cut_45, e_corner + tol)
        << "corner-cut at L=0.45 (" << cut_45 << " m) exceeded the sagitta envelope "
        << e_corner << " m";

    // Sanity: there IS a real, nonzero cut here (unlike the smooth arc) -- the test has teeth.
    EXPECT_GT(cut_45, 0.02) << "expected a measurable corner-cut at a heading discontinuity";
}

// ---- terminal goal-alignment taper + deadband/hysteresis latch ---------------------------------
// The final goal-heading alignment uses a PROPORTIONAL taper (pivotToGoal: omega = clamp(k_head * herr,
// +-omega_pivot), with k_head = omega_pivot / (kHeadingSaturateFactor * goal_heading_tol)) plus a
// deadband/hysteresis latch on done(): once latched, the flag holds under position jitter and small
// heading noise, and only re-opens on a LARGE heading drift (> kHeadingReengageFactor * goal_heading_tol)
// WHILE still inside the position tolerance. These four cases cover that taper + latch; the cases above
// (PurePursuitGoal / PurePursuitPivot) pin the mid-path pivot (pivotToward), which is fixed-rate.
//
// Constants mirror the source (pure_pursuit.cpp anonymous namespace): kHeadingSaturateFactor = 2.5,
// kHeadingReengageFactor = 1.5. At nominal gains (omega_pivot = 1.0, goal_heading_tol = 0.10):
//   k_head          = 1.0 / (2.5 * 0.10) = 4.0 rad/s per rad   (taper saturates at 2.5*tol = 0.25 rad)
//   re-engage band  = 1.5 * 0.10 = 0.15 rad                    (wider than the 0.10 stop band)

namespace {
constexpr double kHeadingSaturateFactor = 2.5;   // mirrors pure_pursuit.cpp
constexpr double kHeadingReengageFactor = 1.5;   // mirrors pure_pursuit.cpp
}  // namespace

// (a) Deadband hold. After done() latches, a heading perturbation inside the re-engage band
// (|herr| < 1.5*goal_heading_tol = 0.15) -- even one BEYOND the plain goal_heading_tol (0.10) -- must
// neither un-latch done() nor re-command a pivot. Without the hysteresis a 0.12 rad drift (> 0.10 tol)
// would restart the alignment; the wider re-engage band is exactly what holds it. Commands zero omega.
TEST(PurePursuitGoalAlign, deadbandHoldsUnderSmallHeadingPerturbation) {
    const auto traj = straightForward(/*length=*/1.0);
    const SE2State goal = traj.back().pose;   // {1.0, 0.0, 0.0}
    PurePursuit pp(nominalParams(), traj, 0.05);

    (void)pp.step(SE2State{goal.x, goal.y, 0.0});   // reach goal aligned -> latch done
    ASSERT_TRUE(pp.done());

    // 0.10 (stop band / goal_heading_tol) < 0.12 < 0.15 (re-engage band): the hysteresis "hold" zone.
    const Control u = pp.step(SE2State{goal.x, goal.y, 0.12});
    EXPECT_TRUE(pp.done()) << "deadband/hysteresis must hold done() under sub-re-engage heading noise";
    EXPECT_DOUBLE_EQ(u.v, 0.0);
    EXPECT_DOUBLE_EQ(u.omega, 0.0) << "no re-pivot inside the re-engage band";
}

// (b) Hysteresis re-engage. After done() latches, a LARGE heading drift beyond the re-engage band
// (0.30 > 1.5*goal_heading_tol = 0.15) WHILE within the position tolerance clears done() and
// re-commands a taper pivot; driving that pure rotation to completion re-latches done(). The initial
// re-engage error (0.30) is past the taper saturation point (0.25), so |omega| saturates at omega_pivot.
TEST(PurePursuitGoalAlign, hysteresisReEngagesOnLargeHeadingDrift) {
    const auto traj = straightForward(/*length=*/1.0);
    const SE2State goal = traj.back().pose;
    const PurePursuitParams params = nominalParams();
    const double dt = 0.05;
    PurePursuit pp(params, traj, dt);

    (void)pp.step(SE2State{goal.x, goal.y, 0.0});   // latch done
    ASSERT_TRUE(pp.done());

    SE2State pose{goal.x, goal.y, 0.30};            // large drift, still at the goal position
    const Control reengage = pp.step(pose);
    EXPECT_FALSE(pp.done()) << "large heading drift within pos tol must re-open the latch";
    EXPECT_DOUBLE_EQ(reengage.v, 0.0);
    EXPECT_LT(reengage.omega, 0.0) << "re-commanded pivot turns the short way back toward heading 0";
    // 0.30 rad > saturation point (kHeadingSaturateFactor*tol = 0.25): k_head*0.30 = 1.2 -> clamp to 1.0.
    EXPECT_DOUBLE_EQ(std::abs(reengage.omega), params.omega_pivot);

    // Integrate the pure rotation (v = 0, position fixed) to completion; done() must re-latch in tol.
    int iters = 0;
    const int kMaxIters = 1000;
    for (; iters < kMaxIters && !pp.done(); ++iters) {
        const Control u = pp.step(pose);
        pose.theta = wrapToPi(pose.theta + u.omega * u.dt);
    }
    EXPECT_TRUE(pp.done()) << "follower did not re-latch done() after re-aligning";
    EXPECT_LT(iters, kMaxIters);
    EXPECT_LT(std::abs(wrapToPi(goal.theta - pose.theta)), params.goal_heading_tol);
}

// (c) Proportional taper. At the goal position with a small-but-outside-tol heading error
// (herr = 0.15 rad, between goal_heading_tol = 0.10 and the saturation point 2.5*0.10 = 0.25), the
// terminal alignment commands omega = k_head*herr, strictly below omega_pivot -- proving the taper.
// A bang-bang law would command exactly +-omega_pivot (1.0) here.
//   k_head   = omega_pivot / (kHeadingSaturateFactor * goal_heading_tol) = 1.0 / 0.25 = 4.0
//   |omega|  = 4.0 * 0.15 = 0.60  (< omega_pivot = 1.0)
TEST(PurePursuitGoalAlign, tapersOmegaProportionallyToHeadingError) {
    const PurePursuitParams params = nominalParams();   // omega_pivot = 1.0, goal_heading_tol = 0.10
    const auto traj = straightForward(/*length=*/1.0);
    const SE2State goal = traj.back().pose;             // heading 0
    PurePursuit pp(params, traj, 0.05);

    const double herr = 0.15;                           // > goal_heading_tol, < saturation (0.25)
    const SE2State pose{goal.x, goal.y, -herr};         // headingError(-0.15, 0) = wrapToPi(0.15) = +0.15
    const Control u = pp.step(pose);

    const double k_head  = params.omega_pivot / (kHeadingSaturateFactor * params.goal_heading_tol);  // 4.0
    const double expected = k_head * herr;              // 0.60

    EXPECT_DOUBLE_EQ(u.v, 0.0);
    EXPECT_NEAR(std::abs(u.omega), expected, 1e-9);      // 0.60, NOT the bang-bang omega_pivot = 1.0
    EXPECT_LT(std::abs(u.omega), params.omega_pivot)     // strictly sub-saturation => genuinely tapered
        << "terminal alignment must taper proportionally, not command omega_pivot bang-bang";
    EXPECT_FALSE(pp.done());                             // 0.15 > goal_heading_tol => not yet done
}

// (d) Anti-position-oscillation guard. After done() latches, a position excursion that LEAVES the
// position tolerance (0.25 m > goal_pos_tol = 0.10) with heading still aligned must NOT re-drive the
// base -- re-engagement is in_pos-gated, so the latch holds and the command stays a zero Control. This
// guards the explicit in_pos gate: a transient position kick keeps the base stopped, never
// restarts the drive toward the goal.
TEST(PurePursuitGoalAlign, holdsStoppedUnderPositionExcursion) {
    const auto traj = straightForward(/*length=*/1.0);
    const SE2State goal = traj.back().pose;   // {1.0, 0.0, 0.0}
    PurePursuit pp(nominalParams(), traj, 0.05);

    (void)pp.step(SE2State{goal.x, goal.y, 0.0});   // latch done
    ASSERT_TRUE(pp.done());

    // 0.25 m off the goal (> goal_pos_tol), heading still aligned: in_pos == false, so the re-engage
    // gate (in_pos && large_heading_drift) stays closed and the base holds stopped.
    const Control u = pp.step(SE2State{goal.x - 0.25, goal.y, 0.0});
    EXPECT_TRUE(pp.done()) << "position jitter alone must not un-latch done()";
    EXPECT_DOUBLE_EQ(u.v, 0.0) << "base must stay stopped, not re-drive toward the goal";
    EXPECT_DOUBLE_EQ(u.omega, 0.0);
}

// ---- speed_scale -------------------------------------------------------------------------------

namespace {

// Planar Euler step of a unicycle; enough to close the loop for a scale test.
SE2State eulerStep(const SE2State & p, const Control & u) {
    return SE2State{p.x + u.v * std::cos(p.theta) * u.dt,
                    p.y + u.v * std::sin(p.theta) * u.dt,
                    wrapToPi(p.theta + u.omega * u.dt)};
}

}  // namespace

// ---- endpoint: overshoot guard and smooth slow-down --------------------------------------------

namespace {

PurePursuitParams shippedGains() { return PurePursuitParams{0.45, 0.5, 0.5, 1.0, 0.10, 0.10}; }

std::vector<TrajectoryPoint> reverseStraight(double length, double spacing = 0.1) {
    std::vector<TrajectoryPoint> traj;
    for (double x = 0.0; x <= length + 1e-9; x += spacing) {
        traj.push_back(TrajectoryPoint{SE2State{-x, 0.0, 0.0}, MotionMode::Reverse, -0.5});
    }
    return traj;
}

}  // namespace

TEST(PurePursuitOvershoot, pastTheEndForwardCommandsZero) {
    for (double y : {0.0, 0.01}) {
        PurePursuit pp(shippedGains(), straightForward(1.0), 0.05);
        const Control u = pp.step(SE2State{1.15, y, 0.0});
        EXPECT_EQ(u.v, 0.0) << "y = " << y;
        EXPECT_EQ(u.omega, 0.0) << "y = " << y;
        EXPECT_FALSE(pp.done()) << "y = " << y;
    }
}

TEST(PurePursuitOvershoot, pastTheEndReverseCommandsZero) {
    PurePursuit pp(shippedGains(), reverseStraight(1.0), 0.05);
    const Control u = pp.step(SE2State{-1.15, 0.0, 0.0});
    EXPECT_EQ(u.v, 0.0);
    EXPECT_EQ(u.omega, 0.0);
    EXPECT_FALSE(pp.done());
}

TEST(PurePursuitOvershoot, beforeTheEndStillDrives) {
    PurePursuit pp(shippedGains(), straightForward(1.0), 0.05);
    EXPECT_GT(pp.step(SE2State{0.85, 0.0, 0.0}).v, 0.0);
}

TEST(PurePursuitOvershoot, endpointSlowDownIsLinearInRemainingDistance) {
    const PurePursuitParams params = shippedGains();
    PurePursuit pp(params, straightForward(1.0), 0.05);
    SE2State pose{0.0, 0.0, 0.0};
    const SE2State end{1.0, 0.0, 0.0};
    double prev = std::numeric_limits<double>::infinity();
    int n = 0;
    for (int i = 0; i < 2000 && !pp.done(); ++i) {
        const double d = std::hypot(end.x - pose.x, end.y - pose.y);
        const Control u = pp.step(pose);
        if (d <= params.lookahead) {
            ++n;
            const double speed = std::abs(u.v);
            if (speed == 0.0) break;
            EXPECT_LE(speed, prev + 1e-9) << "step " << i << " d = " << d;
            EXPECT_NEAR(speed, params.v_cruise * d / params.lookahead, 1e-6)
                << "step " << i << " d = " << d;
            prev = speed;
        }
        pose = eulerStep(pose, u);
    }
    EXPECT_GT(n, 3);
}

TEST(PurePursuitSpeedScale, unitScaleIsBitIdenticalToDefault) {
    const double dt = 0.05;
    PurePursuit a(nominalParams(), straightForward(), dt);
    PurePursuit b(nominalParams(), straightForward(), dt, 1.0);
    SE2State pa{0.0, 0.4, 0.3};
    SE2State pb = pa;
    for (int i = 0; i < 400 && !(a.done() && b.done()); ++i) {
        const Control ua = a.step(pa);
        const Control ub = b.step(pb);
        ASSERT_EQ(ua.v, ub.v);
        ASSERT_EQ(ua.omega, ub.omega);
        ASSERT_EQ(a.cursor(), b.cursor());
        pa = eulerStep(pa, ua);
        pb = eulerStep(pb, ub);
    }
    EXPECT_TRUE(a.done());
    EXPECT_TRUE(b.done());
}

TEST(PurePursuitSpeedScale, straightRunStaysUnderScaledCruiseAndReachesGoal) {
    const PurePursuitParams params = nominalParams();
    const double s = 0.2, dt = 0.05;
    PurePursuit pp(params, straightForward(), dt, s);
    SE2State pose{0.0, 0.0, 0.0};
    double vmax_seen = 0.0;
    for (int i = 0; i < 2000 && !pp.done(); ++i) {
        const Control u = pp.step(pose);
        EXPECT_LE(std::abs(u.v), s * params.v_cruise + 1e-12);
        vmax_seen = std::max(vmax_seen, std::abs(u.v));
        pose = eulerStep(pose, u);
    }
    EXPECT_TRUE(pp.done());
    EXPECT_GT(vmax_seen, 0.5 * s * params.v_cruise);  // actually moves, near the scaled ceiling
}

TEST(PurePursuitSpeedScale, pivotRateStaysUnderScaledOmegaPivot) {
    const PurePursuitParams params = nominalParams();
    const double s = 0.2, dt = 0.05;
    // Mid-path pivot (bang-bang) followed by a goal pivot (proportional taper), co-located.
    std::vector<TrajectoryPoint> traj{
        TrajectoryPoint{SE2State{0.0, 0.0, kPi / 2.0}, MotionMode::Pivot, 0.0},
        TrajectoryPoint{SE2State{0.0, 0.0, kPi / 2.0}, MotionMode::Pivot, 0.0}};
    PurePursuit pp(params, traj, dt, s);
    SE2State pose{0.0, 0.0, 0.0};
    for (int i = 0; i < 2000 && !pp.done(); ++i) {
        const Control u = pp.step(pose);
        EXPECT_DOUBLE_EQ(u.v, 0.0);
        EXPECT_LE(std::abs(u.omega), s * params.omega_pivot + 1e-12);
        pose = eulerStep(pose, u);
    }
    EXPECT_TRUE(pp.done());
}

TEST(PurePursuitSpeedScale, rejectsOutOfRangeScale) {
    for (double s : {0.0, -0.1, 1.0001, std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
        EXPECT_THROW(PurePursuit(nominalParams(), straightForward(), 0.05, s),
                     std::invalid_argument)
            << "speed_scale " << s;
    }
}

// ---- reverse-only straight path, closed loop at the shipped controller gains -------------------

namespace {

struct ReverseRun {
    bool   done = false;
    double t_done = 0.0, pos_err = 0.0, head_err = 0.0, max_omega = 0.0, max_abs_v = 0.0;
    bool   v_positive = false;
    std::size_t n_poses = 0;
    double longest_gap = 0.0;  ///< Longest time the follower's cursor stayed unchanged while not done (s).
    double total_time = 0.0;   ///< Time of the step that reported done, or the run limit (s).
    SE2State final_pose{};
};

// Shipped gains: controller.yaml with robot_params.yaml (square 0.6 m => r_in = 0.3, spacing r_in/2 = 0.15,
// lookahead = 3 * 0.15 = 0.45). Path poses follow sampleTrajectory's rule for a straight segment:
// n = ceil(L / spacing) intervals, n + 1 poses, evenly spaced from (0,0) back along -x at theta = 0.
ReverseRun runReverse(double L, SE2State pose, double speed_scale = 0.2) {
    const PurePursuitParams params{0.45, 0.5, 0.5, 1.0, 0.10, 0.10};
    const double dt = 1.0 / 20.0;
    const int n = std::max(1, static_cast<int>(std::ceil(L / 0.15)));
    std::vector<TrajectoryPoint> traj;
    for (int i = 0; i <= n; ++i) {
        traj.push_back(TrajectoryPoint{SE2State{-L * i / n, 0.0, 0.0}, MotionMode::Reverse, -0.5});
    }
    ReverseRun r;
    r.n_poses = traj.size();
    PurePursuit pp(params, traj, dt, speed_scale);
    int steps = 0;
    std::size_t prev_cursor = pp.cursor();
    double last_change = 0.0;
    for (; steps < static_cast<int>(60.0 / dt); ++steps) {
        const Control u = pp.step(pose);
        const double t_after = (steps + 1) * dt;
        if (pp.cursor() != prev_cursor) {
            r.longest_gap = std::max(r.longest_gap, t_after - last_change);
            last_change = t_after;
            prev_cursor = pp.cursor();
        }
        if (pp.done()) { r.done = true; r.longest_gap = std::max(r.longest_gap, t_after - last_change); r.total_time = t_after; break; }
        if (u.v > 0.0) r.v_positive = true;
        r.max_abs_v = std::max(r.max_abs_v, std::abs(u.v));
        r.max_omega = std::max(r.max_omega, std::abs(u.omega));
        if (std::abs(u.omega) < 1e-9) {
            pose.x += u.v * dt * std::cos(pose.theta);
            pose.y += u.v * dt * std::sin(pose.theta);
        } else {
            const double th1 = pose.theta + u.omega * dt;
            pose.x += u.v / u.omega * (std::sin(th1) - std::sin(pose.theta));
            pose.y -= u.v / u.omega * (std::cos(th1) - std::cos(pose.theta));
            pose.theta = th1;
        }
    }
    r.t_done = steps * dt;
    if (!r.done) { r.total_time = steps * dt; r.longest_gap = std::max(r.longest_gap, r.total_time - last_change); }
    r.pos_err = std::hypot(pose.x + L, pose.y);
    r.head_err = std::abs(wrapToPi(pose.theta));
    r.final_pose = pose;
    return r;
}

void checkReverse(double L, SE2State start) {
    const ReverseRun r = runReverse(L, start);
    std::cout << "REVCASE L=" << L << " start=(" << start.x << "," << start.y << "," << start.theta
              << ") poses=" << r.n_poses << " t_done=" << r.t_done << " pos_err=" << r.pos_err
              << " head_err=" << r.head_err << " max_omega=" << r.max_omega
              << " max_abs_v=" << r.max_abs_v << " final=(" << r.final_pose.x << ","
              << r.final_pose.y << "," << r.final_pose.theta << ")\n";
    ::testing::Test::RecordProperty("t_done", std::to_string(r.t_done));
    ::testing::Test::RecordProperty("pos_err", std::to_string(r.pos_err));
    ::testing::Test::RecordProperty("head_err", std::to_string(r.head_err));
    ::testing::Test::RecordProperty("max_omega", std::to_string(r.max_omega));
    EXPECT_TRUE(r.done);
    EXPECT_FALSE(r.v_positive);
    EXPECT_LE(r.max_abs_v, 0.2 * 0.5 + 1e-9);
    EXPECT_LE(r.pos_err, 0.10);
    EXPECT_LE(r.head_err, 0.10);
}

}  // namespace

TEST(PurePursuitReverseOnly, L008) { checkReverse(0.08, SE2State{0.0, 0.0, 0.0}); }
TEST(PurePursuitReverseOnly, L012) { checkReverse(0.12, SE2State{0.0, 0.0, 0.0}); }
TEST(PurePursuitReverseOnly, L020) { checkReverse(0.20, SE2State{0.0, 0.0, 0.0}); }
TEST(PurePursuitReverseOnly, L172) { checkReverse(1.72, SE2State{0.0, 0.0, 0.0}); }
TEST(PurePursuitReverseOnly, L012Offset) { checkReverse(0.12, SE2State{0.01, 0.01, 0.02}); }

// ---- stall margin: longest time the progress cursor stays put while not done ---------------------
// The navigator declares a stall after stall_timeout (3.0 s) without a cursor change and requires
// stall_timeout > crawl(s) = (L/(s*v_cruise)) * max(ln(L/goal_pos_tol), spacing/L), with L the
// lookahead (0.45), spacing 0.15, v_cruise 0.5, goal_pos_tol 0.10.
namespace {
double crawl(double s) { return (0.45 / (s * 0.5)) * std::max(std::log(0.45 / 0.10), 0.15 / 0.45); }
}  // namespace

TEST(PurePursuitReverseOnly, StallMarginTable) {
    const double lengths[] = {0.18, 0.22, 0.30, 0.60, 1.82};
    const double scales[] = {0.2, 0.5, 1.0};
    std::cout << "STALLTABLE  L(m) scale poses longest_gap(s) total(s) crawl(s)\n";
    for (double L : lengths) {
        for (double s : scales) {
            const ReverseRun r = runReverse(L, SE2State{0.0, 0.0, 0.0}, s);
            std::cout << "STALLTABLE  " << L << " " << s << " " << r.n_poses << " " << r.longest_gap << " "
                      << r.total_time << " " << crawl(s) << (r.done ? "" : " NOT_DONE") << "\n";
            if (s < 0.2 + 1e-9) continue;  // scale 0.2: record only
            EXPECT_TRUE(r.done) << "L=" << L << " s=" << s;
            EXPECT_LE(r.longest_gap, crawl(s) + 0.05) << "L=" << L << " s=" << s;
            EXPECT_LT(r.longest_gap, 3.0) << "L=" << L << " s=" << s;
        }
    }
}

// Planned 180-degree pivot at the end of a path: the navigator declares a stall when the follower
// cursor is unchanged for 3 s, so the cursor must keep advancing through the turn. Samples are spaced
// as MotionContext::sampleTrajectory does for the 0.6 m square (r_in = 0.30, r_out = 0.4243): straight
// runs in ceil(length / 0.15) even intervals, pivot samples at most r_in/(2 r_out) = 0.3536 rad apart,
// split evenly (ceil(pi / 0.3536) = 9 intervals -> pi/9 = 0.349 rad). Shipped controller gains,
// dt = 0.05 s, ideal pose integrator. Lengths other than 1.0 m vary where the samples fall.
TEST(PurePursuitPivot, endOfPathHalfTurnFinishesWithinTheStallBound) {
    const PurePursuitParams params{0.45, 0.5, 0.5, 1.0, 0.10, 0.10};
    const double dt = 0.05;
    // Closed-form stall bound from the literal settings: lookahead 0.45 m, cruise 0.5 m/s, goal_pos_tol
    // 0.10 m, spacing 0.15 m, goal_heading_tol 0.10 rad, omega_pivot 1.0 rad/s.
    const double crawl = (0.45 / 0.5) * std::max(std::log(0.45 / 0.10), 0.15 / 0.45);
    const double h_sat = 2.5 * 0.10;
    const double turn = (kPi - h_sat) / 1.0 + (h_sat / 1.0) * std::log(2.5);
    for (double length : {1.0, 0.9, 0.95, 1.05, 1.1, 1.2}) {
        std::vector<TrajectoryPoint> traj;
        const int nd = static_cast<int>(std::ceil(length / 0.15));
        for (int i = 0; i <= nd; ++i)
            traj.push_back(TrajectoryPoint{SE2State{length * i / nd, 0.0, 0.0}, MotionMode::Forward, 0.5});
        const int n = static_cast<int>(std::ceil(kPi / (0.30 / (2.0 * 0.424264))));
        for (int i = 1; i <= n; ++i)
            traj.push_back(TrajectoryPoint{SE2State{length, 0.0, wrapToPi(kPi * i / n)}, MotionMode::Pivot, 0.0});

        PurePursuit pp(params, traj, dt);
        SE2State pose{0.0, 0.0, 0.0};
        double t = 0.0, last_change_t = 0.0, longest_gap = 0.0, arrive_t = -1.0;
        std::size_t last_cursor = pp.cursor();
        for (int i = 0; i < 4000 && !pp.done(); ++i, t += dt) {
            const Control u = pp.step(pose);
            if (pp.cursor() != last_cursor) {
                longest_gap = std::max(longest_gap, t - last_change_t);
                last_change_t = t;
                last_cursor = pp.cursor();
            }
            if (arrive_t < 0.0 && std::hypot(pose.x - length, pose.y) <= params.goal_pos_tol) arrive_t = t;
            pose = eulerStep(pose, u);
        }
        longest_gap = std::max(longest_gap, t - last_change_t);
        std::cout << "[pivot-stall] length " << length << " m, pivot spacing " << kPi / n
                  << " rad: done=" << pp.done() << " longest cursor gap " << longest_gap
                  << " s, arrival-in-position to done " << (arrive_t < 0.0 ? -1.0 : t - arrive_t)
                  << " s, final pose (" << pose.x << ", " << pose.theta << ")\n";
        EXPECT_TRUE(pp.done()) << "length " << length;
        EXPECT_LE(longest_gap, crawl + turn) << "length " << length;
        ASSERT_GE(arrive_t, 0.0) << "length " << length;
        EXPECT_LE(t - arrive_t, turn + 2.0 * dt) << "length " << length;
        EXPECT_LE(t - arrive_t, PurePursuit::goalAlignTimeBound(1.0, 0.10) + 2.0 * dt) << "length " << length;
    }
}

TEST(PurePursuitGoalAlignTimeBound, matchesClosedForm) {
    const double expected = (kPi - 0.25) + 0.25 * std::log(2.5);
    EXPECT_NEAR(PurePursuit::goalAlignTimeBound(1.0, 0.10), expected, 1e-6);
    EXPECT_NEAR(expected, 3.120666, 1e-6);
    const double half = PurePursuit::goalAlignTimeBound(2.0, 0.10);
    EXPECT_NEAR(half, 0.5 * PurePursuit::goalAlignTimeBound(1.0, 0.10), 1e-12 * expected);
}

TEST(PurePursuitGoalAlignTimeBound, neverSaturatedBranch) {
    EXPECT_NEAR(PurePursuit::goalAlignTimeBound(1.0, 1.3), 3.25 * std::log(kPi / 1.3), 1e-9);
}

TEST(PurePursuitGoalAlignTimeBound, rejectsInvalidArguments) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double w : {0.0, -1.0, nan})
        EXPECT_THROW(PurePursuit::goalAlignTimeBound(w, 0.10), std::invalid_argument) << w;
    for (double h : {0.0, -0.1, kPi, kPi + 0.1, nan})
        EXPECT_THROW(PurePursuit::goalAlignTimeBound(1.0, h), std::invalid_argument) << h;
}

TEST(PurePursuitOvershoot, cuspNearTheEndDoesNotTripTheGuard) {
    std::vector<TrajectoryPoint> traj;
    for (double x = 0.0; x <= 1.0 + 1e-9; x += 0.1) {
        traj.push_back(TrajectoryPoint{SE2State{x, 0.0, 0.0}, MotionMode::Forward, 0.5});
    }
    traj.push_back(TrajectoryPoint{SE2State{0.9, 0.0, 0.0}, MotionMode::Reverse, -0.5});
    traj.push_back(TrajectoryPoint{SE2State{0.8, 0.0, 0.0}, MotionMode::Reverse, -0.5});
    PurePursuit pp(shippedGains(), traj, 0.05);
    // 0.15 m from the path end (outside goal_pos_tol); the nearest sample is a Forward one.
    EXPECT_GT(pp.step(SE2State{0.95, 0.0, 0.0}).v, 0.0);
}
