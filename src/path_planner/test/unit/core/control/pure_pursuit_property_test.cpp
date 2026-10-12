#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <memory>
#include <numbers>
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
PurePursuit property tests.

The pure-pursuit law is closed-form; the load-bearing property is that iterating
    u = pursuit.step(pose);  pose = DiffDriveStateSpace::propagate(pose, u);
from an OFF-PATH start pose drives the robot ONTO and ALONG a generated trajectory, and reaches the
goal within tolerance -- for BOTH Forward and Reverse. This is the reverse-sign guard: the law commands
omega = kappa * speed (positive magnitude, not kappa * v), so the follower and the kinematic model must
agree on which way the base turns when backing up. If the reverse sign were wrong, the closed loop would
diverge (cross-track error would grow) and the goal would never be reached -- so we do NOT hand-assert a
sign here; we let the real kinematic model be the oracle and require genuine convergence.

Generators use bounded rc::gen::inRange to keep the geometry tame (gentle curvature, ints exactly
representable as doubles), and each polyline is densely resampled below the lookahead so the carrot walk
behaves as designed. The DiffDriveStateSpace is built with a large workspace so propagate is never
gated by bounds (only propagate is used; inBounds is not consulted by the closed loop).
*/

namespace {

constexpr double kPi = std::numbers::pi;

// Wide-open state space: the closed loop only calls propagate(), never inBounds(), so bounds just need
// to exist. rmin only feeds the (unused-here) distance functor.
DiffDriveStateSpace makeSpace() {
    DiffDriveParams kin;
    kin.rmin = 0.3; kin.vmax = 3.0; kin.atomic_time = 1.0; kin.n_headings = 8;
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    // 1000 x 1000 m workspace centred well away from any test geometry.
    return DiffDriveStateSpace(
        GridMetaData{1000, 1000, 1.0, Eigen::Vector2f(-500.0f, -500.0f)}, dist);
}

PurePursuitParams params() {
    // Small dt for a fine closed loop; gains at the config seeds but a slightly larger a_lat_max so
    // gentle generated arcs are not speed-limited into a crawl (keeps step counts bounded).
    return PurePursuitParams{
        /*lookahead=*/0.75,
        /*v_cruise=*/1.0,
        /*a_lat_max=*/4.0,
        /*omega_pivot=*/1.5,
        /*goal_pos_tol=*/0.10,
        /*goal_heading_tol=*/0.10};
}

// Resample a coarse polyline into dense reference points spaced <= `spacing`, headings tangent to the
// segment. `mode` tags every point (Forward or Reverse). The tangent heading is the geometric travel
// direction of the path regardless of mode (Reverse means the base backs along this same tangent).
std::vector<TrajectoryPoint> densify(const std::vector<Eigen::Vector2d> & pts,
                                     MotionMode mode, double spacing = 0.1) {
    std::vector<TrajectoryPoint> traj;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Eigen::Vector2d a = pts[i];
        const Eigen::Vector2d b = pts[i + 1];
        const Eigen::Vector2d d = b - a;
        const double len = d.norm();
        if (len < 1e-9) continue;
        const double heading = std::atan2(d.y(), d.x());
        const int n = std::max(1, static_cast<int>(std::ceil(len / spacing)));
        for (int k = 0; k < n; ++k) {
            const double t = static_cast<double>(k) / n;
            const Eigen::Vector2d p = a + t * d;
            const double speed = (mode == MotionMode::Reverse) ? -1.0 : 1.0;
            traj.push_back(TrajectoryPoint{SE2State{p.x(), p.y(), heading}, mode, speed});
        }
    }
    // Final point.
    const Eigen::Vector2d last = pts.back();
    const Eigen::Vector2d prev = pts[pts.size() - 2];
    const double heading = std::atan2(last.y() - prev.y(), last.x() - prev.x());
    const double speed = (mode == MotionMode::Reverse) ? -1.0 : 1.0;
    traj.push_back(TrajectoryPoint{SE2State{last.x(), last.y(), heading}, mode, speed});
    return traj;
}

// Min distance from `p` to any reference point in the trajectory (cross-track proxy: the trajectory is
// dense, so nearest-vertex distance approximates perpendicular distance to the path).
double crossTrack(const SE2State & p, const std::vector<TrajectoryPoint> & traj) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & tp : traj) {
        const double dx = tp.pose.x - p.x;
        const double dy = tp.pose.y - p.y;
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best;
}

// Run the closed loop; returns {reached_goal, max_cross_track_after_settling}.
struct LoopResult { bool reached; double settled_xtrack; };

LoopResult runLoop(const std::vector<TrajectoryPoint> & traj, const SE2State & start,
                   MotionMode mode) {
    const DiffDriveStateSpace space = makeSpace();
    const PurePursuitParams pp_params = params();
    const double dt = 0.05;
    PurePursuit pp(pp_params, traj, dt);

    SE2State pose = start;
    const int kMaxSteps = 4000;
    double settled_xtrack = 0.0;
    // "Settled" = after the loop has had time to pull onto the path. We measure the worst cross-track
    // over the LATTER portion of the run (once transient acquisition is over).
    const int settle_after = 400;

    int step = 0;
    for (; step < kMaxSteps && !pp.done(); ++step) {
        const Control u = pp.step(pose);
        pose = space.propagate(pose, u);
        pose.theta = wrapToPi(pose.theta);
        if (step >= settle_after) {
            settled_xtrack = std::max(settled_xtrack, crossTrack(pose, traj));
        }
    }
    (void)mode;
    return LoopResult{pp.done(), settled_xtrack};
}

}  // namespace

// ---- Forward convergence -----------------------------------------------------------------------

RC_GTEST_PROP(PurePursuitLoopForward, convergesOntoStraightAndReachesGoal,
              (int len_dm, int off_dm, int side)) {
    // Straight line along +x of length in [2.0, 6.0] m, start offset laterally by up to +-0.6 m.
    const double length = 2.0 + std::abs(len_dm % 41) / 10.0;   // [2.0, 6.0]
    const double offset = (std::abs(off_dm % 7)) / 10.0;         // [0.0, 0.6]
    const double signed_off = (side % 2 == 0) ? offset : -offset;

    std::vector<Eigen::Vector2d> pts{{0.0, 0.0}, {length, 0.0}};
    const auto traj = densify(pts, MotionMode::Forward);

    // Start off-path, facing roughly along the line (small heading error).
    const SE2State start{0.0, signed_off, 0.0};
    const LoopResult r = runLoop(traj, start, MotionMode::Forward);

    RC_ASSERT(r.reached);
    // On a straight path the follower must settle to well within a lookahead of the line.
    RC_ASSERT(r.settled_xtrack < 0.20);
}

RC_GTEST_PROP(PurePursuitLoopForward, convergesOntoGentleArc, (int r_dm, int off_dm, int side)) {
    // A gentle left arc: radius in [2.0, 5.0] m, quarter turn. Start offset laterally.
    const double radius = 2.0 + std::abs(r_dm % 31) / 10.0;     // [2.0, 5.0]
    const double offset = (std::abs(off_dm % 5)) / 10.0;        // [0.0, 0.4]
    const double signed_off = (side % 2 == 0) ? offset : -offset;

    std::vector<Eigen::Vector2d> pts;
    for (int k = 0; k <= 20; ++k) {
        const double a = (kPi / 2.0) * k / 20.0;
        pts.push_back({radius * std::sin(a), radius * (1.0 - std::cos(a))});
    }
    const auto traj = densify(pts, MotionMode::Forward);

    const SE2State start{0.0, signed_off, 0.0};
    const LoopResult r = runLoop(traj, start, MotionMode::Forward);

    RC_ASSERT(r.reached);
    RC_ASSERT(r.settled_xtrack < 0.30);   // gentle arcs allow a slightly larger steady-state lag
}

// ---- Reverse convergence (the reverse-sign guard) ----------------------------------------------

RC_GTEST_PROP(PurePursuitLoopReverse, convergesOntoStraightAndReachesGoal,
              (int len_dm, int off_dm, int side)) {
    const double length = 2.0 + std::abs(len_dm % 41) / 10.0;   // [2.0, 6.0]
    const double offset = (std::abs(off_dm % 7)) / 10.0;         // [0.0, 0.6]
    const double signed_off = (side % 2 == 0) ? offset : -offset;

    // Trajectory tangent points along +x; Reverse means the base backs down it. The reference points
    // run 0 -> +x, so the base must reverse in the +x world direction (body -x is world +x) => the
    // base faces -x (theta ~= pi). Start off-path facing pi.
    std::vector<Eigen::Vector2d> pts{{0.0, 0.0}, {length, 0.0}};
    const auto traj = densify(pts, MotionMode::Reverse);

    const SE2State start{0.0, signed_off, kPi};
    const LoopResult r = runLoop(traj, start, MotionMode::Reverse);

    // If the reverse sign convention were wrong, the base would peel AWAY from the line and never
    // reach the goal -- so both of these fail loudly on a sign regression.
    RC_ASSERT(r.reached);
    RC_ASSERT(r.settled_xtrack < 0.20);
}

RC_GTEST_PROP(PurePursuitLoopReverse, convergesOntoGentleArc, (int r_dm, int off_dm, int side)) {
    const double radius = 2.5 + std::abs(r_dm % 26) / 10.0;     // [2.5, 5.0]
    const double offset = (std::abs(off_dm % 4)) / 10.0;        // [0.0, 0.3]
    const double signed_off = (side % 2 == 0) ? offset : -offset;

    std::vector<Eigen::Vector2d> pts;
    for (int k = 0; k <= 20; ++k) {
        const double a = (kPi / 2.0) * k / 20.0;
        pts.push_back({radius * std::sin(a), radius * (1.0 - std::cos(a))});
    }
    const auto traj = densify(pts, MotionMode::Reverse);

    // Tangent at the first point is +x; reversing along it means the base faces -x (pi).
    const SE2State start{0.0, signed_off, kPi};
    const LoopResult r = runLoop(traj, start, MotionMode::Reverse);

    RC_ASSERT(r.reached);
    RC_ASSERT(r.settled_xtrack < 0.35);
}
