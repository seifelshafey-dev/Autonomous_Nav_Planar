/**
 * @file pure_pursuit.cpp
 * @author Seif ElShafey
 * @brief Implementation of the pure-pursuit control law (see pure_pursuit.hpp).
 * @version 0.1
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "core/diagnostics/check.hpp"
#include "core/types/math/wrap_angle.hpp"

namespace {

/// Squared planar distance between two poses' positions.
double posDistSq(const SE2State & a, const SE2State & b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return dx * dx + dy * dy;
}

// --- Terminal goal-heading alignment tuning -----------------------------------------------------
//
// The final heading alignment tapers proportionally: omega = clamp(k_head * herr, +-omega_pivot),
// with k_head sized so the command saturates at kHeadingSaturateFactor * goal_heading_tol. That is,
// the base turns at full omega_pivot until it is within a few tolerances of the target heading, then
// decelerates smoothly into it -- no bang-bang overshoot, no hunting.
//
//   k_head = omega_pivot / (kHeadingSaturateFactor * goal_heading_tol)
//
// Deriving k_head from the gains keeps it self-consistent across configs (it always saturates at the
// same multiple of the tolerance, whatever omega_pivot / goal_heading_tol are). At the default gains
// (omega_pivot = 1.0 rad/s, goal_heading_tol = 0.10 rad) this gives k_head = 4.0 rad/s per rad,
// saturating at 0.25 rad (2.5x tol) of heading error.
constexpr double kHeadingSaturateFactor = 2.5;

// Deadband + hysteresis. Alignment is declared done at goal_heading_tol; once done, the flag holds
// under position jitter and small heading noise, and only re-opens (re-engages the pivot) if the
// heading error later exceeds kHeadingReengageFactor * goal_heading_tol WHILE still within the
// position tolerance. The wider re-engage band than the stop band is the hysteresis that keeps
// residual localization jitter around the boundary from restarting the pivot.
constexpr double kHeadingReengageFactor = 1.5;

}  // namespace

PurePursuit::PurePursuit(const PurePursuitParams & params,
                         std::vector<TrajectoryPoint> trajectory,
                         double dt,
                         double speed_scale)
    : params_(params), trajectory_(std::move(trajectory)), dt_(dt), speed_scale_(speed_scale) {
    CHECK(params_.lookahead > 0.0, "PurePursuitParams.lookahead must be > 0");
    CHECK(params_.v_cruise > 0.0, "PurePursuitParams.v_cruise must be > 0");
    CHECK(params_.a_lat_max > 0.0, "PurePursuitParams.a_lat_max must be > 0");
    CHECK(params_.omega_pivot > 0.0, "PurePursuitParams.omega_pivot must be > 0");
    CHECK(params_.goal_pos_tol > 0.0, "PurePursuitParams.goal_pos_tol must be > 0");
    CHECK(params_.goal_heading_tol > 0.0, "PurePursuitParams.goal_heading_tol must be > 0");
    CHECK(dt_ > 0.0, "PurePursuit control period dt must be > 0");
    CHECK(!trajectory_.empty(), "PurePursuit trajectory must be non-empty");
    CHECK(std::isfinite(speed_scale_) && speed_scale_ > 0.0 && speed_scale_ <= 1.0,
          "PurePursuit speed_scale must be in (0, 1]");
}

double PurePursuit::headingError(double from, double to) {
    return wrapToPi(to - from);
}

void PurePursuit::reset() {
    cursor_ = 0;
    done_   = false;
}

void PurePursuit::advanceCursor(const SE2State & pose) {
    // Squared radius under which a reference point is treated as co-located with the base
    // position, i.e. offers no translation to pursue. Real hybrid A* plans stack several
    // leading poses at the exact start position; dense translating trajectories are spaced
    // far above this (0.1 m spacing => 1e-2 m^2, vs 1e-6 m^2 here) so normal points are
    // never treated as co-located.
    constexpr double kColocatedRadiusSq = 1e-6;

    // Phase 1 -- objective-based skip. Leading pivots and poses stacked on the start
    // position translate the base by zero, so the position-nearest scan below can never
    // move off them (their distance stays 0). Retire them by their per-mode objective:
    //   - Pivot: a purely in-place rotation; retired once the base heading has converged
    //            to the pivot's target heading (within goal_heading_tol).
    //   - Forward/Reverse: retired once co-located with the base, since a carrot on top of
    //            the base offers nothing to drive toward.
    // Never advance past the final pose (the goal test in step() owns termination there),
    // and the cursor only ever moves forward, so this stays monotonic.
    while (cursor_ + 1 < trajectory_.size()) {
        const TrajectoryPoint & tp = trajectory_[cursor_];
        bool objective_met = false;
        if (tp.mode == MotionMode::Pivot) {
            objective_met =
                std::abs(headingError(pose.theta, tp.pose.theta)) < params_.goal_heading_tol;
        } else {
            // Skip a co-located drive point only while at least one point remains AFTER the
            // one we would advance onto (cursor_ + 2 < size). This clears leading start-stacked
            // drive points without ever landing the cursor on the final pose, which would zero
            // the endpoint deceleration ramp and stall the base short of the goal. A drive point
            // the base merely arrives at mid-path is left to the position-nearest scan below.
            objective_met = cursor_ + 2 < trajectory_.size() &&
                            posDistSq(pose, tp.pose) < kColocatedRadiusSq;
        }
        if (!objective_met) break;
        ++cursor_;
    }

    // Phase 2 -- position-nearest scan. From the (possibly advanced) cursor, snap to the
    // nearest reference point at or after it. Monotonic: only ever moved forward, so passed
    // points are never re-tracked. This is the tracking behaviour for translating paths.
    double      best_dist = posDistSq(pose, trajectory_[cursor_].pose);
    std::size_t best_idx  = cursor_;
    for (std::size_t i = cursor_ + 1; i < trajectory_.size(); ++i) {
        const double d = posDistSq(pose, trajectory_[i].pose);
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }
    cursor_ = best_idx;
}

Control PurePursuit::pivotToward(double current_theta, double target_theta) const {
    const double err = headingError(current_theta, target_theta);
    const double sign = (err >= 0.0) ? 1.0 : -1.0;
    return Control{0.0, sign * speed_scale_ * params_.omega_pivot, dt_};
}

double PurePursuit::goalAlignTimeBound(double omega_pivot, double goal_heading_tol) {
    CHECK(std::isfinite(omega_pivot) && omega_pivot > 0.0, "omega_pivot must be finite and > 0");
    CHECK(std::isfinite(goal_heading_tol) && goal_heading_tol > 0.0 && goal_heading_tol < M_PI,
          "goal_heading_tol must be finite, > 0 and < pi");
    const double h_sat = kHeadingSaturateFactor * goal_heading_tol;
    if (h_sat >= M_PI) return (h_sat / omega_pivot) * std::log(M_PI / goal_heading_tol);
    return (M_PI - h_sat) / omega_pivot +
           (h_sat / omega_pivot) * std::log(kHeadingSaturateFactor);
}

Control PurePursuit::pivotToGoal(double heading_error) const {
    // Proportional taper, saturating at +-speed_scale*omega_pivot; the gain stays derived from the
    // unscaled omega_pivot. For |herr| beyond the saturation point this is
    // identical to the fixed-rate pivot (sign * omega_pivot); as |herr| falls below it the rate eases
    // toward zero, so the base decelerates into the goal heading instead of overshooting and hunting.
    const double k_head =
        params_.omega_pivot / (kHeadingSaturateFactor * params_.goal_heading_tol);
    const double omega_max = speed_scale_ * params_.omega_pivot;
    const double omega = std::clamp(k_head * heading_error, -omega_max, omega_max);
    return Control{0.0, omega, dt_};
}

Control PurePursuit::driveTowardCarrot(const SE2State & pose, MotionMode mode) const {
    const double dir = (mode == MotionMode::Reverse) ? -1.0 : 1.0;

    // Pursuit heading: the direction of travel. Forward drives along theta; reverse drives
    // along theta+pi, so the carrot (behind the robot in body frame) is ahead in the pursuit
    // frame. Feeding the resulting Control to propagate then curves the base onto the path.
    const double psi = (mode == MotionMode::Reverse) ? wrapToPi(pose.theta + M_PI)
                                                     : pose.theta;

    // Walk forward from the cursor to the first reference point at least `lookahead` away.
    // Clamp to the last point near the goal so the carrot never runs off the trajectory.
    const double L      = params_.lookahead;
    const double L_sq   = L * L;
    std::size_t  carrot = cursor_;
    for (std::size_t i = cursor_; i < trajectory_.size(); ++i) {
        carrot = i;
        if (posDistSq(pose, trajectory_[i].pose) >= L_sq) break;
    }
    const SE2State & c = trajectory_[carrot].pose;

    // Carrot in the pursuit (travel-direction) body frame.
    const double dx_w = c.x - pose.x;
    const double dy_w = c.y - pose.y;
    const double cp   = std::cos(psi);
    const double sp   = std::sin(psi);
    const double x_c  =  cp * dx_w + sp * dy_w;
    const double y_c  = -sp * dx_w + cp * dy_w;

    const double ld_sq = x_c * x_c + y_c * y_c;
    if (ld_sq < 1e-9) {
        // Carrot coincides with the robot: nothing to pursue this tick.
        return Control{0.0, 0.0, dt_};
    }

    if (carrot == trajectory_.size() - 1 && x_c <= 0.0 && trajectory_[carrot].mode == mode) {
        // Path end is beside or behind the robot in its travel direction: pursuing it
        // would drive away from the goal. Only when the final segment is the one being driven;
        // otherwise a direction switch lies ahead and the clamped carrot is not the goal.
        return Control{0.0, 0.0, dt_};
    }

    // Pure-pursuit curvature of the arc through the carrot, expressed in the pursuit frame.
    const double kappa = 2.0 * y_c / ld_sq;

    // Speed magnitude: cruise ceiling, curvature (lateral-accel) clamp, endpoint ramp.
    const double v_max = speed_scale_ * params_.v_cruise;
    double speed = v_max;
    const double abs_k = std::abs(kappa);
    if (abs_k > 1e-9) {
        speed = std::min(speed, std::sqrt(params_.a_lat_max / abs_k));
    }

    // Endpoint ramp: linearly ease |v| to zero over the last lookahead of distance
    // remaining, measured from the robot to the next sample after the cursor and then along
    // the path, so the base decelerates into the goal rather than overshooting yet never
    // stalls short of it.
    const std::size_t j = std::min(cursor_ + 1, trajectory_.size() - 1);
    double remaining = std::sqrt(posDistSq(pose, trajectory_[j].pose));
    for (std::size_t i = j; i + 1 < trajectory_.size(); ++i) {
        remaining += std::sqrt(posDistSq(trajectory_[i].pose, trajectory_[i + 1].pose));
    }
    if (remaining < L) {
        speed = std::min(speed, v_max * (remaining / L));
    }

    // Body angular rate. Curvature is taken in the pursuit (travel-direction) frame, and the
    // travel heading psi differs from the body heading theta by a constant (0 forward, pi
    // reverse), so dpsi/dt == dtheta/dt == omega. The rate of the travel heading is
    // kappa * (forward travel speed) = kappa * speed (always the positive magnitude). Using
    // the signed v here would flip omega's sign in reverse and drive the base off the path.
    const double v     = dir * speed;
    const double omega = kappa * speed;
    return Control{v, omega, dt_};
}

Control PurePursuit::step(const SE2State & pose) {
    const TrajectoryPoint & goal = trajectory_.back();
    const bool in_pos =
        posDistSq(pose, goal.pose) <= params_.goal_pos_tol * params_.goal_pos_tol;

    // Terminal latch + hysteresis. Once done, hold a zero command; stay latched under position
    // jitter (a transient that leaves the position tolerance keeps us stopped, not re-driving) and
    // small heading noise. Re-open the alignment only on a genuine LARGE heading deviation while
    // still within the position tolerance -- the wide re-engage band keeps boundary jitter from
    // restarting the pivot, so done() does not oscillate on residual localization noise.
    if (done_) {
        const bool large_heading_drift =
            std::abs(headingError(pose.theta, goal.pose.theta)) >
            kHeadingReengageFactor * params_.goal_heading_tol;
        if (in_pos && large_heading_drift) {
            done_ = false;  // fall through and re-align proportionally
        } else {
            return Control{0.0, 0.0, dt_};
        }
    }

    advanceCursor(pose);

    // Goal handling: within position tolerance of the final reference point, align heading to the
    // goal. Deadband at goal_heading_tol -> latch done and stop; otherwise taper proportionally
    // into the target heading (pivotToGoal) so the base eases in instead of slamming at full rate.
    if (in_pos) {
        const double herr = headingError(pose.theta, goal.pose.theta); 
        if (std::abs(herr) < params_.goal_heading_tol) {
            done_ = true;
            return Control{0.0, 0.0, dt_};
        }
        return pivotToGoal(herr);
    }

    const TrajectoryPoint & tp = trajectory_[cursor_];
    if (tp.mode == MotionMode::Pivot) {
        return pivotToward(pose.theta, tp.pose.theta);
    }
    return driveTowardCarrot(pose, tp.mode);
}
