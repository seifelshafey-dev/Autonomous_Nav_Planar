/**
 * @file pure_pursuit.hpp
 * @author Seif ElShafey
 * @brief Pure, ROS-free pure-pursuit control law that tracks a flattened SE(2) trajectory.
 * @version 0.1
 * @date 2026-07-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstddef>
#include <vector>

#include "core/types/state.hpp"
#include "core/types/control.hpp"

/**
 * @brief Per-pose motion intent carried by a trajectory point.
 *
 * Selects which control law the follower applies at a given point: `Forward`/`Reverse`
 * drive along an arc toward a carrot; `Pivot` rotates in place.
 */
enum class MotionMode { Forward, Reverse, Pivot };

/**
 * @brief One point of a flattened trajectory the follower tracks.
 */
struct TrajectoryPoint {
    SE2State   pose;   ///< World-frame SE(2) pose of the reference point.
    MotionMode mode;   ///< Motion intent at this point (drive direction or in-place pivot).
    double     speed;  ///< Signed reference speed (m/s) the planner assigned to this point. Retained for diagnostics; the drive direction is taken from `mode`, the magnitude from the follower's own speed policy.
};

/**
 * @brief Tuning gains for the pure-pursuit follower. All values must be strictly positive.
 */
struct PurePursuitParams {
    double lookahead;         ///< Carrot lookahead distance L (m).
    double v_cruise;          ///< Hard ceiling on the commanded speed magnitude |v| (m/s), before `speed_scale`.
    double a_lat_max;         ///< Lateral-acceleration bound (m/s^2); slows the robot on tight arcs so that v^2*|kappa| <= a_lat_max.
    double omega_pivot;       ///< In-place turn rate magnitude used for pivots and heading alignment (rad/s), before `speed_scale`.
    double goal_pos_tol;      ///< Position tolerance for declaring the goal reached (m).
    double goal_heading_tol;  ///< Heading tolerance for the final goal-alignment pivot (rad).
};

/**
 * @brief Closed-form pure-pursuit path follower over a flattened SE(2) trajectory.
 *
 * Constructed with a set of gains and a trajectory (ordered start->goal). Each call to
 * `step` takes the robot's current world pose and returns the control to apply for one
 * control period. The only mutable state is a monotonic progress cursor that never walks
 * backward, so passed reference points are not re-tracked.
 *
 * The law is deterministic and dependency-free: no ROS, no TF, no I/O. Feed the returned
 * `Control` to a differential-drive kinematic `propagate` to advance the robot; iterating
 * `step`->`propagate` from an off-path start converges onto and follows the trajectory in
 * both forward and reverse.
 *
 * Preconditions (validated at construction):
 * - every `PurePursuitParams` field is strictly positive;
 * - the trajectory is non-empty.
 */
class PurePursuit {
public:
    /**
     * @brief Construct a follower bound to a trajectory.
     *
     * @param params     Pursuit gains (all fields must be > 0).
     * @param trajectory Ordered reference points start->goal (must be non-empty).
     * @param dt         Control period (s) reported on the returned `Control` (must be > 0).
     * @param speed_scale Fraction in (0, 1] of `v_cruise` and `omega_pivot` this follower may
     *                   command: |v| never exceeds `speed_scale * v_cruise` and the pivot rate never
     *                   exceeds `speed_scale * omega_pivot`. 1.0 applies the gains unchanged.
     * @throws std::invalid_argument if any precondition is violated, including `speed_scale`
     *         non-finite or outside (0, 1].
     */
    PurePursuit(const PurePursuitParams & params,
                std::vector<TrajectoryPoint> trajectory,
                double dt,
                double speed_scale = 1.0);

    /**
     * @brief Compute the control to apply for one control period at the given pose.
     *
     * Advances the progress cursor (monotonically): on a driving stretch to the reference point
     * nearest the pose at or after the current one, and on an in-place turn as each turn point's
     * heading is reached within `goal_heading_tol`; in both cases it also moves to any later
     * reference point strictly nearer the pose. It then either drives toward a carrot at the lookahead distance, pivots in place, or,
     * once within the goal position tolerance, aligns heading to the goal and reports
     * completion. While driving, the speed is limited by `sqrt(a_lat_max / |curvature|)` and,
     * over the last `lookahead` of remaining path distance, tapers to at most
     * `speed_scale * v_cruise * remaining / lookahead`. The terminal alignment tapers the turn rate proportionally as the heading
     * error shrinks (rather than commanding a fixed rate), so the base decelerates smoothly
     * into the goal heading instead of overshooting and hunting around the tolerance.
     *
     * When the robot is on the final drive segment and the path end is beside or behind it in
     * its travel direction while still outside the position tolerance, the command is zero
     * (the follower does not turn back); the caller's no-progress handling must deal with
     * that case.
     *
     * @param pose Current world-frame SE(2) pose of the robot.
     * @return Control to apply for the configured period. Returns a zero control once the
     *         goal position and heading tolerances are both satisfied.
     */
    Control step(const SE2State & pose);

    /**
     * @brief Rewind the follower to the start of its trajectory.
     *
     * Resets the progress cursor and the done latch; the bound trajectory and gains are unchanged.
     */
    void reset();

    /**
     * @brief Whether the follower has reached the goal within tolerance.
     *
     * Latches true once a `step` has satisfied both the position and heading goal tolerances,
     * and stays latched under position jitter and small heading noise (a hysteresis band guards
     * the boundary). It re-opens only if the heading later deviates by more than the re-engage
     * band while still within the position tolerance, so a genuine large disturbance is
     * re-aligned while boundary jitter never flips the flag.
     *
     * @return true while the follower is holding at the goal within tolerance.
     */
    bool done() const { return done_; }

    /**
     * @brief Index of the trajectory point currently being tracked.
     *
     * Never decreases except through `reset()`.
     *
     * @return Index in `[0, trajectory size)`.
     */
    std::size_t cursor() const { return cursor_; }

    /**
     * @brief Upper bound on the time the final in-place alignment needs for a half turn.
     *
     * Time in seconds for the follower's in-position alignment to bring a heading error of
     * pi radians inside `goal_heading_tol`, at speed scale 1, for an ideal robot that turns at
     * exactly the commanded rate (no acceleration limit, no control-period rounding).
     *
     * @param omega_pivot       Pivot rate limit in rad/s; finite and > 0.
     * @param goal_heading_tol  Heading tolerance in rad; finite, > 0 and < pi.
     * @return Time bound in seconds.
     * @throws std::invalid_argument if a parameter violates its constraint.
     */
    static double goalAlignTimeBound(double omega_pivot, double goal_heading_tol);

private:
    // Signed heading error wrapped to [-pi, pi]: shortest rotation from `from` to `to`.
    static double headingError(double from, double to);

    // Advance cursor_ toward `pose`, monotonically. First retires any leading reference points
    // whose per-mode objective is already met -- a pivot whose target heading the base has
    // reached, or a drive point co-located with the base position -- then snaps to the nearest
    // remaining point at or after the current cursor. Retiring stacked start-position poses is
    // what lets the follower rotate to the plan's initial heading and then start driving instead
    // of stalling on a leading in-place pivot.
    void advanceCursor(const SE2State & pose);

    // In-place rotation toward a target heading at a fixed rate (bang-bang +-speed_scale*omega_pivot);
    // commands v=0. Used for mid-path Pivot reference points, where an overshoot is harmless
    // (the cursor retires the pivot once its heading is met and the base moves on).
    Control pivotToward(double current_theta, double target_theta) const;

    // In-place rotation for the FINAL goal-heading alignment; commands v=0. Unlike pivotToward,
    // the rate tapers proportionally with the heading error (clamped to +-speed_scale*omega_pivot) so the
    // base eases into the goal heading instead of slamming at full rate and hunting around the
    // tolerance. `heading_error` is the signed shortest rotation to the goal heading (as returned
    // by headingError).
    Control pivotToGoal(double heading_error) const;

    // Carrot-following arc for a drive mode (Forward/Reverse).
    Control driveTowardCarrot(const SE2State & pose, MotionMode mode) const;

    PurePursuitParams            params_;
    std::vector<TrajectoryPoint> trajectory_;
    double                       dt_;
    double                       speed_scale_;
    std::size_t                  cursor_ = 0;   ///< Monotonic progress index into trajectory_.
    bool                         done_   = false;
};
