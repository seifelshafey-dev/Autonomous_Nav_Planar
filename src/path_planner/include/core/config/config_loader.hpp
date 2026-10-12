/**
 * @file config_loader.hpp
 * @author Seif ElShafey
 * @brief YAML loaders that populate typed configuration structs for the path planner.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <string>

#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/escape/escape_search.hpp"

/**
 * @brief Pure-pursuit follower gains loaded from `config/controller/controller.yaml`.
 *
 * Field-for-field the tuning surface of the follower's control law: the ROS ControllerNode maps
 * this directly into a `PurePursuitParams`. Frames and control rate are ROS params, not part of
 * this struct. All fields are strictly positive (validated at load).
 */
struct ControllerParams {
    double lookahead_gain;   ///< Unitless carrot-lookahead gain k; the absolute lookahead L = k * (r_in / 2) is derived at the ControllerNode from the robot footprint.
    double v_cruise;         ///< Hard ceiling on the commanded speed magnitude |v| (m/s). Must not exceed the diff-drive `vmax`; checked when the navigator and controller nodes start, not by the loader.
    double a_lat_max;        ///< Lateral-acceleration bound (m/s^2) used to slow the base on tight arcs.
    double omega_pivot;      ///< In-place turn rate magnitude for pivots and heading alignment (rad/s).
    double goal_pos_tol;     ///< Position tolerance for declaring the goal reached (m).
    double goal_heading_tol; ///< Heading tolerance for the final goal-alignment pivot (rad).
};

/**
 * @brief Tuning for the reactive /scan safety guard, loaded from `config/safety/scan_safety_guard.yaml`.
 *
 * The guard stops a velocity command when a scan return lies in the area the robot's footprint
 * covers while it stops: along the motion, the distance covered during the reaction time plus the
 * braking distance plus `forward_pad`; sideways, the footprint grown by `side_pad`.
 */
struct ScanGuardParams {
    double linear_decel;       ///< Deceleration (m/s^2) the drive achieves when commanded to stop. Finite, > 0.
    double angular_decel;      ///< Angular deceleration (rad/s^2) the drive achieves when commanded to stop. Finite, > 0.
    double forward_pad;        ///< Extra distance (m) along the motion beyond the stopping distance. Finite, >= 0.
    double side_pad;           ///< Sideways growth (m) of the footprint at every sampled pose; not a sensor-noise allowance. Finite, > 0.
    int    stop_count;         ///< Consecutive violating cmd_vel ticks required before engaging STOP. Always >= 1.
    int    clear_count;        ///< Consecutive clear cmd_vel ticks required before resuming PASS. Always >= 1.
    double stale_scan_timeout; ///< Max age (s) of the cached scan before a fail-safe STOP, independent of the debounce counters. Always > 0.
    double tf_lookup_timeout;  ///< Max wait (s) for the scan-frame->robot-frame TF before giving up on a scan message. Always > 0.
};

/**
 * @brief Tuning for the navigator, loaded from `config/navigation/navigator.yaml`.
 *
 * The navigator sequences planning, driving and clearing a blocked start. Every field is required.
 */
struct NavigatorParams {
    double tick_period_sec;       ///< Period (s) of the navigator's timer, which runs the map check and the stall and retry rules. Finite and > 0.
    double stall_timeout;         ///< Time (s) without progress along the driven path after which the robot is treated as stalled. Finite, > 0 and greater than tick_period_sec; the navigator node also requires it to exceed the follower's final approach and turn times when it starts.
    double blocked_retry_period;  ///< Minimum time (s) between plan requests while the robot is blocked. Finite and > 0.
    int    max_clearing_attempts; ///< Consecutive clearing moves allowed for one goal before giving up. At least 1.
    int    max_stall_replans;     ///< Number of replans allowed per goal after the robot made no progress for `stall_timeout`, before the navigator gives up and reports that it is stuck. A count, at least 0 (0 = give up at the first stall). The count resets only on a new goal or on arrival; movement along the path does not reset it.
    EscapeConfig escape;          ///< Tuning of the straight move that clears a blocked start; see `EscapeConfig` for the ranges.
    double escape_speed_scale;    ///< Fraction of full speed, in (0, 1], at which the clearing move is driven.
};

/**
 * @brief Reads robot vertices from the given YAML file and returns the entire robot geometry
 *        (half-planes, r_in, r_out) derived from the vertices.
 *
 * @param file_path YAML config file path as a string.
 * @return RobotParams Struct defining the robot geometry.
 */
RobotParams loadRobotParams(const std::string & file_path);

/**
 * @brief Reads the robot's kinematic limits and returns the parameters required for the
 *        differential drive motion model.
 *
 * @param file_path YAML config file path as a string.
 * @return DiffDriveParams Struct defining differential-drive kinematic parameters.
 */
DiffDriveParams loadDiffDriveParams(const std::string & file_path);

/**
 * @brief Reads costmap penalty parameters (clearance and unknown-cell weighting) from a YAML file.
 *
 * @param file_path YAML config file path as a string.
 * @return CostPolicy Struct holding clearance radius, decay, maximum clearance cost, and unknown-space cost factor.
 *
 * @note All four keys (clearance_radius, clearance_decay, max_clearance_cost, unknown_cost_factor)
 *       must be present in the YAML source; the loader throws YAML::TypedBadConversion if any is missing
 *       or null. `unknown_cost_factor` must be finite and at least 1, and the former `unknown_cost`
 *       key is rejected. Fill the YAML file before invoking this loader.
 */
CostPolicy loadCostPolicy(const std::string & file_path);

/**
 * @brief Reads hybrid A* planner tuning parameters (tolerances, expansion limits, shot config, reserves)
 *        from a YAML file. Missing or null fields fall back to the AStarConfig default values.
 *
 * @param file_path YAML config file path as a string.
 * @return AStarConfig Struct holding tunable planner parameters.
 */
AStarConfig loadAStarConfig(const std::string & file_path);

/**
 * @brief Reads heuristic tuning parameters (weight) from a YAML file. Missing or null fields fall
 *        back to the HeuristicConfig default values.
 *
 * @param file_path YAML config file path as a string.
 * @return HeuristicConfig Struct holding tunable heuristic parameters.
 */
HeuristicConfig loadHeuristicConfig(const std::string & file_path);

/**
 * @brief Reads pure-pursuit follower gains from a YAML file (`controller:` section).
 *
 * @param file_path YAML config file path as a string.
 * @return ControllerParams Struct holding the follower tuning gains.
 *
 * @note All six keys (lookahead_gain, v_cruise, a_lat_max, omega_pivot, goal_pos_tol, goal_heading_tol)
 *       must be present under the `controller:` root; the loader throws if any is missing or null.
 *       Every gain is validated to be strictly positive.
 */
ControllerParams loadControllerParams(const std::string & file_path);

/**
 * @brief Reads reactive-safety-guard tuning from a YAML file (`scan_guard:` section).
 *
 * @param file_path YAML config file path as a string.
 * @return ScanGuardParams Struct holding the guard's tuning.
 *
 * @note All eight keys (linear_decel, angular_decel, forward_pad, side_pad, stop_count, clear_count,
 *       stale_scan_timeout, tf_lookup_timeout) must be present under the `scan_guard:` root; the
 *       loader throws if any is missing or null. linear_decel/angular_decel/side_pad must be finite
 *       and > 0; forward_pad finite and >= 0; stop_count/clear_count >= 1;
 *       stale_scan_timeout/tf_lookup_timeout > 0.
 */
ScanGuardParams loadScanGuardParams(const std::string & file_path);

/**
 * @brief Reads navigator tuning from a YAML file (`navigator:` section).
 *
 * @param file_path YAML config file path as a string.
 * @return NavigatorParams Struct holding the navigator's tuning.
 *
 * @note Every key (tick_period_sec, stall_timeout, blocked_retry_period, max_clearing_attempts, max_stall_replans,
 *       escape_speed_scale, and escape.step_fraction, escape.slack_cells, escape.max_travel_factor)
 *       must be present under the `navigator:` root; the loader throws if any is missing or null, or
 *       if a value is not finite or is outside the range documented on `NavigatorParams` and
 *       `EscapeConfig`. Also throws if stall_timeout is not greater than tick_period_sec. The relation of stall_timeout to the
 *       follower's approach and turn times, and of v_cruise to vmax, is checked by the node, not here.
 */
NavigatorParams loadNavigatorParams(const std::string & file_path);
