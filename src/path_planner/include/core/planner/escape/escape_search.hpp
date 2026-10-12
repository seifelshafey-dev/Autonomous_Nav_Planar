/**
 * @file escape_search.hpp
 * @brief Straight reverse or forward move that takes a robot standing in or too close to an
 *        obstacle to a nearby pose with clearance.
 */

#pragma once

#include <optional>
#include "core/collision_checker/collision_checker.hpp"
#include "core/motion/motion_context.hpp"
#include "core/planner/path.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"

/**
 * @brief Tuning of the straight escape move. Every field is validated by `findEscape`.
 */
struct EscapeConfig {
    double step_fraction     = 0.4; ///< Length of one step as a fraction of the grid resolution; in (0, 1).
    int    slack_cells       = 1;   ///< Extra clearance aimed for at the end pose, in grid cells (each one grid resolution, in metres); >= 0. A target, not a requirement.
    double max_travel_factor = 5.0; ///< Longest straight travel as a multiple of the footprint's outer radius; finite and > 0.
};

/**
 * @brief Find a straight reverse or forward move, at constant heading, that ends at a pose clear of
 *        obstacles.
 *
 * The robot may start in collision. A step never touches an occupied cell the robot was not
 * already touching, and never moves further into one it was touching. Unknown cells and cells
 * outside the grid do not block. Both directions are tried; a pose with `slack_cells` extra cells of
 * clearance (measured at the earliest pose the follower may stop at) is preferred over one that only passes the start check, then the shorter move, then
 * reverse over forward. When no pose with that clearance is reached in a direction, the first move
 * that passes the start check is the one returned for it.
 *
 * @param start Start pose in the map frame (m, m, rad).
 * @param checker Collision oracle over the occupancy map. Its map, resolution and origin are used
 *                for every cell test.
 * @param motion Footprint and kinematics used to sweep the move. Its grid metadata is not used.
 * @param kin Differential-drive limits; the move runs at `vmax`.
 * @param config Tuning; each field must be in the range documented on `EscapeConfig`,
 *               otherwise a CHECK throws `std::invalid_argument`.
 * @param stop_short How far (m) before the end of the path the follower may stop; finite and >= 0,
 *                   otherwise a CHECK throws `std::invalid_argument`. Pass the follower's goal position
 *                   tolerance. Every step pose (spaced `step_fraction` grid resolutions apart along
 *                   the move) within `stop_short` of the path end passes `checker.startClear(pose)`.
 *                   Poses between two step poses are not checked, so a planner start taken there can
 *                   still be reported blocked.
 *                   The path is longer than the plain escape by about this distance.
 * @return A path from `start` to a pose that passes `checker.startClear(pose)`: a single straight
 *         segment (negative speed for reverse), or a path with no segments and the single node
 *         `start` if the start is already clear. `std::nullopt` if neither direction reaches such a
 *         pose within `max_travel_factor` footprint outer radii.
 *         The segment cost is the control cost `sqrt(v^2 + rmin^2 * omega^2) * dt` with no map term.
 */
std::optional<Path> findEscape(const SE2State & start, const CollisionChecker & checker,
                               const MotionContext & motion, const DiffDriveParams & kin,
                               const EscapeConfig & config, double stop_short);
