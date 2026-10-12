/**
 * @file collision_checker.hpp
 * @author Seif ElShafey
 * @brief Checks whether a set of grid cells (from a state or robot trajectory) are collision-free in the grid-based workspace.
 * @version 0.1
 * @date 2026-05-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "core/types/state.hpp"
#include "core/types/grid_coords.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/robot/robot_params.hpp"

/**
 * @brief Result of a quick distance-field test: `COLLISION` (certainly in collision), `VALID`
 *        (certainly collision-free), or `UNCERTAIN` (the distance field cannot decide).
 */
 enum class Validity {
    COLLISION, ///< Certainly in collision.
    VALID, ///< Certainly collision-free.
    UNCERTAIN ///< The distance field cannot decide.
};

/**
 * @brief Outcome of `CollisionChecker::checkTrajectory`, naming the stage that decided it.
 */
enum class TrajectoryVerdict {
    FAST_ACCEPT,  ///< At least one sample was given and every sample is farther from any obstacle than its supplied clearance; the motion is collision-free.
    FAST_REJECT,  ///< A sample is so close to an obstacle that the robot certainly collides there given its supplied clearance; the motion collides.
    SCAN_CLEAR,   ///< The per-cell scan found no OCCUPIED cell among the supplied swept cells; the motion is collision-free when those cells cover it.
    SCAN_BLOCKED  ///< The per-cell scan found an OCCUPIED cell among the supplied swept cells. The motion collides only if those cells are exactly the cells it touches; if they are a conservative cover, the result is inconclusive.
};

/**
 * @brief Collision oracle for the planning stack: answers whether the robot, at a pose or along a
 *        swept trajectory, touches an OCCUPIED cell of the grid.
 *
 * Consumers pass raw body-frame cell lists; the checker translates them by the robot's cell and decides
 * from the distance field where it can, otherwise from the cells themselves.
 *
 */
class CollisionChecker{
    std::shared_ptr<const OccupancyGrid> grid_;
    std::shared_ptr<const EDT> edt_;
    RobotParams body_;

    public:
    /**
     * @brief Construct a new Collision Checker object, which provides easy access for validity checking given a certain map and robot.
     *
     * @param grid A shared pointer to the current occupancy grid for which we will check collision
     * @param edt A shared pointer to the current EDT grid, used for quick accept/reject.
     * @param params Robot parameters defining the minimum and maximum circular footprint of the robot. Used for quick reject/accept.
     */
     
        CollisionChecker(std::shared_ptr<const OccupancyGrid> grid,
                     std::shared_ptr<const EDT> edt,
                     RobotParams params) : grid_(std::move(grid)), edt_(std::move(edt)), body_(params){}

        /**
         * @brief Validate a single SE(2) pose by rasterizing its footprint and checking occupancy.
         *
         @note Out-of-grid and unknown cells are treated as free.
         @note An OCCUPIED cell whose centre lies within half a cell diagonal of the footprint counts as a
               collision, so the test is conservative for the continuous pose.
               Cost is O(1) in the common case and O(N) in the footprint cell count otherwise.
         *
         * @param state Robot state in world coordinates (meters).
         * @return true If no collision is detected.
         * @return false If a collision exists.
         */
        bool checkState(const SE2State & state) const;

        /**
         * @brief Whether the footprint at `start`, widened by a safety margin, touches no OCCUPIED cell.
         *
         * With `h` the grid resolution (m), let `m = (√2/2 + 1/2)·h + extra_margin` (m).
         * If the result is true, no OCCUPIED cell centre lies within distance `m` of the footprint at
         * `start`. Equivalently, if any OCCUPIED cell centre lies within `m` of the footprint, the result
         * is false.
         *
         * If the result is false, some OCCUPIED cell centre lies within `m` outside every edge line of the
         * footprint (the edge lines extended without limit). Near a footprint corner such a centre can be
         * further than `m` from the footprint, up to `m` divided by the sine of half the corner's interior
         * angle (`√2·m` for a right-angle corner). The result is therefore exact except for OCCUPIED cells in
         * that corner band, beyond distance `m` but inside every edge line pushed out by `m`: for those
         * either answer is possible.
         *
         * UNKNOWN and out-of-grid cells never fail it. Stricter than
         * `checkState(start)`: a pose that passes it also passes `checkState`. Raising `extra_margin`
         * can only turn true into false.
         *
         * @param start Robot pose in world coordinates (m, m, rad).
         * @param extra_margin Additional clearance beyond the base margin (m); finite and >= 0, else
         *                     `std::invalid_argument` is thrown (via CHECK).
         * @return true if no OCCUPIED cell centre lies within distance `m` of the footprint, subject to the
         *         corner band described above.
         */
        bool startClear(const SE2State & start, double extra_margin = 0.0) const;

        /**
         * @brief Validate a swept trajectory described by raw body-frame cell lists.
         *
         * Each sample carries the obstacle distance it needs to be certainly clear. The primitive library
         * supplies this per sample so that cell snapping and heading quantization are covered.
         * A sample rejects the trajectory when its obstacle distance `d`
         * satisfies `d + (sample_clearance - r_out) < r_in`; it is clear when `d > sample_clearance`;
         * otherwise the swept cells are scanned.
         *
         @note Cost is O(M) in the sample count when the distance field decides, O(N) in the swept-cell
               count otherwise.
         *
         * @param swept_body Body-frame swept cell offsets covering the full trajectory footprint (cells;
         *                   e.g. `Primitive::swept_cells[k]`). May be a conservative cover of the motion
         *                   rather than its exact cells; see `TrajectoryVerdict::SCAN_BLOCKED`.
         * @param samples_body Body-frame sample cell offsets along the trajectory (cells); may be empty,
         *                     in which case the swept cells are scanned. A sample outside the grid is
         *                     inconclusive and triggers the full scan.
         * @param sample_clearance Per-sample obstacle distance (m), same length and order as
         *                         `samples_body`: the distance between that sample cell's centre and the
         *                         nearest OCCUPIED cell's centre beyond which the robot is certainly clear
         *                         there. Must have `samples_body.size()` elements, and every entry
         *                         must be >= `r_out`; the amount above `r_out` is the uncertainty of the
         *                         sample centre's position, and is what the reject test allows for.
         *                         Violating either throws `std::invalid_argument`; entries are checked in
         *                         order, and none is checked after the first sample that rejects, lies
         *                         outside the grid, or is not clear.
         * @param robot_cell Robot's world-frame grid cell (cell indices); added to every body-frame offset.
         * @return The stage that decided the outcome. `TrajectoryVerdict::FAST_ACCEPT` and
         *         `TrajectoryVerdict::SCAN_CLEAR` prove the motion is collision-free (given that
         *         `swept_body` covers it). `TrajectoryVerdict::FAST_REJECT` proves it collides.
         *         `TrajectoryVerdict::SCAN_BLOCKED` proves a collision only when `swept_body` is exactly
         *         the cells the motion touches; when it is a conservative cover the result is
         *         inconclusive, and the caller may check the motion more precisely (e.g. `checkCells`
         *         over an exact cell set).
         */
        TrajectoryVerdict checkTrajectory(const std::vector<GridCoords> & swept_body,
                                          const std::vector<GridCoords> & samples_body,
                                          const std::vector<double> & sample_clearance,
                                          const GridCoords & robot_cell) const;

        /**
         * @brief Validate a single absolute (world-frame) grid cell.
         *
         * @param cell Map-grid cell indices (world-frame). A cell outside the grid counts as UNKNOWN.
         * @param reject_unknown If true, an UNKNOWN cell fails. If false, only an OCCUPIED cell can fail.
         * @return false if `cell` is OCCUPIED, or if `reject_unknown` is true and `cell`
         *         is UNKNOWN; true otherwise.
         */
        bool checkCell(const GridCoords & cell,
                       bool reject_unknown = false) const;

        /**
         * @brief Validate a list of absolute (world-frame) grid cells by scanning each one.
         *
         * @param cells_world Absolute map-grid cells (indices) to check, e.g. a rasterized trajectory.
         *                    Cells outside the grid read as UNKNOWN.
         * @param reject_unknown If true, UNKNOWN cells count as collisions (KNOWN-FREE semantics, e.g. for shots).
         *                       If false, only OCCUPIED cells count (default search semantics).
         * @return true If no cell collides under the chosen policy (also for an empty list).
         * @return false If any cell collides.
         */
        bool checkCells(const std::vector<GridCoords> & cells_world,
                        bool reject_unknown = false) const;

        /**
         * @brief Quick, possibly inconclusive collision test for a pose, from the distance field alone.
         * @param current_state Robot state in world coordinates (m, m, rad).
         * @return `Validity::VALID` if certainly collision-free, `Validity::COLLISION` if certainly in
         *         collision, `Validity::UNCERTAIN` otherwise.
         */
        Validity fastAcceptReject(const SE2State & current_state) const;

        /**
         * @brief Quick, possibly inconclusive collision test for a robot whose centre lies in `cell`.
         *
         * With `h` the grid resolution and `d` the distance from the cell's centre to the nearest
         * OCCUPIED cell's centre: `VALID` iff `d > r_out + √2·h`; `COLLISION` iff `d + (√2/2)·h < r_in`;
         * otherwise `UNCERTAIN`.
         *
         * @param cell World-frame grid cell containing the robot's centre.
         * @return The verdict. A cell outside the grid is always `Validity::UNCERTAIN`.
         */
        Validity fastAcceptReject(const GridCoords & cell) const;

        /**
         * @brief Metadata of the occupancy grid this checker tests against: resolution (m per cell),
         *        dimensions (`rows`, `cols`, cells) and the world position of the corner of cell (0, 0) with the smallest x and y (`origin`,
         *        m, map frame).
         *
         * Use it to convert world positions to the cell indices that `checkCell` and `startClear`
         * expect.
         *
         * @return Reference valid for the lifetime of this checker.
         */
        const GridMetaData & gridMeta() const noexcept { return grid_->getMetadata(); }
};
