/**
 * @file motion_context.hpp
 * @author Seif ElShafey
 * @brief Context bundling robot geometry, kinematic state-space, and grid resolution for trajectory sampling and swept-footprint computation.
 * @version 0.1
 * @date 2026-05-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "core/diagnostics/check.hpp"
#include "core/map/rasterize.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/grid_coords.hpp"
#include "core/types/geometry.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/state_space_interface.hpp"
#include "core/map/grid.hpp"

/**
 * @brief Bundles the robot body geometry, kinematic state-space, and grid metadata required to
 *        sample a trajectory and compute its swept footprint.
 *
 * Trajectory sampling and footprint sweeping repeatedly need the same trio of inputs: the body
 * polygon (for inscribed/circumscribed radii and rasterization), the kinematic state-space (for
 * propagating controls), and the grid metadata (resolution + origin for rasterizing into the
 * occupancy grid). This class holds them together so they're passed as one argument instead of
 * three, and centralizes origin handling so callers don't thread `cell_origin` through every call.
 *
 * @note The robot body is held by value (cheap copy); the state-space is shared via
 *       `shared_ptr<const ...>` so multiple MotionContexts can reference one kinematic model
 *       without lifetime juggling. `GridMetaData` is held by value — it's a small POD and is
 *       fixed for the lifetime of a single planner query.
 */
class MotionContext{
    public:
    /**
     * @brief Bundle returned by `computeExactCover` containing the covering cell set,
     *        the sampled poses, and the per-sample robot-center cells.
     *
     *  - `cells`        — sorted, duplicate-free set of covering cells. Not index-matched to the other fields.
     *  - `samples`      — SE(2) poses along the trajectory, in the same frame as `start`.
     *  - `sample_cells` — index-matched to `samples`: the grid cell containing the robot's reference
     *                     point at each sampled pose.
     */
        struct SweptResult{
            std::vector<GridCoords> cells;        ///< Sorted, duplicate-free cells the footprint can touch along the trajectory.
            std::vector<SE2State>   samples;      ///< Poses sampled along the trajectory; consecutive inscribed disks overlap.
            std::vector<GridCoords> sample_cells; ///< Per-sample robot-center cell; size == samples.size().
        };

    /**
     * @brief Construct a MotionContext from the robot's body geometry, kinematic state-space, and grid metadata.
     *
     * @param body Body-frame robot geometry (vertices, half-planes, inscribed/circumscribed radii); stored by value.
     * @param space Kinematic state-space describing how the robot moves under a control input;
     *              shared via `shared_ptr<const ...>` for cheap reuse across MotionContexts.
     * @param grid_meta Grid metadata (resolution + origin) matching the occupancy grid this context
     *                  will produce cells against. Pass a `GridMetaData` with `origin = (0, 0)` to
     *                  obtain body-frame cells suitable for the primitive cache.
     */
        MotionContext(RobotParams body,
                      std::shared_ptr<const State_Space<SE2State>> space,
                      GridMetaData grid_meta)
                : body_(std::move(body)), space_(std::move(space)), grid_meta_(std::move(grid_meta)) {}

    /**
     * @brief Produces a sequence of SE(2) poses along the kinematic trajectory defined by `start` and `u`.
     *
     * Spacing between consecutive poses is at most `r_in / 2` in arc length, and the rotation between them
     * is at most `r_in / (2 * r_out)` rad, so consecutive inscribed disks overlap.
     * Poses are equally spaced in time. A control with both `v` and `omega` nonzero yields at least two intervals; any other yields at least one.
     *
     * @param start The pose from which the trajectory is integrated.
     * @param u The control input applied for `u.dt`.
     * @return std::vector<SE2State> The sampled SE(2) poses including `start` and the endpoint.
     */
        std::vector<SE2State> sampleTrajectory(const SE2State & start, const Control & u) const;

    /**
     * @brief Returns every grid cell the robot footprint can touch while executing a control, plus the
     *        sampled poses and their robot-center cells.
     *
     * `cells` contains every grid cell the footprint touches at any instant while executing `u` from
     * the continuous pose `start`, and every cell the footprint rasterization returns at each pose in
     * `samples`. `cells` is sorted ascending and free of duplicates. The frame of the returned cells
     * follows the context's `GridMetaData`: with `origin = (0, 0)` the cells are body-frame and suitable
     * for the primitive cache; with the map's origin they align directly with the OccupancyGrid's indexing.
     *
     * @param start Pose from which the control is applied: x (m), y (m), theta (rad), in the frame of
     *              the context's grid (body frame for an origin of (0, 0), map frame otherwise).
     * @param u Control `(v [m/s], omega [rad/s], dt [s])` applied for `u.dt`. `u.dt` must be finite and >= 0.
     * @return SweptResult with the covering `cells`, the sampled `samples`, and `sample_cells`, one
     *         grid cell per sample.
     * @throws std::invalid_argument if `u.dt` is negative or not finite.
     * @note Body-frame cells can be integer-translated by the robot's map-grid cell to recover map-grid cells.
     * @note With (`v == 0` and `omega == 0`) or `dt == 0`, `cells` is the footprint at `start`.
     */
        SweptResult computeExactCover(const SE2State & start, const Control & u) const;

    /**
     * @brief Visits, one at a time, the grid cells the robot footprint can touch while executing a
     *        control, without building a cell list.
     *
     * The set of cells visited equals `computeExactCover(start, u).cells` as a set. A cell can be
     * visited more than once, and the visiting order is unspecified. Intended for testing a motion
     * against a map and stopping at the first blocked cell.
     *
     * @tparam Visit Callable taking a `GridCoords` and returning `bool`.
     * @param start Pose from which the control is applied: x (m), y (m), theta (rad), in the frame of
     *              the context's grid (body frame for an origin of (0, 0), map frame otherwise).
     * @param u Control `(v [m/s], omega [rad/s], dt [s])` applied for `u.dt`. `u.dt` must be finite and >= 0.
     * @param visit Called per covered cell, with the cell expressed in the frame of the context's
     *              `GridMetaData` (same frame as `computeExactCover` cells). Return true to continue,
     *              false to stop. An exception thrown by `visit` propagates.
     * @return false if `visit` returned false and the walk stopped early; true otherwise.
     * @throws std::invalid_argument if `u.dt` is negative or not finite; nothing is visited in that case.
     * @note With (`v == 0` and `omega == 0`) or `dt == 0`, the cells of the footprint at `start` are visited.
     */
        template <typename Visit>
        bool visitExactCover(const SE2State & start, const Control & u, Visit && visit) const
        {
            CHECK(std::isfinite(u.dt) && u.dt >= 0.0, "visitExactCover: u.dt must be finite and >= 0");
            const double h = grid_meta_.resolution;
            const double reach = (std::abs(u.v) + body_.r_out * std::abs(u.omega)) * u.dt;
            const int n = std::max(1, static_cast<int>(std::ceil(reach / h)));
            const double m_s = reach / (2.0 * n);
            CHECK(m_s <= h / 2 + 1e-12, "arc margin exceeds half a cell");

            const Vec2 cell_origin = grid_meta_.origin.cast<double>();
            auto walk = [&](const SE2State & p, double margin) {
                const auto geo = transformGeometry(body_, p, cell_origin);
                return visitPolygonCells(geo.half_planes, geo.aabb, h, margin, visit);
            };
            for (int j = 0; j <= n; ++j) {
                if (!walk(space_->propagate(start, Control{u.v, u.omega, u.dt * j / n}), m_s)) return false;
            }
            for (const auto & p : sampleTrajectory(start, u)) {
                if (!walk(p, 0.0)) return false;
            }
            return true;
        }

        const RobotParams & body() const noexcept { return body_; }     ///< Access to the bundled body geometry. @return The body geometry.
        const State_Space<SE2State> & space() const noexcept { return *space_; }    ///< Access to the bundled kinematic state-space. @return The state-space.
        std::shared_ptr<const State_Space<SE2State>> spacePtr() const noexcept { return space_; } ///< Shared ownership handle for the kinematic state-space. @return Shared handle to the state-space.
        const GridMetaData & gridMeta() const noexcept { return grid_meta_; } ///< Access to the bundled grid metadata. @return The grid metadata given at construction.
        double cellSize() const noexcept { return grid_meta_.resolution; } ///< Access to the bundled grid resolution (m). @return Grid resolution (m per cell).

    private:
        RobotParams body_;                                              ///< Body-frame robot geometry.
        std::shared_ptr<const State_Space<SE2State>> space_;            ///< Kinematic model (e.g. differential drive); shared across contexts.
        GridMetaData grid_meta_;                                        ///< Grid metadata (resolution + origin) for cell production.
};
