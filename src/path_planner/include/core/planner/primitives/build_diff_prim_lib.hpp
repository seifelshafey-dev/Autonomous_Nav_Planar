/**
 * @file build_diff_prim_lib.hpp
 * @author Seif ElShafey
 * @brief Cached motion-primitive library for differential-drive kinematics.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <vector>

#include "core/robot/diff_drive_parameters.hpp"
#include "core/motion/motion_context.hpp"
#include "core/planner/primitives/control_primitives.hpp"

/**
 * @brief Cached primitive library for differential-drive motion. Holds one `Primitive` per `Control`,
 *        with `swept_cells[k]` and `sample_cells[k]` populated for `k in [0, n_headings)`.
 *
 * The library is built once at construction. Consumers access the cached primitives via `primitives()`
 * and map continuous headings to bin indices via `headingBin(theta)`.
 *
 * @note Heading bins are anchored at `theta_k = -pi + k * (2*pi/n_headings)` for `k in [0, n_headings)`,
 *       matching the signed wrapped range `[-pi, pi)` produced by `wrapToPi`.
 *
 * @note Although state is continuous, headings are binned into discrete values for the cached
 *       collision-check data only (`swept_cells`, `sample_cells`). For every pose whose heading lies in
 *       bin `k`, `swept_cells[k]` covers every cell the robot touches while executing the control, so
 *       a cache free of occupied cells proves the motion collision-free.
 */
class DiffDrivePrimLib{
    public:
    /**
     * @brief Constructs the library and builds every primitive eagerly. After construction the library
     *        is ready for use; no further build calls are required.
     *
     * @param motion Bundled robot geometry, kinematic state-space, and grid resolution used to sweep trajectories.
     * @param kin Differential-drive kinematic parameters; `kin.atomic_time` and `kin.n_headings`
     *            govern the per-primitive control duration and the heading-bin discretization.
     */
        DiffDrivePrimLib(MotionContext motion, DiffDriveParams kin);

        /**
         * @brief Returns the cached primitive library. Size equals the number of distinct controls in the
         *        underlying control set. Each `Primitive` carries per-bin swept-cell and sample-cell data
         *        for collision checking.
         *
         * @return const std::vector<Primitive>& Reference to the cached library; valid for the lifetime of this object.
         */
        const std::vector<Primitive>& primitives() const noexcept { return primitives_; }

        /**
         * @brief Returns the number of heading bins used by the library's swept-footprint cache.
         *
         * @return Number of heading bins.
         */
        int nHeadings() const noexcept { return kin_.n_headings; }

        /**
         * @brief Maps a continuous heading angle to its bin index in `[0, n_headings)`.
         *
         * @param theta Heading angle in radians; any finite value is accepted (wrapped internally).
         * @return int16_t Bin index in `[0, n_headings)`.
         */
        int16_t headingBin(double theta) const noexcept;

        /**
         * @brief Returns the wall-clock time spent building the library.
         *
         * Measured once during construction; it covers every primitive across all heading bins and does
         * not change afterwards.
         *
         * @return Build time in milliseconds (fractional, >= 0).
         */
        double buildTimeMs() const noexcept { return build_time_ms_; }

    private:
        /**
         * @brief Builds the differential-drive control set from the minimum turning radius: a forward
         *        and reverse straight, in-place rotations, and left/right arcs at both tight and wide radii.
         */
        std::vector<Control> buildControlSet() const;

        /**
         * @brief Builds a single `Primitive` for the given control input, populating swept cells and
         *        sample cells for all heading bins.
         */
        Primitive buildPrimitive(const Control & u) const;

        /** Cell cover of control `u` for heading bin `k`, sorted and unique. */
        std::vector<GridCoords> buildCover(const Control & u, int k) const;

        MotionContext         motion_;       ///< Bundled kinematic + geometric context used to sweep trajectories.
        DiffDriveParams       kin_;          ///< Differential-drive kinematic parameters (rmin, vmax, atomic_time, n_headings).
        std::vector<Primitive> primitives_;  ///< Cached primitive library built at construction.
        double                build_time_ms_ = 0.0; ///< Wall-clock build time in milliseconds.
};
