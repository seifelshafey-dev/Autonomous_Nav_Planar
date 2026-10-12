/**
 * @file kinematic_model.hpp
 * @author Seif ElShafey
 * @brief Aggregates every dependency needed to reason about how the robot moves through SE(2).
 * @version 0.1
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>

#include "core/motion/motion_context.hpp"
#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/robot/robot_params.hpp"

/**
 * @brief Bundles every dependency needed to reason about how the robot moves through SE(2):
 *        on-demand rasterization and sampling (MotionContext), the cached primitive library
 *        (DiffDrivePrimLib), and the raw kinematic constants (DiffDriveParams).
 *
 * The code that assembles the planner builds one instance and shares it across every
 * motion-aware consumer: planners, repair strategies, RRT*, goal-shot generators, etc. Heavy
 * members are held by `shared_ptr<const ...>` for cheap sharing; `DiffDriveParams` is small
 * and held by value.
 *
 * @note This is a plain aggregate. Consumers may copy it freely; the underlying objects
 *       remain shared via the contained shared pointers.
 */
struct KinematicModel {
    std::shared_ptr<const MotionContext>    motion;    ///< On-demand `sampleTrajectory` / `computeExactCover`.
    std::shared_ptr<const DiffDrivePrimLib> prim_lib;  ///< Cached expansion primitives + `headingBin`.
    DiffDriveParams                         kin;       ///< Differential-drive constants and discretization knobs.

    /// Grid resolution (m) this bundle was built with; matches the `cell_size` passed to `buildKinematicModel`.
    /// @return Grid resolution (m).
    double cellSize() const noexcept { return motion->cellSize(); }
};

/**
 * @brief Build the stable kinematic bundle (motion context, primitive library, kinematic constants)
 *        from the robot's body geometry, kinematic parameters, and the grid resolution.
 *
 * @param body Body-frame robot geometry (vertices, half-planes, inscribed/circumscribed radii).
 * @param kin Differential-drive kinematic parameters (rmin, vmax, atomic_time, n_headings).
 * @param cell_size Grid resolution (m). Must match the SLAM grid resolution.
 * @return KinematicModel Bundle suitable for sharing across motion-aware consumers.
 *
 * @note The returned `motion` field is anchored at a placeholder origin (0, 0); planner factories
 *       that consume this bundle (e.g. `buildHybridAstar`) replace `motion` with a snapshot-aware
 *       variant carrying the SLAM grid's real origin so world-frame trajectory rasterization works
 *       correctly. The cached primitive library is body-frame and depends only on `cell_size` and
 *       `kin`, so it is reused across snapshots without rebuild.
 */
KinematicModel buildKinematicModel(const RobotParams & body,
                                   const DiffDriveParams & kin,
                                   double cell_size);
