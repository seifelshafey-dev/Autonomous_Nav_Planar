/**
 * @file kinematic_model.cpp
 * @author Seif ElShafey
 * @brief Factory that assembles the stable kinematic bundle (MotionContext + DiffDrivePrimLib +
 *        DiffDriveParams) from a robot body, kinematic constants, and the grid resolution.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/motion/kinematic_model.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/map/grid.hpp"

KinematicModel buildKinematicModel(const RobotParams & body,
                                   const DiffDriveParams & kin,
                                   double cell_size) {
    // SE(2) distance with rmin^2 rotational weight: dimensionally aligns angular displacement
    // (squared arc length at the minimum turning radius) with Cartesian displacement.
    auto se2_dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);

    // Placeholder grid metadata: only resolution is consumed at construction (the primitive
    // library is body-frame). Snapshot-aware origin/dims are wired in by buildHybridAstar. Dims are
    // a 1x1 stub rather than 0x0 to satisfy the GridMetaData positive-dimension invariant; the actual
    // values are unused in body-frame caching.
    GridMetaData stub_meta{1, 1, cell_size, Eigen::Vector2f::Zero()};

    auto space = std::make_shared<const DiffDriveStateSpace>(stub_meta, se2_dist);

    // Body-frame MotionContext feeds the primitive library; consumed once at DiffDrivePrimLib
    // construction and never accessed again.
    MotionContext motion_for_lib(body, space, stub_meta);
    auto prim_lib = std::make_shared<const DiffDrivePrimLib>(std::move(motion_for_lib), kin);

    auto motion = std::make_shared<const MotionContext>(body, space, stub_meta);

    return KinematicModel{motion, prim_lib, kin};
}
