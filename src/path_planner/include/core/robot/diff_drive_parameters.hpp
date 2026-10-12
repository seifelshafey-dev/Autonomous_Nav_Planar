/**
 * @file diff_drive_parameters.hpp
 * @author Seif ElShafey
 * @brief Interface representing core parameters in the differential drive kinematic model.
 * @version 0.1
 * @date 2026-05-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

/**
 * @brief Kinematic parameters of the differential-drive robot bundled with the discretization knobs
 *        that the cached primitive library uses (per-primitive application time, heading-bin count).
 *
 * `rmin` and `vmax` are physical limits of the robot; `atomic_time` and `n_headings` are the
 * planner-side discretization choices the cached primitive library is built against.
 */
struct DiffDriveParams{
    double rmin        = 0;    ///< Minimum turning radius of the robot (m). `omega_max = vmax / rmin`.
    double vmax        = 0;    ///< Maximum tangential velocity of the robot (m/s).
    double atomic_time = 0.6;  ///< Time (s) each primitive's control input is applied for; shared across every control in the library.
    int    n_headings  = 16;   ///< Number of heading bins for swept-footprint caching; must match the closed-set bin keying.
};