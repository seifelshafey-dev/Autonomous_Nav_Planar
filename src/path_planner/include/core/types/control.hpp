/**
 * @file control.hpp
 * @author Seif ElShafey
 * @brief Holds the control input structure
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

/**
 * @brief Represents the canonical structure of a control input and the time it is applied for.
 *
 * The control input of a planar wheeled robot is represented by tangential velocity and
 * rotational velocity along the plane. Their values are constrained by the inversely
 * proportional relationship defined by the turning radius R.
 *
 * @note Canonical for planar wheeled robots: differential-drive, Ackermann/bicycle, and omni-without-strafe
 *       all reduce to (v, omega, dt). Non-wheeled platforms (full omni with lateral velocity, aerial,
 *       articulated) would require a different Control variant and a corresponding State_Space specialization.
 */
struct Control{
    double v; ///< Tangential velocity applied to the robot
    double omega; ///< Rotation velocity applied to the robot about its central axis
    double dt; ///< Time for which the control input is applied
};