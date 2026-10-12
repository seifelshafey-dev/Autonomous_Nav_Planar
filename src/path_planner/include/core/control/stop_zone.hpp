/**
 * @file stop_zone.hpp
 * @author Seif ElShafey
 * @brief Pure, ROS-free test of whether scan returns lie in the area the robot sweeps while stopping.
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <vector>

#include "core/robot/robot_params.hpp"
#include "core/types/geometry.hpp"

/**
 * @brief Stopping behaviour and safety margins used to size the stop zone.
 */
struct StopZoneParams {
    double linear_decel;  ///< Deceleration the drive achieves when commanded to stop (m/s^2), > 0.
    double angular_decel; ///< Angular deceleration the drive achieves when commanded to stop (rad/s^2), > 0.
    double forward_pad;   ///< Extra travel of the fastest footprint point after the modelled stop (m), >= 0.
    double side_pad;      ///< Inflation of the footprint at every sampled pose (m), > 0; it makes the sampled poses cover the continuous motion and sets how far outside the true zone a point may still be reported: within `side_pad * (1/sin(a/2) + 1/2)` of the footprint at some instant of the modelled motion, where `a` is the footprint's smallest interior angle (about `1.9 * side_pad` for a rectangle). It is not a sensor-noise allowance.
};

/**
 * @brief Report whether any scan return lies in the area the footprint covers while the robot
 *        stops from the commanded velocity.
 *
 * The zone is the footprint, inflated by `side_pad` at poses sampled over the whole motion: the reaction time at
 * the commanded velocity, then linear and angular speed each braking independently at its
 * configured deceleration until both are zero, then a further motion at the commanded velocity
 * until the fastest footprint point has travelled `forward_pad`. For a robot that brakes each axis
 * at exactly the configured deceleration, every point the footprint touches during the stop is
 * reported as inside. A point outside may also be reported if it lies within
 * `side_pad * (1/sin(a/2) + 1/2)` of the footprint at some instant of the modelled motion, where
 * `a` is the footprint's smallest interior angle (about `1.9 * side_pad` for a rectangle).
 * Points on the zone boundary count as inside.
 *
 * @param robot         Footprint geometry (body frame, metres).
 * @param points_body   Scan returns in the robot body frame (base_link), metres. May be empty.
 *                      Non-finite points are never reported as inside.
 * @param v             Commanded linear velocity (m/s), signed; negative is reverse.
 * @param omega         Commanded angular velocity (rad/s), signed; positive is counter-clockwise.
 * @param reaction_time Delay before braking starts (s), >= 0.
 * @param params        Deceleration and padding parameters.
 * @return `true` if at least one point lies in the stop zone; `false` if none does, if
 *         `points_body` is empty, or if `v` and `omega` are both zero.
 * @throws std::invalid_argument if `v`, `omega`, `reaction_time` or any field of `params` is not
 *         finite, `reaction_time` is negative, `linear_decel` or `angular_decel` is not positive,
 *         `forward_pad` is negative, or `side_pad` is not positive. The arguments are validated
 *         even when `v` and `omega` are both zero.
 */
bool motionBlocked(const RobotParams & robot, const std::vector<Vec2> & points_body,
                   double v, double omega, double reaction_time, const StopZoneParams & params);
