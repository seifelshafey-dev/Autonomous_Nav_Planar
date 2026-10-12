/**
 * @file state.hpp
 * @author Seif ElShafey
 * @brief Represents state formulation in the SE(2) space where planar robots exist
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

/**
 * @brief Canonical representation of a wheeled robot's state in a plane.
 *
 * X and Y define the position of the robot in the plane relative to the plane's origin.
 * Theta defines the heading angle of the robot: the CCW angle between the x-axis and the
 * line passing through the robot's origin perpendicular to the rear-axle line.
 *
 * @note Theta is expected in [-pi, pi] (radians). Values produced by wrapToPi and
 *       State_Space::interpolate are; State_Space::propagate returns theta unwrapped.
 */
struct SE2State{
    double x; ///< X position (m) in the frame the state is expressed in (the world frame for planner poses).
    double y; ///< Y position (m) in the frame the state is expressed in (the world frame for planner poses).
    double theta; ///< Heading (rad), counter-clockwise from the x-axis of that frame.
};