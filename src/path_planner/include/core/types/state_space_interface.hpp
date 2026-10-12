/**
 * @file state_space_interface.hpp
 * @author Seif ElShafey
 * @brief Kinematic representation of the robot's state-space configuration. Allows for multiple configurations. SE(2) is used throughout.
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "core/types/control.hpp"


/**
 * @brief Interface defining the kinematic properties of the robot's state and configuration space.
 *
 * Covers the relationships between states: bounds within the workspace, kinematic distance,
 * interpolation, and state propagation under a control.
 *
 * @note Kinematic only; carries no geometric or mathematical footprint representation.
 *
 * @tparam StateT State-space representation of the robot. SE(2) is the default for wheeled planar robots.
 */
template <typename StateT>
class State_Space{
    public:
        virtual ~State_Space() {};
        /**
         * @brief Whether a state lies inside the workspace.
         * @param a State to test, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @return True if `a` lies strictly inside the workspace; borders count as out of bounds.
         */
        virtual bool inBounds(const StateT & a) const = 0;
        /**
         * @brief Kinematic displacement cost between two states (not straight-line Euclidean).
         * @param a First state, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @param b Second state, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @return Kinematic distance between `a` and `b`; the unit is defined by the implementation.
         */
        virtual double distance(const StateT & a, const StateT & b) const = 0;
        /**
         * @brief State sampled along the trajectory from `a` to `b` at fraction `t`.
         * @param a State at `t = 0`, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @param b State at `t = 1`, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @param t Fraction along the trajectory; `t` in [0, 1] lies between `a` and `b`. Not clamped.
         * @return Interpolated state. For SE(2) states the heading is wrapped to [-pi, pi].
         */
        virtual StateT interpolate(const StateT & a, const StateT & b, double t) const = 0;
        /**
         * @brief State reached by applying control `b` from state `a`.
         * @param a Starting state, in the frame the states are expressed in (for SE(2) states: m, m, rad).
         * @param b Control applied from `a`: velocities in the robot body frame (for wheeled robots: m/s, rad/s), duration in seconds.
         * @return End state in the same frame as `a`. The heading is not wrapped: wrap it with wrapToPi when a
         *         value in [-pi, pi] is needed.
         */
        virtual StateT propagate(const StateT & a, const Control & b) const = 0;
};
