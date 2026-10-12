/**
 * @file distance_functions.hpp
 * @author Seif ElShafey
 * @brief Weighted-Euclidean SE(2) distance metrics and the differential-drive control-cost functor.
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include "core/types/state.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/types/control.hpp"
#include "core/robot/diff_drive_parameters.hpp"



/**
 * @brief Polymorphic SE(2) distance functor. Concrete implementations define a scalar distance
 *        between two poses; consumers (heuristics, state-space metrics) plug them in via injection.
 */
class SE2Distance{
    public:
        virtual ~SE2Distance() = default;

        /**
         * @brief Evaluate the distance from pose `a` to pose `b`.
         *
         * @param a Source pose in world coordinates.
         * @param b Target pose in world coordinates.
         * @return double Non-negative scalar distance in meters.
         */
        virtual double operator()(const SE2State & a, const SE2State& b) const = 0;
};

/**
 * @brief Weighted Euclidean SE(2) distance between two poses (m):
 *        `sqrt(dx^2 + dy^2 + weight * dtheta^2)`, with `dtheta` the wrapped heading difference (rad).
 * @note The angular delta is wrapped to the signed range [-pi, pi] before squaring, so a small
 *       clockwise rotation contributes the same cost as the equivalent counter-clockwise rotation.
 */
class PointsWeightedEuclideanSE2 : public SE2Distance{
    double weight; ///< Proportionality constant on the squared angular displacement. For diff-drive, the canonical choice is `rmin^2` (units m^2/rad^2), so the rotational term reads as the squared arc length at the minimum turning radius.
    public:
        /**
         * @brief Construct with weight `w` on the squared heading difference.
         * @param w Weight (m^2/rad^2).
         */
        explicit PointsWeightedEuclideanSE2(double w) : weight(w) {};
        /**
         * @brief Distance from pose `a` to pose `b`.
         * @param a Source pose (m, m, rad).
         * @param b Target pose (m, m, rad), in the same frame as `a`.
         * @return Distance in metres.
         */
        double operator()(const SE2State & a, const SE2State & b) const override{
            double dx = b.x - a.x;
            double dy =  b.y - a.y;
            double dth = wrapToPi(b.theta - a.theta);
            return std::sqrt(dx*dx + dy*dy + weight*dth*dth);
        }
};

/**
 * @brief Cost of one control, in metres: `sqrt(v^2 + w^2 * omega^2) * dt`, with `w` in m/rad.
 *        With `w = rmin`, a pivot at `vmax / rmin` costs the same as a straight at `vmax` for the same duration.
 */
class VelWeightedEuclideanSE2{
    double weight; ///< Defines the proportionality constant punishing the rotational displacement. Defined by the minimal radius of robot rotation (Vmax/Wmax).
    public:
        /**
         * @brief Construct with weight `w` on the rotational rate.
         * @param w Weight (m/rad).
         */
        explicit VelWeightedEuclideanSE2(double w) : weight(w) {};
        /**
         * @brief Cost of one control.
         * @param a Control: `v` (m/s), `omega` (rad/s), `dt` (s).
         * @return Cost in metres.
         */
        double operator()(const Control & a) const{
            double path = sqrt(pow(a.v, 2) + pow(a.omega,2) * pow(weight, 2));
            return path * a.dt;
        }
};

/**
 * @brief Construct the canonical edge-cost functor for a differential-drive robot.
 *
 * @param kin Differential-drive kinematic parameters; the rotational weight is set from `kin.rmin`.
 * @return VelWeightedEuclideanSE2 Functor mapping a `Control{v, omega, dt}` to its weighted arc length.
 */
inline VelWeightedEuclideanSE2 makeControlCost(const DiffDriveParams & kin) {
    return VelWeightedEuclideanSE2{kin.rmin};
}