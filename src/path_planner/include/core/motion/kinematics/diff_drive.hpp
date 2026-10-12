/**
 * @file diff_drive.hpp
 * @author Seif ElShafey
 * @brief Defines the robot's state-space in the context of a differential drive wheeled robot in SE(2)
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <memory>

#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/geometry.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/types/state_space_interface.hpp"
#include "core/map/grid.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/motion/distance_functions.hpp"


/**
 * @brief Inherited state-space interface redefining kinematic properties of the SE(2) state-space based on the
 differential-drive kinematic model. See state_space_interface.hpp for the core properties of a state space.
 */

class DiffDriveStateSpace : public State_Space<SE2State>{
    public:

    /**
     * @brief Construct a new Diff Drive State Space object, defining the workspace boundaries and distance function.
     *
     * @param grid Grid metadata defining the workspace extent in world coordinates. Bounds are derived as
     *             `[origin, origin + cols*resolution] x [origin, origin + rows*resolution]`.
     * @param dist Distance function returned by `distance`; must be non-null. Its rotational weight is whatever
     *             the caller built it with.
     */
    DiffDriveStateSpace(const GridMetaData & grid, std::shared_ptr<const SE2Distance> dist) :
        bounds_{
            Vec2{static_cast<double>(grid.origin.x()),
                 static_cast<double>(grid.origin.y())},
            Vec2{static_cast<double>(grid.origin.x()) + grid.cols * grid.resolution,
                 static_cast<double>(grid.origin.y()) + grid.rows * grid.resolution}
        },
        dist_(std::move(dist)) {}

    /**
     * @brief True if `a` lies strictly inside the workspace; a state exactly on an edge is out of bounds.
     * @param a State to test; x, y (m) in the frame of the grid given at construction.
     * @return True if `a.x` and `a.y` are strictly inside the bounds; heading is ignored.
     */
    bool inBounds(const SE2State & a) const override {
        // Strictly interior: a state exactly on a workspace edge is out of bounds.
        return (a.x > bounds_.min.x() && a.x < bounds_.max.x() &&
                a.y > bounds_.min.y() && a.y < bounds_.max.y());
    }

    /**
     * @brief Distance from `a` to `b` as computed by the distance function given at construction.
     * @param a First state (m, m, rad).
     * @param b Second state (m, m, rad), in the same frame as `a`.
     * @return The injected distance functor's value for `(a, b)`, in the functor's unit (metres per the `SE2Distance` contract).
     */
    double distance(const SE2State & a, const SE2State & b) const override{
        return (*dist_)(a, b);
    }

    /**
     * @brief Position interpolated linearly; heading follows the shortest arc from `a` to `b`, wrapped to [-pi, pi].
     * @param a State at `t = 0` (m, m, rad).
     * @param b State at `t = 1` (m, m, rad), in the same frame as `a`.
     * @param t Fraction from `a` to `b`; not clamped.
     * @return Interpolated state, heading in [-pi, pi].
     */
    SE2State interpolate(const SE2State & a, const SE2State & b, double t) const override{
        double x_int = (1-t)*a.x + t*b.x;
        double y_int = (1-t)*a.y + t*b.y;
        // Heading follows the shortest arc between the two orientations: advance from a.theta by the
        // fraction t of the wrapped (geodesic) delta, then renormalize to [-pi, pi].
        double theta_int = wrapToPi(a.theta + t * wrapToPi(b.theta - a.theta));
        SE2State interp = {x_int, y_int, theta_int};
        return interp;
    }

    /**
     * @brief Moves the robot in an arc-like trajectory defined by the joint-velocity kinematic model.
     @note Differential drive allows instantaneous acceleration changes, so the only constraint is on the velocity/omega tradeoff.
     * 
     * @param a Starting state of the robot.
     * @param u A set of velocity control inputs applied to the robot at the current state. 
     * @return SE2State The end state reached by the robot after successful application of the control input.
     *         The heading is `a.theta + u.omega * u.dt`, not wrapped.
     */

    SE2State propagate(const SE2State & a, const Control & u) const override{
        double dx; double dy; double dtheta = u.omega * u.dt;

        // Straight line motion
        if(u.omega == 0){
            dx = u.v * u.dt;
            dy = 0.0;
        }
        else{
            double R = (u.v / u.omega);
            dx = R * std::sin(u.omega * u.dt);
            dy = R * (1-std::cos(u.omega * u.dt));
        }
        
        double c = std::cos(a.theta);
        double s = std::sin(a.theta);
        return SE2State{
            a.x + c*dx - s*dy,
            a.y + s*dx + c*dy,
            a.theta + dtheta
        };
    }
    
private:
    AABB bounds_; ///< Axis-aligned workspace bounds in world meters, derived from the grid extent.
    std::shared_ptr<const SE2Distance> dist_; ///< Distance function in SE(2); rotational weight is rmin^2 for dimensional consistency.
};