/**
 * @file control_primitives.hpp
 * @author Seif ElShafey
 * @brief Precomputed per-control motion primitive: body-frame endpoint offset, cost, and per-heading-bin cached collision cells.
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <vector>
#include "core/types/control.hpp"
#include "core/types/state.hpp"
#include "core/types/grid_coords.hpp"

/**
 * @brief Encapsulates the state-independent result of applying one control input to the robot.
 *        Applied from any state, a given control produces the same body-frame outcome: the robot moves
 *        the same distance along the same arc and sweeps the same local footprint. A primitive caches
 *        that per-control data so it is computed once rather than at every expansion.
 *
 * @note Graph-based search expands a fixed, predetermined primitive set rather than arbitrary control
 *       inputs, for efficiency and consistency.
 *
 * @note `endpoint_offset` and `cost` are heading-invariant (computed in body-frame at `theta = 0`; identical
        across heading bins by symmetry of the kinematic model), while `swept_cells[k]` and `sample_cells[k]`
        carry per-heading-bin data indexed by the heading bin `k`. Bin `k` is anchored at `theta_k = -pi + k * (2*pi/N)`,
        spanning the signed wrapped range `[-pi, pi)` consistent with `wrapToPi`. `swept_cells[k]` is a guaranteed
        cover for any heading in bin `k` and any position within the robot's cell.

 *
 */

struct Primitive{
    Control control; ///< Robot velocity control input
    SE2State endpoint_offset; ///< Body-frame endpoint at theta=0; bin-invariant.
    double cost; ///< Control cost `sqrt(v^2 + rmin^2 * omega^2) * dt` [m], with no map term; bin-invariant.
    /// Body-frame cells covering the motion, one list per heading bin (outer size == n_headings), sorted and unique.
    /// Translate by the robot's cell: if none of these cells is occupied, the motion is collision-free from
    /// any pose in that cell whose heading lies in bin `k`. An occupied cell does not prove a collision.
    std::vector<std::vector<GridCoords>> swept_cells;
    /// Body-frame cell of the robot centre at each trajectory sample, evaluated at the mid-bin heading, one
    /// list per heading bin (outer size == n_headings). Every list has the same length as `sample_clearance`.
    std::vector<std::vector<GridCoords>> sample_cells;
    /// Obstacle-free distance (m) required around each sample's centre cell for that sample to be safe; same
    /// length and order as each `sample_cells[k]`, bin-invariant. Pass to `CollisionChecker::checkTrajectory`.
    std::vector<double> sample_clearance;
};
