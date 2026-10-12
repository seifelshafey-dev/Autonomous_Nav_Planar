/**
 * @file path.hpp
 * @author Seif ElShafey
 * @brief Output representation of a planned trajectory: a sequence of control-applied segments and
 *        the SE(2) poses at their boundaries.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <vector>
#include "core/types/state.hpp"
#include "core/types/control.hpp"

/**
 * @brief One segment of a planned path: a single control input applied for `u.dt`, the SE(2) poses
 *        sampled densely along it, and the cost contribution of that segment.
 */
struct PathSegment{
    Control u; ///< Control input applied along this segment.
    std::vector<SE2State> samples_poses; ///< Densely sampled poses; consecutive entries are within the rasterizer overlap.
    double cost; ///< Cost contribution of this segment.
};

/**
 * @brief A complete planned path: an ordered list of segments, the SE(2) poses at each node
 *        boundary, and the total path cost.
 *
 * @note `node_points.size() == segments.size() + 1` — `node_points.front()` is the start pose,
 *       `node_points.back()` is the final pose (the goal, or a state within the goal tolerances), and `node_points[i+1]` is the pose at the end of
 *       `segments[i]`.
 * @note For a path returned by `findEscape`, `node_points.back()` is a pose clear of obstacles, not a goal.
 */
struct Path{
    std::vector<PathSegment> segments; ///< Segments connecting consecutive waypoints.
    std::vector<SE2State> node_points; ///< Pose at each node boundary; size == segments.size() + 1.
    double total_cost; ///< Sum of segment costs.
};
