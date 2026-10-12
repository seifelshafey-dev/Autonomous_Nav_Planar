/**
 * @file geometry.hpp
 * @author Seif ElShafey
 * @brief Planar geometric primitives (Vec2/Mat2/Rot2, HalfPlane, AABB) shared across map, motion, and robot modules.
 * @version 0.1
 * @date 2026-05-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

using Vec2 = Eigen::Vector2d; ///< 2D vector of doubles (x, y).
using Mat2 = Eigen::Matrix2d; ///< 2x2 matrix of doubles.
using Rot2 = Eigen::Rotation2Dd; ///< 2D rotation by an angle in radians.

/// @brief One half-plane of the robot's convex footprint boundary.
struct HalfPlane {
    Vec2 normal; ///< Outward unit normal (pointing away from the robot) of the polygon edge.
    double offset; ///< Perpendicular distance from the robot's origin to the plane, along the normal.
};

/**
 * @brief Axis-aligned bounding box spanning the extremities (leftmost, downmost, rightmost, and
 *        upmost vertices) of the robot in global (map-origin) coordinates (m). Used as the search
 *        area during rasterization (rasterize.hpp), but usable elsewhere.
 *
 * The min and max corners are element-independent: the minimum-x vertex need not be the minimum-y
 * vertex, so up to four distinct vertices (more precisely, their x or y values) can define the box.
 *
 * @note The box is state-aware: rotated by theta and translated by the robot's global x and y.
 */
struct AABB {
    Vec2 min; ///< Lower-left corner of the bounding box (minimum x and minimum y, element-independent).
    Vec2 max; ///< Upper-right corner of the bounding box (maximum x and maximum y, element-independent).
};
