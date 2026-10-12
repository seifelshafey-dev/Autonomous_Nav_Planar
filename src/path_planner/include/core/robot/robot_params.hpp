/**
 * @file robot_params.hpp
 * @author Seif ElShafey
 * @brief Robot footprint geometry: convex polygon, outward half-planes, and inner/outer radii.
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <vector>
#include "core/types/geometry.hpp"

/**
 * @brief Defines the robot geometry using vertices, convex half-planes representing the exterior
 *        structure, and an approximate footprint via inner/outer bounding circles.
 *        The footprint must be a convex polygon of at least 3 vertices.
 *
 */
struct RobotParams{
    std::vector<Vec2> vertices; ///< Robot polygonal vertices along a plane
    std::vector<HalfPlane> half_planes; ///< The set of all convex half-planes defining the external structure
    double r_in; ///< Radius (m) of the largest circle centered on the robot's origin that fits inside the polygon
    double r_out; ///< Minimum radius of circle centered around robot's origin such that all robot vertices fit inside the circle
};

/**
 * @brief Validate that a vertex list describes a footprint usable by buildRobotParams: at least
 *        3 vertices forming a strictly convex polygon wound counter-clockwise (CCW).
 *
 * Strict convexity rejects collinear (redundant) vertices; CCW ordering is required so that the
 * outward half-plane normals derived in buildRobotParams carry the correct sign.
 *
 * @param v Body-centered polygon vertices.
 * @throws std::invalid_argument if v has fewer than 3 vertices, is non-convex, contains collinear
 *         vertices, or is wound clockwise.
 */
void validateConvexCCWPolygon(const std::vector<Vec2> & v);

/**
 * @brief Factory method for constructing robot geometry from a set of vertices. The vertices
 *        are inherited from an external config file provided by the user. Method should only be called
 *        once per robot.
 *
 * @param v Body-centered (around origin) vertices defining a strictly convex, CCW-wound polygon
 *          that contains the body origin. These are preconditions, enforced via validateConvexCCWPolygon
 *          and an origin-containment check; violations throw std::invalid_argument.
 *
 * @return RobotParams fully defined robot geometry based on polygonal properties
 * @throws std::invalid_argument if v is not a valid convex CCW polygon or does not contain the origin.
 */

RobotParams buildRobotParams(std::vector<Vec2> v);
