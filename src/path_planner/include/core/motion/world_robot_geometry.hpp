/**
 * @file world_robot_geometry.hpp
 * @author Seif ElShafey
 * @brief Defines the polygonal geometry of our robot in world coordinates (m).
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/robot/robot_params.hpp"

/**
 * @brief Robot polygon transformed into world (or map-grid-aligned) coordinates: state-aware
 *        half-planes plus the bounding AABB. Produced by `transformGeometry` and consumed by
 *        `rasterizePolygon`. `r_in` / `r_out` (robot_params.hpp) are frame-independent scalars and
 *        are therefore not carried here.
 */

struct WorldRobotGeometry{
    std::vector<HalfPlane> half_planes; ///< State-aware half-planes in world coordinates (m)
    AABB aabb; ///< Grid bounding box around the robot at given state (4 values in m)
};

/**
 * @brief Utility function that translates and rotates the vertices and half-planes of the robot into the
 configuration space, along with computing its surrounding AABB.

 Requires the robot to be defined locally, and the state to be valid. The optional `cell_origin`
 parameter shifts the output frame so that the produced geometry is anchored relative to it: pass
 `(0, 0)` (the default) for body-frame or "world (0, 0)" output, or pass the OccupancyGrid's
 `info.origin` so the output is in map-grid-aligned frame and the downstream rasterizer's cell
 indices line up directly with `worldToGrid` / `findCellStatus`.

 * @param p Locally-defined robot geometry constructed on robot instantiation. Inherently static (robot does not change shape).
 * @param state Robot state in the parent (map) frame [m, m, rad]. Passed from a localizer/SLAM node at defined intervals.
 * @param cell_origin World position of grid cell (0, 0); subtracted from `state` translation so the
 *                    output geometry sits in map-grid-aligned frame. Defaults to (0, 0).
 * @return WorldRobotGeometry Robot geometry translated by `(state.x - cell_origin.x, state.y - cell_origin.y)`
 *                            and rotated by `state.theta`.
 */

WorldRobotGeometry transformGeometry(const RobotParams & p, const SE2State & state,
                                     const Vec2 & cell_origin = Vec2::Zero());