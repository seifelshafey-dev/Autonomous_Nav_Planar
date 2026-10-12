/**
 * @file world_robot_geometry.cpp
 * @author Seif ElShafey
 * @brief Implementation of the transformGeometry() function declared in world_robot_geometry.hpp
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "core/motion/world_robot_geometry.hpp"

// The translation absorbs `cell_origin` so the output geometry sits in map-grid-aligned frame when
// the caller supplies a non-zero origin.
WorldRobotGeometry transformGeometry(const RobotParams & p, const SE2State & state,
                                     const Vec2 & cell_origin){
    Vec2 trans_ = {state.x - cell_origin.x(), state.y - cell_origin.y()};
    Rot2 rot_(state.theta);

    std::vector<Vec2> v_world;
    std::vector<HalfPlane> hp_world;

    v_world.reserve(p.vertices.size());

    for(auto &x : p.vertices){
        v_world.push_back((rot_* x) + trans_);
    }

    AABB bbox;
    bbox.min = v_world[0];
    bbox.max = v_world[0];
    for(const auto & x : v_world){
        bbox.min = bbox.min.cwiseMin(x);
        bbox.max = bbox.max.cwiseMax(x);
    }

    for(const auto &x : p.half_planes){
        Vec2 norm_world = rot_ * x.normal;
        double offset_world = x.offset + norm_world.dot(trans_);
        hp_world.push_back({norm_world, offset_world});
    }

    return WorldRobotGeometry{hp_world, bbox};
}