/**
 * @file robot_params.cpp
 * @author Seif ElShafey
 * @brief Implementation of the RobotParams factory method defined in robot_params.hpp
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "core/robot/robot_params.hpp"
#include "core/diagnostics/check.hpp"

namespace {
// Strict-convexity threshold on the signed cross product of consecutive edges. Any value at or
// below this (negative for clockwise/concave turns, ~0 for collinear vertices) is rejected.
constexpr double kConvexCrossEps = 1e-9;
}

void validateConvexCCWPolygon(const std::vector<Vec2> & v){
    const int n = static_cast<int>(v.size());
    CHECK(n >= 3, "polygon must have at least 3 vertices");

    // For a strictly convex, CCW-wound polygon every consecutive edge pair turns left, i.e. the
    // signed 2D cross product edge_i x edge_{i+1} is strictly positive. A clockwise winding makes
    // all crosses negative; a concave vertex makes at least one negative; a collinear vertex makes
    // one zero. The single strict-positivity test covers all three rejection cases.
    for (int i = 0; i < n; i++){
        Vec2 edge_curr = v[(i + 1) % n] - v[i];
        Vec2 edge_next = v[(i + 2) % n] - v[(i + 1) % n];
        double cross = edge_curr.x() * edge_next.y() - edge_curr.y() * edge_next.x();
        CHECK(cross > kConvexCrossEps,
              "polygon must be strictly convex with counter-clockwise (CCW) vertex ordering");
    }
}

RobotParams buildRobotParams(std::vector<Vec2> v){

    validateConvexCCWPolygon(v);

    std::vector<Vec2> edges;
    std::vector<HalfPlane> half_p;

    int n_vertices = v.size();

    for(int i = 0; i<n_vertices; i++){
        edges.push_back((v[(i+1) % n_vertices] - v[i]));
    }

    int n_edges = edges.size();

    for (int i = 0; i<n_edges; i++){
       Vec2 norm = Vec2(edges[i].y(), -edges[i].x()).normalized();
       double offset = norm.dot(v[i]);

       half_p.push_back({norm, offset});
    }

    /* Since r_in defines the smallest circle inside the polygon, we define the radius as the
     * smallest distance from the robot's origin to an external wall (half-plane). Any r_in greater would
     * inherently grow out of the polygon.
     * 
     * As for r_out, since the circle inscribes all the vertices, the minimum radius is defined by the position
     * of the farthest vertex from the origin. Any smaller and the vertex isn't included, and larger and the 
     * circle is too large.
     */
    
    double r_i = std::numeric_limits<double>::infinity();
    double r_o = 0.0;

    for(auto & x : v){
        r_o = std::max(r_o, x.norm());
    }

    for(auto & hp : half_p){
        CHECK(hp.offset > 0, "origin must lie strictly inside the robot footprint");
        r_i = std::min(r_i, hp.offset);
    }

    RobotParams p = {.vertices = v, .half_planes = half_p, .r_in= r_i, .r_out = r_o};

    return p;
}
