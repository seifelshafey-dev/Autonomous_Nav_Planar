#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "core/motion/world_robot_geometry.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"

/*
Property-based tests for transformGeometry (RapidCheck). Oracles are independent invariants, not a
re-run of the n_world = rot*n / d_world = d + n.trans formula (that would be tautological).

1. halfPlaneSignedDistanceIsRigidInvariant: a rigid motion preserves the signed half-plane value.
   For a body point q and its world image Q = R*q + trans, every k must satisfy
   n_world_k . Q - d_world_k == n_k . q - d_k (orthogonality of R makes the trans terms cancel).
   This pins down BOTH the normal rotation and the offset shift via geometry, independent of the
   point transform. A wrong normal direction or a missing n.trans term in the offset fails it.

2. aabbContainsAndIsTightOnVertices: the AABB contains every transformed vertex and each of its four
   bounds is attained by some vertex (independent min/max reduction over the transformed vertices).

3. cellOriginIsAdditiveShift: transformGeometry(p, {x,y,th}, {ox,oy}) equals
   transformGeometry(p, {x-ox, y-oy, th}, {0,0}) -- a metamorphic relation independent of the formula.
*/

namespace {
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}
double coord(){ return *rc::gen::inRange(-50, 51) / 10.0; }       // -5.0 .. 5.0
double angle(){ return *rc::gen::inRange(-31415, 31416) / 10000.0; } // ~ -pi .. pi
}

// 1. Signed half-plane value is invariant under the rigid transform.
RC_GTEST_PROP(WorldGeomProp, halfPlaneSignedDistanceIsRigidInvariant, ()){
    RobotParams p = buildRobotParams(ccwRect());
    SE2State s{coord(), coord(), angle()};
    Vec2 origin(coord(), coord());
    WorldRobotGeometry g = transformGeometry(p, s, origin);

    Rot2 R(s.theta);
    Vec2 trans(s.x - origin.x(), s.y - origin.y());
    Vec2 q(*rc::gen::inRange(-100, 101) / 100.0, *rc::gen::inRange(-100, 101) / 100.0);
    Vec2 Q = R * q + trans;

    RC_ASSERT(g.half_planes.size() == p.half_planes.size());
    for (std::size_t k = 0; k < p.half_planes.size(); ++k){
        double body  = p.half_planes[k].normal.dot(q) - p.half_planes[k].offset;
        double world = g.half_planes[k].normal.dot(Q) - g.half_planes[k].offset;
        RC_ASSERT(std::abs(world - body) < 1e-9);
    }
}

// 2. AABB contains all transformed vertices and is tight on them.
RC_GTEST_PROP(WorldGeomProp, aabbContainsAndIsTightOnVertices, ()){
    RobotParams p = buildRobotParams(ccwRect());
    SE2State s{coord(), coord(), angle()};
    Vec2 origin(coord(), coord());
    WorldRobotGeometry g = transformGeometry(p, s, origin);

    Rot2 R(s.theta);
    Vec2 trans(s.x - origin.x(), s.y - origin.y());

    const double inf = std::numeric_limits<double>::infinity();
    double minx = inf, miny = inf, maxx = -inf, maxy = -inf;
    for (const auto & v : p.vertices){
        Vec2 V = R * v + trans;
        RC_ASSERT(V.x() >= g.aabb.min.x() - 1e-9);
        RC_ASSERT(V.y() >= g.aabb.min.y() - 1e-9);
        RC_ASSERT(V.x() <= g.aabb.max.x() + 1e-9);
        RC_ASSERT(V.y() <= g.aabb.max.y() + 1e-9);
        minx = std::min(minx, V.x()); miny = std::min(miny, V.y());
        maxx = std::max(maxx, V.x()); maxy = std::max(maxy, V.y());
    }
    // Tightness: each bound is achieved by a vertex.
    RC_ASSERT(std::abs(g.aabb.min.x() - minx) < 1e-9);
    RC_ASSERT(std::abs(g.aabb.min.y() - miny) < 1e-9);
    RC_ASSERT(std::abs(g.aabb.max.x() - maxx) < 1e-9);
    RC_ASSERT(std::abs(g.aabb.max.y() - maxy) < 1e-9);
    RC_ASSERT(g.aabb.min.x() <= g.aabb.max.x());
    RC_ASSERT(g.aabb.min.y() <= g.aabb.max.y());
}

// 3. cell_origin is a pure additive shift of the state translation.
RC_GTEST_PROP(WorldGeomProp, cellOriginIsAdditiveShift, ()){
    RobotParams p = buildRobotParams(ccwRect());
    double x = coord(), y = coord(), th = angle();
    double ox = coord(), oy = coord();

    WorldRobotGeometry a = transformGeometry(p, SE2State{x, y, th}, Vec2(ox, oy));
    WorldRobotGeometry b = transformGeometry(p, SE2State{x - ox, y - oy, th}, Vec2::Zero());

    RC_ASSERT(std::abs(a.aabb.min.x() - b.aabb.min.x()) < 1e-9);
    RC_ASSERT(std::abs(a.aabb.min.y() - b.aabb.min.y()) < 1e-9);
    RC_ASSERT(std::abs(a.aabb.max.x() - b.aabb.max.x()) < 1e-9);
    RC_ASSERT(std::abs(a.aabb.max.y() - b.aabb.max.y()) < 1e-9);

    RC_ASSERT(a.half_planes.size() == b.half_planes.size());
    for (std::size_t k = 0; k < a.half_planes.size(); ++k){
        RC_ASSERT(std::abs(a.half_planes[k].normal.x() - b.half_planes[k].normal.x()) < 1e-9);
        RC_ASSERT(std::abs(a.half_planes[k].normal.y() - b.half_planes[k].normal.y()) < 1e-9);
        RC_ASSERT(std::abs(a.half_planes[k].offset     - b.half_planes[k].offset)     < 1e-9);
    }
}
