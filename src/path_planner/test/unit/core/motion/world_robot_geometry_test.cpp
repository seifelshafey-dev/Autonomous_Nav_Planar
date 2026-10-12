#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include <vector>

#include "core/motion/world_robot_geometry.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"

/*
transformGeometry contract:

Pure rigid-body transform of a (precondition-valid) RobotParams into world/grid-aligned frame.
    trans  = (state.x - cell_origin.x, state.y - cell_origin.y)
    rot    = Rot2(state.theta)
    AABB   = element-wise min/max of (rot * vertex_i + trans)
    per half-plane (n, d): n_world = rot * n,  d_world = d + n_world . trans

Preconditions are caller-owned: p comes from buildRobotParams (>=3 vertices, convex CCW, contains
the origin) and state is finite, so no input-validation tests here. Expected values below are computed
by hand from concrete states, not by re-running the transform formula.
*/

namespace {
constexpr double kPi = std::numbers::pi;

// A CCW, strictly convex 0.6 x 0.4 rectangle centered on the origin. Non-square so a 90-degree
// rotation visibly swaps the AABB extents.
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}

// The half-plane whose unit normal matches (nx, ny); fails the test if none. Decouples assertions
// from the order in which buildRobotParams emits edges.
HalfPlane hpWithNormal(const std::vector<HalfPlane> & hps, double nx, double ny){
    for (const auto & hp : hps)
        if (std::abs(hp.normal.x() - nx) < 1e-6 && std::abs(hp.normal.y() - ny) < 1e-6) return hp;
    ADD_FAILURE() << "no half-plane with normal (" << nx << ", " << ny << ")";
    return HalfPlane{Vec2::Zero(), 0.0};
}
}

// ---- Identity: zero state, zero origin -> output equals body geometry ----

TEST(TransformGeometry, identityReturnsBodyGeometry){
    RobotParams p = buildRobotParams(ccwRect());
    WorldRobotGeometry g = transformGeometry(p, SE2State{0.0, 0.0, 0.0});

    EXPECT_NEAR(g.aabb.min.x(), -0.30, 1e-9);
    EXPECT_NEAR(g.aabb.min.y(), -0.20, 1e-9);
    EXPECT_NEAR(g.aabb.max.x(),  0.30, 1e-9);
    EXPECT_NEAR(g.aabb.max.y(),  0.20, 1e-9);

    ASSERT_EQ(g.half_planes.size(), p.half_planes.size());
    for (std::size_t k = 0; k < p.half_planes.size(); ++k){
        EXPECT_NEAR(g.half_planes[k].normal.x(), p.half_planes[k].normal.x(), 1e-9);
        EXPECT_NEAR(g.half_planes[k].normal.y(), p.half_planes[k].normal.y(), 1e-9);
        EXPECT_NEAR(g.half_planes[k].offset,     p.half_planes[k].offset,     1e-9);
    }
}

// ---- Pure translation: normals unchanged, offsets shift by n . trans, AABB shifts by trans ----

TEST(TransformGeometry, pureTranslation){
    RobotParams p = buildRobotParams(ccwRect());
    WorldRobotGeometry g = transformGeometry(p, SE2State{1.0, 2.0, 0.0});

    EXPECT_NEAR(g.aabb.min.x(), 0.70, 1e-9);
    EXPECT_NEAR(g.aabb.min.y(), 1.80, 1e-9);
    EXPECT_NEAR(g.aabb.max.x(), 1.30, 1e-9);
    EXPECT_NEAR(g.aabb.max.y(), 2.20, 1e-9);

    // offset_world = d + n . (1, 2)
    EXPECT_NEAR(hpWithNormal(g.half_planes,  1,  0).offset,  1.30, 1e-9); // 0.3 + 1
    EXPECT_NEAR(hpWithNormal(g.half_planes, -1,  0).offset, -0.70, 1e-9); // 0.3 - 1
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0,  1).offset,  2.20, 1e-9); // 0.2 + 2
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0, -1).offset, -1.80, 1e-9); // 0.2 - 2
}

// ---- Pure rotation by +90 degrees: extents swap, normals rotate, offsets unchanged (trans = 0) ----

TEST(TransformGeometry, pureRotationNinetyDegrees){
    RobotParams p = buildRobotParams(ccwRect());
    WorldRobotGeometry g = transformGeometry(p, SE2State{0.0, 0.0, kPi / 2.0});

    // 0.6 x 0.4 body box becomes 0.4 x 0.6 after a quarter turn.
    EXPECT_NEAR(g.aabb.min.x(), -0.20, 1e-9);
    EXPECT_NEAR(g.aabb.min.y(), -0.30, 1e-9);
    EXPECT_NEAR(g.aabb.max.x(),  0.20, 1e-9);
    EXPECT_NEAR(g.aabb.max.y(),  0.30, 1e-9);

    // Rot90 maps body normal (1,0)->(0,1), (0,1)->(-1,0), (-1,0)->(0,-1), (0,-1)->(1,0);
    // offsets are unchanged because the translation is zero.
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0,  1).offset, 0.30, 1e-6); // from (1,0), d=0.3
    EXPECT_NEAR(hpWithNormal(g.half_planes, -1,  0).offset, 0.20, 1e-6); // from (0,1), d=0.2
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0, -1).offset, 0.30, 1e-6); // from (-1,0), d=0.3
    EXPECT_NEAR(hpWithNormal(g.half_planes,  1,  0).offset, 0.20, 1e-6); // from (0,-1), d=0.2
}

// ---- Combined rotation + translation: hand-worked +90 deg, trans (1,2) ----

TEST(TransformGeometry, combinedRotationAndTranslation){
    RobotParams p = buildRobotParams(ccwRect());
    WorldRobotGeometry g = transformGeometry(p, SE2State{1.0, 2.0, kPi / 2.0});

    EXPECT_NEAR(g.aabb.min.x(), 0.80, 1e-9);
    EXPECT_NEAR(g.aabb.min.y(), 1.70, 1e-9);
    EXPECT_NEAR(g.aabb.max.x(), 1.20, 1e-9);
    EXPECT_NEAR(g.aabb.max.y(), 2.30, 1e-9);

    // offset_world = d + n_world . (1, 2)
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0,  1).offset,  2.30, 1e-6); // 0.3 + (0,1).(1,2)
    EXPECT_NEAR(hpWithNormal(g.half_planes, -1,  0).offset, -0.80, 1e-6); // 0.2 + (-1,0).(1,2)
    EXPECT_NEAR(hpWithNormal(g.half_planes,  0, -1).offset, -1.70, 1e-6); // 0.3 + (0,-1).(1,2)
    EXPECT_NEAR(hpWithNormal(g.half_planes,  1,  0).offset,  1.20, 1e-6); // 0.2 + (1,0).(1,2)
}

// ---- cell_origin is a pure additive shift of the state translation ----

