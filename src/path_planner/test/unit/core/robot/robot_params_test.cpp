#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "core/robot/robot_params.hpp"
#include "core/types/geometry.hpp"

/*
robot_params contract:

validateConvexCCWPolygon(v):
    Precondition predicate. Passes iff v has >= 3 vertices forming a strictly convex polygon
    wound CCW. Throws std::invalid_argument on: <3 vertices, clockwise winding, concavity,
    or collinear (redundant) vertices.

buildRobotParams(v):
    Enforces validateConvexCCWPolygon, then requires the body origin to lie strictly inside the
    footprint. Returns derived geometry: one outward unit half-plane per edge, r_in = min
    origin-to-edge distance, r_out = max origin-to-vertex distance.
*/

namespace {
// A CCW, strictly convex rectangle centered on the origin.
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}
}

// ---- validateConvexCCWPolygon: accept ----

TEST(ValidatePolygon, acceptsConvexCCWRectangle){
    EXPECT_NO_THROW(validateConvexCCWPolygon(ccwRect()));
}

TEST(ValidatePolygon, acceptsTriangle){
    // >= 3 vertices is allowed; a CCW triangle is valid.
    std::vector<Vec2> tri = {{1, -1}, {1, 1}, {-1, 0}};
    EXPECT_NO_THROW(validateConvexCCWPolygon(tri));
}

// ---- validateConvexCCWPolygon: reject ----

TEST(ValidatePolygon, rejectsFewerThanThreeVertices){
    EXPECT_THROW(validateConvexCCWPolygon({}), std::invalid_argument);
    EXPECT_THROW(validateConvexCCWPolygon({{0, 0}}), std::invalid_argument);
    EXPECT_THROW(validateConvexCCWPolygon({{0, 0}, {1, 1}}), std::invalid_argument);
}

TEST(ValidatePolygon, rejectsClockwiseWinding){
    std::vector<Vec2> cw = ccwRect();
    std::reverse(cw.begin(), cw.end()); // CCW reversed -> CW
    EXPECT_THROW(validateConvexCCWPolygon(cw), std::invalid_argument);
}

TEST(ValidatePolygon, rejectsConcavePolygon){
    // Diamond with one vertex pulled in toward the origin -> reflex (concave) corner.
    std::vector<Vec2> concave = {{2, 0}, {0.2, 0.2}, {0, 2}, {-2, 0}, {0, -2}};
    EXPECT_THROW(validateConvexCCWPolygon(concave), std::invalid_argument);
}

TEST(ValidatePolygon, rejectsCollinearVertex){
    // Square with a redundant vertex (1,0) on the bottom edge -> zero cross at that vertex.
    std::vector<Vec2> collinear = {{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}};
    EXPECT_THROW(validateConvexCCWPolygon(collinear), std::invalid_argument);
}

// ---- buildRobotParams: input -> output for valid geometry ----

TEST(BuildRobotParams, derivesGeometryForRectangle){
    RobotParams p = buildRobotParams(ccwRect());

    EXPECT_EQ(p.vertices.size(), 4u);
    EXPECT_EQ(p.half_planes.size(), 4u);

    // r_in = nearest edge distance = 0.20; r_out = farthest vertex = sqrt(0.3^2 + 0.2^2).
    EXPECT_NEAR(p.r_in, 0.20, 1e-9);
    EXPECT_NEAR(p.r_out, std::sqrt(0.13), 1e-9);

    // Every half-plane normal is a unit vector and faces outward (positive offset).
    for (const auto & hp : p.half_planes){
        EXPECT_NEAR(hp.normal.norm(), 1.0, 1e-9);
        EXPECT_GT(hp.offset, 0.0);
    }
}

TEST(BuildRobotParams, acceptsTriangleContainingOrigin){
    std::vector<Vec2> tri = {{1, -1}, {1, 1}, {-1, 0}};
    RobotParams p = buildRobotParams(tri);
    EXPECT_EQ(p.half_planes.size(), 3u);
    EXPECT_GT(p.r_in, 0.0);
}

// ---- buildRobotParams: rejection paths ----

TEST(BuildRobotParams, rejectsInvalidPolygon){
    // Clockwise winding fails inside validateConvexCCWPolygon before any geometry is derived.
    std::vector<Vec2> cw = ccwRect();
    std::reverse(cw.begin(), cw.end());
    EXPECT_THROW(buildRobotParams(cw), std::invalid_argument);
}

TEST(BuildRobotParams, rejectsPolygonNotContainingOrigin){
    // Valid convex CCW square, but shifted into the first quadrant so the origin is outside.
    std::vector<Vec2> shifted = {{1, 1}, {2, 1}, {2, 2}, {1, 2}};
    EXPECT_THROW(buildRobotParams(shifted), std::invalid_argument);
}
