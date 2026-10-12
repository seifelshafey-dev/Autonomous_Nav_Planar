#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

#include "core/map/rasterize.hpp"
#include "core/types/geometry.hpp"

/*
rasterizePolygon(half_planes, aabb, cell_size):
    Inputs are caller-guaranteed valid (from world_geometry). A cell is included iff its center
    satisfies every half-plane inflated by clearance = (sqrt(2)/2)*cell_size. This conservatively
    covers any cell the true polygon touches; cells inside the AABB but outside the polygon are
    excluded.
*/

namespace {
HalfPlane hp(double nx, double ny, double off){ return HalfPlane{Vec2(nx, ny), off}; }

// Axis-aligned rectangle [x0,x1] x [y0,y1].
std::vector<HalfPlane> rectHalfPlanes(double x0, double y0, double x1, double y1){
    return {hp(1, 0, x1), hp(-1, 0, -x0), hp(0, 1, y1), hp(0, -1, -y0)};
}

// Diamond {|x| + |y| <= r}.
std::vector<HalfPlane> diamondHalfPlanes(double r){
    const double s = 1.0 / std::sqrt(2.0);
    return {hp(s, s, r * s), hp(s, -s, r * s), hp(-s, s, r * s), hp(-s, -s, r * s)};
}

bool contains(const std::vector<GridCoords> & v, GridCoords c){
    return std::find(v.begin(), v.end(), c) != v.end();
}
}

// ===== rasterizePolygon =====

TEST(RasterizePolygon, axisAlignedSquareCoversInteriorCells){
    auto cells = rasterizePolygon(rectHalfPlanes(0, 0, 2, 2), AABB{Vec2(0, 0), Vec2(2, 2)}, 1.0);
    EXPECT_TRUE(contains(cells, {0, 0}));
    EXPECT_TRUE(contains(cells, {1, 0}));
    EXPECT_TRUE(contains(cells, {0, 1}));
    EXPECT_TRUE(contains(cells, {1, 1}));
}

// The key contract case: cells within the AABB scan but outside the polygon are dropped.
TEST(RasterizePolygon, diamondExcludesAABBCornerCells){
    // {|x|+|y|<=3}, AABB [-3,3]^2, cs=1. Cell (2,2) center (2.5,2.5) is inside the AABB but
    // outside the diamond (5 > 3) by more than the clearance, so it must be excluded.
    auto cells = rasterizePolygon(diamondHalfPlanes(3.0), AABB{Vec2(-3, -3), Vec2(3, 3)}, 1.0);
    EXPECT_TRUE(contains(cells, {0, 0}));    // interior
    EXPECT_FALSE(contains(cells, {2, 2}));   // inside AABB, outside polygon
    EXPECT_FALSE(contains(cells, {-3, -3})); // far AABB corner
}

TEST(RasterizePolygon, subCellPolygonReturnsContainingCell){
    auto cells = rasterizePolygon(rectHalfPlanes(0.4, 0.4, 0.6, 0.6),
                                  AABB{Vec2(0.4, 0.4), Vec2(0.6, 0.6)}, 1.0);
    EXPECT_TRUE(contains(cells, {0, 0}));
    EXPECT_FALSE(cells.empty());
}

TEST(RasterizePolygonMargin, zeroMarginEqualsThreeArgumentCall){
    const auto hps = diamondHalfPlanes(3.0);
    const AABB box{Vec2(-3, -3), Vec2(3, 3)};
    const auto a = rasterizePolygon(hps, box, 0.5);
    const auto b = rasterizePolygon(hps, box, 0.5, 0.0);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t n = 0; n < a.size(); ++n) EXPECT_EQ(a[n], b[n]);
}

TEST(RasterizePolygonMargin, marginWidensCoverageByWholeCells){
    // Square [0, 0.2]^2 at cs = 0.1. Cell centres within (sqrt2/2)*cs + margin of the square are kept:
    // margin 0 -> i, j in [-1, 2]; margin 0.1 -> i, j in [-2, 3].
    const auto hps = rectHalfPlanes(0, 0, 0.2, 0.2);
    const AABB box{Vec2(0, 0), Vec2(0.2, 0.2)};
    auto range = [](const std::vector<GridCoords> & v){
        int lo = 1 << 30, hi = -(1 << 30);
        for (const auto & c : v) { lo = std::min({lo, c.i, c.j}); hi = std::max({hi, c.i, c.j}); }
        return std::make_pair(lo, hi);
    };
    EXPECT_EQ(range(rasterizePolygon(hps, box, 0.1, 0.0)), std::make_pair(-1, 2));
    EXPECT_EQ(range(rasterizePolygon(hps, box, 0.1, 0.1)), std::make_pair(-2, 3));
}

TEST(RasterizePolygonMargin, negativeOrNonFiniteMarginThrows){
    const auto hps = rectHalfPlanes(0, 0, 1, 1);
    const AABB box{Vec2(0, 0), Vec2(1, 1)};
    EXPECT_THROW(rasterizePolygon(hps, box, 0.1, -0.01), std::invalid_argument);
    EXPECT_THROW(rasterizePolygon(hps, box, 0.1, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(rasterizePolygon(hps, box, 0.1, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(RasterizePolygonCellSize, nonPositiveOrNonFiniteCellSizeThrows){
    const auto hps = rectHalfPlanes(0, 0, 1, 1);
    const AABB box{Vec2(0, 0), Vec2(1, 1)};
    EXPECT_THROW(rasterizePolygon(hps, box, 0.0), std::invalid_argument);
    EXPECT_THROW(rasterizePolygon(hps, box, -0.1), std::invalid_argument);
    EXPECT_THROW(rasterizePolygon(hps, box, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

// ===== visitPolygonCells =====

namespace {
std::vector<GridCoords> visitAll(const std::vector<HalfPlane> & hps, const AABB & box, double cs, double margin,
                                 bool * result = nullptr){
    std::vector<GridCoords> out;
    const bool r = visitPolygonCells(hps, box, cs, margin, [&](GridCoords c){ out.push_back(c); return true; });
    if (result) *result = r;
    return out;
}
struct VisitorError {};
}

TEST(VisitPolygonCells, matchesRasterizePolygonElementForElement){
    const auto rect = rectHalfPlanes(0, 0, 2, 2);
    const AABB rbox{Vec2(0, 0), Vec2(2, 2)};
    const auto dia = diamondHalfPlanes(3.0);
    const AABB dbox{Vec2(-3, -3), Vec2(3, 3)};
    struct Case { const std::vector<HalfPlane> * hps; const AABB * box; double cs, margin; };
    for (const Case & k : {Case{&rect, &rbox, 1.0, 0.0}, Case{&dia, &dbox, 1.0, 0.0}, Case{&dia, &dbox, 0.5, 0.3}}){
        bool ret = false;
        const auto got = visitAll(*k.hps, *k.box, k.cs, k.margin, &ret);
        EXPECT_TRUE(ret);
        EXPECT_EQ(got, rasterizePolygon(*k.hps, *k.box, k.cs, k.margin));
    }
}

TEST(VisitPolygonCells, stopsAtFirstFalseVisit){
    const auto dia = diamondHalfPlanes(3.0);
    const AABB box{Vec2(-3, -3), Vec2(3, 3)};
    ASSERT_GE(rasterizePolygon(dia, box, 1.0).size(), 4u);
    for (int k : {1, 3}){
        int calls = 0;
        const bool r = visitPolygonCells(dia, box, 1.0, 0.0, [&](GridCoords){ return ++calls < k; });
        EXPECT_FALSE(r);
        EXPECT_EQ(calls, k);
    }
}

TEST(VisitPolygonCells, alwaysTrueVisitorOnSubCellPolygonReturnsTrue){
    bool ret = false;
    const auto got = visitAll(rectHalfPlanes(0.4, 0.4, 0.6, 0.6), AABB{Vec2(0.4, 0.4), Vec2(0.6, 0.6)}, 1.0, 0.0, &ret);
    EXPECT_TRUE(ret);
    EXPECT_FALSE(got.empty());
}

TEST(VisitPolygonCells, invalidCellSizeOrMarginThrowsBeforeAnyVisit){
    const auto hps = rectHalfPlanes(0, 0, 1, 1);
    const AABB box{Vec2(0, 0), Vec2(1, 1)};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    int calls = 0;
    auto v = [&](GridCoords){ ++calls; return true; };
    EXPECT_THROW(visitPolygonCells(hps, box, 0.0, 0.0, v), std::invalid_argument);
    EXPECT_THROW(visitPolygonCells(hps, box, -0.1, 0.0, v), std::invalid_argument);
    EXPECT_THROW(visitPolygonCells(hps, box, nan, 0.0, v), std::invalid_argument);
    EXPECT_THROW(visitPolygonCells(hps, box, 0.1, -0.01, v), std::invalid_argument);
    EXPECT_THROW(visitPolygonCells(hps, box, 0.1, nan, v), std::invalid_argument);
    EXPECT_THROW(visitPolygonCells(hps, box, 0.1, inf, v), std::invalid_argument);
    EXPECT_EQ(calls, 0);
}

TEST(VisitPolygonCells, visitorExceptionPropagates){
    EXPECT_THROW(visitPolygonCells(rectHalfPlanes(0, 0, 2, 2), AABB{Vec2(0, 0), Vec2(2, 2)}, 1.0, 0.0,
                                   [](GridCoords) -> bool { throw VisitorError{}; }),
                 VisitorError);
}
