#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "core/map/grid.hpp"

/*
GridMetaData contract:
    Invariants (CHECK, throws std::invalid_argument): cols > 0, rows > 0, resolution > 0.
    inBounds(gc): true iff 0 <= gc.i < cols AND 0 <= gc.j < rows (half-open).
    worldToGrid(x,y): floor-based snap; cell (i,j) covers [i*res+ox,(i+1)*res+ox) x [j*res+oy,(j+1)*res+oy).
    gridToWorld(gc): center of cell (gc.i, gc.j) = ((i+0.5)*res+ox, (j+0.5)*res+oy).
    world_width/height: ceil(cols*res) / ceil(rows*res) -- a deliberate whole-meter over-estimate.
*/

namespace {
Eigen::Vector2f origin(float x, float y){ return Eigen::Vector2f{x, y}; }
}

// ---- world dimensions (ceil) ----

TEST(GridMetaData, worldDimsAreCeilOfExtent){
    GridMetaData exact(10, 10, 0.1, origin(0, 0)); // 10*0.1 = 1.0
    EXPECT_FLOAT_EQ(exact.world_width, 1.0f);
    EXPECT_FLOAT_EQ(exact.world_height, 1.0f);

    GridMetaData rounded(10, 10, 0.15, origin(0, 0)); // 10*0.15 = 1.5 -> ceil 2.0
    EXPECT_FLOAT_EQ(rounded.world_width, 2.0f);
    EXPECT_FLOAT_EQ(rounded.world_height, 2.0f);
}

// ---- inBounds ----

TEST(GridMetaData, inBoundsHalfOpenRange){
    GridMetaData meta(4, 10, 1.0, origin(0, 0)); // cols=4, rows=10 (non-square on purpose)
    EXPECT_TRUE(meta.inBounds({0, 0}));
    EXPECT_TRUE(meta.inBounds({3, 9}));   // cols-1, rows-1
    EXPECT_FALSE(meta.inBounds({4, 0}));  // i == cols
    EXPECT_FALSE(meta.inBounds({0, 10})); // j == rows
    EXPECT_FALSE(meta.inBounds({-1, 0}));
    EXPECT_FALSE(meta.inBounds({0, -1}));
}

// Non-square grid distinguishes the i<->cols, j<->rows pairing (catches a swapped comparison).
TEST(GridMetaData, inBoundsDoesNotSwapColsAndRows){
    GridMetaData meta(4, 10, 1.0, origin(0, 0));
    EXPECT_FALSE(meta.inBounds({5, 0})); // i=5 >= cols=4
    EXPECT_TRUE(meta.inBounds({0, 5}));  // j=5 <  rows=10
}

// ---- worldToGrid ----

TEST(GridMetaData, worldToGridFloorAndBoundaries){
    GridMetaData meta(10, 10, 1.0, origin(0, 0));
    EXPECT_EQ(meta.worldToGrid(0.0, 0.0), (GridCoords{0, 0}));
    EXPECT_EQ(meta.worldToGrid(0.5, 0.5), (GridCoords{0, 0}));
    EXPECT_EQ(meta.worldToGrid(1.0, 0.0), (GridCoords{1, 0})); // boundary -> upper cell
    EXPECT_EQ(meta.worldToGrid(9.9, 9.9), (GridCoords{9, 9}));
}

TEST(GridMetaData, worldToGridNegativeUsesFloorNotTruncation){
    GridMetaData meta(10, 10, 1.0, origin(0, 0));
    // A point just left/below the origin must land in cell -1, not 0 (floor, not toward-zero).
    EXPECT_EQ(meta.worldToGrid(-0.5, -0.5), (GridCoords{-1, -1}));
    EXPECT_EQ(meta.worldToGrid(-1.0, -1.0), (GridCoords{-1, -1})); // boundary -> upper cell
}

TEST(GridMetaData, worldToGridAppliesOrigin){
    GridMetaData meta(10, 10, 1.0, origin(5, 5));
    EXPECT_EQ(meta.worldToGrid(5.0, 5.0), (GridCoords{0, 0}));
    EXPECT_EQ(meta.worldToGrid(4.5, 5.0), (GridCoords{-1, 0}));
    EXPECT_EQ(meta.worldToGrid(15.0, 5.0), (GridCoords{10, 0}));
}

// ---- gridToWorld ----

TEST(GridMetaData, gridToWorldReturnsCellCenter){
    GridMetaData meta(10, 10, 1.0, origin(0, 0));
    EXPECT_TRUE(meta.gridToWorld({0, 0}).isApprox(origin(0.5f, 0.5f)));
    EXPECT_TRUE(meta.gridToWorld({2, 3}).isApprox(origin(2.5f, 3.5f)));
    EXPECT_TRUE(meta.gridToWorld({-1, -1}).isApprox(origin(-0.5f, -0.5f)));
}

TEST(GridMetaData, gridToWorldAppliesOrigin){
    GridMetaData meta(10, 10, 1.0, origin(5, 5));
    EXPECT_TRUE(meta.gridToWorld({0, 0}).isApprox(origin(5.5f, 5.5f)));
}

// ---- invariant checks ----
TEST(GridMetaData, rejectsNonPositiveDimensions){
    Eigen::Vector2f o = origin(0, 0);
    EXPECT_THROW(GridMetaData(0, 10, 1.0, o), std::invalid_argument);    // cols == 0
    EXPECT_THROW(GridMetaData(10, 0, 1.0, o), std::invalid_argument);    // rows == 0
    EXPECT_THROW(GridMetaData(-1, 10, 1.0, o), std::invalid_argument);   // cols < 0
    EXPECT_THROW(GridMetaData(10, -1, 1.0, o), std::invalid_argument);   // rows < 0
    EXPECT_THROW(GridMetaData(10, 10, 0.0, o), std::invalid_argument);   // resolution == 0
    EXPECT_THROW(GridMetaData(10, 10, -1.0, o), std::invalid_argument);  // resolution < 0
    EXPECT_THROW(GridMetaData(10, 10, std::nan(""), o), std::invalid_argument);  // resolution NaN
}

TEST(GridMetaData, rejectsNonFiniteResolutionAndOrigin){
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(GridMetaData(10, 10, std::numeric_limits<double>::infinity(), origin(0, 0)), std::invalid_argument);
    EXPECT_THROW(GridMetaData(10, 10, 1e39, origin(0, 0)), std::invalid_argument);  // +inf as float
    EXPECT_THROW(GridMetaData(10, 10, 0.05, origin(nan, 0)), std::invalid_argument);
    EXPECT_THROW(GridMetaData(10, 10, 0.05, origin(0, nan)), std::invalid_argument);
    EXPECT_THROW(GridMetaData(10, 10, 0.05, origin(inf, 0)), std::invalid_argument);
    EXPECT_THROW(GridMetaData(10, 10, 0.05, origin(0, -inf)), std::invalid_argument);
    EXPECT_NO_THROW(GridMetaData(10, 10, 0.05, origin(-1e6f, 1e6f)));
}
