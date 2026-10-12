#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/map/edt.hpp"
#include "core/map/grid.hpp"

/*
EDT contract:
    distanceAt(col,row): raw euclidean distance (meters), row-major index row*cols+col, clamped to
                         maximum_distance (read-time). getData() returns the raw, unclamped field.
buildEDTGrid(cs_grid, clamp):
    Obstacle cell (raw >= occupied_threshold) -> 0. Unknown (-1) treated as free -> distance to
    nearest obstacle. No obstacle in grid -> every cell reports clamp distance.
*/

namespace {
const float SQRT2 = std::sqrt(2.0f);
}

// ===== EDT class (mocked field, no OpenCV) =====

class EDTFixture : public ::testing::Test {
protected:
    GridMetaData meta3{3, 3, 1.0, Eigen::Vector2f{0, 0}};
    std::vector<float> centerField; // single obstacle at center (col=1,row=1), res=1

    void SetUp() override {
        centerField = {SQRT2, 1, SQRT2,
                       1,     0, 1,
                       SQRT2, 1, SQRT2};
    }

    EDT makeCenterEDT(double max_dist){ return EDT(meta3, centerField, max_dist); }
};

TEST_F(EDTFixture, distanceAtReadsCenterObstacleField){
    EDT edt = makeCenterEDT(1e6);
    EXPECT_FLOAT_EQ(edt.distanceAt(1, 1), 0.0f);
    EXPECT_FLOAT_EQ(edt.distanceAt(1, 0), 1.0f);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 1), 1.0f);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), SQRT2);
    EXPECT_FLOAT_EQ(edt.distanceAt(2, 2), SQRT2);
}

// Row-major indexing on a non-square grid (catches a transposed index).
TEST_F(EDTFixture, distanceAtRowMajorNonSquare){
    std::vector<float> field(12);
    for (int k = 0; k < 12; ++k) field[k] = static_cast<float>(k);
    EDT edt(GridMetaData(4, 3, 1.0, Eigen::Vector2f{0, 0}), field, 1e6); // cols=4, rows=3
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(edt.distanceAt(3, 0), 3.0f);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 1), 4.0f);  // row 1 -> 1*4 + 0
    EXPECT_FLOAT_EQ(edt.distanceAt(3, 2), 11.0f);
}

TEST_F(EDTFixture, distanceAtClampsToMax){
    EDT edt = makeCenterEDT(1.0); // clamp at 1.0
    EXPECT_FLOAT_EQ(edt.distanceAt(1, 1), 0.0f);  // 0 unaffected
    EXPECT_FLOAT_EQ(edt.distanceAt(1, 0), 1.0f);  // exactly at max
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), 1.0f);  // sqrt(2) clamped down to 1.0
}

TEST_F(EDTFixture, getDataIsRawUnclamped){
    EDT edt = makeCenterEDT(1.0);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), 1.0f); // distanceAt clamps...
    EXPECT_FLOAT_EQ(edt.getData()[0], SQRT2);    // ...but stored field is the true distance
}

TEST_F(EDTFixture, setMaxDistChangesClampAtReadTime){
    EDT edt = makeCenterEDT(1.0);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), 1.0f);   // clamped
    edt.setMaxDist(2.0);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), SQRT2);  // sqrt(2) < 2, now reported in full
    edt.setMaxDist(0.5);
    EXPECT_FLOAT_EQ(edt.distanceAt(0, 0), 0.5f);   // clamped harder
}

// ===== buildEDTGrid (factory, uses OpenCV) =====

class BuildEDTFixture : public ::testing::Test {
protected:
    OccupancyGrid gridFrom(std::vector<int8_t> data, int cols = 3, int rows = 3, double res = 1.0){
        return OccupancyGrid(GridMetaData(cols, rows, res, Eigen::Vector2f{0, 0}), std::move(data));
    }
};

TEST_F(BuildEDTFixture, singleObstacleMatchesExactField){
    EDT edt = buildEDTGrid(gridFrom({0, 0, 0,
                                     0, 100, 0,
                                     0, 0, 0}), 1e6);
    EXPECT_NEAR(edt.distanceAt(1, 1), 0.0f, 1e-4);
    EXPECT_NEAR(edt.distanceAt(1, 0), 1.0f, 1e-4);
    EXPECT_NEAR(edt.distanceAt(0, 0), SQRT2, 1e-4);
}

TEST_F(BuildEDTFixture, obstacleCellsAreZero){
    EDT edt = buildEDTGrid(gridFrom({100, 0, 0,
                                     0, 0, 0,
                                     0, 0, 100}), 1e6);
    EXPECT_NEAR(edt.distanceAt(0, 0), 0.0f, 1e-4);
    EXPECT_NEAR(edt.distanceAt(2, 2), 0.0f, 1e-4);
}

TEST_F(BuildEDTFixture, unknownCellsTreatedAsFree){
    // Obstacle at (0,0); cell (2,2) is unknown (-1). It must get its true distance, not 0.
    EDT edt = buildEDTGrid(gridFrom({100, 0, 0,
                                     0, 0, 0,
                                     0, 0, -1}), 1e6);
    EXPECT_NEAR(edt.distanceAt(0, 0), 0.0f, 1e-4);
    EXPECT_NEAR(edt.distanceAt(2, 2), 2.0f * SQRT2, 1e-4); // dist (2,2)->(0,0) = sqrt(8)
}

TEST_F(BuildEDTFixture, allObstaclesAllZero){
    EDT edt = buildEDTGrid(gridFrom(std::vector<int8_t>(4, 100), 2, 2), 1e6);
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c)
            EXPECT_NEAR(edt.distanceAt(c, r), 0.0f, 1e-4);
}

TEST_F(BuildEDTFixture, noObstacleFillsClampDistance){
    EDT free_edt = buildEDTGrid(gridFrom(std::vector<int8_t>(9, 0)), 5.0); // all free
    OccupancyGrid unknown_grid(GridMetaData(3, 3, 1.0, Eigen::Vector2f{0, 0}), std::nullopt); // all -1
    EDT unknown_edt = buildEDTGrid(unknown_grid, 5.0);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c){
            EXPECT_FLOAT_EQ(free_edt.distanceAt(c, r), 5.0f);
            EXPECT_FLOAT_EQ(unknown_edt.distanceAt(c, r), 5.0f);
        }
}

TEST_F(BuildEDTFixture, distancesScaleWithResolution){
    EDT edt = buildEDTGrid(gridFrom({0, 0, 0,
                                     0, 100, 0,
                                     0, 0, 0}, 3, 3, 0.1), 1e6); // resolution 0.1 m/cell
    EXPECT_NEAR(edt.distanceAt(1, 0), 0.1f, 1e-4);
    EXPECT_NEAR(edt.distanceAt(0, 0), 0.1f * SQRT2, 1e-4);
}

// ===== validation =====

static_assert(std::is_same_v<decltype(std::declval<const EDT&>().getMaxDist()), double>);

TEST_F(EDTFixture, ctorRejectsFieldSizeMismatch){
    std::vector<float> small(8, 1.0f), big(10, 1.0f);   // meta3 is 3 x 3 = 9 cells
    EXPECT_THROW(EDT(meta3, small, 1.0), std::invalid_argument);
    EXPECT_THROW(EDT(meta3, big, 1.0), std::invalid_argument);
}

TEST_F(EDTFixture, ctorRejectsNonPositiveOrNaNClamp){
    EXPECT_THROW(EDT(meta3, centerField, 0.0), std::invalid_argument);
    EXPECT_THROW(EDT(meta3, centerField, -1.0), std::invalid_argument);
    EXPECT_THROW(EDT(meta3, centerField, std::nan("")), std::invalid_argument);
    EXPECT_NO_THROW(EDT(meta3, centerField, std::numeric_limits<double>::infinity()));
}

TEST_F(EDTFixture, setMaxDistRejectsInvalidAndKeepsValue){
    EDT edt = makeCenterEDT(2.5);
    EXPECT_THROW(edt.setMaxDist(0.0), std::invalid_argument);
    EXPECT_THROW(edt.setMaxDist(-1.0), std::invalid_argument);
    EXPECT_THROW(edt.setMaxDist(std::nan("")), std::invalid_argument);
    EXPECT_EQ(edt.getMaxDist(), 2.5);
    edt.setMaxDist(4.0);
    EXPECT_EQ(edt.getMaxDist(), 4.0);
}

TEST_F(EDTFixture, maxDistKeepsDoublePrecision){
    EDT e(meta3, centerField, 0.1);
    EXPECT_EQ(e.getMaxDist(), 0.1);   // a float round trip would give 0.100000001490116...
}

TEST(EDTBuild, buildEDTGridRejectsInvalidClamp){
    OccupancyGrid grid(GridMetaData(3, 3, 1.0, Eigen::Vector2f{0, 0}), std::nullopt);
    EXPECT_THROW(buildEDTGrid(grid, 0.0), std::invalid_argument);
    EXPECT_THROW(buildEDTGrid(grid, -1.0), std::invalid_argument);
    EXPECT_THROW(buildEDTGrid(grid, std::nan("")), std::invalid_argument);
}
