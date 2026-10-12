#include <gtest/gtest.h>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/map/grid.hpp"

/*
OccupancyGrid contract:
    Default (no data): rows*cols cells initialized to -1 (UNKNOWN).
    Data invariant (CHECK): supplied data size must equal rows*cols (ctor and setData throw otherwise).
    at(gc): raw cell value; OOB -> -1 (UNKNOWN). Row-major index = gc.j*cols + gc.i.
    findCellStatus(gc): -1 -> UNKNOWN; else raw >= occupied_threshold -> OCCUPIED, otherwise FREE.
    rawAccess(i): cells_[i] if 0 <= i < size, else -1.
    Moved-from grid: 0 x 0, no data, at() == -1 / UNKNOWN everywhere, resolution and origin kept.
*/

namespace {
GridMetaData meta(int cols, int rows){
    return GridMetaData(cols, rows, 1.0, Eigen::Vector2f{0, 0});
}
}

// ---- default fill ----

TEST(OccupancyGrid, defaultFillIsUnknown){
    OccupancyGrid g(meta(3, 3), std::nullopt);
    EXPECT_EQ(g.getData().size(), 9u);
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i){
            EXPECT_EQ(g.at({i, j}), -1);
            EXPECT_EQ(g.findCellStatus({i, j}), UNKNOWN);
        }
}

// ---- at(): row-major indexing on a NON-SQUARE grid (catches transposition) ----

TEST(OccupancyGrid, atUsesRowMajorIndex){
    // cols=4, rows=3 -> 12 cells. data[idx] = idx, so at({i,j}) must equal j*cols + i.
    std::vector<int8_t> data(12);
    for (int k = 0; k < 12; ++k) data[k] = static_cast<int8_t>(k);
    OccupancyGrid g(meta(4, 3), data);

    EXPECT_EQ(g.at({0, 0}), 0);
    EXPECT_EQ(g.at({3, 0}), 3);  // last col of row 0
    EXPECT_EQ(g.at({0, 1}), 4);  // 1*4 + 0  (transposition would give 0*3+1 = 1)
    EXPECT_EQ(g.at({3, 2}), 11); // 2*4 + 3
}

// ---- at(): out of bounds -> unknown ----

TEST(OccupancyGrid, atOutOfBoundsReturnsMinusOne){
    std::vector<int8_t> data(12, 7);
    OccupancyGrid g(meta(4, 3), data);
    EXPECT_EQ(g.at({4, 0}), -1);  // i == cols
    EXPECT_EQ(g.at({0, 3}), -1);  // j == rows
    EXPECT_EQ(g.at({-1, 0}), -1);
    EXPECT_EQ(g.at({0, -1}), -1);
    // in-bounds corners still read the data
    EXPECT_EQ(g.at({0, 0}), 7);
    EXPECT_EQ(g.at({3, 2}), 7);
}

// ---- findCellStatus(): threshold boundary ----

TEST(OccupancyGrid, findCellStatusThresholdBoundary){
    std::vector<int8_t> data = {-1, 0, 49, 50, 100}; // 5 cells in a 5x1 grid
    OccupancyGrid g(meta(5, 1), data);
    EXPECT_EQ(g.findCellStatus({0, 0}), UNKNOWN);  // -1
    EXPECT_EQ(g.findCellStatus({1, 0}), FREE);     // 0
    EXPECT_EQ(g.findCellStatus({2, 0}), FREE);     // 49
    EXPECT_EQ(g.findCellStatus({3, 0}), OCCUPIED); // 50 == threshold
    EXPECT_EQ(g.findCellStatus({4, 0}), OCCUPIED); // 100
}

TEST(OccupancyGrid, findCellStatusHonorsCustomThreshold){
    GridMetaData m = meta(2, 1);
    m.occupied_threshold = 80;
    std::vector<int8_t> data = {50, 80};
    OccupancyGrid g(m, data);
    EXPECT_EQ(g.findCellStatus({0, 0}), FREE);     // 50 < 80
    EXPECT_EQ(g.findCellStatus({1, 0}), OCCUPIED); // 80 == threshold
}

TEST(OccupancyGrid, findCellStatusOutOfBoundsIsUnknown){
    std::vector<int8_t> data(4, 100); // all occupied in-bounds
    OccupancyGrid g(meta(2, 2), data);
    EXPECT_EQ(g.findCellStatus({5, 5}), UNKNOWN); // OOB -> at()==-1 -> UNKNOWN
}

// ---- rawAccess() ----

TEST(OccupancyGrid, rawAccessBounds){
    std::vector<int8_t> data = {10, 20, 30, 40};
    OccupancyGrid g(meta(2, 2), data);
    EXPECT_EQ(g.rawAccess(-1), -1);
    EXPECT_EQ(g.rawAccess(0), 10);
    EXPECT_EQ(g.rawAccess(3), 40);   // size - 1
    EXPECT_EQ(g.rawAccess(4), -1);   // size
}

// ---- data-size invariant (CHECK -> throw) ----

TEST(OccupancyGrid, constructorRejectsWrongDataSize){
    std::vector<int8_t> wrong(5, 0); // 3x3 expects 9
    EXPECT_THROW(OccupancyGrid(meta(3, 3), wrong), std::invalid_argument);
}

TEST(OccupancyGrid, setDataRejectsWrongDataSize){
    OccupancyGrid g(meta(3, 3), std::nullopt);
    std::vector<int8_t> bad(3, 1);
    EXPECT_THROW(g.setData(bad), std::invalid_argument);
}

TEST(OccupancyGrid, setDataReplacesCells){
    OccupancyGrid g(meta(3, 3), std::nullopt);
    std::vector<int8_t> good(9, 42);
    g.setData(good);
    EXPECT_EQ(g.at({0, 0}), 42);
    EXPECT_EQ(g.at({2, 2}), 42);
}

// ---- getters ----


// ---- move semantics ----

static_assert(std::is_nothrow_move_constructible_v<OccupancyGrid> && std::is_nothrow_move_assignable_v<OccupancyGrid>);

namespace {
OccupancyGrid moveFixture(){
    return OccupancyGrid(GridMetaData(3, 2, 0.5, Eigen::Vector2f{1.0f, -2.0f}),
                         std::vector<int8_t>{0, 100, -1, 50, 49, 0});
}

void expectMovedFrom(const OccupancyGrid & g){
    EXPECT_EQ(g.rows(), 0);
    EXPECT_EQ(g.cols(), 0);
    EXPECT_TRUE(g.getData().empty());
    EXPECT_EQ(g.resolution(), 0.5f);
    EXPECT_EQ(g.origin(), (Eigen::Vector2f{1.0f, -2.0f}));
    EXPECT_EQ(g.getMetadata().world_width, 0.0f);
    EXPECT_EQ(g.getMetadata().world_height, 0.0f);
    EXPECT_EQ(g.getData().size(), static_cast<std::size_t>(g.rows()) * g.cols());
    for (int j = -2; j <= 4; ++j)
        for (int i = -2; i <= 4; ++i){
            EXPECT_EQ(g.at({i, j}), -1);
            EXPECT_EQ(g.findCellStatus({i, j}), UNKNOWN);
        }
    for (int i : {-1, 0, 5, 6}) EXPECT_EQ(g.rawAccess(i), -1);
}

void expectFixtureContents(const OccupancyGrid & g){
    const std::vector<int8_t> expected{0, 100, -1, 50, 49, 0};
    EXPECT_EQ(g.rows(), 2);
    EXPECT_EQ(g.cols(), 3);
    EXPECT_EQ(g.getData(), expected);
    EXPECT_EQ(g.getData().size(), static_cast<std::size_t>(g.rows()) * g.cols());
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 3; ++i) EXPECT_EQ(g.at({i, j}), expected[j * 3 + i]);
}
}

TEST(OccupancyGrid, moveConstructTransfersAndEmptiesSource){
    OccupancyGrid src = moveFixture();
    OccupancyGrid dst(std::move(src));
    expectFixtureContents(dst);
    expectMovedFrom(src);  // NOLINT(bugprone-use-after-move)
}

TEST(OccupancyGrid, moveAssignTransfersAndEmptiesSource){
    OccupancyGrid src = moveFixture();
    OccupancyGrid dst(GridMetaData(1, 1, 1.0, Eigen::Vector2f{0, 0}), std::nullopt);
    dst = std::move(src);
    expectFixtureContents(dst);
    expectMovedFrom(src);  // NOLINT(bugprone-use-after-move)
}

TEST(OccupancyGrid, selfMoveAssignLeavesGridUnchanged){
    OccupancyGrid g = moveFixture();
    OccupancyGrid & ref = g;
    g = std::move(ref);
    expectFixtureContents(g);
}

TEST(OccupancyGrid, movedFromSetDataAcceptsOnlyEmpty){
    OccupancyGrid src = moveFixture();
    OccupancyGrid dst(std::move(src));
    std::vector<int8_t> empty;
    EXPECT_NO_THROW(src.setData(empty));  // NOLINT(bugprone-use-after-move)
    std::vector<int8_t> one(1, 0);
    EXPECT_THROW(src.setData(one), std::invalid_argument);
}

TEST(OccupancyGrid, movedFromGridIsReusableByAssignment){
    OccupancyGrid src = moveFixture();
    OccupancyGrid dst(std::move(src));
    src = moveFixture();  // NOLINT(bugprone-use-after-move)
    expectFixtureContents(src);
}

TEST(OccupancyGrid, rawAccessOnLiveGrid){
    OccupancyGrid g = moveFixture();
    EXPECT_EQ(g.rawAccess(-1), -1);
    EXPECT_EQ(g.rawAccess(6), -1);
    EXPECT_EQ(g.rawAccess(5), 0);
    EXPECT_EQ(g.rawAccess(1), 100);
}
