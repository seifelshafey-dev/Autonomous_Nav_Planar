#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/motion/motion_context.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/grid_coords.hpp"
#include "core/map/grid.hpp"
#include "core/map/rasterize.hpp"
#include "core/motion/world_robot_geometry.hpp"

/*
MotionContext tests. Collaborators (propagate, transformGeometry, rasterizePolygon, worldToGrid) are independently unit-tested and trusted here; assertions are scoped to what MotionContext
itself owns:
  - sampleTrajectory: the sample COUNT (n_intervals+1, computed before any propagate call, so it is
    propagate-independent) and the endpoint contract (first == start, last == propagate(start, u)).
  - computeExactCover: the matched-list / sorted-unique / sample-cover invariants, zero-motion, a
    numeric example and origin threading.
We do NOT re-assert collaborator correctness. r_in = 0.20, r_out = sqrt(0.13) for the rectangle below,
so the translation step bound is r_in/2 = 0.10 and the rotation step bound is r_in/(2*r_out) ~ 0.27735.
*/

namespace {
std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}
DiffDriveParams kinParams(){
    DiffDriveParams k;
    k.rmin = 2.0; k.vmax = 1.0; k.atomic_time = 1.0; k.n_headings = 8;
    return k;
}
}

class MotionContextTest : public ::testing::Test {
protected:
    RobotParams    body  = buildRobotParams(ccwRect());
    DiffDriveParams kin  = kinParams();
    // Body-frame model: motion has resolution 0.1 and origin (0, 0).
    KinematicModel model = buildKinematicModel(body, kin, 0.1);

    // A MotionContext over the same kinematics but with a chosen resolution/origin. propagate does not
    // depend on the grid, so reusing the model's state-space is sound.
    MotionContext makeContext(double resolution, float ox, float oy) const {
        return MotionContext(body, model.motion->spacePtr(),
                             GridMetaData{1, 1, resolution, Eigen::Vector2f(ox, oy)});
    }
};

// ---- sampleTrajectory: sample count per branch (independent of the kinematics) ----

TEST_F(MotionContextTest, translationOnlySampleCount){
    // arc = 1.0 * 0.35 = 0.35; n = ceil(0.35 / 0.10) = 4 -> 5 poses.
    auto poses = model.motion->sampleTrajectory(SE2State{0, 0, 0}, Control{1.0, 0.0, 0.35});
    EXPECT_EQ(poses.size(), 5u);
}

TEST_F(MotionContextTest, rotationOnlySampleCount){
    // rot = 1.0 * 1.0 = 1.0; n = ceil(1.0 / 0.27735) = ceil(3.6056) = 4 -> 5 poses.
    auto poses = model.motion->sampleTrajectory(SE2State{0, 0, 0}, Control{0.0, 1.0, 1.0});
    EXPECT_EQ(poses.size(), 5u);
}

TEST_F(MotionContextTest, combinedSampleCountTakesTheTighterBound){
    // arc = 0.45 -> ceil(4.5) = 5; rot = 0.9 -> ceil(3.245) = 4; max(2,5,4) = 5 -> 6 poses.
    auto poses = model.motion->sampleTrajectory(SE2State{0, 0, 0}, Control{1.0, 2.0, 0.45});
    EXPECT_EQ(poses.size(), 6u);
}

TEST_F(MotionContextTest, zeroControlYieldsTwoCoincidentSamples){
    // omega == 0, arc == 0 -> n = max(1, 0) = 1 -> 2 poses, both equal to start. No division by zero.
    SE2State start{0.4, -0.2, 0.3};
    auto poses = model.motion->sampleTrajectory(start, Control{0.0, 0.0, 1.0});
    ASSERT_EQ(poses.size(), 2u);
    for (const auto & p : poses){
        EXPECT_NEAR(p.x, start.x, 1e-12);
        EXPECT_NEAR(p.y, start.y, 1e-12);
        EXPECT_NEAR(p.theta, start.theta, 1e-12);
    }
}

// ---- sampleTrajectory: endpoints span the full control ----

TEST_F(MotionContextTest, endpointsAreStartAndFullPropagation){
    SE2State start{0.5, -0.3, 0.2};
    Control u{1.0, 0.5, 1.0};
    auto poses = model.motion->sampleTrajectory(start, u);
    ASSERT_GE(poses.size(), 2u);

    EXPECT_DOUBLE_EQ(poses.front().x, start.x);
    EXPECT_DOUBLE_EQ(poses.front().y, start.y);
    EXPECT_DOUBLE_EQ(poses.front().theta, start.theta);

    SE2State full = model.motion->space().propagate(start, u);
    EXPECT_NEAR(poses.back().x, full.x, 1e-9);
    EXPECT_NEAR(poses.back().y, full.y, 1e-9);
    EXPECT_NEAR(poses.back().theta, full.theta, 1e-9);
}

// ---- computeExactCover: invariants MotionContext owns ----

TEST_F(MotionContextTest, sweptResultIsConsistentWithItsOwnPieces){
    SE2State start{0.5, -0.3, 0.2};
    Control u{1.0, 0.5, 0.8};

    MotionContext::SweptResult r = model.motion->computeExactCover(start, u);
    std::vector<SE2State> poses = model.motion->sampleTrajectory(start, u);

    // samples are exactly sampleTrajectory's output.
    ASSERT_EQ(r.samples.size(), poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i){
        EXPECT_DOUBLE_EQ(r.samples[i].x, poses[i].x);
        EXPECT_DOUBLE_EQ(r.samples[i].y, poses[i].y);
        EXPECT_DOUBLE_EQ(r.samples[i].theta, poses[i].theta);
    }

    // One center cell per sample, matching worldToGrid.
    ASSERT_EQ(r.sample_cells.size(), r.samples.size());
    for (std::size_t i = 0; i < r.samples.size(); ++i)
        EXPECT_EQ(r.sample_cells[i], model.motion->gridMeta().worldToGrid(r.samples[i].x, r.samples[i].y));

    // Cells are non-empty, sorted ascending and duplicate-free.
    EXPECT_FALSE(r.cells.empty());
    EXPECT_TRUE(std::is_sorted(r.cells.begin(), r.cells.end()));
    EXPECT_TRUE(std::adjacent_find(r.cells.begin(), r.cells.end()) == r.cells.end());
    std::set<std::pair<int, int>> cellset;
    for (const auto & c : r.cells) cellset.insert({c.i, c.j});
    EXPECT_EQ(cellset.size(), r.cells.size());

    // Each sample's center cell lies within the swept footprint.
    for (const auto & sc : r.sample_cells)
        EXPECT_EQ(cellset.count({sc.i, sc.j}), 1u);
}

// Footprint raster of `body` at `p` with `margin`, origin (0, 0), via the public rasterizer.
static std::set<std::pair<int, int>> rasterAt(const RobotParams & body, const SE2State & p, double h, double margin){
    const auto geo = transformGeometry(body, p);
    std::set<std::pair<int, int>> out;
    for (const auto & c : rasterizePolygon(geo.half_planes, geo.aabb, h, margin)) out.insert({c.i, c.j});
    return out;
}

static std::set<std::pair<int, int>> asSet(const std::vector<GridCoords> & cells){
    std::set<std::pair<int, int>> out;
    for (const auto & c : cells) out.insert({c.i, c.j});
    return out;
}

TEST_F(MotionContextTest, coversTheZeroMarginRasterAtEverySample){
    SE2State start{0.53, -0.27, 0.9};
    for (const Control & u : {Control{1.0, 0.5, 0.8}, Control{-1.0, 0.0, 0.5}, Control{0.0, 1.5, 1.0}}){
        auto r = model.motion->computeExactCover(start, u);
        const auto have = asSet(r.cells);
        for (const auto & p : r.samples)
            for (const auto & c : rasterAt(body, p, 0.1, 0.0))
                EXPECT_EQ(have.count(c), 1u) << "cell (" << c.first << "," << c.second << ")";
    }
}

TEST_F(MotionContextTest, zeroMotionControlYieldsTheFootprintAtStart){
    SE2State start{0.47, 0.31, 0.6};
    auto r = model.motion->computeExactCover(start, Control{0.0, 0.0, 0.8});
    EXPECT_EQ(asSet(r.cells), rasterAt(body, start, 0.1, 0.0));
}

TEST(MotionContextExactCover, reverseStraightExample){
    // 0.2 m square, h = 0.1, origin (0, 0). reach = 0.3 m -> n = 3, m_s = 0.3 / 6 = 0.05; the poses
    // t_j = dt*j/n are x = 0.55, 0.45, 0.35, 0.25 (y = 0.55, theta = 0).
    RobotParams sq = buildRobotParams({Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
    DiffDriveParams k; k.rmin = 0.5; k.vmax = 1.0; k.atomic_time = 0.3; k.n_headings = 8;
    KinematicModel m = buildKinematicModel(sq, k, 0.1);
    const SE2State start{0.55, 0.55, 0.0};
    const Control u{-1.0, 0.0, 0.3};

    std::set<std::pair<int, int>> expected;
    for (double x : {0.55, 0.45, 0.35, 0.25})
        for (const auto & c : rasterAt(sq, SE2State{x, 0.55, 0.0}, 0.1, 0.05)) expected.insert(c);
    for (const auto & p : m.motion->sampleTrajectory(start, u))
        for (const auto & c : rasterAt(sq, p, 0.1, 0.0)) expected.insert(c);

    EXPECT_EQ(asSet(m.motion->computeExactCover(start, u).cells), expected);
}

TEST_F(MotionContextTest, exactCoverRejectsNegativeOrNonFiniteDuration){
    const SE2State start{0.5, -0.3, 0.2};
    EXPECT_THROW(model.motion->computeExactCover(start, Control{1.0, 0.5, -0.1}), std::invalid_argument);
    EXPECT_THROW(model.motion->computeExactCover(start, Control{1.0, 0.5, std::nan("")}), std::invalid_argument);
}

// ---- computeExactCover: origin is threaded into both the footprint and the sample cells ----

TEST_F(MotionContextTest, originShiftsEveryCellConsistently){
    // Resolution 1.0 with integer origins -> an exact integer cell shift, no boundary FP ambiguity.
    // Origin (2, -3) maps a world point to cell (floor(x)-2, floor(y)+3), i.e. a {-2, +3} shift.
    MotionContext a = makeContext(1.0, 0.0f,  0.0f);
    MotionContext b = makeContext(1.0, 2.0f, -3.0f);
    const GridCoords shift{-2, 3};

    SE2State start{0.3, 0.4, 0.5};
    Control u{1.0, 0.5, 1.0};
    auto ra = a.computeExactCover(start, u);
    auto rb = b.computeExactCover(start, u);

    ASSERT_EQ(ra.samples.size(), rb.samples.size());
    ASSERT_EQ(ra.sample_cells.size(), rb.sample_cells.size());
    for (std::size_t i = 0; i < ra.sample_cells.size(); ++i)
        EXPECT_EQ(rb.sample_cells[i], ra.sample_cells[i] + shift);

    ASSERT_EQ(ra.cells.size(), rb.cells.size());
    std::set<std::pair<int, int>> shifted;
    for (const auto & c : ra.cells) shifted.insert({c.i + shift.i, c.j + shift.j});
    std::set<std::pair<int, int>> setB;
    for (const auto & c : rb.cells) setB.insert({c.i, c.j});
    EXPECT_EQ(shifted, setB);
}

// ---- visitExactCover: visits exactly the computeExactCover cell set ----

namespace {
std::set<std::pair<int, int>> visitedSet(const MotionContext & m, const SE2State & s, const Control & u, bool & ret){
    std::set<std::pair<int, int>> out;
    ret = m.visitExactCover(s, u, [&](GridCoords c){ out.insert({c.i, c.j}); return true; });
    return out;
}
}

TEST_F(MotionContextTest, visitExactCoverSetEqualsComputeExactCover){
    const SE2State start{0.53, -0.27, 0.9};
    for (const Control & u : {Control{1.0, 0.0, 0.8}, Control{0.0, 1.5, 1.0}, Control{1.0, 0.5, 0.8}}){
        bool ret = false;
        const auto got = visitedSet(*model.motion, start, u, ret);
        EXPECT_TRUE(ret);
        EXPECT_EQ(got, asSet(model.motion->computeExactCover(start, u).cells));
    }
}

TEST_F(MotionContextTest, visitExactCoverStopsEarly){
    const SE2State start{0.53, -0.27, 0.9};
    for (int k : {1, 5}){
        int calls = 0;
        const bool ret = model.motion->visitExactCover(start, Control{1.0, 0.5, 0.8},
                                                       [&](GridCoords){ return ++calls < k; });
        EXPECT_FALSE(ret);
        EXPECT_EQ(calls, k);
    }
}

TEST_F(MotionContextTest, visitExactCoverZeroMotion){
    const SE2State start{0.47, 0.31, 0.6};
    for (const Control & u : {Control{1.0, 0.5, 0.0}, Control{0.0, 0.0, 0.8}}){
        bool ret = false;
        const auto got = visitedSet(*model.motion, start, u, ret);
        EXPECT_TRUE(ret);
        EXPECT_EQ(got, asSet(model.motion->computeExactCover(start, u).cells));
        EXPECT_EQ(got, rasterAt(body, start, 0.1, 0.0));
    }
}

TEST_F(MotionContextTest, visitExactCoverRejectsBadDt){
    const SE2State start{0.5, -0.3, 0.2};
    for (double dt : {-0.1, std::nan("")}){
        int calls = 0;
        EXPECT_THROW(model.motion->visitExactCover(start, Control{1.0, 0.5, dt}, [&](GridCoords){ ++calls; return true; }),
                     std::invalid_argument);
        EXPECT_EQ(calls, 0);
    }
}
