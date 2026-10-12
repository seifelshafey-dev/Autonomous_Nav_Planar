#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

#include "core/collision_checker/collision_checker.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/map/rasterize.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"
#include "core/types/grid_coords.hpp"

/*
CollisionChecker tests (real OccupancyGrid + EDT constructed with controlled
contents drive every branch). The checker owns: EDT fast-accept (>r_out) / fast-reject (<r_in) /
UNCERTAIN classification, the UNCERTAIN -> full per-cell scan fallback, the body->world cell
translation (checkTrajectory), and the reject_unknown policy of the pure scan (checkCells). Occupancy threshold is 50;
-1 / OOB read as UNKNOWN.

For the EDT-threshold tests we hand-build RobotParams with r_in=0.25, r_out=0.5 (both exactly
representable) so the strict > / < boundary cases are unambiguous. checkState needs a real footprint,
so it uses buildRobotParams on a rectangle (r_in=0.20, r_out=sqrt(0.13)).
*/

namespace {
const Eigen::Vector2f kZero = Eigen::Vector2f::Zero();

std::vector<Vec2> ccwRect(){
    return {{0.30, 0.20}, {-0.30, 0.20}, {-0.30, -0.20}, {0.30, -0.20}};
}

// RobotParams with only the bounding radii set (vertices/half-planes unused by the radius-based paths).
RobotParams discRobot(double r_in, double r_out){
    RobotParams p;
    p.r_in = r_in;
    p.r_out = r_out;
    return p;
}

std::shared_ptr<const OccupancyGrid> gridFrom(int cols, int rows, double res, std::vector<int8_t> data){
    return std::make_shared<const OccupancyGrid>(GridMetaData{cols, rows, res, kZero},
                                                 std::move(data));
}
std::shared_ptr<const OccupancyGrid> uniformGrid(int cols, int rows, double res, int8_t v){
    return gridFrom(cols, rows, res, std::vector<int8_t>(cols * rows, v));
}
std::shared_ptr<const EDT> edtFrom(int cols, int rows, double res, std::vector<float> df){
    return std::make_shared<const EDT>(GridMetaData{cols, rows, res, kZero},
                                       std::move(df), 5.0);
}
std::shared_ptr<const EDT> uniformEDT(int cols, int rows, double res, float d){
    return edtFrom(cols, rows, res, std::vector<float>(cols * rows, d));
}
}  // namespace

// Per-sample clearance vector for hand-built trajectories. The default r_out of discRobot(0.25, 0.5)
// reproduces the single-pose accept threshold, so the reject rule reduces to edt < r_in and the
// control-flow tests keep their pre-clearance verdicts.
static std::vector<double> clr(const std::vector<GridCoords> & samples, double c = 0.5){
    return std::vector<double>(samples.size(), c);
}

// ---- fastAcceptReject(cell): EDT threshold classification ----

// Single-pose thresholds with h the grid resolution: VALID iff edt > r_out + sqrt2*h, COLLISION iff
// edt + (sqrt2/2)*h < r_in. For discRobot(0.25, 0.5) at h = 0.1: VALID above 0.6414, COLLISION below 0.1793.

TEST(FastAcceptReject, classifiesByEdtAgainstRadii){
    auto grid = uniformGrid(4, 1, 0.1, 0);
    RobotParams body = discRobot(0.25, 0.5);

    // cell distances: far(0.75)=VALID, near(0.1)=COLLISION, mid(0.375)=UNCERTAIN, 0.5 (the old
    // accept boundary r_out) is now UNCERTAIN.
    CollisionChecker far(grid, edtFrom(4, 1, 0.1, {0.75f, 0.1f, 0.375f, 0.5f}), body);
    EXPECT_EQ(far.fastAcceptReject(GridCoords{0, 0}), Validity::VALID);
    EXPECT_EQ(far.fastAcceptReject(GridCoords{1, 0}), Validity::COLLISION);
    EXPECT_EQ(far.fastAcceptReject(GridCoords{2, 0}), Validity::UNCERTAIN);
    EXPECT_EQ(far.fastAcceptReject(GridCoords{3, 0}), Validity::UNCERTAIN);
}

TEST(FastAcceptReject, acceptBoundaryIsRoutPlusSqrt2Resolution){
    auto grid = uniformGrid(2, 1, 0.1, 0);
    CollisionChecker c(grid, edtFrom(2, 1, 0.1, {0.64f, 0.65f}), discRobot(0.25, 0.5));  // threshold 0.6414
    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, 0}), Validity::UNCERTAIN);
    EXPECT_EQ(c.fastAcceptReject(GridCoords{1, 0}), Validity::VALID);
}

TEST(FastAcceptReject, rejectBoundaryIsRinMinusHalfDiagonal){
    auto grid = uniformGrid(2, 1, 0.1, 0);
    CollisionChecker c(grid, edtFrom(2, 1, 0.1, {0.17f, 0.18f}), discRobot(0.25, 0.5));  // threshold 0.1793
    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, 0}), Validity::COLLISION);
    EXPECT_EQ(c.fastAcceptReject(GridCoords{1, 0}), Validity::UNCERTAIN);
}

TEST(FastAcceptReject, coarseGridNeverRejectsWhenHalfDiagonalExceedsRin){
    // h = 1.0: (sqrt2/2)*h = 0.707 > r_in, so even an adjacent obstacle (edt 0) cannot be rejected.
    CollisionChecker c(uniformGrid(1, 1, 1.0, 0), edtFrom(1, 1, 1.0, {0.0f}), discRobot(0.25, 0.5));
    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, 0}), Validity::UNCERTAIN);
}

TEST(FastAcceptReject, stateOverloadMapsThroughWorldToGrid){
    // res 0.1: state (0.25, 0.35) -> cell (2, 3). Only that cell is COLLISION-near; elsewhere VALID-far.
    std::vector<float> df(5 * 5, 5.0f);
    df[3 * 5 + 2] = 0.1f;  // cell (i=2, j=3)
    auto grid = uniformGrid(5, 5, 0.1, 0);
    CollisionChecker c(grid, edtFrom(5, 5, 0.1, df), discRobot(0.25, 0.5));

    EXPECT_EQ(c.fastAcceptReject(SE2State{0.25, 0.35, 0.0}), Validity::COLLISION);
    EXPECT_EQ(c.fastAcceptReject(SE2State{0.05, 0.05, 0.0}), Validity::VALID);
}

// ---- fastAcceptReject: out-of-grid cells (frontier expansion) must not read the EDT OOB ----

TEST(FastAcceptReject, outOfGridCellIsUncertain){
    // Every in-grid cell would fast-accept (EDT 5.0 > r_out). OOB cells must NOT read distance_field;
    // they return UNCERTAIN so the caller falls back to the OOB-safe per-cell scan.
    auto grid = uniformGrid(4, 4, 1.0, 0);
    CollisionChecker c(grid, uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));

    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, 0}), Validity::VALID);       // in-grid sanity
    EXPECT_EQ(c.fastAcceptReject(GridCoords{-1, 0}), Validity::UNCERTAIN);  // col < 0
    EXPECT_EQ(c.fastAcceptReject(GridCoords{4, 0}), Validity::UNCERTAIN);   // col >= cols
    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, -1}), Validity::UNCERTAIN);  // row < 0
    EXPECT_EQ(c.fastAcceptReject(GridCoords{0, 4}), Validity::UNCERTAIN);   // row >= rows
    EXPECT_EQ(c.fastAcceptReject(GridCoords{99, 99}), Validity::UNCERTAIN); // far OOB
}

TEST(FastAcceptReject, outOfGridStateIsUncertain){
    auto grid = uniformGrid(4, 4, 1.0, 0);
    CollisionChecker c(grid, uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    // worldToGrid maps these poses outside [0,4) -> UNCERTAIN, no OOB EDT read.
    EXPECT_EQ(c.fastAcceptReject(SE2State{-1.0, 2.0, 0.0}), Validity::UNCERTAIN);
    EXPECT_EQ(c.fastAcceptReject(SE2State{10.0, 2.0, 0.0}), Validity::UNCERTAIN);
}

TEST(CheckTrajectory, oobSampleDoesNotMaskInGridOccupiedSweptCell){
    // The safety crux of returning UNCERTAIN (not VALID) for OOB: a fast-accept off the OOB sample
    // would skip the swept scan and miss the in-grid OCCUPIED cell on the same trajectory.
    std::vector<int8_t> occ(4 * 4, 0);
    occ[1 * 4 + 1] = 100;  // cell (i=1, j=1) OCCUPIED
    auto grid = gridFrom(4, 4, 1.0, std::move(occ));
    CollisionChecker c(grid, uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));

    std::vector<GridCoords> samples = {GridCoords{99, 99}};                  // OOB -> UNCERTAIN
    std::vector<GridCoords> swept   = {GridCoords{0, 0}, GridCoords{1, 1}};  // (1,1) occupied
    EXPECT_EQ(c.checkTrajectory(swept, samples, clr(samples), GridCoords{0, 0}), TrajectoryVerdict::SCAN_BLOCKED);
}

TEST(CheckTrajectory, oobSampleWithFreeSweptScansAndAccepts){
    auto grid = uniformGrid(4, 4, 1.0, 0);  // all free
    CollisionChecker c(grid, uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    std::vector<GridCoords> samples = {GridCoords{99, 99}};                  // OOB -> UNCERTAIN
    std::vector<GridCoords> swept   = {GridCoords{0, 0}, GridCoords{1, 1}};  // in-grid, free
    EXPECT_EQ(c.checkTrajectory(swept, samples, clr(samples), GridCoords{0, 0}), TrajectoryVerdict::SCAN_CLEAR);
}

// ---- checkState: short-circuits + UNCERTAIN footprint scan ----

class CheckStateTest : public ::testing::Test {
protected:
    // 20x20 @ 0.1m, origin 0. Robot at (1,1) -> center cell (10,10); footprint stays in bounds.
    RobotParams body = buildRobotParams(ccwRect());   // r_in=0.20, r_out=sqrt(0.13)~0.3606
    SE2State state{1.0, 1.0, 0.0};
    std::vector<int8_t> occupiedAtCenter(){
        std::vector<int8_t> d(20 * 20, 0);
        d[10 * 20 + 10] = 100;  // OCCUPIED at the robot center cell
        return d;
    }
};

TEST_F(CheckStateTest, validFastAcceptSkipsScan){
    // EDT far -> VALID -> returns true WITHOUT scanning, even though the center cell is OCCUPIED.
    CollisionChecker c(gridFrom(20, 20, 0.1, occupiedAtCenter()), uniformEDT(20, 20, 0.1, 1.0f), body);
    EXPECT_TRUE(c.checkState(state));
}

TEST_F(CheckStateTest, collisionFastRejectSkipsScan){
    // EDT near -> COLLISION -> false even though the grid is entirely free.
    CollisionChecker c(uniformGrid(20, 20, 0.1, 0), uniformEDT(20, 20, 0.1, 0.05f), body);
    EXPECT_FALSE(c.checkState(state));
}

TEST_F(CheckStateTest, uncertainScansFootprintClear){
    CollisionChecker c(uniformGrid(20, 20, 0.1, 0), uniformEDT(20, 20, 0.1, 0.3f), body);
    EXPECT_TRUE(c.checkState(state));
}

TEST_F(CheckStateTest, uncertainScansFootprintObstacle){
    CollisionChecker c(gridFrom(20, 20, 0.1, occupiedAtCenter()), uniformEDT(20, 20, 0.1, 0.3f), body);
    EXPECT_FALSE(c.checkState(state));
}

// ---- checkTrajectory: translation, sample fast-path, swept fallback ----

TEST(CheckTrajectory, allSamplesValidSkipsSweptScan){
    // Samples fast-accept -> true even though a swept cell is OCCUPIED (scan is skipped).
    std::vector<int8_t> d(10 * 10, 0);
    d[2 * 10 + 2] = 100;  // OCCUPIED at swept cell (2,2)
    CollisionChecker c(gridFrom(10, 10, 1.0, d), uniformEDT(10, 10, 1.0, 1.0f), discRobot(0.25, 0.5));

    std::vector<GridCoords> swept = {{2, 2}};
    std::vector<GridCoords> samples = {{1, 1}, {3, 3}};
    EXPECT_EQ(c.checkTrajectory(swept, samples, clr(samples), GridCoords{0, 0}), TrajectoryVerdict::FAST_ACCEPT);
}

TEST(CheckTrajectory, collisionSampleRejectsImmediately){
    std::vector<float> df(10 * 10, 1.0f);
    df[5 * 10 + 5] = 0.05f;  // near obstacle at sample cell (5,5)
    CollisionChecker c(uniformGrid(10, 10, 1.0, 0), edtFrom(10, 10, 1.0, df), discRobot(0.25, 0.5));

    EXPECT_EQ(c.checkTrajectory({{5, 5}}, {{5, 5}}, {0.5}, GridCoords{0, 0}), TrajectoryVerdict::FAST_REJECT);
}

TEST(CheckTrajectory, uncertainFallsBackToSweptScan){
    auto edt = uniformEDT(10, 10, 1.0, 0.375f);  // all UNCERTAIN -> always scan swept
    std::vector<int8_t> occ(10 * 10, 0);
    occ[4 * 10 + 4] = 100;

    CollisionChecker clear(uniformGrid(10, 10, 1.0, 0), edt, discRobot(0.25, 0.5));
    EXPECT_EQ(clear.checkTrajectory({{4, 4}}, {{4, 4}}, {0.5}, GridCoords{0, 0}), TrajectoryVerdict::SCAN_CLEAR);

    CollisionChecker blocked(gridFrom(10, 10, 1.0, occ), edt, discRobot(0.25, 0.5));
    EXPECT_EQ(blocked.checkTrajectory({{4, 4}}, {{4, 4}}, {0.5}, GridCoords{0, 0}), TrajectoryVerdict::SCAN_BLOCKED);
}

TEST(CheckTrajectory, robotCellTranslatesBodyCells){
    // Near-obstacle only at world (5,5). A body cell (0,0) reaches it only with robot_cell (5,5).
    std::vector<float> df(10 * 10, 1.0f);
    df[5 * 10 + 5] = 0.05f;
    CollisionChecker c(uniformGrid(10, 10, 1.0, 0), edtFrom(10, 10, 1.0, df), discRobot(0.25, 0.5));

    EXPECT_EQ(c.checkTrajectory({{0, 0}}, {{0, 0}}, {0.5}, GridCoords{5, 5}), TrajectoryVerdict::FAST_REJECT);  // translated onto obstacle
    EXPECT_EQ(c.checkTrajectory({{0, 0}}, {{0, 0}}, {0.5}, GridCoords{0, 0}), TrajectoryVerdict::FAST_ACCEPT);   // reads clear (0,0)
}

// ---- checkCells: pure scan, reject_unknown policy ----

namespace {
// 10x10 @1.0 grid with (2,2) set to `v` and everything else free.
CollisionChecker oneCellChecker(int8_t v){
    std::vector<int8_t> d(10 * 10, 0);
    d[2 * 10 + 2] = v;
    return CollisionChecker(gridFrom(10, 10, 1.0, d), uniformEDT(10, 10, 1.0, 1.0f), discRobot(0.25, 0.5));
}
}  // namespace

TEST(CheckCells, occupiedCellBlocks){
    auto c = oneCellChecker(100);
    EXPECT_FALSE(c.checkCells({{2, 2}}, false));
    EXPECT_FALSE(c.checkCells({{2, 2}}, true));
    EXPECT_TRUE(c.checkCells({{3, 3}}, false));
}

TEST(CheckCells, unknownPassesUnlessRejectUnknown){
    auto c = oneCellChecker(-1);
    EXPECT_TRUE(c.checkCells({{2, 2}}, false));
    EXPECT_FALSE(c.checkCells({{2, 2}}, true));
    EXPECT_TRUE(c.checkCells({{2, 2}}));   // default policy is reject_unknown = false
}

TEST(CheckCells, outOfGridCellReadsAsUnknown){
    auto c = oneCellChecker(0);
    EXPECT_TRUE(c.checkCells({{-1, 0}, {10, 10}}, false));
    EXPECT_FALSE(c.checkCells({{-1, 0}}, true));
}

TEST(CheckCells, emptyListIsClearUnderBothPolicies){
    auto c = oneCellChecker(100);
    EXPECT_TRUE(c.checkCells({}, false));
    EXPECT_TRUE(c.checkCells({}, true));
}

TEST(CheckCells, ignoresDistanceField){
    // EDT says everything is near an obstacle; the grid is free, so the scan alone decides.
    CollisionChecker c(uniformGrid(10, 10, 1.0, 0), uniformEDT(10, 10, 1.0, 0.01f), discRobot(0.25, 0.5));
    EXPECT_TRUE(c.checkCells({{2, 2}, {3, 3}}, true));
}


// ---- checkTrajectory: one example per verdict ----

TEST(CheckTrajectoryVerdict, emptySamplesAlwaysScanSwept){
    // No samples means no distance-field evidence, so the swept cells decide.
    std::vector<int8_t> occ(10 * 10, 0); occ[4 * 10 + 4] = 100;
    CollisionChecker blocked(gridFrom(10, 10, 1.0, occ), uniformEDT(10, 10, 1.0, 0.375f), discRobot(0.25, 0.5));
    EXPECT_EQ(blocked.checkTrajectory({{4, 4}}, {}, {}, GridCoords{0, 0}), TrajectoryVerdict::SCAN_BLOCKED);
    CollisionChecker clear(uniformGrid(10, 10, 1.0, 0), uniformEDT(10, 10, 1.0, 0.375f), discRobot(0.25, 0.5));
    EXPECT_EQ(clear.checkTrajectory({{4, 4}}, {}, {}, GridCoords{0, 0}), TrajectoryVerdict::SCAN_CLEAR);
}

TEST(CheckTrajectoryVerdict, collisionSampleIsFastReject){
    std::vector<float> df(10 * 10, 1.0f); df[5 * 10 + 5] = 0.05f;  // EDT < r_in at (5,5)
    CollisionChecker c(uniformGrid(10, 10, 1.0, 0), edtFrom(10, 10, 1.0, df), discRobot(0.25, 0.5));
    EXPECT_EQ(c.checkTrajectory({{5, 5}}, {{5, 5}}, {0.5}, GridCoords{0, 0}), TrajectoryVerdict::FAST_REJECT);
}

TEST(CheckTrajectoryVerdict, oobSampleScansToClearOrBlocked){
    const std::vector<GridCoords> samples = {{99, 99}};  // OOB -> inconclusive -> scan
    const std::vector<GridCoords> swept   = {{0, 0}, {1, 1}};
    CollisionChecker clear(uniformGrid(4, 4, 1.0, 0), uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    EXPECT_EQ(clear.checkTrajectory(swept, samples, clr(samples), GridCoords{0, 0}), TrajectoryVerdict::SCAN_CLEAR);

    std::vector<int8_t> occ(4 * 4, 0); occ[1 * 4 + 1] = 100;
    CollisionChecker blocked(gridFrom(4, 4, 1.0, occ), uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    EXPECT_EQ(blocked.checkTrajectory(swept, samples, clr(samples), GridCoords{0, 0}), TrajectoryVerdict::SCAN_BLOCKED);
}

// ---- checkTrajectory with a clearance larger than r_out ----
// discRobot(0.25, 0.5) with clearance c = 0.6: accept iff edt > 0.6; reject iff edt + (c - r_out) < r_in,
// i.e. edt < 0.15; otherwise scan.

namespace {
TrajectoryVerdict verdictAtEdt(float edt, int8_t occupied_value = 0){
    std::vector<int8_t> occ(4 * 4, 0);
    occ[2 * 4 + 2] = occupied_value;
    CollisionChecker c(gridFrom(4, 4, 1.0, occ), uniformEDT(4, 4, 1.0, edt), discRobot(0.25, 0.5));
    return c.checkTrajectory({{2, 2}}, {{1, 1}}, {0.6}, GridCoords{0, 0});
}
}  // namespace

TEST(CheckTrajectoryClearance, edtAboveClearanceFastAccepts){
    // 0.55 would already be accepted at clearance r_out (0.5) but must not be at 0.6.
    EXPECT_EQ(verdictAtEdt(0.625f, 100), TrajectoryVerdict::FAST_ACCEPT);
    EXPECT_EQ(verdictAtEdt(0.55f, 100), TrajectoryVerdict::SCAN_BLOCKED);
}

TEST(CheckTrajectoryClearance, edtBetweenRejectAndAcceptThresholdsScans){
    EXPECT_EQ(verdictAtEdt(0.17f), TrajectoryVerdict::SCAN_CLEAR);   // just above r_in - (c - r_out) = 0.15
    EXPECT_EQ(verdictAtEdt(0.5f), TrajectoryVerdict::SCAN_CLEAR);
}

TEST(CheckTrajectoryClearance, edtPlusExcessBelowRinFastRejects){
    // 0.1 + (0.6 - 0.5) = 0.2 < r_in = 0.25. At clearance r_out this edt (0.2) would also reject; at
    // 0.12 below it would scan at r_out but rejects here.
    EXPECT_EQ(verdictAtEdt(0.1f), TrajectoryVerdict::FAST_REJECT);
    EXPECT_EQ(verdictAtEdt(0.2f), TrajectoryVerdict::SCAN_CLEAR);  // 0.2 + 0.1 = 0.3 >= r_in
}

TEST(CheckTrajectoryClearance, sizeMismatchThrows){
    CollisionChecker c(uniformGrid(4, 4, 1.0, 0), uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    EXPECT_THROW(c.checkTrajectory({{1, 1}}, {{1, 1}, {2, 2}}, {0.6}, GridCoords{0, 0}), std::invalid_argument);
    EXPECT_THROW(c.checkTrajectory({{1, 1}}, {{1, 1}}, {}, GridCoords{0, 0}), std::invalid_argument);
}

TEST(CheckTrajectoryClearance, clearanceBelowRoutThrows){
    CollisionChecker c(uniformGrid(4, 4, 1.0, 0), uniformEDT(4, 4, 1.0, 5.0f), discRobot(0.25, 0.5));
    EXPECT_THROW(c.checkTrajectory({{1, 1}}, {{1, 1}}, {0.49}, GridCoords{0, 0}), std::invalid_argument);
    EXPECT_NO_THROW(c.checkTrajectory({{1, 1}}, {{1, 1}}, {0.5}, GridCoords{0, 0}));  // exactly r_out is allowed
}

// ---- checkCell: single-cell policy (truth table) ----

namespace {
// 10x10 grid: (1,1) FREE, (2,2) OCCUPIED, (3,3) UNKNOWN, rest FREE.
CollisionChecker threeStatusChecker(){
    std::vector<int8_t> d(10 * 10, 0);
    d[2 * 10 + 2] = 100;
    d[3 * 10 + 3] = -1;
    return CollisionChecker(gridFrom(10, 10, 1.0, d), uniformEDT(10, 10, 1.0, 1.0f), discRobot(0.25, 0.5));
}
const GridCoords kFree{1, 1}, kOcc{2, 2}, kUnk{3, 3}, kOob{-1, 0};
}

TEST(CheckCell, freeCellPasses){
    auto c = threeStatusChecker();
    for (bool ru : {false, true}) EXPECT_TRUE(c.checkCell(kFree, ru));
}

TEST(CheckCell, occupiedCellBlocks){
    auto c = threeStatusChecker();
    for (bool ru : {false, true}) EXPECT_FALSE(c.checkCell(kOcc, ru));
}

TEST(CheckCell, unknownCellBlocksOnlyWhenRejectUnknown){
    auto c = threeStatusChecker();
    EXPECT_TRUE(c.checkCell(kUnk, false));
    EXPECT_FALSE(c.checkCell(kUnk, true));
}

TEST(CheckCell, outOfGridCellReadsAsUnknown){
    auto c = threeStatusChecker();
    EXPECT_TRUE(c.checkCell(kOob, false));
    EXPECT_FALSE(c.checkCell(kOob, true));
}

TEST(CheckCell, checkCellsIsAllOfCheckCell){
    auto c = threeStatusChecker();
    const std::vector<std::vector<GridCoords>> lists = {
        {kFree}, {kFree, kOcc}, {kFree, kUnk}, {kFree, kOob}, {kFree, kOcc, kUnk, kOob}, {kOcc, kUnk}};
    for (const auto & list : lists)
        for (bool ru : {false, true})
        {
            bool all = true;
            for (const auto & cell : list) all = all && c.checkCell(cell, ru);
            EXPECT_EQ(c.checkCells(list, ru), all);
        }
}
