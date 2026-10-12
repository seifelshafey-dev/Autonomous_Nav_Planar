#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/map/map_provider.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/types/grid_coords.hpp"
#include "core/types/state.hpp"

/*
ObstacleHeuristic unit suite. Collaborators are REAL and controlled: a hand-built OccupancyGrid -> real
EDT (buildEDTGrid) -> real CostMap (buildCostMap). Oracles are independent of the Dijkstra code:
straight-line Euclidean distance (a lower bound the field must exceed when a detour is forced) and the
UNREACHABLE sentinel for cells with no route to the goal. The differential test relies only on the
monotone dependence of the field on cost, not on the exact value.
*/

namespace {

constexpr double CELL = 0.1;

GridMetaData meta(int cols, int rows){
    return GridMetaData{cols, rows, CELL,
                        Eigen::Vector2f::Zero()};
}

std::shared_ptr<MapSnapshot> snapshot(int cols, int rows, std::vector<int8_t> occ){
    OccupancyGrid g(meta(cols, rows), std::move(occ));
    EDT e = buildEDTGrid(g, 5.0);
    return std::make_shared<MapSnapshot>(MapSnapshot{std::move(g), std::move(e)});
}

std::vector<int8_t> freeOcc(int cols, int rows){
    return std::vector<int8_t>(static_cast<std::size_t>(cols) * rows, 0);
}

void fillBlock(std::vector<int8_t> & occ, int cols, int c0, int r0, int c1, int r1, int8_t v){
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
            occ[static_cast<std::size_t>(r) * cols + c] = v;
}

// CostPolicy that yields an all-zero costmap (no clearance inflation, factor 1 with zero clearance gives no unknown penalty), so the
// field is a pure geometric (octile) distance and the Euclidean oracle is exact.
CostPolicy zeroPolicy(){ return CostPolicy{0.0f, 0.0f, 0.0f, 1.0f}; }

ObstacleHeuristic makeH(std::shared_ptr<MapSnapshot> snap, const CostPolicy & p,
                        double costmap_weight = 1.0, double weight = 1.0){
    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto cm   = std::make_shared<const CostMap>(buildCostMap(*snap, p, 0.1, costmap_weight));
    return ObstacleHeuristic(grid, std::move(cm), costmap_weight, weight);
}

// World-frame center of a cell, as an SE2 pose.
SE2State center(const GridCoords & gc){
    return SE2State{(gc.i + 0.5) * CELL, (gc.j + 0.5) * CELL, 0.0};
}

double cellEuclid(const GridCoords & a, const GridCoords & b){
    const double di = a.i - b.i, dj = a.j - b.j;
    return std::sqrt(di * di + dj * dj) * CELL;
}

bool isUnreachable(double v){ return v >= ObstacleHeuristic::UNREACHABLE; }

// Independent shortest-path oracle: a Bellman-Ford fixed-point over the 8-connected grid graph, rooted
// at the goal, sharing NO code with the production Dijkstra (no buckets, no priority queue — just relax
// every edge until nothing changes). Edge cost INTO a cell charges that cell's costmap value times the
// geometric step, matching the cost model in obstacle_heuristic.cpp. This is the reference the bucket
// queue must reproduce. Returns a row-major rows*cols field; UNREACHABLE for obstacle/walled-off cells.
std::vector<double> bellmanFordField(int cols, int rows, const std::vector<int8_t> & occ,
                                     const CostMap & cm, double costmap_weight, int gi, int gj){
    constexpr double INF = ObstacleHeuristic::UNREACHABLE;
    const double ortho = CELL, diag = std::sqrt(2.0) * CELL;
    auto occupied = [&](int i, int j){ return occ[static_cast<std::size_t>(j) * cols + i] >= 50; };

    std::vector<double> d(static_cast<std::size_t>(cols) * rows, INF);
    d[static_cast<std::size_t>(gj) * cols + gi] = 0.0;

    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool changed = true;
    while (changed) {
        changed = false;
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < cols; ++i) {
                if (occupied(i, j)) { continue; }                 // obstacles are impassable
                const std::size_t idx = static_cast<std::size_t>(j) * cols + i;
                const double rate = costmap_weight * static_cast<double>(cm.costAt(i, j));
                for (int k = 0; k < 8; ++k) {
                    const int ni = i + di[k], nj = j + dj[k];
                    if (ni < 0 || ni >= cols || nj < 0 || nj >= rows) { continue; }
                    const double dn = d[static_cast<std::size_t>(nj) * cols + ni];
                    if (dn >= INF) { continue; }
                    const double step = (di[k] != 0 && dj[k] != 0) ? diag : ortho;
                    const double cand = dn + step * (1.0 + rate);  // edge into (i,j) charges costAt(i,j)
                    if (cand + 1e-12 < d[idx]) { d[idx] = cand; changed = true; }
                }
            }
        }
    }
    return d;
}

}  // namespace

// ---- field values ---------------------------------------------------------------------------

TEST(ObstacleHeuristic, zeroAtGoalCell){
    auto h = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy());
    h.prepare(center({0, 0}));
    EXPECT_DOUBLE_EQ(h.fieldAt({0, 0}), 0.0);
}

TEST(ObstacleHeuristic, freeGridApproximatesStraightLineAndIsMonotone){
    auto h = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy());
    const GridCoords goal{0, 0};
    h.prepare(center(goal));

    // Monotone decrease toward the goal along a straight free corridor.
    EXPECT_GT(h.fieldAt({3, 0}), h.fieldAt({2, 0}));
    EXPECT_GT(h.fieldAt({2, 0}), h.fieldAt({1, 0}));
    EXPECT_GT(h.fieldAt({1, 0}), h.fieldAt({0, 0}));

    // 8-connected distance is bounded by [euclidean, sqrt(2)*euclidean].
    const GridCoords p{3, 4};
    const double e = cellEuclid(p, goal);
    EXPECT_GE(h.fieldAt(p), e - 1e-9);
    EXPECT_LE(h.fieldAt(p), e * std::sqrt(2.0) + 1e-9);
}

TEST(ObstacleHeuristic, wallForcesDetourExceedingStraightLine){
    // Vertical wall (col 10, rows 0..14) with a gap at the top (rows 15..19). A probe directly across
    // the wall from the goal must route up to the gap and back, so its field far exceeds the
    // straight-line distance. Oracle = straight-line distance (independent of the Dijkstra code).
    std::vector<int8_t> occ = freeOcc(20, 20);
    fillBlock(occ, 20, 10, 0, 10, 14, 100);
    auto h = makeH(snapshot(20, 20, std::move(occ)), zeroPolicy());

    const GridCoords goal{15, 5}, probe{5, 5};
    h.prepare(center(goal));
    EXPECT_GT(h.fieldAt(probe), cellEuclid(probe, goal) + 0.5);  // straight line is 1.0 m; detour ~3 m
}

TEST(ObstacleHeuristic, higherPenaltyRaisesFieldThroughCostlyRegion){
    // Identical geometry; only the unknown penalty differs. The probe is separated from the goal by an
    // UNKNOWN band it must cross, so the field is larger under the costlier policy.
    std::vector<int8_t> occ = freeOcc(20, 20);
    fillBlock(occ, 20, 8, 0, 11, 19, -1);  // unknown band, full height
    auto snap = snapshot(20, 20, std::move(occ));

    auto hA = makeH(snap, CostPolicy{0.0f, 0.0f, 0.0f, 1.0f});
    auto hB = makeH(snap, CostPolicy{0.0f, 0.0f, 0.0f, 51.0f});
    const GridCoords goal{18, 5}, probe{2, 5};
    hA.prepare(center(goal));
    hB.prepare(center(goal));
    EXPECT_GT(hB.fieldAt(probe), hA.fieldAt(probe));
}

// ---- unreachable handling -------------------------------------------------------------------

TEST(ObstacleHeuristic, occupiedAndWalledOffAndOffGridCellsAreUnreachable){
    std::vector<int8_t> occ = freeOcc(20, 20);
    fillBlock(occ, 20, 10, 10, 10, 10, 100);  // a single occupied cell
    fillBlock(occ, 20, 4, 4, 6, 6, 100);      // a ring...
    occ[static_cast<std::size_t>(5) * 20 + 5] = 0;  // ...around a carved-free cell (5,5)
    auto h = makeH(snapshot(20, 20, std::move(occ)), zeroPolicy());
    h.prepare(SE2State{1.5, 1.5, 0.0});  // goal at (15,15), free and reachable

    EXPECT_TRUE(isUnreachable(h.fieldAt({10, 10})));  // occupied cell: never relaxed
    EXPECT_TRUE(isUnreachable(h.fieldAt({5, 5})));     // free but enclosed: no route to goal
    EXPECT_TRUE(isUnreachable(h.fieldAt({-1, 0})));    // off-grid
    EXPECT_TRUE(isUnreachable(h.fieldAt({100, 100})));  // off-grid
}

// ---- operator() policy: prune in-grid dead ends, abstain off-grid / for off-grid goals ------

TEST(ObstacleHeuristic, operatorPrunesInGridUnreachableCell){
    std::vector<int8_t> occ = freeOcc(20, 20);
    fillBlock(occ, 20, 4, 4, 6, 6, 100);
    occ[static_cast<std::size_t>(5) * 20 + 5] = 0;  // enclosed free cell (5,5)
    auto h = makeH(snapshot(20, 20, std::move(occ)), zeroPolicy());
    h.prepare(SE2State{1.5, 1.5, 0.0});  // in-grid goal -> active field
    EXPECT_GE(h(center({5, 5})), ObstacleHeuristic::UNREACHABLE);  // weight 1.0 -> steer away
}

TEST(ObstacleHeuristic, operatorAbstainsForOffGridState){
    auto h = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy());
    h.prepare(SE2State{1.0, 1.0, 0.0});           // in-grid goal -> active
    EXPECT_DOUBLE_EQ(h(SE2State{-5.0, -5.0, 0.0}), 0.0);  // off-grid frontier -> abstain
}

TEST(ObstacleHeuristic, operatorAbstainsEntirelyWhenGoalOffGrid){
    auto h = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy());
    h.prepare(SE2State{100.0, 100.0, 0.0});  // goal off-grid -> field unrooted (inactive)
    EXPECT_DOUBLE_EQ(h(SE2State{0.5, 0.5, 0.0}), 0.0);
    EXPECT_DOUBLE_EQ(h(SE2State{1.9, 1.9, 0.0}), 0.0);
}

TEST(ObstacleHeuristic, appliesWeightToReachableField){
    auto h1 = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy(), 1.0, 1.0);
    auto h3 = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy(), 1.0, 3.0);
    h1.prepare(center({0, 0}));
    h3.prepare(center({0, 0}));
    const SE2State probe = center({5, 3});
    EXPECT_NEAR(h3(probe), 3.0 * h1(probe), 1e-9);
}

// ---- Dial's bucket queue vs an independent shortest-path oracle ------------------------------
// These lock the bucket-queue Dijkstra to the same field a from-scratch Bellman-Ford produces, on
// hand-built grids.

// Exact hand-computed value on a tiny free grid (zero costmap -> field is pure octile distance). From
// goal (0,0) to (3,2): the octile path takes min(3,2)=2 diagonal steps + |3-2|=1 orthogonal step =
// 2*sqrt(2)*CELL + 1*CELL. No code-derived oracle: this is arithmetic. Tolerance is 1e-6 (not 1e-9)
// because GridMetaData::resolution is a float: the field steps by the float-widened resolution
// (~0.10000000149), so a few accumulated steps drift ~6e-9 from the exact-double octile value. 1e-6
// pins the value to six decimals while absorbing that float-resolution gap.
TEST(ObstacleHeuristicDial, freeGridMatchesHandComputedOctileDistance){
    auto snap = snapshot(8, 8, freeOcc(8, 8));
    auto h = makeH(snap, zeroPolicy());
    h.prepare(center({0, 0}));

    const double expected = 2.0 * std::sqrt(2.0) * CELL + 1.0 * CELL;
    EXPECT_NEAR(h.fieldAt({3, 2}), expected, 1e-6);
}

// Cell-by-cell agreement with the independent Bellman-Ford oracle on a grid with a wall (forcing a
// detour) and a NONZERO costmap (cost-weighted edges). Every cell — reachable or walled-off — must
// match, which pins both the bucket index arithmetic and the unreachable handling.
TEST(ObstacleHeuristicDial, matchesIndependentBellmanFordWithWallAndCostmap){
    constexpr int COLS = 16, ROWS = 16;
    std::vector<int8_t> occ = freeOcc(COLS, ROWS);
    fillBlock(occ, COLS, 8, 0, 8, 11, 100);  // vertical wall col 8, rows 0..11; gap rows 12..15

    auto snap = snapshot(COLS, ROWS, occ);
    const CostPolicy policy{0.3f, 5.0f, 20.0f, 1.0f};   // clearance inflation -> nonzero per-cell cost
    const double costmap_weight = 1.0;

    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto cm   = std::make_shared<const CostMap>(buildCostMap(*snap, policy, 0.1, costmap_weight));
    ObstacleHeuristic h(grid, cm, costmap_weight, 1.0);

    const int gi = 13, gj = 5;  // free cell on the right of the wall
    h.prepare(center({gi, gj}));

    const std::vector<double> oracle =
        bellmanFordField(COLS, ROWS, occ, *cm, costmap_weight, gi, gj);

    for (int j = 0; j < ROWS; ++j) {
        for (int i = 0; i < COLS; ++i) {
            const double got = h.fieldAt({i, j});
            const double exp = oracle[static_cast<std::size_t>(j) * COLS + i];
            if (isUnreachable(exp)) {
                EXPECT_TRUE(isUnreachable(got)) << "cell (" << i << "," << j << ") should be unreachable";
            } else {
                EXPECT_NEAR(got, exp, 1e-6) << "cell (" << i << "," << j << ") field mismatch";
            }
        }
    }
}

// ---- prepare(): same-goal-cell reuse never serves a stale field ----

namespace {
std::vector<double> fullField(const ObstacleHeuristic & h, int cols, int rows){
    std::vector<double> f;
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i) f.push_back(h.fieldAt({i, j}));
    return f;
}
std::vector<int8_t> wallOcc(){
    std::vector<int8_t> occ = freeOcc(20, 20);
    fillBlock(occ, 20, 10, 2, 10, 15, 100);
    return occ;
}
}

TEST(ObstacleHeuristicReuse, sameCellDifferentPoseMatchesFreshPrepare){
    auto snap = snapshot(20, 20, wallOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    h.prepare(SE2State{1.52, 1.53, 0.0});
    h.prepare(SE2State{1.58, 1.51, 2.0});   // same cell (15,15)
    fresh.prepare(SE2State{1.52, 1.53, 0.0});
    EXPECT_EQ(fullField(h, 20, 20), fullField(fresh, 20, 20));
    for (const GridCoords & c : {GridCoords{2, 2}, GridCoords{5, 17}, GridCoords{18, 3}})
        EXPECT_EQ(h(center(c)), fresh(center(c)));
}

TEST(ObstacleHeuristicReuse, aThenBThenAEachMatchesFresh){
    auto snap = snapshot(20, 20, wallOcc());
    auto h = makeH(snap, zeroPolicy());
    const SE2State A = center({15, 15}), B = center({3, 4});
    for (const SE2State & g : {A, B, A}){
        h.prepare(g);
        auto fresh = makeH(snap, zeroPolicy());
        fresh.prepare(g);
        EXPECT_EQ(fullField(h, 20, 20), fullField(fresh, 20, 20));
    }
}

TEST(ObstacleHeuristicReuse, offGridGoalTwiceStaysInactive){
    auto h = makeH(snapshot(20, 20, freeOcc(20, 20)), zeroPolicy());
    for (const SE2State & g : {SE2State{100.0, 100.0, 0.0}, SE2State{100.02, 100.03, 1.0}}){
        h.prepare(g);
        EXPECT_DOUBLE_EQ(h(center({3, 3})), 0.0);
        EXPECT_TRUE(isUnreachable(h.fieldAt({3, 3})));
    }
}

TEST(ObstacleHeuristicReuse, offGridThenInGridGoalActivates){
    auto snap = snapshot(20, 20, wallOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    h.prepare(SE2State{100.0, 100.0, 0.0});
    h.prepare(center({15, 15}));
    fresh.prepare(center({15, 15}));
    EXPECT_EQ(fullField(h, 20, 20), fullField(fresh, 20, 20));
    EXPECT_GT(h(center({2, 2})), 0.0);
}

TEST(ObstacleHeuristicReuse, occupiedGoalTwiceInactiveThenFreeGoalMatchesFresh){
    auto snap = snapshot(20, 20, wallOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    const SE2State occGoal = center({10, 5});
    h.prepare(occGoal);
    EXPECT_DOUBLE_EQ(h(center({2, 2})), 0.0);
    h.prepare(occGoal);
    EXPECT_DOUBLE_EQ(h(center({2, 2})), 0.0);
    h.prepare(center({15, 15}));
    fresh.prepare(center({15, 15}));
    EXPECT_EQ(fullField(h, 20, 20), fullField(fresh, 20, 20));
}

// ---- on-demand field: values equal the full build whatever the query order ------------------

namespace {
// Expected field of a free zero-cost 1-row corridor, k steps from the goal: repeated addition of the
// grid resolution (the same accumulation the wave performs), not k * resolution.
double corridorValue(const OccupancyGrid & g, int k){
    const double res = g.getMetadata().resolution;
    double d = 0.0;
    for (int s = 0; s < k; ++s) d += res * 1.0;
    return d;
}
std::vector<int8_t> splitOcc(){  // 10x10, full-height wall at col 4: left half walled off from goal (8,5)
    std::vector<int8_t> occ = freeOcc(10, 10);
    fillBlock(occ, 10, 4, 0, 4, 9, 100);
    return occ;
}
}

TEST(ObstacleHeuristicLazy, corridorValuesEqualRepeatedAdditionInAnyOrder){
    auto snap = snapshot(10, 1, freeOcc(10, 1));
    auto h = makeH(snap, zeroPolicy());
    h.prepare(center({0, 0}));
    // Query far cell first, then nearer ones, then a middle one: wave resumes, values fixed.
    for (int k : {9, 3, 0, 6, 1}) EXPECT_EQ(h.fieldAt({k, 0}), corridorValue(snap->grid_, k)) << "k=" << k;
}

TEST(ObstacleHeuristicLazy, occupiedCellIsUnreachable){
    std::vector<int8_t> occ = freeOcc(10, 10);
    occ[static_cast<std::size_t>(3) * 10 + 3] = 100;
    auto h = makeH(snapshot(10, 10, std::move(occ)), zeroPolicy(), 1.0, 2.0);
    h.prepare(center({8, 8}));
    EXPECT_EQ(h.fieldAt({3, 3}), ObstacleHeuristic::UNREACHABLE);
    EXPECT_EQ(h(center({3, 3})), 2.0 * ObstacleHeuristic::UNREACHABLE);
}

TEST(ObstacleHeuristicLazy, walledOffCellsUnreachableAndReachableCellStillCorrect){
    auto snap = snapshot(10, 10, splitOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    h.prepare(center({8, 5}));
    fresh.prepare(center({8, 5}));
    EXPECT_EQ(h.fieldAt({1, 1}), ObstacleHeuristic::UNREACHABLE);  // exhausts the wave
    EXPECT_EQ(h.fieldAt({2, 7}), ObstacleHeuristic::UNREACHABLE);  // second walled-off cell
    // Two orthogonal steps from the goal: 0 + res + res.
    const double res = snap->grid_.getMetadata().resolution;
    EXPECT_EQ(h.fieldAt({6, 5}), res + res);
    EXPECT_EQ(h.fieldAt({6, 5}), fresh.fieldAt({6, 5}));
    EXPECT_EQ(fullField(h, 10, 10), fullField(fresh, 10, 10));
}

TEST(ObstacleHeuristicLazy, preparingSameGoalCellAgainKeepsValues){
    auto snap = snapshot(10, 10, splitOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    h.prepare(center({8, 5}));
    fresh.prepare(center({8, 5}));
    const double a = h.fieldAt({6, 2});
    h.prepare(center({8, 5}));
    const double b = h.fieldAt({9, 9});
    EXPECT_EQ(a, fresh.fieldAt({6, 2}));
    EXPECT_EQ(b, fresh.fieldAt({9, 9}));
    EXPECT_EQ(h.fieldAt({6, 2}), a);
}

TEST(ObstacleHeuristicLazy, newGoalAfterQueriesMatchesFreshObject){
    auto snap = snapshot(10, 10, splitOcc());
    auto h = makeH(snap, zeroPolicy());
    auto fresh = makeH(snap, zeroPolicy());
    h.prepare(center({8, 5}));
    (void)h.fieldAt({9, 0});
    (void)h.fieldAt({1, 1});
    h.prepare(center({1, 8}));   // goal moves to the other side of the wall
    fresh.prepare(center({1, 8}));
    for (const GridCoords & c : {GridCoords{0, 0}, GridCoords{9, 9}, GridCoords{3, 5}, GridCoords{8, 5}})
        EXPECT_EQ(h.fieldAt(c), fresh.fieldAt(c));
    EXPECT_EQ(fullField(h, 10, 10), fullField(fresh, 10, 10));
}

TEST(ObstacleHeuristicLazy, offGridOrOccupiedGoalIsInactive){
    std::vector<int8_t> occ = freeOcc(10, 10);
    occ[static_cast<std::size_t>(5) * 10 + 5] = 100;
    auto h = makeH(snapshot(10, 10, std::move(occ)), zeroPolicy());
    for (const SE2State & g : {SE2State{100.0, 100.0, 0.0}, center({5, 5})}){
        h.prepare(g);
        EXPECT_EQ(h(center({2, 2})), 0.0);
        EXPECT_EQ(h.fieldAt({2, 2}), ObstacleHeuristic::UNREACHABLE);
    }
}
