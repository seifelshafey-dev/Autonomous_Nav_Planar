#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/hybrid_astar/search.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/planner_interface.hpp"
#include "core/planner/path.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/map/map_provider.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/math/wrap_angle.hpp"

/*
HybridAstar integration suite. Collaborators are REAL and composed via
the shared scenario seam (buildScenario over a programmatically-built ScenarioSpec) — the SAME factory
chain the data-driven integration tier uses, so these tests and test/integration/scenarios/*.yaml share one construction
path. Assertions here target planner-level outcomes that are NOT covered by the universal Tier-0 oracle
in test/integration/scenario_plan_test.cpp: exact analytic-shot termination, shot-disabled expansion
structure, branch short-circuits, and RELATIONAL comparisons (two configs over one map).

Cases whose ONLY assertion is a Tier-0 invariant over a fixed map (collision-free detour, enclosed-box
no path, expansion-cap no path, free-space reachability, all-UNKNOWN traversal, plain determinism) were
MIGRATED to test/integration/scenarios/*.yaml and are exercised by the integration tier — see the scenario files for
the 1:1 mapping. What remains below cannot be a single-scenario expect_found case:
either it asserts something tighter/different than the oracle, or it is comparative.
*/

namespace {

constexpr double CELL = 0.1;

DiffDriveParams makeKin(){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.3; kin.n_headings = 16;
    return kin;
}

GridMetaData meta(int cols, int rows){
    return GridMetaData{cols, rows, CELL,
                        Eigen::Vector2f::Zero()};
}

AStarConfig baseCfg(){
    AStarConfig c;
    c.node_pool_reserve = 4096;   // smaller reserves keep the test fixtures light
    c.closed_set_reserve = 4096;
    return c;
}
CostPolicy basePolicy(){ return CostPolicy{0.3f, 5.0f, 100.0f, 1.0f}; }  // unknown_cost_factor 1 (smallest legal) unless overridden

// Build a ScenarioSpec from controlled inputs, then route it through the production factory chain via
// buildScenario — the one shared seam. Returns the owning BuiltScenario; callers MUST keep it alive for
// the planner's lifetime (it aliases the snapshot). `occ` is row-major, sized cols*rows.
//
// Existing scenarios were written for the weighted-Euclidean heuristic; default to it so their
// expansion semantics are unchanged. The obstacle heuristic is opted into per-case.
BuiltScenario build(int cols, int rows, std::vector<int8_t> occ, AStarConfig cfg,
                    CostPolicy cp = basePolicy(),
                    HeuristicConfig hc = HeuristicConfig{1.0, false},
                    DiffDriveParams kin = makeKin()){
    ScenarioSpec spec{"search_test", meta(cols, rows), std::move(occ)};
    spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
    spec.diff_drive = kin;
    spec.a_star = cfg;
    spec.heuristic = hc;
    spec.cost_policy = cp;
    return buildScenario(spec);
}

BuiltScenario buildFree(int cols, int rows, AStarConfig cfg,
                        CostPolicy cp = basePolicy(),
                        HeuristicConfig hc = HeuristicConfig{1.0, false},
                        DiffDriveParams kin = makeKin()){
    return build(cols, rows, std::vector<int8_t>(static_cast<size_t>(cols) * rows, 0),
                 cfg, cp, hc, kin);
}

// occupy the inclusive cell rectangle [c0,c1] x [r0,r1]
void fillBlock(std::vector<int8_t> & occ, int cols, int c0, int r0, int c1, int r1, int8_t v){
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
            occ[static_cast<size_t>(r) * cols + c] = v;
}

bool withinGoalTol(const SE2State & a, const SE2State & goal, const AStarConfig & c){
    const double dx = goal.x - a.x, dy = goal.y - a.y;
    return (dx * dx + dy * dy) <= c.goal_pos_tol * c.goal_pos_tol + 1e-9 &&
           std::abs(wrapToPi(goal.theta - a.theta)) <= c.goal_heading_tol + 1e-9;
}

}  // namespace

// ---- Start/goal validity short-circuits (branch-targeted; not a map-geometry scenario) ------

TEST(HybridAstarSearch, startInCollisionReturnsStartBlocked){
    // Entire grid occupied -> start and goal both collide; the start gate is checked first.
    auto b = build(30, 30, std::vector<int8_t>(900, 100), baseCfg());
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.0, 1.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::StartBlocked);
    EXPECT_FALSE(r.path.has_value());
}

TEST(HybridAstarSearch, goalInCollisionReturnsGoalBlocked){
    std::vector<int8_t> occ(900, 0);
    fillBlock(occ, 30, 18, 8, 22, 12, 100);  // occupied block around goal cell (20,10)
    auto b = build(30, 30, std::move(occ), baseCfg());
    // start (0.5,1.0) is far from the block and free.
    const auto r = b.planner->plan(SE2State{0.5, 1.0, 0.0}, SE2State{2.0, 1.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::GoalBlocked);
    EXPECT_FALSE(r.path.has_value());
}

// ---- Trivial termination (single-node path structure; tighter than the goal-tol oracle) -----

TEST(HybridAstarSearch, startAlreadyAtGoalReturnsSingleNodePath){
    auto b = buildFree(30, 30, baseCfg());
    SE2State s{1.5, 1.5, 0.0};
    auto path = b.planner->plan(s, s).path;  // goal == start -> atGoal on first pop
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->node_points.size(), 1u);
    EXPECT_TRUE(path->segments.empty());
    EXPECT_DOUBLE_EQ(path->total_cost, 0.0);
}

// ---- Analytic shot exactness (1e-9, tighter than the integration goal-tolerance oracle) -----

TEST(HybridAstarSearch, freeSpaceShotReachesGoalExactly){
    auto b = buildFree(60, 60, baseCfg());  // shot enabled
    SE2State start{1.0, 1.0, 0.0}, goal{3.0, 1.0, 0.0};
    auto path = b.planner->plan(start, goal).path;
    ASSERT_TRUE(path.has_value());
    // The shot terminates the path exactly at the goal pose (tighter than the expansion tolerance).
    EXPECT_NEAR(path->node_points.back().x, goal.x, 1e-9);
    EXPECT_NEAR(path->node_points.back().y, goal.y, 1e-9);
    EXPECT_NEAR(wrapToPi(path->node_points.back().theta - goal.theta), 0.0, 1e-9);
    EXPECT_NEAR(path->node_points.front().x, start.x, 1e-9);
    EXPECT_NEAR(path->node_points.front().y, start.y, 1e-9);
    EXPECT_GT(path->total_cost, 0.0);
}

TEST(HybridAstarSearch, shotCrossesUnknownWhenAllowed){
    // All-UNKNOWN grid with shot_allow_unknown=true flips the shot to optimistic (UNKNOWN treated as
    // traversable). The shot fires on the first pop and terminates EXACTLY at the goal pose (1e-9) —
    // distinguishing it from the conservative case (migrated to unknown_field_conservative.yaml), which
    // only reaches the goal within the expansion tolerance. The exact 1e-9 termination + the
    // allow-unknown branch are why this stays a unit test rather than an expect_found scenario.
    auto cfg = baseCfg();
    cfg.shot_allow_unknown = true;
    auto b = build(30, 30, std::vector<int8_t>(900, -1), cfg);
    SE2State start{0.5, 1.0, 0.0}, goal{2.5, 1.0, 0.0};
    auto path = b.planner->plan(start, goal).path;
    ASSERT_TRUE(path.has_value());
    EXPECT_NEAR(path->node_points.back().x, goal.x, 1e-9);
    EXPECT_NEAR(path->node_points.back().y, goal.y, 1e-9);
    EXPECT_NEAR(wrapToPi(path->node_points.back().theta - goal.theta), 0.0, 1e-9);
}

// ---- Shot-disabled expansion structure (>=1 expansion segment; not a Tier-0 invariant) ------

TEST(HybridAstarSearch, expansionReachesGoalWithinToleranceWhenShotDisabled){
    auto cfg = baseCfg();
    cfg.enable_shot = false;
    auto b = buildFree(60, 60, cfg);
    SE2State start{1.0, 1.0, 0.0}, goal{2.8, 1.0, 0.0};  // reachable by a straight chain of 0.3 m steps
    const auto res = b.planner->plan(start, goal);
    EXPECT_EQ(res.outcome, PlanOutcome::Found);
    const auto & path = res.path;
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->node_points.size(), path->segments.size() + 1);
    EXPECT_GE(path->segments.size(), 1u);  // shot off -> the path is built entirely from primitives
    EXPECT_TRUE(withinGoalTol(path->node_points.back(), goal, cfg));
}

// ---- Budget: zero time budget (dt=0 cannot be expressed in a scenario YAML) ------------------

TEST(HybridAstarSearch, zeroTimeBudgetReturnsBudgetExhausted){
    // Shot disabled, so the only way plan() can return is the time-budget gate. The deadline equals the
    // plan() entry time and steady_clock has advanced by the time the gate runs, so the search returns
    // BudgetExhausted before expanding any node. The loader rejects max_plan_time_sec <= 0, so this is
    // necessarily a programmatic unit case, not a scenario.
    auto cfg = baseCfg();
    cfg.max_plan_time_sec = 0.0;
    cfg.enable_shot = false;
    auto b = buildFree(60, 60, cfg);
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{5.0, 1.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::BudgetExhausted);
    EXPECT_FALSE(r.path.has_value());
}

// ---- RELATIONAL: costmap penalty flows into path cost (two policies, one map) ----------------

TEST(HybridAstarSearch, unknownPenaltyRaisesPathCost){
    // A full-height UNKNOWN band the straight route must cross (no cheaper detour exists in-grid or via
    // the frontier). With unknown_cost_factor 1 vs 100 the geometry is identical but cost B > cost A. This is a
    // comparison BETWEEN two runs, so it cannot be a single-scenario expect_found case.
    std::vector<int8_t> occ(30 * 30, 0);
    fillBlock(occ, 30, 12, 0, 17, 29, -1);  // unknown band, full height
    auto cfg = baseCfg();
    cfg.enable_shot = false;

    SE2State start{0.5, 1.0, 0.0}, goal{2.5, 1.0, 0.0};
    auto bA = build(30, 30, occ, cfg, CostPolicy{0.3f, 5.0f, 100.0f, 1.0f});
    auto bB = build(30, 30, occ, cfg, CostPolicy{0.3f, 5.0f, 100.0f, 100.0f});
    auto a = bA.planner->plan(start, goal).path;
    auto b = bB.planner->plan(start, goal).path;
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_GT(b->total_cost, a->total_cost);
}

// ---- RELATIONAL: obstacle heuristic vs. obstacle-pocket trap (two heuristics, one map) -------

TEST(HybridAstarSearch, obstacleHeuristicEscapesPocketWithFewerExpansions){
    // A vertical wall with a gap at the top forces a long detour. The straight-line heuristic floods the
    // pocket in front of the wall; the obstacle-aware heuristic routes up to the gap. Over the same
    // map/cfg, the obstacle heuristic explores far fewer nodes. Comparative -> stays a unit test.
    constexpr int COLS = 50, ROWS = 50;
    std::vector<int8_t> occ(static_cast<size_t>(COLS) * ROWS, 0);
    fillBlock(occ, COLS, 25, 0, 25, 42, 100);  // wall col 25, rows 0..42; gap rows 43..49

    auto cfg = baseCfg();
    cfg.max_expansions = 40000;
    const SE2State start{0.5, 1.0, 0.0}, goal{4.5, 1.0, 0.0};  // both low, opposite sides of the wall

    auto bOn  = build(COLS, ROWS, occ, cfg, basePolicy(), HeuristicConfig{1.0, true});
    auto bOff = build(COLS, ROWS, occ, cfg, basePolicy(), HeuristicConfig{1.0, false});
    auto onPath = bOn.planner->plan(start, goal).path;
    (void)bOff.planner->plan(start, goal).path;

    ASSERT_TRUE(onPath.has_value());  // obstacle heuristic finds the detour within budget

    const auto * onH  = dynamic_cast<const HybridAstar *>(bOn.planner.get());
    const auto * offH = dynamic_cast<const HybridAstar *>(bOff.planner.get());
    ASSERT_NE(onH, nullptr);
    ASSERT_NE(offH, nullptr);
    EXPECT_LT(onH->nodePool().size(), offH->nodePool().size());
}

// ---- Closed-set dedup cell (AStarConfig::closed_cell_size) -----------------------------------
// The closed key floors map-resolution grid coords to a coarser dedup cell of stride =
// max(1, round(closed_cell_size / resolution)) whole map cells (search.cpp), decoupling pruning
// granularity from map resolution. stride == 1 (closed_cell_size == resolution) reproduces the
// original map-resolution key exactly; a coarser cell merges nearby states into one key, pruning
// more aggressively. These cases pin both ends, using the shared nodePool() proxy for search effort.

namespace {
// One planner run at a given dedup cell over a free 60x60 map. Shot disabled so the search is driven
// entirely by primitive expansion, where dedup granularity actually decides how many child nodes
// survive the closed-set check. `keep` is the owning BuiltScenario the caller must hold alive while
// inspecting the returned planner pointer; `pool` is the generated-node count (SIZE_MAX if no path).
struct ClosedCellRun {
    BuiltScenario keep;
    std::optional<Path> path;
    std::size_t pool;
};
ClosedCellRun runAtClosedCell(double closed_cell_size,
                              const SE2State & start, const SE2State & goal){
    auto cfg = baseCfg();
    cfg.enable_shot       = false;          // force primitive expansion
    cfg.closed_cell_size  = closed_cell_size;
    cfg.max_expansions    = 200000;
    // The factory requires vmax * atomic_time > sqrt(2) * effective cell; 0.6 m admits the 3x CELL run (0.4243).
    DiffDriveParams kin = makeKin();
    kin.atomic_time = 0.6;
    auto b = buildFree(60, 60, cfg, basePolicy(), HeuristicConfig{1.0, false}, kin);
    auto path = b.planner->plan(start, goal).path;
    std::size_t pool = static_cast<std::size_t>(-1);
    if (path.has_value()) {
        const auto * h = dynamic_cast<const HybridAstar *>(b.planner.get());
        if (h != nullptr) { pool = h->nodePool().size(); }
    }
    return ClosedCellRun{std::move(b), std::move(path), pool};
}
}  // namespace

TEST(HybridAstarClosedCell, strideOneReproducesMapResolutionKeyAndIsDeterministic){
    // closed_cell_size == resolution -> stride 1 -> keyOf floors by 1 (identity) -> the closed key is
    // exactly packClosedKey(gc.i, gc.j, k), the original map-resolution key. This is the byte-identical
    // anchor: the run is deterministic and a replan on the same instance is bit-for-bit identical.
    auto cfg = baseCfg();
    cfg.enable_shot      = false;
    cfg.closed_cell_size = CELL;            // == grid resolution -> stride 1
    auto b = buildFree(60, 60, cfg);
    const SE2State start{1.0, 1.0, 0.0}, goal{4.0, 3.0, 0.0};

    auto first = b.planner->plan(start, goal).path;
    ASSERT_TRUE(first.has_value());
    const auto * h = dynamic_cast<const HybridAstar *>(b.planner.get());
    ASSERT_NE(h, nullptr);
    const std::size_t pool_first = h->nodePool().size();

    auto second = b.planner->plan(start, goal).path;  // plan() resets all state; stride-1 key is stable
    ASSERT_TRUE(second.has_value());
    EXPECT_DOUBLE_EQ(second->total_cost, first->total_cost);
    ASSERT_EQ(second->node_points.size(), first->node_points.size());
    for (std::size_t i = 0; i < first->node_points.size(); ++i){
        EXPECT_DOUBLE_EQ(second->node_points[i].x, first->node_points[i].x) << "node " << i;
        EXPECT_DOUBLE_EQ(second->node_points[i].y, first->node_points[i].y) << "node " << i;
        EXPECT_DOUBLE_EQ(second->node_points[i].theta, first->node_points[i].theta) << "node " << i;
    }
    EXPECT_EQ(h->nodePool().size(), pool_first);  // generated-node count reproduces exactly
}

// Dedup regression guard — the relational substitute for a warehouse cost ceiling (warehouse is
// gitignored / not in the CI corpus, and an output-derived cost_max violates the Expectations
// contract). Coarsening the closed cell only ever PRUNES states, so path cost is monotone
// non-decreasing in the cell size while search effort drops. Pins both directions WITHOUT any map
// binary: a change that lets coarsening cheat cost below the baseline or blow the loose suboptimality
// bound fails loudly.
TEST(HybridAstarClosedCell, coarseningRaisesCostMonotonicallyWithinLooseBoundAndCutsExpansions){
    const SE2State start{1.0, 1.0, 0.0}, goal{4.0, 3.0, 0.0};

    ClosedCellRun s1 = runAtClosedCell(1.0 * CELL, start, goal);  // stride 1 (map-res baseline)
    ClosedCellRun s2 = runAtClosedCell(2.0 * CELL, start, goal);  // stride 2
    ClosedCellRun s3 = runAtClosedCell(3.0 * CELL, start, goal);  // stride 3

    ASSERT_TRUE(s1.path.has_value());
    ASSERT_TRUE(s2.path.has_value());
    ASSERT_TRUE(s3.path.has_value());

    const double c1 = s1.path->total_cost, c2 = s2.path->total_cost, c3 = s3.path->total_cost;

    // (1) Cost is monotone non-decreasing in the dedup cell: a coarser cell can only prune, never
    //     shortcut, so it cannot beat the finer baseline (small epsilon for float accumulation).
    EXPECT_GE(c2, c1 - 1e-9) << "stride 2 cost dipped below the stride-1 baseline";
    EXPECT_GE(c3, c2 - 1e-9) << "stride 3 cost dipped below the stride-2 cost";

    // (2) Suboptimality stays within a loose upper factor of the baseline (author-chosen sanity bound,
    //     NOT a planner-dumped value): the dedup must not silently produce a wildly worse path.
    EXPECT_LE(c3, 1.5 * c1) << "stride-3 path cost exceeded 1.5x the stride-1 baseline";

    // (3) The point of coarsening: less search effort. The coarsest run expands fewer nodes than the
    //     finest baseline (the established nodePool() proxy).
    EXPECT_LT(s3.pool, s1.pool) << "coarser dedup must cut expansions vs the map-resolution baseline";
}

// ---- Straight-primitive length vs closed-set dedup cell ---------------------------------------
// buildHybridAstar requires vmax * atomic_time > sqrt(2) * effective cell, where the effective cell
// is closedStride(closed_cell_size, resolution) * resolution [m]. Fixture resolution CELL = 0.1 m.
namespace {
void buildWithStraightLen(double vmax, double atomic_time, double closed_cell_size){
    auto cfg = baseCfg();
    cfg.closed_cell_size = closed_cell_size;
    DiffDriveParams kin = makeKin();
    kin.vmax = vmax; kin.atomic_time = atomic_time;
    auto b = buildFree(30, 30, cfg, basePolicy(), HeuristicConfig{1.0, false}, kin);
}
}  // namespace

TEST(HybridAstarBuild, straightPrimitiveLongerThanRootTwoEffectiveCellBuilds){
    EXPECT_NO_THROW(buildWithStraightLen(0.5, 0.6, 0.1));    // stride 1, cell 0.1, bound 0.1414 < 0.30
    EXPECT_NO_THROW(buildWithStraightLen(0.5, 0.6, 0.24));   // stride 2 (2.4), cell 0.2, bound 0.2828 < 0.30 (raw 0.24 would give 0.339)
    EXPECT_NO_THROW(buildWithStraightLen(0.5, 0.57, 0.2));   // stride 2, cell 0.2, bound 0.2828 < 0.285 (just over)
    EXPECT_NO_THROW(buildWithStraightLen(0.5, 0.3, 0.01));   // clamp: stride 1, cell 0.1, bound 0.1414 < 0.15
}

TEST(HybridAstarBuild, straightPrimitiveNotLongerThanRootTwoEffectiveCellThrows){
    EXPECT_THROW(buildWithStraightLen(0.5, 0.6, 0.3), std::invalid_argument);    // stride 3, cell 0.3, bound 0.4243 > 0.30
    EXPECT_THROW(buildWithStraightLen(0.5, 0.56, 0.16), std::invalid_argument);  // stride 2 (1.6), cell 0.2, bound 0.2828 > 0.28 (raw 0.16 gives 0.226, would pass)
    EXPECT_THROW(buildWithStraightLen(0.5, 0.56, 0.2), std::invalid_argument);   // stride 2, cell 0.2, bound 0.2828 > 0.28 (just under)
    EXPECT_THROW(buildWithStraightLen(0.5, 0.28, 0.01), std::invalid_argument);  // clamp: stride 1, cell 0.1, bound 0.1414 > 0.14
}

TEST(HybridAstarBuild, coarseMapUsesMapResolutionAsMinimumCell){
    // h = 0.25: closed_cell_size 0.15 rounds to stride 1 (0.6), cell 0.25, bound 0.3536 > 0.30.
    ScenarioSpec spec{"coarse", GridMetaData{20, 20, 0.25f, Eigen::Vector2f::Zero()},
                      std::vector<int8_t>(400, 0)};
    spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
    spec.diff_drive = makeKin();
    spec.diff_drive.vmax = 0.5; spec.diff_drive.atomic_time = 0.6;
    spec.a_star = baseCfg();
    spec.a_star.closed_cell_size = 0.15;
    spec.heuristic = HeuristicConfig{1.0, false};
    spec.cost_policy = basePolicy();
    EXPECT_THROW(buildScenario(spec), std::invalid_argument);
}

TEST(HybridAstarClosedStride, roundsToNearestWholeMapCellsAndClampsToOne){
    EXPECT_EQ(HybridAstar::closedStride(0.15, 0.03), 5);
    EXPECT_EQ(HybridAstar::closedStride(0.15, 0.05), 3);
    EXPECT_EQ(HybridAstar::closedStride(0.16, 0.1), 2);    // 1.6 (an exact .5 tie is avoided: 0.15 / 0.1 is 1.4999... in binary)
    EXPECT_EQ(HybridAstar::closedStride(0.15, 0.25), 1);   // 0.6 rounds to 1
    EXPECT_EQ(HybridAstar::closedStride(0.01, 0.05), 1);   // 0.2 rounds to 0, clamped to 1
    EXPECT_EQ(HybridAstar::closedStride(0.16, 0.05), 3);   // 3.2
    EXPECT_EQ(HybridAstar::closedStride(0.2, 0.05), 4);
}

// ---- Budget covers heuristic prepare() (deadline anchored at plan() entry) -------------------
// plan() anchors the wall-clock deadline BEFORE calling heuristic->prepare() (search.cpp), so
// max_plan_time_sec bounds the WHOLE call including a heavy prep (e.g. ObstacleHeuristic's full-grid
// Dijkstra). If prep alone exhausts the budget, the search loop must never start: plan() returns
// BudgetExhausted with only the start node yet to be pushed — i.e. ZERO loop expansions. These cases inject a
// heuristic whose prepare() deliberately blocks, so the deadline is provably crossed inside prep.

namespace {

// Heuristic wrapper whose prepare() sleeps for a fixed duration (simulating a heavy wavefront prep),
// then delegates to a real inner heuristic for operator(). Counts prepare() calls so a test can
// confirm prep actually ran. Real-object composition: the inner heuristic is a genuine
// DistanceHeuristic, so a search that DOES start behaves exactly like the production weighted-Euclidean
// configuration.
class BlockingHeuristic : public Heuristic {
public:
    BlockingHeuristic(std::shared_ptr<const Heuristic> inner, std::chrono::milliseconds block)
        : Heuristic(1.0), inner_(std::move(inner)), block_(block) {}

    void prepare(const SE2State & goal) const override {
        ++prepare_calls_;
        std::this_thread::sleep_for(block_);
        inner_->prepare(goal);
    }
    double operator()(const SE2State & current) const override { return (*inner_)(current); }

    int prepareCalls() const noexcept { return prepare_calls_.load(); }

private:
    std::shared_ptr<const Heuristic> inner_;
    std::chrono::milliseconds        block_;
    mutable std::atomic<int>         prepare_calls_{0};
};

// Live HybridAstar over a controlled free map with an INJECTED heuristic. Replicates the production
// dependency wiring (buildHybridAstar) so the only substitution is the heuristic — every other
// collaborator (kinematic model, collision checker, state space, costmap) is real. Returned by value;
// holds nothing the caller must keep alive beyond the planner itself (the snapshot is owned via the
// aliasing shared_ptrs captured inside the planner's collaborators).
struct WiredPlanner {
    std::shared_ptr<const MapSnapshot> snapshot;  // kept alive for the planner's collaborators
    std::shared_ptr<BlockingHeuristic> heuristic;
    std::unique_ptr<HybridAstar>       planner;
};

WiredPlanner wireBlockingPlanner(int cols, int rows, std::chrono::milliseconds block, AStarConfig cfg){
    const GridMetaData gm = meta(cols, rows);
    OccupancyGrid grid(gm, std::vector<int8_t>(static_cast<size_t>(cols) * rows, 0));  // all free
    EDT edt = buildEDTGrid(grid, 5.0);
    auto snap = std::make_shared<const MapSnapshot>(MapSnapshot{std::move(grid), std::move(edt)});

    RobotParams body = buildRobotParams(
        {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
    const DiffDriveParams kin = makeKin();
    KinematicModel km = buildKinematicModel(body, kin, gm.resolution);

    auto se2_dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(snap, &snap->edt_);
    auto checker  = std::make_shared<const CollisionChecker>(grid_ptr, edt_ptr, km.motion->body());
    auto space    = std::make_shared<const DiffDriveStateSpace>(gm, se2_dist);
    auto costmap  = std::make_shared<const CostMap>(buildCostMap(*snap, basePolicy(), body.r_in, cfg.costmap_weight));

    // Snapshot-aware motion context (matches buildHybridAstar) so world-frame shot rasterization works.
    KinematicModel runtime_kin = km;
    runtime_kin.motion = std::make_shared<const MotionContext>(km.motion->body(), space, gm);

    auto inner = std::make_shared<const DistanceHeuristic>(se2_dist, 1.0);
    auto blocking = std::make_shared<BlockingHeuristic>(inner, block);

    auto planner = std::make_unique<HybridAstar>(
        std::move(runtime_kin), checker, space, blocking, costmap, gm, cfg);

    return WiredPlanner{snap, blocking, std::move(planner)};
}

}  // namespace

TEST(HybridAstarBudget, prepBlowingBudgetReturnsBudgetExhaustedWithZeroExpansions){
    // prepare() blocks ~120 ms; the budget is 5 ms. The deadline (anchored at plan() entry) is already
    // past when prep returns, so plan() returns BudgetExhausted before the start node is pushed — ZERO loop
    // expansions occurred and the node pool stays empty.
    auto cfg = baseCfg();
    cfg.max_plan_time_sec = 0.005;  // 5 ms
    WiredPlanner w = wireBlockingPlanner(60, 60, std::chrono::milliseconds(120), cfg);

    const auto r = w.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{5.0, 1.0, 0.0});

    EXPECT_EQ(r.outcome, PlanOutcome::BudgetExhausted);
    EXPECT_FALSE(r.path.has_value()) << "prep alone exceeded the budget -> no path";
    EXPECT_EQ(w.heuristic->prepareCalls(), 1) << "prepare() must have been called (and blocked)";
    // The post-prep deadline check returns BEFORE the start node is ever pushed, so the pool is empty.
    EXPECT_EQ(w.planner->nodePool().size(), 0u)
        << "search loop must not run when prep already blew the budget";
}

TEST(HybridAstarBudget, moderateBudgetEntersLoopAndPlans){
    // Same wiring, negligible prep block (1 ms) and a generous budget: prep finishes well inside the
    // deadline, so the search loop runs and finds a path. Confirms the deadline-covers-prep guard does
    // not spuriously abort a normal plan, and that the start node IS pushed once the loop is entered.
    auto cfg = baseCfg();
    cfg.max_plan_time_sec = 5.0;
    WiredPlanner w = wireBlockingPlanner(60, 60, std::chrono::milliseconds(1), cfg);

    auto path = w.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{3.0, 1.0, 0.0}).path;

    ASSERT_TRUE(path.has_value()) << "a moderate budget must let the search run to a solution";
    EXPECT_EQ(w.heuristic->prepareCalls(), 1);
    EXPECT_GE(w.planner->nodePool().size(), 1u) << "the loop ran -> at least the start node was pushed";
}

// ---- Cancellation: a pre-stopped token aborts before any expansion ----------------------------

TEST(HybridAstarSearch, preStoppedTokenReturnsStopped){
    // A token that is already stop-requested returns Stopped without expanding (deterministic — no
    // clock or scenario dependence).
    auto cfg = baseCfg();
    auto b = buildFree(60, 60, cfg);
    std::stop_source ss;
    ss.request_stop();
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{5.0, 1.0, 0.0},
                                   nullptr, ss.get_token());
    EXPECT_EQ(r.outcome, PlanOutcome::Stopped);
    EXPECT_FALSE(r.path.has_value());
}

// ---- Start gate: a start overlapping a mapped wall is blocked ---------------------------------
// Geometry: a full-height OCCUPIED wall whose left face is at x = 1.30 (cells i >= 13). A start centred
// at x = 1.25, theta = 0 has its footprint [1.15, 1.35] penetrating the wall face by 0.05 m. The goal
// sits behind the robot in free space.

namespace {

constexpr int ESCAPE_COLS = 60, ESCAPE_ROWS = 30;
constexpr int WALL_C0 = 13, WALL_C1 = 20;      // x in [1.30, 2.10): footprint front edge (1.35) enters col 13
const SE2State ESCAPE_GOAL{0.5, 1.5, 0.0};     // behind the robot, in free space

// Free grid with a single full-height OCCUPIED wall column band.
std::vector<int8_t> escapeWallOcc(){
    std::vector<int8_t> occ(static_cast<size_t>(ESCAPE_COLS) * ESCAPE_ROWS, 0);
    fillBlock(occ, ESCAPE_COLS, WALL_C0, 0, WALL_C1, ESCAPE_ROWS - 1, 100);  // full-height wall
    return occ;
}

// A collision oracle over the built snapshot, independent of the planner.
CollisionChecker makeOracle(const BuiltScenario & b){
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(b.snapshot, &b.snapshot->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(b.snapshot, &b.snapshot->edt_);
    return CollisionChecker(grid_ptr, edt_ptr, b.robot);
}

}  // namespace

TEST(HybridAstarStartGate, startFootprintOverlappingWallReturnsStartBlocked){
    auto b = build(ESCAPE_COLS, ESCAPE_ROWS, escapeWallOcc(), baseCfg());
    const SE2State start{1.25, 1.5, 0.0};

    CollisionChecker oracle = makeOracle(b);
    ASSERT_FALSE(oracle.checkState(start)) << "fixture error: start footprint must overlap the wall";
    ASSERT_TRUE(oracle.checkState(ESCAPE_GOAL)) << "fixture error: goal must be collision-free";

    const auto r = b.planner->plan(start, ESCAPE_GOAL);
    EXPECT_EQ(r.outcome, PlanOutcome::StartBlocked);
    EXPECT_FALSE(r.path.has_value());
    EXPECT_EQ(dynamic_cast<const HybridAstar *>(b.planner.get())->lastPlanStats().expansions, 0);
}

// With the start clear of the wall on the same map, planning succeeds.
TEST(HybridAstarStartGate, startClearPlanUnaffected){
    auto cfg = baseCfg();
    auto b = build(ESCAPE_COLS, ESCAPE_ROWS, escapeWallOcc(), cfg);
    const SE2State start{0.8, 1.5, 0.0};  // footprint [0.7, 0.9] — clear of the wall face at 1.30

    CollisionChecker oracle = makeOracle(b);
    ASSERT_TRUE(oracle.checkState(start)) << "fixture error: this start must be collision-free";

    auto path = b.planner->plan(start, ESCAPE_GOAL).path;
    ASSERT_TRUE(path.has_value()) << "a clear start behind the wall must plan normally";
    EXPECT_NEAR(path->node_points.front().x, start.x, 1e-9);
    EXPECT_TRUE(withinGoalTol(path->node_points.back(), ESCAPE_GOAL, cfg));
    // No pose along a clear-start plan is ever in collision.
    for (const auto & np : path->node_points){
        EXPECT_TRUE(oracle.checkState(np))
            << "clear-start plan must have no in-collision poses at (" << np.x << ", " << np.y << ")";
    }
}

// ---- PlanStats (HybridAstar::lastPlanStats) ---------------------------------------------------
// Pins: per-candidate verdict counters partition the candidate motions of every expanded node. The
// control set has P = 12 primitives (see ControlSet.hasTwelveControls in build_diff_prim_lib_test.cpp).
// A successful shot-off plan pops the goal node (counted in expansions) and returns before generating
// its children, hence sum + duplicate_skipped == P * (expansions - 1); a plan cut off by the budget
// expands every counted node fully, hence sum + duplicate_skipped == P * expansions.

namespace {

constexpr std::int64_t kNumPrims = 12;

struct CountingObserver : SearchObserver {
    std::int64_t n = 0;
    void onExpand(const SearchExpansion &) override { ++n; }
};

const HybridAstar & concrete(const BuiltScenario & b){
    const auto * h = dynamic_cast<const HybridAstar *>(b.planner.get());
    EXPECT_NE(h, nullptr);
    return *h;
}

std::int64_t verdictSum(const PlanStats & s){
    return s.fast_accept + s.fast_reject + s.scan_clear + s.scan_blocked;
}

AStarConfig shotOffCfg(){
    auto cfg = baseCfg();
    cfg.enable_shot = false;
    return cfg;
}

void expectStatsEqual(const PlanStats & a, const PlanStats & b){
    EXPECT_EQ(a.expansions, b.expansions);
    EXPECT_EQ(a.fast_accept, b.fast_accept);
    EXPECT_EQ(a.fast_reject, b.fast_reject);
    EXPECT_EQ(a.scan_clear, b.scan_clear);
    EXPECT_EQ(a.scan_blocked, b.scan_blocked);
    EXPECT_EQ(a.exact_accept, b.exact_accept);
    EXPECT_EQ(a.exact_reject, b.exact_reject);
    EXPECT_EQ(a.duplicate_skipped, b.duplicate_skipped);
}

void expectAllZero(const PlanStats & s){ expectStatsEqual(s, PlanStats{}); }

}  // namespace

TEST(HybridAstarPlanStats, allZeroBeforeAnyPlan){
    auto b = buildFree(60, 60, shotOffCfg());
    expectAllZero(concrete(b).lastPlanStats());
}

TEST(HybridAstarPlanStats, freeMapSuccessPartitionsCandidates){
    auto b = buildFree(60, 60, shotOffCfg());
    auto obs = std::make_shared<CountingObserver>();
    auto path = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.8, 1.0, 0.0}, obs).path;
    ASSERT_TRUE(path.has_value());
    const PlanStats s = concrete(b).lastPlanStats();
    ASSERT_GT(s.expansions, 1);
    EXPECT_EQ(verdictSum(s) + s.duplicate_skipped, kNumPrims * (s.expansions - 1));
    EXPECT_EQ(s.scan_blocked, 0);
    EXPECT_EQ(s.fast_reject, 0);
    EXPECT_EQ(s.exact_accept, 0);
    EXPECT_EQ(s.exact_reject, 0);
    EXPECT_EQ(s.expansions, obs->n);
}

TEST(HybridAstarPlanStats, budgetCutoffCountsEveryExpandedNodeFully){
    auto cfg = shotOffCfg();
    cfg.max_expansions = 5;
    auto b = buildFree(60, 60, cfg);
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{5.0, 1.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::BudgetExhausted);
    EXPECT_FALSE(r.path.has_value());
    const PlanStats s = concrete(b).lastPlanStats();
    EXPECT_EQ(s.expansions, 5);
    EXPECT_EQ(verdictSum(s) + s.duplicate_skipped, kNumPrims * s.expansions);
}

TEST(HybridAstarPlanStats, rejectedGoalLeavesAllZeroAndResetsPriorStats){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 38, 8, 42, 12, 100);  // block around goal cell (40,10)
    auto b = build(60, 60, std::move(occ), shotOffCfg());
    const SE2State start{1.0, 1.0, 0.0};

    EXPECT_EQ(b.planner->plan(start, SE2State{4.0, 1.0, 0.0}).outcome, PlanOutcome::GoalBlocked);
    expectAllZero(concrete(b).lastPlanStats());

    // Prior successful plan (goal clear of the block), then a rejected one: stats must be reset.
    ASSERT_TRUE(b.planner->plan(start, SE2State{2.8, 1.0, 0.0}).path.has_value());
    ASSERT_GT(concrete(b).lastPlanStats().expansions, 0);
    EXPECT_EQ(b.planner->plan(start, SE2State{4.0, 1.0, 0.0}).outcome, PlanOutcome::GoalBlocked);
    expectAllZero(concrete(b).lastPlanStats());
}

TEST(HybridAstarPlanStats, observerDoesNotChangePathOrStats){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 18, 11, 20, 14, 100);
    const SE2State start{1.0, 1.0, 0.0}, goal{2.8, 1.0, 0.0};

    auto bPlain = build(60, 60, occ, shotOffCfg());
    auto bObs   = build(60, 60, occ, shotOffCfg());
    const auto plain_result = bPlain.planner->plan(start, goal);
    const auto watched_result = bObs.planner->plan(start, goal, std::make_shared<CountingObserver>());
    EXPECT_EQ(plain_result.outcome, watched_result.outcome);
    const auto & plain = plain_result.path;
    const auto & watched = watched_result.path;
    ASSERT_TRUE(plain.has_value());
    ASSERT_TRUE(watched.has_value());

    ASSERT_EQ(plain->node_points.size(), watched->node_points.size());
    for (std::size_t i = 0; i < plain->node_points.size(); ++i){
        EXPECT_EQ(plain->node_points[i].x, watched->node_points[i].x);
        EXPECT_EQ(plain->node_points[i].y, watched->node_points[i].y);
        EXPECT_EQ(plain->node_points[i].theta, watched->node_points[i].theta);
    }
    EXPECT_EQ(plain->total_cost, watched->total_cost);
    expectStatsEqual(concrete(bPlain).lastPlanStats(), concrete(bObs).lastPlanStats());
}

TEST(HybridAstarPlanStats, nearbyObstacleTriggersScansAndIdentityHolds){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 18, 11, 20, 14, 100);  // 0.1-0.4 m off the y=1.0 route
    auto b = build(60, 60, std::move(occ), shotOffCfg());
    ASSERT_TRUE(b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.8, 1.0, 0.0}).path.has_value());
    const PlanStats s = concrete(b).lastPlanStats();
    EXPECT_GT(s.scan_clear + s.scan_blocked, 0);
    ASSERT_GT(s.expansions, 1);
    EXPECT_EQ(verdictSum(s) + s.duplicate_skipped, kNumPrims * (s.expansions - 1));
}

// ---- Exact re-check of scan-blocked motions ---------------------------------------------------
// A motion the cached swept cells flag as blocked is pushed unconfirmed and re-checked over the exact
// cover of the motion only if taken for expansion, so the exact counters are bounded by scan_blocked.

TEST(HybridAstarPlanStats, exactCountersBoundedByScanBlocked){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 18, 11, 20, 14, 100);
    auto b = build(60, 60, std::move(occ), shotOffCfg());
    ASSERT_TRUE(b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.8, 1.0, 0.0}).path.has_value());
    const PlanStats s = concrete(b).lastPlanStats();
    ASSERT_GT(s.scan_blocked, 0) << "fixture must produce scan-blocked candidates";
    ASSERT_GT(s.exact_accept, 0) << "fixture must pop unconfirmed nodes that are exactly clear";
    ASSERT_GT(s.exact_reject, 0) << "fixture must pop unconfirmed nodes that really collide";
    EXPECT_LE(s.exact_accept + s.exact_reject, s.scan_blocked);
    EXPECT_LE(s.exact_accept, s.expansions);
    ASSERT_GT(s.expansions, 1);
    EXPECT_EQ(verdictSum(s) + s.duplicate_skipped, kNumPrims * (s.expansions - 1));
}

namespace {

struct RecordingObserver : SearchObserver {
    std::vector<SearchExpansion> events;
    void onExpand(const SearchExpansion & e) override { events.push_back(e); }
};

// Independent exact-cover check of every segment of `path` against the built snapshot.
bool segmentsExactlyClear(const BuiltScenario & b, const Path & path, bool reject_unknown){
    const KinematicModel km = buildKinematicModel(b.robot, makeKin(), CELL);
    const CollisionChecker checker = makeOracle(b);
    for (std::size_t i = 0; i < path.segments.size(); ++i){
        const auto cells = km.motion->computeExactCover(path.node_points[i], path.segments[i].u).cells;
        if (!checker.checkCells(cells, reject_unknown)) return false;
    }
    return true;
}

void expectPathsIdentical(const Path & a, const Path & b){
    ASSERT_EQ(a.node_points.size(), b.node_points.size());
    for (std::size_t i = 0; i < a.node_points.size(); ++i){
        EXPECT_EQ(a.node_points[i].x, b.node_points[i].x);
        EXPECT_EQ(a.node_points[i].y, b.node_points[i].y);
        EXPECT_EQ(a.node_points[i].theta, b.node_points[i].theta);
    }
    EXPECT_EQ(a.total_cost, b.total_cost);
}

std::vector<int8_t> scanBlockedOcc(){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 18, 11, 20, 14, 100);
    return occ;
}

}  // namespace

TEST(HybridAstarPlanStats, duplicateSkippedIsPositiveOnARevisitingSearch){
    auto b = buildFree(60, 60, shotOffCfg());
    ASSERT_TRUE(b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.8, 1.0, 0.0}).path.has_value());
    const PlanStats s = concrete(b).lastPlanStats();
    ASSERT_GT(s.expansions, 1);
    EXPECT_GT(s.duplicate_skipped, 0);
}

TEST(HybridAstarUnconfirmedNodes, observerDoesNotChangePathOrStatsAndSeesOneEventPerExpansion){
    const SE2State start{1.0, 1.0, 0.0}, goal{2.8, 1.0, 0.0};
    auto bPlain = build(60, 60, scanBlockedOcc(), shotOffCfg());
    auto bObs   = build(60, 60, scanBlockedOcc(), shotOffCfg());
    auto rec = std::make_shared<RecordingObserver>();
    auto plain = bPlain.planner->plan(start, goal).path;
    auto watched = bObs.planner->plan(start, goal, rec).path;
    ASSERT_TRUE(plain.has_value());
    ASSERT_TRUE(watched.has_value());
    ASSERT_GT(concrete(bPlain).lastPlanStats().scan_blocked, 0) << "fixture must produce scan-blocked candidates";
    expectPathsIdentical(*plain, *watched);
    expectStatsEqual(concrete(bPlain).lastPlanStats(), concrete(bObs).lastPlanStats());
    EXPECT_EQ(static_cast<std::int64_t>(rec->events.size()), concrete(bObs).lastPlanStats().expansions);
}

TEST(HybridAstarUnconfirmedNodes, everyObservedMotionIsExactlyClear){
    auto b = build(60, 60, scanBlockedOcc(), shotOffCfg());
    auto rec = std::make_shared<RecordingObserver>();
    ASSERT_TRUE(b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.8, 1.0, 0.0}, rec).path.has_value());
    ASSERT_GT(concrete(b).lastPlanStats().scan_blocked, 0);
    ASSERT_GT(concrete(b).lastPlanStats().exact_reject, 0)
        << "fixture must pop colliding unconfirmed nodes, else the pop-time check is untested";
    const KinematicModel km = buildKinematicModel(b.robot, makeKin(), CELL);
    const CollisionChecker checker = makeOracle(b);
    const auto & pool = concrete(b).nodePool();
    const auto & prims = km.prim_lib->primitives();
    std::size_t checked = 0;
    for (const auto & e : rec->events){
        if (e.parent_id < 0) continue;
        const Node & child = pool[static_cast<std::size_t>(e.id)];
        const auto cells = km.motion->computeExactCover(pool[static_cast<std::size_t>(e.parent_id)].state,
                                                        prims[static_cast<std::size_t>(child.prim_idx)].control).cells;
        EXPECT_TRUE(checker.checkCells(cells, false)) << "observed node id " << e.id;
        ++checked;
    }
    EXPECT_GT(checked, 0u);
}

TEST(HybridAstarDeterminism, repeatedAndInterleavedPlansOnASymmetricMapAreIdentical){
    auto b = buildFree(60, 60, shotOffCfg());
    const SE2State start{1.0, 3.0, 0.0}, goal{3.0, 3.0, 0.0};   // on the y = 3.0 axis of symmetry
    auto first = b.planner->plan(start, goal).path;
    auto second = b.planner->plan(start, goal).path;
    ASSERT_TRUE(b.planner->plan(start, SE2State{2.0, 4.0, 0.0}).path.has_value());
    auto third = b.planner->plan(start, goal).path;
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_TRUE(third.has_value());
    expectPathsIdentical(*first, *second);
    expectPathsIdentical(*first, *third);
}

TEST(HybridAstarShot, firstExpansionShotIsFoundAndExactlyClear){
    auto b = buildFree(60, 60, baseCfg());
    auto path = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{3.0, 1.0, 0.0}).path;
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(concrete(b).lastPlanStats().expansions, 1);
    EXPECT_TRUE(segmentsExactlyClear(b, *path, !baseCfg().shot_allow_unknown));
}

TEST(HybridAstarShot, shotThroughObstacleNeverReturnsAnUnclearSegment){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 19, 5, 21, 15, 100);   // straight line y = 1.0 from x = 1 to 3 crosses it; robot can detour above
    auto b = build(60, 60, std::move(occ), baseCfg());
    auto path = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{3.0, 1.0, 0.0}).path;
    ASSERT_GT(concrete(b).lastPlanStats().expansions, 1) << "the direct shot must have been refused";
    ASSERT_TRUE(path.has_value());
    EXPECT_TRUE(segmentsExactlyClear(b, *path, !baseCfg().shot_allow_unknown));
}

namespace {

// Shot from (1.0, 1.0) to (3.0, 1.0), both heading +x, past a block of occupied cells whose lowest row
// is `block_row` (cells 0.1 m, robot half-width 0.1 m: the footprint top edge is y = 1.1, the block's
// nearest edge is y = 0.1 * block_row). Returns {motion clear by the independent check, plan expansions}.
struct ShotBeside {
    bool independently_clear;
    std::int64_t expansions;
    bool found;
    bool path_clear;
};
ShotBeside shotBesideBlock(int block_row){
    std::vector<int8_t> occ(60 * 60, 0);
    fillBlock(occ, 60, 15, block_row, 25, block_row + 5, 100);
    auto b = build(60, 60, std::move(occ), baseCfg());
    const SE2State start{1.0, 1.0, 0.0}, goal{3.0, 1.0, 0.0};
    const KinematicModel km = buildKinematicModel(b.robot, makeKin(), CELL);
    const double d = goal.x - start.x, vmax = makeKin().vmax;
    const auto cells = km.motion->computeExactCover(start, Control{vmax, 0.0, d / vmax}).cells;
    ShotBeside r{};
    r.independently_clear = makeOracle(b).checkCells(cells, !baseCfg().shot_allow_unknown);
    auto path = b.planner->plan(start, goal).path;
    r.expansions = concrete(b).lastPlanStats().expansions;
    r.found = path.has_value();
    r.path_clear = !r.found || segmentsExactlyClear(b, *path, !baseCfg().shot_allow_unknown);
    return r;
}

}  // namespace

// Footprint top edge y = 1.1, block bottom edge y = 1.2: gap 0.1 m = 1 cell. Clear, so the shot is taken.
TEST(HybridAstarShot, shotPassingCloseToAnObstacleIsStillTaken){
    const ShotBeside r = shotBesideBlock(12);
    ASSERT_TRUE(r.independently_clear);
    EXPECT_TRUE(r.found);
    EXPECT_EQ(r.expansions, 1);
    EXPECT_TRUE(r.path_clear);
}

// Block bottom edge y = 1.1: gap 0, the footprint touches the block. Colliding, so the shot is refused.
TEST(HybridAstarShot, shotGrazingAnObstacleIsRefused){
    const ShotBeside r = shotBesideBlock(11);
    ASSERT_FALSE(r.independently_clear);
    EXPECT_GT(r.expansions, 1);
    EXPECT_TRUE(r.path_clear);
}

namespace {

// Planner over an arbitrary resolution, kinematics and body, shot and heuristic as given.
BuiltScenario buildCustom(const GridMetaData & gm, std::vector<int8_t> occ, const DiffDriveParams & kin,
                          const std::vector<Vec2> & verts, AStarConfig cfg){
    ScenarioSpec spec{"search_test_custom", gm, std::move(occ)};
    spec.robot_vertices = verts;
    spec.diff_drive = kin;
    spec.a_star = cfg;
    spec.heuristic = HeuristicConfig{1.0, false};
    spec.cost_policy = basePolicy();
    return buildScenario(spec);
}

const std::vector<Vec2> kSquare{Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};

// Plain full-footprint checker over the built snapshot.
CollisionChecker plainChecker(const BuiltScenario & b){
    return makeOracle(b);
}

}  // namespace

// ---- Goal reachable by a straight drive with the shot off ------------------------------

namespace {
void expectStraightGoalReached(const DiffDriveParams & kin, const SE2State & start, double ahead){
    const GridMetaData gm{100, 100, CELL, Eigen::Vector2f::Zero()};   // 10 m square
    auto cfg = baseCfg();
    cfg.enable_shot = false;
    cfg.max_expansions = 200000;
    auto b = buildCustom(gm, std::vector<int8_t>(100 * 100, 0), kin, kSquare, cfg);
    const SE2State goal{start.x + ahead * std::cos(start.theta), start.y + ahead * std::sin(start.theta), start.theta};
    auto path = b.planner->plan(start, goal).path;
    ASSERT_TRUE(path.has_value()) << "no path; expansions=" << concrete(b).lastPlanStats().expansions;
    EXPECT_TRUE(withinGoalTol(path->node_points.back(), goal, cfg));
}
DiffDriveParams goalAheadKin(double rmin, double vmax, double dt){
    DiffDriveParams k; k.rmin = rmin; k.vmax = vmax; k.atomic_time = dt; k.n_headings = 16; return k;
}
}  // namespace

TEST(HybridAstarGoalReachability, goalStraightAheadShotOffA){
    expectStraightGoalReached(goalAheadKin(0.3, 0.5, 0.6), SE2State{4.29, 3.89, -2.85}, 0.6);
}
TEST(HybridAstarGoalReachability, goalStraightAheadShotOffB){
    expectStraightGoalReached(goalAheadKin(0.3, 0.5, 0.6), SE2State{3.07, 3.59, -1.7726}, 0.9);
}
TEST(HybridAstarGoalReachability, goalStraightAheadShotOffRmin05KinematicsA){
    expectStraightGoalReached(goalAheadKin(0.5, 1.0, 0.3), SE2State{4.29, 3.89, -2.85}, 0.6);
}
TEST(HybridAstarGoalReachability, goalStraightAheadShotOffRmin05KinematicsB){
    expectStraightGoalReached(goalAheadKin(0.5, 1.0, 0.3), SE2State{3.07, 3.59, -1.7726}, 0.9);
}

// ---- Narrow doorway: a body 0.20 m wide through a 0.30 m door ---------------------------------

namespace {
struct Doorway {
    BuiltScenario b;
    AStarConfig cfg;
};
Doorway buildDoorway(bool shot){
    constexpr double H = 0.05;
    constexpr int COLS = 120, ROWS = 80;
    const GridMetaData gm{COLS, ROWS, H, Eigen::Vector2f::Zero()};
    std::vector<int8_t> occ(static_cast<size_t>(COLS) * ROWS, 0);
    fillBlock(occ, COLS, 59, 0, 60, ROWS - 1, 100);       // wall x in [2.95, 3.05), 2 cells thick
    fillBlock(occ, COLS, 59, 37, 60, 42, 0);              // door: 6 cells, y in [1.85, 2.15) = 0.30 m
    auto cfg = baseCfg();
    cfg.enable_shot = shot;
    cfg.max_expansions = 200000;
    return Doorway{buildCustom(gm, std::move(occ), makeKin(), kSquare, cfg), cfg};
}
const SE2State DOOR_START{2.35, 2.0, 0.0}, DOOR_GOAL{3.65, 2.0, 0.0};
}  // namespace

TEST(HybridAstarNarrowDoorway, shotOffPassesWithExactRecheckAndEverySamplePoseClear){
    Doorway d = buildDoorway(false);
    CollisionChecker plain = plainChecker(d.b);
    ASSERT_TRUE(plain.checkState(DOOR_START));
    ASSERT_TRUE(plain.checkState(DOOR_GOAL));
    auto path = d.b.planner->plan(DOOR_START, DOOR_GOAL).path;
    ASSERT_TRUE(path.has_value()) << "expansions=" << concrete(d.b).lastPlanStats().expansions;
    EXPECT_GT(concrete(d.b).lastPlanStats().exact_accept, 0);
    EXPECT_TRUE(withinGoalTol(path->node_points.back(), DOOR_GOAL, d.cfg));
    for (size_t i = 0; i < path->segments.size(); ++i)
        for (const auto & p : path->segments[i].samples_poses)
            EXPECT_TRUE(plain.checkState(p)) << "segment " << i << " pose (" << p.x << ", " << p.y << ", " << p.theta << ")";
}

TEST(HybridAstarNarrowDoorway, shotOnReturnsPath){
    Doorway d = buildDoorway(true);
    EXPECT_TRUE(d.b.planner->plan(DOOR_START, DOOR_GOAL).path.has_value());
}

// ---- Boxed-in start ---------------------------------------------------------------------------

TEST(HybridAstarBoxedInStart, collisionFreeStartInsideClosedRingReturnsNoPath){
    constexpr int COLS = 60, ROWS = 40;
    std::vector<int8_t> occ(static_cast<size_t>(COLS) * ROWS, 0);
    fillBlock(occ, COLS, 16, 16, 25, 25, 100);            // ring outer cells 16..25 (2 thick)
    fillBlock(occ, COLS, 18, 18, 23, 23, 0);              // interior cells 18..23 = 0.6 m square
    auto cfg = shotOffCfg();
    cfg.max_expansions = 3000;
    auto b = build(COLS, ROWS, std::move(occ), cfg);
    const SE2State start{2.1, 2.1, 0.0};
    ASSERT_TRUE(plainChecker(b).checkState(start)) << "fixture: start must be collision-free";
    const auto r = b.planner->plan(start, SE2State{5.0, 2.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::NoPath);
    EXPECT_FALSE(r.path.has_value());
}

// ---- PlanOutcome: stop during the search and gate precedence -----------------------------------

namespace {
struct StopAfterFirstExpansion : SearchObserver {
    std::stop_source ss;
    void onExpand(const SearchExpansion &) override { ss.request_stop(); }
};
}  // namespace

TEST(HybridAstarOutcome, stopDuringSearchReturnsStopped){
    // The token is polled every 1024 loop iterations, so the search must outlast that: the goal
    // (opposite heading) takes thousands of expansions to reach.
    auto b = buildFree(60, 60, shotOffCfg());
    auto obs = std::make_shared<StopAfterFirstExpansion>();
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{5.0, 5.0, 3.0}, obs, obs->ss.get_token());
    EXPECT_EQ(r.outcome, PlanOutcome::Stopped);
    EXPECT_FALSE(r.path.has_value());
    EXPECT_GE(concrete(b).lastPlanStats().expansions, 1);
}

TEST(HybridAstarOutcome, stopAtEntryWithBlockedStartReturnsStopped){
    auto b = build(30, 30, std::vector<int8_t>(900, 100), baseCfg());
    std::stop_source ss;
    ss.request_stop();
    const auto r = b.planner->plan(SE2State{1.0, 1.0, 0.0}, SE2State{2.0, 1.0, 0.0}, nullptr, ss.get_token());
    EXPECT_EQ(r.outcome, PlanOutcome::Stopped);
    EXPECT_FALSE(r.path.has_value());
}

TEST(HybridAstarOutcome, clearStartBlockedGoalZeroBudgetReturnsGoalBlocked){
    std::vector<int8_t> occ(900, 0);
    fillBlock(occ, 30, 18, 8, 22, 12, 100);
    auto cfg = baseCfg();
    cfg.max_plan_time_sec = 0.0;
    auto b = build(30, 30, std::move(occ), cfg);
    const auto r = b.planner->plan(SE2State{0.5, 1.0, 0.0}, SE2State{2.0, 1.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::GoalBlocked);
    EXPECT_FALSE(r.path.has_value());
}

// ---- Near-wall start gate: a wall just beyond the front footprint edge --------------------------
// Body 0.2 m square (front edge at start.x + 0.1), h = 0.05. An OCCUPIED wall fills columns >= 44
// (face at x = 44*h = 2.20, first wall cell centre at 2.225). The start is placed at
// x = 2.20 - 0.10 - g so the face is exactly g ahead of the front edge; the wall cell centre is
// g + h/2 from the edge line. The gate clears the start iff that exceeds (sqrt2/2 + 1/2)h = 1.207h,
// i.e. g > 0.707h. The goal is 0.6 m straight behind, shot off.

namespace {
struct NearWall {
    BuiltScenario b;
    SE2State start;
};
NearWall buildNearWall(double gap_in_h){
    constexpr int N = 80;
    constexpr double h = 0.05;
    std::vector<int8_t> occ(static_cast<size_t>(N) * N, 0);
    fillBlock(occ, N, 44, 0, N - 1, N - 1, 100);
    ScenarioSpec spec{"near_wall", GridMetaData{N, N, h, Eigen::Vector2f::Zero()}, std::move(occ)};
    spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
    spec.diff_drive = makeKin();
    spec.a_star = baseCfg();
    spec.a_star.enable_shot = false;
    spec.heuristic = HeuristicConfig{1.0, false};
    spec.cost_policy = basePolicy();
    return {buildScenario(spec), SE2State{44 * h - 0.1 - gap_in_h * h, 2.0, 0.0}};
}
}  // namespace

TEST(HybridAstarNearWall, gap075hIsFound){
    auto w = buildNearWall(0.75);
    const auto r = w.b.planner->plan(w.start, SE2State{w.start.x - 0.6, 2.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::Found) << "startClear=" << makeOracle(w.b).startClear(w.start);
    EXPECT_TRUE(r.path.has_value());
}

TEST(HybridAstarNearWall, gap070hIsStartBlockedWithZeroExpansions){
    auto w = buildNearWall(0.70);
    const auto r = w.b.planner->plan(w.start, SE2State{w.start.x - 0.6, 2.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::StartBlocked) << "startClear=" << makeOracle(w.b).startClear(w.start);
    EXPECT_FALSE(r.path.has_value());
    EXPECT_EQ(concrete(w.b).lastPlanStats().expansions, 0);
}

TEST(HybridAstarNearWall, gap020hIsStartBlockedWithZeroExpansions){
    auto w = buildNearWall(0.20);
    const auto r = w.b.planner->plan(w.start, SE2State{w.start.x - 0.6, 2.0, 0.0});
    EXPECT_EQ(r.outcome, PlanOutcome::StartBlocked) << "startClear=" << makeOracle(w.b).startClear(w.start);
    EXPECT_FALSE(r.path.has_value());
    EXPECT_EQ(concrete(w.b).lastPlanStats().expansions, 0);
}
