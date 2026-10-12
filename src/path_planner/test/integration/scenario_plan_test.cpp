#include <gtest/gtest.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_loader.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/planner_interface.hpp"
#include "core/planner/path.hpp"
#include "core/planner/hybrid_astar/search.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/types/state.hpp"
#include "core/types/math/wrap_angle.hpp"

/*
Data-driven whole-planner integration tier.

One ctest case is produced PER scenario YAML in TEST_SCENARIO_CORPUS_DIR (a PRIVATE compile define
owned by this test target; the core library is scenario-agnostic). The corpus is runtime-globbed by
discoverScenarioFiles at instantiation, so dropping a new .yaml into the corpus dir is picked up with
no CMake reconfigure. Each case is named by its scenario
(spec.name / file stem) so a failure reads `ScenarioPlan/ScenarioPlanTest.PlansCorrectly/enclosed_box`.

The oracle is layered:
  - Tier-0 universal invariants in assertPlanValid() below, written ONCE and applied to every
    scenario. These need no per-scenario authoring; they are the truths that hold for any feasible
    hybrid-A* output (collision-free, structural soundness, cost-sum, goal-tol-when-found,
    determinism, expansion budget).
  - Tier-1 coarse design-intent scalars from the YAML expectations: block (expect_found + loose
    cost/expansion ceilings). These are author-chosen bounds, never the planner's own dumped output.

Empty-corpus guard: a separate non-parameterized test FAILS if discovery returns nothing, so a
missing/empty corpus is a hard red rather than a vacuous green (no scenarios -> zero parameterized
cases would otherwise look passing).
*/

namespace {

// ---- Corpus discovery (evaluated once at static-init for INSTANTIATE_TEST_SUITE_P) --------------

const std::vector<std::string> & corpusFiles() {
    static const std::vector<std::string> files = discoverScenarioFiles(TEST_SCENARIO_CORPUS_DIR);
    return files;
}

// ---- Tier-0 universal invariants ---------------------------------------------------------------

// Propagation-only state space rebuilt from the spec; propagate() depends solely on the control and
// the start heading, so the workspace bounds / distance functor here are inert for reconnection.
DiffDriveStateSpace makePropagationSpace(const ScenarioSpec & spec) {
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(spec.diff_drive.rmin *
                                                                   spec.diff_drive.rmin);
    return DiffDriveStateSpace(spec.meta, std::move(dist));
}

bool withinGoalTol(const SE2State & a, const SE2State & goal, const AStarConfig & c) {
    const double dx = goal.x - a.x, dy = goal.y - a.y;
    return (dx * dx + dy * dy) <= c.goal_pos_tol * c.goal_pos_tol + 1e-9 &&
           std::abs(wrapToPi(goal.theta - a.theta)) <= c.goal_heading_tol + 1e-9;
}

// Re-plan the same query and compare bit-exact. plan() resets all internal search state at entry, so a
// second call on the same instance must reproduce the first result exactly. Kept here (not inlined)
// so the determinism assertion reads as one named invariant.
void assertDeterministic(BuiltScenario & built, const ScenarioSpec & spec,
                         const std::optional<Path> & first) {
    auto second = built.planner->plan(spec.start, spec.goal).path;
    ASSERT_EQ(first.has_value(), second.has_value()) << "replan changed found/not-found";
    if (!first.has_value()) return;
    ASSERT_EQ(first->node_points.size(), second->node_points.size());
    EXPECT_DOUBLE_EQ(first->total_cost, second->total_cost);
    for (size_t i = 0; i < first->node_points.size(); ++i) {
        EXPECT_DOUBLE_EQ(first->node_points[i].x, second->node_points[i].x) << "node " << i;
        EXPECT_DOUBLE_EQ(first->node_points[i].y, second->node_points[i].y) << "node " << i;
        EXPECT_DOUBLE_EQ(first->node_points[i].theta, second->node_points[i].theta) << "node " << i;
    }
}

// The reusable oracle: asserts every Tier-0 invariant that must hold for ANY scenario, then the
// Tier-1 design-intent expectations from the YAML. `result` is the path from plan(spec.start,
// spec.goal); `built` is held by the caller (it aliases the snapshot) and is replanned here for the
// determinism check.
void assertPlanValid(const ScenarioSpec & spec, BuiltScenario & built,
                     const std::optional<Path> & result) {
    // --- Tier-1: existence matches design intent ---
    EXPECT_EQ(result.has_value(), spec.expect.expect_found)
        << "scenario '" << spec.name << "' expect_found=" << spec.expect.expect_found
        << " but planner " << (result.has_value() ? "found" : "did not find") << " a path";

    // nodePool() is the record of every GENERATED node from the last plan(). NOTE: it is deliberately
    // NOT asserted against a_star.max_expansions — that cap bounds POPPED nodes (`expansions` in the
    // search loop), and each pop generates many children, so |nodePool| routinely exceeds
    // max_expansions even on a successful, in-budget plan. The only budget assertion made here is the
    // optional, author-chosen Tier-1 expansions_max ceiling below (a loose upper bound on search
    // effort, not a hard invariant). hybrid is fetched once for that and for the determinism replan.
    const auto * hybrid = dynamic_cast<const HybridAstar *>(built.planner.get());
    ASSERT_NE(hybrid, nullptr) << "buildScenario must produce a HybridAstar for nodePool() inspection";

    if (!result.has_value()) {
        // Determinism still applies to the not-found verdict.
        assertDeterministic(built, spec, result);
        return;
    }

    const Path & path = *result;
    const OccupancyGrid & grid = built.snapshot->grid_;
    const GridMetaData & gmeta = grid.getMetadata();

    // --- Tier-0: structural soundness ---
    ASSERT_EQ(path.node_points.size(), path.segments.size() + 1)
        << "node_points.size() must equal segments.size() + 1";
    ASSERT_FALSE(path.node_points.empty());

    // Start node is the requested start pose (exact: the planner seeds it directly).
    EXPECT_NEAR(path.node_points.front().x, spec.start.x, 1e-9);
    EXPECT_NEAR(path.node_points.front().y, spec.start.y, 1e-9);
    EXPECT_NEAR(wrapToPi(path.node_points.front().theta - spec.start.theta), 0.0, 1e-9);

    // --- Tier-0: reconnection — re-propagating each segment's control reconnects to the next node ---
    DiffDriveStateSpace space = makePropagationSpace(spec);
    for (size_t i = 0; i < path.segments.size(); ++i) {
        SE2State re = space.propagate(path.node_points[i], path.segments[i].u);
        const SE2State & nxt = path.node_points[i + 1];
        EXPECT_NEAR(re.x, nxt.x, 1e-6) << "segment " << i << " reconnection (x)";
        EXPECT_NEAR(re.y, nxt.y, 1e-6) << "segment " << i << " reconnection (y)";
        EXPECT_NEAR(wrapToPi(re.theta - nxt.theta), 0.0, 1e-6) << "segment " << i << " (theta)";
    }

    // --- Tier-0: cost consistency — total_cost == sum of segment costs; each non-negative ---
    double sum = 0.0;
    for (const auto & seg : path.segments) {
        EXPECT_GE(seg.cost, -1e-9) << "segment cost must be non-negative";
        sum += seg.cost;
    }
    EXPECT_NEAR(path.total_cost, sum, 1e-6) << "total_cost must equal the segment cost sum";

    // --- Tier-0: collision-free — no sampled pose center lands in an OCCUPIED cell ---
    for (const auto & seg : path.segments) {
        for (const auto & p : seg.samples_poses) {
            EXPECT_NE(grid.findCellStatus(gmeta.worldToGrid(p.x, p.y)), CellState::OCCUPIED)
                << "sampled pose (" << p.x << ", " << p.y << ") lands in an OCCUPIED cell";
        }
    }

    // --- Tier-0: full-footprint collision-free — every sampled pose passes the plain footprint check. ---
    {
        auto grid_ptr = std::shared_ptr<const OccupancyGrid>(built.snapshot, &built.snapshot->grid_);
        auto edt_ptr  = std::shared_ptr<const EDT>(built.snapshot, &built.snapshot->edt_);
        const CollisionChecker plain(grid_ptr, edt_ptr, built.robot);
        for (size_t i = 0; i < path.segments.size(); ++i)
            for (const auto & p : path.segments[i].samples_poses)
                EXPECT_TRUE(plain.checkState(p))
                    << "segment " << i << " pose (" << p.x << ", " << p.y << ", " << p.theta
                    << ") fails the plain footprint check";
    }

    // --- Tier-0: goal attainment within the configured tolerances ---
    EXPECT_TRUE(withinGoalTol(path.node_points.back(), spec.goal, spec.a_star))
        << "final pose not within goal tolerance";

    // --- Tier-1: loose design-intent ceilings, only when authored ---
    if (spec.expect.cost_max.has_value()) {
        EXPECT_LE(path.total_cost, *spec.expect.cost_max) << "total_cost exceeds the authored ceiling";
    }
    if (spec.expect.expansions_max.has_value()) {
        EXPECT_LE(static_cast<int64_t>(hybrid->nodePool().size()),
                  static_cast<int64_t>(*spec.expect.expansions_max))
            << "node pool exceeds the authored expansions ceiling";
    }

    if (spec.expect.unknown_poses_max.has_value()) {
        const GridMetaData gm = built.snapshot->grid_.getMetadata();
        int count = 0;
        for (const auto & seg : path.segments)
            for (const auto & p : seg.samples_poses) {
                const GridCoords c = gm.worldToGrid(p.x, p.y);
                if (!gm.inBounds(c) || built.snapshot->grid_.findCellStatus(c) == CellState::UNKNOWN) ++count;
            }
        EXPECT_LE(count, *spec.expect.unknown_poses_max) << "too many path poses in unknown/off-map cells";
    }

    // --- Tier-0: determinism (replan is bit-identical) ---
    assertDeterministic(built, spec, result);
}

}  // namespace

// ---- Empty-corpus guard ------------------------------------------------------------------------

// A missing or empty corpus would leave the parameterized suite with zero cases — green by omission.
// This makes that condition an explicit failure: no vacuous green.
TEST(ScenarioCorpus, IsNotEmpty) {
    EXPECT_FALSE(corpusFiles().empty())
        << "no scenario YAMLs discovered under TEST_SCENARIO_CORPUS_DIR=" << TEST_SCENARIO_CORPUS_DIR
        << " — the integration tier has nothing to run";
}

// ---- Parameterized whole-planner runner --------------------------------------------------------

class ScenarioPlanTest : public ::testing::TestWithParam<std::string> {};

TEST_P(ScenarioPlanTest, PlansCorrectly) {
    const std::string & file = GetParam();
    ScenarioSpec spec = loadScenario(file);

    // Hold the BuiltScenario for the whole test: the planner aliases the snapshot's grid + EDT.
    BuiltScenario built = buildScenario(spec);
    const PlanResult plan = built.planner->plan(spec.start, spec.goal);
    const auto & result = plan.path;

    // The outcome must agree with the path, and with the scenario's expectation.
    EXPECT_EQ(plan.outcome == PlanOutcome::Found, result.has_value());
    if (spec.expect.expect_found) {
        EXPECT_EQ(plan.outcome, PlanOutcome::Found);
    } else {
        EXPECT_NE(plan.outcome, PlanOutcome::Found);
    }

    assertPlanValid(spec, built, result);
}

// Name each ctest case by the scenario's file stem so failures are self-locating
// (e.g. .../enclosed_box). GoogleTest requires parameter names to be alphanumeric/underscore, so any
// other character is sanitized to '_'.
std::string scenarioName(const ::testing::TestParamInfo<std::string> & info) {
    std::string stem = std::filesystem::path(info.param).stem().string();
    for (char & c : stem) {
        if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
    }
    if (stem.empty()) stem = "scenario_" + std::to_string(info.index);
    return stem;
}

INSTANTIATE_TEST_SUITE_P(ScenarioCorpus, ScenarioPlanTest,
                         ::testing::ValuesIn(corpusFiles()), scenarioName);
