#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "core/config/config_loader.hpp"
#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/path.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/hybrid_astar/search.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/grid.hpp"
#include "core/types/math/wrap_angle.hpp"

// The shipped configuration (config/robot, config/planner, config/cost_policy) is loaded with the real
// loaders and used to plan over a synthetic 8 m x 8 m map with one wall the route must go around.

namespace {

// <path_planner>/test/integration/shipped_config_test.cpp -> up 3 dirs to <path_planner>.
std::string shipped(const std::string & dir, const std::string & file){
    namespace fs = std::filesystem;
    const fs::path root = fs::path(__FILE__).parent_path().parent_path().parent_path();
    return (root / "config" / dir / file).string();
}

class ShippedConfig : public ::testing::Test {
protected:
    RobotParams robot;
    DiffDriveParams kin;
    AStarConfig cfg;
    HeuristicConfig heur;
    CostPolicy policy;

    void SetUp() override {
        robot  = loadRobotParams(shipped("robot", "robot_params.yaml"));
        kin    = loadDiffDriveParams(shipped("robot", "robot_params.yaml"));
        cfg    = loadAStarConfig(shipped("planner", "planner.yaml"));
        heur   = loadHeuristicConfig(shipped("planner", "planner.yaml"));
        policy = loadCostPolicy(shipped("cost_policy", "cost_policy.yaml"));
    }

    // 8 m x 8 m, 0.05 m cells, origin (0, 0); wall x in [3.9, 4.1], y in [0, 5.0].
    BuiltScenario buildWallMap() const {
        constexpr int N = 160;
        std::vector<int8_t> occ(static_cast<size_t>(N) * N, 0);
        for (int r = 0; r < 100; ++r)
            for (int c = 78; c < 82; ++c)
                occ[static_cast<size_t>(r) * N + c] = 100;
        ScenarioSpec spec{"shipped_config", GridMetaData{N, N, 0.05, Eigen::Vector2f::Zero()}, occ};
        spec.robot_vertices = {Vec2(0.3, 0.3), Vec2(-0.3, 0.3), Vec2(-0.3, -0.3), Vec2(0.3, -0.3)};
        spec.diff_drive = kin;
        spec.a_star = cfg;
        spec.heuristic = heur;
        spec.cost_policy = policy;
        return buildScenario(spec);
    }
};

}  // namespace

TEST_F(ShippedConfig, shippedFilesLoadAndPinPrimitiveValues){
    EXPECT_DOUBLE_EQ(kin.rmin, 0.3);
    EXPECT_DOUBLE_EQ(kin.vmax, 0.5);
    EXPECT_DOUBLE_EQ(kin.atomic_time, 0.6);
    EXPECT_EQ(kin.n_headings, 16);
    EXPECT_DOUBLE_EQ(cfg.closed_cell_size, 0.15);
    ASSERT_EQ(robot.vertices.size(), 4u);
    for (const auto & v : robot.vertices){
        EXPECT_FLOAT_EQ(std::abs(v.x()), 0.3f);
        EXPECT_FLOAT_EQ(std::abs(v.y()), 0.3f);
    }
}

TEST_F(ShippedConfig, shippedConfigPlansAroundAWall){
    BuiltScenario b = buildWallMap();
    const SE2State start{1.5, 1.5, 0.0}, goal{6.5, 1.5, 0.0};
    auto path = b.planner->plan(start, goal).path;
    ASSERT_TRUE(path.has_value());
    ASSERT_FALSE(path->segments.empty());

    EXPECT_DOUBLE_EQ(path->node_points.front().x, start.x);
    EXPECT_DOUBLE_EQ(path->node_points.front().y, start.y);
    EXPECT_DOUBLE_EQ(path->node_points.front().theta, start.theta);
    const SE2State & last = path->node_points.back();
    EXPECT_LE(std::hypot(last.x - goal.x, last.y - goal.y), cfg.goal_pos_tol + 1e-9);
    EXPECT_LE(std::abs(wrapToPi(last.theta - goal.theta)), cfg.goal_heading_tol + 1e-9);

    // Independent exact check of every segment, in the map frame.
    const GridMetaData gm = b.snapshot->grid_.getMetadata();
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(b.snapshot, &b.snapshot->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(b.snapshot, &b.snapshot->edt_);
    const CollisionChecker checker(grid_ptr, edt_ptr, b.robot);
    auto dist  = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    auto space = std::make_shared<const DiffDriveStateSpace>(gm, dist);
    const MotionContext motion(b.robot, space, gm);
    for (size_t i = 0; i < path->segments.size(); ++i){
        const auto & seg = path->segments[i];
        ASSERT_FALSE(seg.samples_poses.empty()) << "segment " << i;
        EXPECT_TRUE(checker.checkCells(
            motion.computeExactCover(seg.samples_poses.front(), seg.u).cells, false))
            << "segment " << i << " touches an OCCUPIED cell";
    }

    const auto * h = dynamic_cast<const HybridAstar *>(b.planner.get());
    ASSERT_NE(h, nullptr);
    const PlanStats s = h->lastPlanStats();
    const std::int64_t P = static_cast<std::int64_t>(
        buildKinematicModel(b.robot, kin, gm.resolution).prim_lib->primitives().size());
    EXPECT_EQ(s.fast_accept + s.fast_reject + s.scan_clear + s.scan_blocked + s.duplicate_skipped,
              P * (s.expansions - 1));
    EXPECT_LE(s.exact_accept + s.exact_reject, s.scan_blocked);
}

TEST_F(ShippedConfig, shippedConfigIsDeterministic){
    BuiltScenario b = buildWallMap();
    const SE2State start{1.5, 1.5, 0.0}, goal{6.5, 1.5, 0.0};
    auto p1 = b.planner->plan(start, goal).path;
    auto p2 = b.planner->plan(start, goal).path;
    ASSERT_TRUE(p1.has_value());
    ASSERT_TRUE(p2.has_value());
    EXPECT_DOUBLE_EQ(p1->total_cost, p2->total_cost);
    ASSERT_EQ(p1->node_points.size(), p2->node_points.size());
    for (size_t i = 0; i < p1->node_points.size(); ++i){
        EXPECT_DOUBLE_EQ(p1->node_points[i].x, p2->node_points[i].x) << "node " << i;
        EXPECT_DOUBLE_EQ(p1->node_points[i].y, p2->node_points[i].y) << "node " << i;
        EXPECT_DOUBLE_EQ(p1->node_points[i].theta, p2->node_points[i].theta) << "node " << i;
    }
}

TEST_F(ShippedConfig, unknownCostIsDerivedFromFactorAndNeverBelowFreeSpace){
    EXPECT_FLOAT_EQ(policy.unknown_cost_factor_, 1.03f);

    const double h = 0.05, r_in = robot.r_in, w = cfg.costmap_weight;
    EXPECT_NEAR(r_in, 0.3, 1e-6);
    const double d_min = std::max(0.0, r_in - std::sqrt(2.0) / 2.0 * h);
    const double b = d_min < policy.clearance_radius_
                         ? policy.max_clearance_cost_ * std::exp(-policy.clearance_decay_ * d_min) : 0.0;
    const double u = w > 0.0 ? (policy.unknown_cost_factor_ * (1.0 + w * b) - 1.0) / w : 0.0;

    BuiltScenario built = buildWallMap();
    const CostMap cm = buildCostMap(*built.snapshot, policy, r_in, w);
    EXPECT_NEAR(cm.unknownCost(), u, 1e-4);
    EXPECT_GE(1.0 + w * cm.unknownCost(), (1.0 + w * b) * (1.0 - 1e-6));
}

TEST(ShippedNavigatorConfig, loadsShippedValues) {
    const NavigatorParams p = loadNavigatorParams(shipped("navigation", "navigator.yaml"));
    EXPECT_DOUBLE_EQ(p.tick_period_sec, 0.5);
    EXPECT_DOUBLE_EQ(p.stall_timeout, 5.0);
    EXPECT_DOUBLE_EQ(p.blocked_retry_period, 2.0);
    EXPECT_EQ(p.max_clearing_attempts, 2);
    EXPECT_DOUBLE_EQ(p.escape_speed_scale, 0.5);
    EXPECT_DOUBLE_EQ(p.escape.step_fraction, 0.4);
    EXPECT_EQ(p.escape.slack_cells, 1);
    EXPECT_DOUBLE_EQ(p.escape.max_travel_factor, 5.0);
}
