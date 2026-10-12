#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "core/config/config_loader.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/map/map_provider.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/planner/hybrid_astar/build_hybrid_astar.hpp"
#include "core/planner/primitives/control_primitives.hpp"
#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_planner.hpp"

/*
buildHybridAstar over the shipped configuration: it builds, and it rejects a snapshot whose distance-field
limit is too small.
*/

namespace {

// <path_planner>/test/unit/core/planner/hybrid_astar/x.cpp -> up 6 dirs to <path_planner>.
std::string shipped(const std::string & dir, const std::string & file){
    namespace fs = std::filesystem;
    fs::path root = fs::path(__FILE__);
    for (int i = 0; i < 6; ++i) root = root.parent_path();
    return (root / "config" / dir / file).string();
}

}  // namespace

namespace {

class BuildHybridAstar : public ::testing::Test {
protected:
    RobotParams robot;
    DiffDriveParams kin;
    AStarConfig cfg;
    HeuristicConfig heur;
    CostPolicy policy;
    BuiltScenario built;
    double h = 0.05;

    void SetUp() override {
        kin    = loadDiffDriveParams(shipped("robot", "robot_params.yaml"));
        cfg    = loadAStarConfig(shipped("planner", "planner.yaml"));
        heur   = loadHeuristicConfig(shipped("planner", "planner.yaml"));
        policy = loadCostPolicy(shipped("cost_policy", "cost_policy.yaml"));
        // 4 m x 4 m free map, non-zero origin.
        constexpr int N = 80;
        ScenarioSpec spec{"build_hybrid_astar", GridMetaData{N, N, h, Eigen::Vector2f(-2.0f, -3.0f)},
                          std::vector<int8_t>(static_cast<size_t>(N) * N, 0)};
        spec.robot_vertices = {Vec2(0.3, 0.3), Vec2(-0.3, 0.3), Vec2(-0.3, -0.3), Vec2(0.3, -0.3)};
        spec.diff_drive = kin;
        spec.a_star = cfg;
        spec.heuristic = heur;
        spec.cost_policy = policy;
        built = buildScenario(spec);
        robot = built.robot;
    }

    // Smallest distance-field limit the factory accepts is strictly above this (m).
    double requiredLimit(const KinematicModel & km) const {
        double c_max = 0.0;
        for (const auto & p : km.prim_lib->primitives())
            for (double c : p.sample_clearance) c_max = std::max(c_max, c);
        return std::max(robot.r_out + std::sqrt(2.0) * h, c_max);
    }

    std::shared_ptr<const MapSnapshot> withLimit(double limit) const {
        auto s = std::make_shared<MapSnapshot>(MapSnapshot{built.snapshot->grid_, built.snapshot->edt_});
        s->edt_.setMaxDist(limit);
        return s;
    }
};

}  // namespace

TEST_F(BuildHybridAstar, shippedConfigBuilds){
    const KinematicModel km = buildKinematicModel(robot, kin, h);
    EXPECT_NO_THROW(buildHybridAstar(km, cfg, heur, policy, built.snapshot));
}

TEST_F(BuildHybridAstar, distanceFieldLimitIsEnforced){
    const KinematicModel km = buildKinematicModel(robot, kin, h);
    const double need = requiredLimit(km);
    EXPECT_THROW(buildHybridAstar(km, cfg, heur, policy, withLimit(robot.r_out)), std::invalid_argument);
    EXPECT_THROW(buildHybridAstar(km, cfg, heur, policy, withLimit(need - 0.01)), std::invalid_argument);
    EXPECT_NO_THROW(buildHybridAstar(km, cfg, heur, policy, withLimit(need + 0.01)));
}

// The clearance cost must reach beyond the circumscribed radius; the bound is strict.
TEST_F(BuildHybridAstar, clearanceRadiusMustExceedCircumscribedRadius){
    const KinematicModel km = buildKinematicModel(robot, kin, h);
    CostPolicy p = policy;
    // clearance_radius_ is a float: use the largest float <= r_out, and the next float above it.
    float at = static_cast<float>(robot.r_out);
    if (static_cast<double>(at) > robot.r_out) at = std::nextafter(at, 0.0f);
    p.clearance_radius_ = at;
    EXPECT_THROW(buildHybridAstar(km, cfg, heur, p, built.snapshot), std::invalid_argument);
    p.clearance_radius_ = std::nextafter(at, 1.0f);
    EXPECT_NO_THROW(buildHybridAstar(km, cfg, heur, p, built.snapshot));
}

// Independent of requiredLimit: the bound is hand-computed.
TEST_F(BuildHybridAstar, distanceFieldLimitHandComputedBound){
    const KinematicModel km = buildKinematicModel(robot, kin, h);
    // Every sample clearance is at least r_out + 2*sqrt(2)*h, so r_out + sqrt(2)*h is too small.
    ASSERT_DOUBLE_EQ(h, 0.05);
    EXPECT_THROW(buildHybridAstar(km, cfg, heur, policy, withLimit(robot.r_out + std::sqrt(2.0) * 0.05)),
                 std::invalid_argument);
    // The shipped edt_max_dist.
    EXPECT_NO_THROW(buildHybridAstar(km, cfg, heur, policy, withLimit(5.0)));
}
