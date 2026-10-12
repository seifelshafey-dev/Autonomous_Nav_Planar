#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Core>

#include "core/planner/validation/path_validity.hpp"
#include "core/planner/path.hpp"
#include "core/planner/planner_interface.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/map/map_provider.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/math/wrap_angle.hpp"

namespace {
CollisionChecker makeChecker(const std::shared_ptr<const OccupancyGrid> & grid) {
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid, 5.0));
    auto params = buildRobotParams({{0.3, 0.3}, {-0.3, 0.3}, {-0.3, -0.3}, {0.3, -0.3}});
    return CollisionChecker(grid, edt, params);
}

Path onePosePath(const SE2State & p) {
    Path path;
    path.segments.push_back(PathSegment{Control{0.0, 0.0, 0.0}, {p}, 0.0});
    path.node_points = {p, p};
    path.total_cost = 0.0;
    return path;
}
}  // namespace

TEST(PathValidity, freeSpacePathIsValid) {
    GridMetaData meta(20, 20, 0.1, Eigen::Vector2f{0.0f, 0.0f});
    auto grid = std::make_shared<const OccupancyGrid>(meta, std::vector<int8_t>(20 * 20, 0));
    auto checker = makeChecker(grid);
    EXPECT_TRUE(pathValid(onePosePath(SE2State{1.0, 1.0, 0.0}), checker));
}

TEST(PathValidity, pathThroughObstacleIsInvalid) {
    GridMetaData meta(20, 20, 0.1, Eigen::Vector2f{0.0f, 0.0f});
    std::vector<int8_t> cells(20 * 20, 0);
    for (int j = 7; j <= 13; ++j)
        for (int i = 7; i <= 13; ++i) cells[j * 20 + i] = 100;  // occupied block around (1.0,1.0)
    auto grid = std::make_shared<const OccupancyGrid>(meta, cells);
    auto checker = makeChecker(grid);
    EXPECT_FALSE(pathValid(onePosePath(SE2State{1.0, 1.0, 0.0}), checker));
}

// ---- pathValid on planned paths: a collision-free plan from a clear start is accepted --------
//
// Fixture: a full-height OCCUPIED wall whose left face is at x = 1.30 (cells i >= 13); the goal sits
// left of it in free space.

namespace {

constexpr double ESC_CELL = 0.1;
constexpr int    ESC_COLS = 60, ESC_ROWS = 30;
constexpr int    ESC_WALL_C0 = 13, ESC_WALL_C1 = 20;  // x in [1.30, 2.10)
constexpr int    ESC_NHEAD = 16;
const SE2State   ESC_START{1.25, 1.5, 0.0};           // footprint [1.15,1.35] overlaps the wall face at 1.30
const SE2State   ESC_GOAL{0.5, 1.5, 0.0};             // behind the robot, in free space

DiffDriveParams escKin(){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.3; kin.n_headings = ESC_NHEAD;
    return kin;
}

GridMetaData escMeta(){
    return GridMetaData{ESC_COLS, ESC_ROWS, ESC_CELL,
                        Eigen::Vector2f::Zero()};
}

std::vector<int8_t> escapeWallOcc(){
    std::vector<int8_t> occ(static_cast<size_t>(ESC_COLS) * ESC_ROWS, 0);
    for (int r = 0; r < ESC_ROWS; ++r)
        for (int c = ESC_WALL_C0; c <= ESC_WALL_C1; ++c)
            occ[static_cast<size_t>(r) * ESC_COLS + c] = 100;  // full-height wall
    return occ;
}

// Build the escape scenario through the production factory chain (buildScenario), the same seam the
// search test and the data-driven integration tier use. Callers keep the returned BuiltScenario alive
// for the planner's lifetime (it aliases the snapshot).
BuiltScenario buildEscape(){
    ScenarioSpec spec{"path_validity_escape", escMeta(), escapeWallOcc()};
    spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
    spec.diff_drive = escKin();
    AStarConfig cfg;
    cfg.node_pool_reserve = 4096;
    cfg.closed_set_reserve = 4096;
    spec.a_star = cfg;
    spec.heuristic = HeuristicConfig{1.0, false};
    spec.cost_policy = CostPolicy{0.3f, 5.0f, 100.0f, 1.0f};  // unknown_cost_factor 1
    return buildScenario(spec);
}

// A collision oracle over the built snapshot, independent of the planner.
CollisionChecker makeOracle(const BuiltScenario & b){
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(b.snapshot, &b.snapshot->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(b.snapshot, &b.snapshot->edt_);
    return CollisionChecker(grid_ptr, edt_ptr, b.robot);
}

}  // namespace

// A plan from a clear start is accepted by pathValid.
TEST(PathValidity, planFromClearStartIsValid){
    auto b = buildEscape();
    CollisionChecker oracle = makeOracle(b);

    const SE2State clear_start{0.8, 1.5, 0.0};   // footprint [0.7,0.9] clear of the wall face at 1.30
    ASSERT_TRUE(oracle.startClear(clear_start)) << "fixture: this start must pass the start gate";

    auto path = b.planner->plan(clear_start, ESC_GOAL).path;
    ASSERT_TRUE(path.has_value()) << "a clear start behind the wall must plan normally";
    EXPECT_TRUE(pathValid(*path, oracle))
        << "pathValid must accept a fully clear valid path";
}
