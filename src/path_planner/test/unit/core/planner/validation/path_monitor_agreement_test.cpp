#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
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
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"

/*
Monitor agreement: every path plan() returns is accepted by `pathValid(path, checker)`. A query whose
start fails `startClear` must return StartBlocked and is not counted as a path. Fixed-seed loop over
small cluttered random maps (3 m x 3 m at 0.05 m, 0.2 m square body), shot enabled on half the maps.
About one query in five starts close to an obstacle and one in ten starts inside one.

Any disagreement is reported with the map seed, query, failing pose, the occupied cell and the segment.
*/

namespace {

constexpr int DIM = 60;
constexpr double H = 0.05;
constexpr double PI = std::numbers::pi;
constexpr int kTargetPaths = 300;
constexpr int kMaxMaps = 4000;
constexpr int kQueriesPerMap = 40;

DiffDriveParams makeKin(){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.3; kin.n_headings = 16;
    return kin;
}

struct Block { int c0, r0, w, h; };

}  // namespace

TEST(PathMonitorAgreement, everyReturnedPathIsAcceptedByTheMonitor){
    const auto t0 = std::chrono::steady_clock::now();
    long paths = 0, queries = 0, rejected = 0, blocked_starts = 0, wrong_outcome = 0, shot_paths = 0, noshot_paths = 0;
    std::ostringstream first_reports;
    int reported = 0;

    for (int m = 0; m < kMaxMaps && paths < kTargetPaths; ++m){
        std::mt19937_64 rng(777000u + static_cast<unsigned>(m));
        auto ui = [&](int lo, int hi){ return std::uniform_int_distribution<int>(lo, hi)(rng); };
        auto ur = [&](double lo, double hi){ return std::uniform_real_distribution<double>(lo, hi)(rng); };

        std::vector<int8_t> occ(DIM * DIM, 0);
        std::vector<Block> blocks;
        const int nb = ui(3, 8);
        for (int k = 0; k < nb; ++k){
            Block b{ui(0, DIM - 3), ui(0, DIM - 3), ui(2, 12), ui(2, 12)};
            blocks.push_back(b);
            for (int r = b.r0; r < std::min(DIM, b.r0 + b.h); ++r)
                for (int c = b.c0; c < std::min(DIM, b.c0 + b.w); ++c) occ[r * DIM + c] = 100;
        }
        const bool shot = (m % 2) == 0;

        ScenarioSpec spec{"monitor_agreement", GridMetaData{DIM, DIM, H, Eigen::Vector2f::Zero()}, occ};
        spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
        spec.diff_drive = makeKin();
        spec.a_star.node_pool_reserve = 8192;
        spec.a_star.closed_set_reserve = 8192;
        spec.a_star.enable_shot = shot;
        spec.a_star.max_expansions = 4000;
        spec.heuristic = HeuristicConfig{1.0, true};
        spec.cost_policy = CostPolicy{0.3f, 5.0f, 100.0f, 1.0f};
        BuiltScenario built = buildScenario(spec);
        auto grid_ptr = std::shared_ptr<const OccupancyGrid>(built.snapshot, &built.snapshot->grid_);
        auto edt_ptr  = std::shared_ptr<const EDT>(built.snapshot, &built.snapshot->edt_);
        const CollisionChecker checker(grid_ptr, edt_ptr, built.robot);

        auto nearOccupied = [&](const SE2State & s, int reach){
            const GridCoords gc = grid_ptr->getMetadata().worldToGrid(s.x, s.y);
            for (int dj = -reach; dj <= reach; ++dj)
                for (int di = -reach; di <= reach; ++di)
                    if (grid_ptr->findCellStatus(GridCoords{gc.i + di, gc.j + dj}) == CellState::OCCUPIED) return true;
            return false;
        };

        for (int q = 0; q < kQueriesPerMap && paths < kTargetPaths; ++q){
            const int kind = q % 10;   // 0..1 grazing, 2 in collision, else free
            SE2State start{0, 0, 0}, goal{0, 0, 0};
            bool ok = false;
            for (int attempt = 0; attempt < 200 && !ok; ++attempt){
                start = SE2State{ur(0.3, 2.7), ur(0.3, 2.7), ur(-PI, PI)};
                goal = SE2State{ur(0.3, 2.7), ur(0.3, 2.7), ur(-PI, PI)};
                const double d = std::hypot(goal.x - start.x, goal.y - start.y);
                if (d < 0.4 || d > 2.5 || !checker.checkState(goal)) continue;
                const bool clear = checker.checkState(start);
                if (kind <= 1)      ok = clear && nearOccupied(start, 4);
                else if (kind == 2) ok = !clear;
                else                ok = clear;
            }
            if (!ok) continue;
            ++queries;

            const PlanResult result = built.planner->plan(start, goal);
            if (!checker.startClear(start)) {
                ++blocked_starts;
                if (result.outcome != PlanOutcome::StartBlocked || result.path.has_value()) ++wrong_outcome;
                continue;
            }
            const auto & path = result.path;
            if (!path.has_value()) continue;
            ++paths;
            (shot ? shot_paths : noshot_paths)++;

            if (pathValid(*path, checker)) continue;

            ++rejected;
            if (reported++ >= 5) continue;
            // Locate the first failing pose and an occupied cell under it.
            for (size_t i = 0; i < path->segments.size(); ++i)
                for (const auto & p : path->segments[i].samples_poses)
                    if (!checker.checkState(p)){
                        first_reports << "map_seed=" << 777000 + m << " shot=" << shot << " query=" << q
                                      << " start=(" << start.x << "," << start.y << "," << start.theta << ")"
                                      << " goal=(" << goal.x << "," << goal.y << "," << goal.theta << ")"
                                      << " segment=" << i << " u=(" << path->segments[i].u.v << ","
                                      << path->segments[i].u.omega << "," << path->segments[i].u.dt << ")"
                                      << " pose=(" << p.x << "," << p.y << "," << p.theta << ")\n";
                        goto next_report;
                    }
            next_report:;
        }
    }

    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("monitor agreement: queries=%ld paths=%ld (shot %ld, no shot %ld) blocked_starts=%ld rejected=%ld wall=%.1fs\n",
                queries, paths, shot_paths, noshot_paths, blocked_starts, rejected, secs);
    EXPECT_GE(paths, kTargetPaths);
    EXPECT_EQ(wrong_outcome, 0) << "a start failing startClear must return StartBlocked";
    EXPECT_EQ(rejected, 0) << first_reports.str();
}
