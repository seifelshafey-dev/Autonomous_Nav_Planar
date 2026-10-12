#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/hybrid_astar/config.hpp"
#include "core/planner/hybrid_astar/heuristics.hpp"
#include "core/planner/planner_interface.hpp"
#include "core/planner/path.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/map/grid.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/math/wrap_angle.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/edt.hpp"
#include "core/map/rasterize.hpp"
#include "core/motion/kinematic_model.hpp"
#include "core/motion/world_robot_geometry.hpp"

/*
Hybrid A* expansion soundness over random obstacle maps. The analytic shot is DISABLED, so every
returned segment is a primitive expansion (a shot would otherwise end the search on the first pop for
any goal inside shot_max_distance).

Generated range: 100x100 grid at 0.1 m (10 m square); 0-4 occupied rectangles of 2-15 cells per side
anywhere in the grid; start position in [3, 7] m x [3, 7] m with any heading; goal = start translated
n * 0.3 m along the start heading (n in 1..6, same heading), i.e. the end of a straight chain of
forward primitives; cases whose start lies within 0.3 m of an occupied cell are discarded (start
recovery may pass through cells under the start). max_expansions is capped at 20000 so unreachable cases terminate deterministically.

Invariants, asserted for every returned path:
 - reconnection: re-propagating each segment's control from its start node lands on the next node;
 - structure: node_points.size() == segments.size()+1, front == start, total_cost == sum of segment
   costs, every segment cost >= 0;
 - goal: the final pose is within the goal position / heading tolerance;
 - collision: no node's centre cell is OCCUPIED;
 - determinism: replanning the same query on the same instance gives a bit-identical path.
Existence is asserted on obstacle-free maps (the goal is the end of a straight chain of forward
primitives); on maps with obstacles the invariants apply to whichever path is returned.
*/

namespace {

constexpr double CELL = 0.1;
constexpr int DIM = 100;  // 10 m grid

DiffDriveParams makeKin(){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.3; kin.n_headings = 16;
    return kin;
}
GridMetaData meta(){
    return GridMetaData{DIM, DIM, CELL,
                        Eigen::Vector2f::Zero()};
}

ScenarioSpec specWith(std::vector<int8_t> occ){
    ScenarioSpec spec{"search_property_obstacles", meta(), std::move(occ)};
    spec.robot_vertices = {Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)};
    spec.diff_drive = makeKin();
    spec.a_star.node_pool_reserve = 8192;
    spec.a_star.closed_set_reserve = 8192;
    spec.a_star.enable_shot = false;
    spec.a_star.max_expansions = 20000;
    spec.heuristic = HeuristicConfig{1.0, true};
    spec.cost_policy = CostPolicy{0.3f, 5.0f, 100.0f, 1.0f};
    return spec;
}

// Propagation-only state space (propagate depends solely on the control + start heading).
const DiffDriveStateSpace & space(){
    static auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(makeKin().rmin * makeKin().rmin);
    static DiffDriveStateSpace s(meta(), dist);
    return s;
}

const AStarConfig kCfg{};  // default tolerances 0.10 / 0.10

std::vector<int8_t> genOccupancy(){
    std::vector<int8_t> occ(static_cast<size_t>(DIM) * DIM, 0);
    const int n_rects = *rc::gen::inRange(0, 5);
    for (int k = 0; k < n_rects; ++k){
        const int w = *rc::gen::inRange(2, 16), h = *rc::gen::inRange(2, 16);
        const int c0 = *rc::gen::inRange(0, DIM - w + 1), r0 = *rc::gen::inRange(0, DIM - h + 1);
        for (int r = r0; r < r0 + h; ++r)
            for (int c = c0; c < c0 + w; ++c)
                occ[static_cast<size_t>(r) * DIM + c] = 100;
    }
    return occ;
}

bool occupiedAt(const std::vector<int8_t> & occ, double x, double y){
    const int c = static_cast<int>(std::floor(x / CELL)), r = static_cast<int>(std::floor(y / CELL));
    if (c < 0 || r < 0 || c >= DIM || r >= DIM) return true;  // off-grid counts as blocked
    return occ[static_cast<size_t>(r) * DIM + c] >= 50;
}

// True if any cell within `r` metres (Chebyshev) of (x, y) is occupied.
bool nearOccupied(const std::vector<int8_t> & occ, double x, double y, double r){
    const int reach = static_cast<int>(std::ceil(r / CELL));
    for (int dr = -reach; dr <= reach; ++dr)
        for (int dc = -reach; dc <= reach; ++dc)
            if (occupiedAt(occ, x + dc * CELL, y + dr * CELL)) return true;
    return false;
}

bool reachesGoal(const SE2State & a, const SE2State & g){
    const double dx = g.x - a.x, dy = g.y - a.y;
    return (dx * dx + dy * dy) <= kCfg.goal_pos_tol * kCfg.goal_pos_tol + 1e-9 &&
           std::abs(wrapToPi(g.theta - a.theta)) <= kCfg.goal_heading_tol + 1e-9;
}

}  // namespace

RC_GTEST_PROP(HybridAstarProp, pathIsSoundOverRandomObstacleMaps, ()){
    const std::vector<int8_t> occ = genOccupancy();
    const double sx = *rc::gen::inRange(300, 701) / 100.0, sy = *rc::gen::inRange(300, 701) / 100.0;
    const double th = *rc::gen::inRange(-31415, 31416) / 10000.0;
    const int n = *rc::gen::inRange(1, 7);
    const SE2State start{sx, sy, th};
    const SE2State goal{sx + n * 0.3 * std::cos(th), sy + n * 0.3 * std::sin(th), th};

    // A start whose footprint touches an obstacle triggers collision-tolerant start recovery, which
    // deliberately lets the path pass through the cells under the start; keep the start clear.
    RC_PRE(!nearOccupied(occ, sx, sy, 0.3));

    BuiltScenario built = buildScenario(specWith(occ));
    auto path = built.planner->plan(start, goal).path;

    RC_CLASSIFY(path.has_value(), "path found");
    const bool obstacle_free = std::none_of(occ.begin(), occ.end(), [](int8_t c){ return c != 0; });
    if (obstacle_free) RC_ASSERT(path.has_value());
    if (!path.has_value()) return;

    // reconnection
    for (size_t i = 0; i < path->segments.size(); ++i){
        SE2State re  = space().propagate(path->node_points[i], path->segments[i].u);
        SE2State nxt = path->node_points[i + 1];
        RC_ASSERT(std::abs(re.x - nxt.x) < 1e-6);
        RC_ASSERT(std::abs(re.y - nxt.y) < 1e-6);
        RC_ASSERT(std::abs(wrapToPi(re.theta - nxt.theta)) < 1e-6);
    }

    // structure
    RC_ASSERT(path->node_points.size() == path->segments.size() + 1);
    RC_ASSERT(std::abs(path->node_points.front().x - start.x) < 1e-9);
    RC_ASSERT(std::abs(path->node_points.front().y - start.y) < 1e-9);
    RC_ASSERT(std::abs(wrapToPi(path->node_points.front().theta - start.theta)) < 1e-9);
    double sum = 0.0;
    for (const auto & s : path->segments){
        RC_ASSERT(s.cost >= -1e-9);
        sum += s.cost;
    }
    RC_ASSERT(std::abs(path->total_cost - sum) < 1e-6);

    // goal + collision
    RC_ASSERT(reachesGoal(path->node_points.back(), goal));
    for (const auto & p : path->node_points) RC_ASSERT(!occupiedAt(occ, p.x, p.y));

    // determinism
    auto again = built.planner->plan(start, goal).path;
    RC_ASSERT(again.has_value());
    RC_ASSERT(again->node_points.size() == path->node_points.size());
    RC_ASSERT(std::abs(again->total_cost - path->total_cost) < 1e-12);
    for (size_t i = 0; i < path->node_points.size(); ++i){
        RC_ASSERT(again->node_points[i].x == path->node_points[i].x);
        RC_ASSERT(again->node_points[i].y == path->node_points[i].y);
        RC_ASSERT(again->node_points[i].theta == path->node_points[i].theta);
    }
}

/*
Start gate vs. the exact cover's first pose: whenever startClear(start) holds, for every primitive the
footprint at the start, rasterised with that primitive's arc margin m_s = reach / (2 n)
(reach = (|v| + r_out |omega|) dt, n = max(1, ceil(reach / h))), holds no OCCUPIED cell. The gate's
margin h/2 bounds m_s, so a clear start can never be rejected by the first pose of a primitive's cover.
(visitExactCover cannot isolate its first pose, so the first-pose raster is reproduced directly.)
*/
RC_GTEST_PROP(HybridAstarProp, clearStartFirstPoseRasterOfEveryPrimitiveIsFree, ()){
    const std::vector<int8_t> occ = genOccupancy();
    const double sx = *rc::gen::inRange(300, 701) / 100.0, sy = *rc::gen::inRange(300, 701) / 100.0;
    const double th = *rc::gen::inRange(-31415, 31416) / 10000.0;
    const SE2State start{sx, sy, th};

    BuiltScenario built = buildScenario(specWith(occ));
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(built.snapshot, &built.snapshot->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(built.snapshot, &built.snapshot->edt_);
    const CollisionChecker checker(grid_ptr, edt_ptr, built.robot);
    RC_PRE(checker.startClear(start));

    const KinematicModel km = buildKinematicModel(built.robot, makeKin(), CELL);
    const auto geo0 = transformGeometry(built.robot, start, Vec2(0.0, 0.0));
    for (const auto & prim : km.prim_lib->primitives()){
        const Control & u = prim.control;
        const double reach = (std::abs(u.v) + built.robot.r_out * std::abs(u.omega)) * u.dt;
        const int n = std::max(1, static_cast<int>(std::ceil(reach / CELL)));
        const double m_s = reach / (2.0 * n);
        RC_ASSERT(m_s <= CELL / 2 + 1e-12);
        for (const auto & c : rasterizePolygon(geo0.half_planes, geo0.aabb, CELL, m_s))
            RC_ASSERT(built.snapshot->grid_.findCellStatus(c) != CellState::OCCUPIED);
    }
}
