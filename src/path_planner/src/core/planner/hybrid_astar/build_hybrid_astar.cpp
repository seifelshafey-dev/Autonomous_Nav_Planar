/**
 * @file build_hybrid_astar.cpp
 * @author Seif ElShafey
 * @brief Implementation of the hybrid A* planner factory.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "core/planner/hybrid_astar/build_hybrid_astar.hpp"

#include "core/planner/hybrid_astar/search.hpp"
#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/motion_context.hpp"
#include "core/diagnostics/check.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

std::unique_ptr<Planner>
buildHybridAstar(const KinematicModel & kin_model,
                 const AStarConfig & cfg,
                 const HeuristicConfig & heur_cfg,
                 const CostPolicy & cost_policy,
                 std::shared_ptr<const MapSnapshot> snapshot) {

    const GridMetaData grid_meta = snapshot->grid_.getMetadata();

    // Single-source the grid resolution: the kinematic model's primitive library and swept-cell
    // cache are precomputed against the cell size it was built with, so planning against a snapshot
    // of a different resolution silently misaligns cells. Compare with a float-width tolerance —
    // GridMetaData::resolution is a float and cellSize() is a double-widened float, so exact
    // equality would trip on the double->float->double round-trip rather than a real mismatch.
    const double kin_cell_size = kin_model.cellSize();
    const double snap_resolution = static_cast<double>(grid_meta.resolution);
    CHECK(std::abs(kin_cell_size - snap_resolution) < 1e-6,
          "KinematicModel cell size (" + std::to_string(kin_cell_size) +
          " m) does not match the map snapshot grid resolution (" +
          std::to_string(snap_resolution) +
          " m); rebuild the kinematic model against this snapshot's resolution.");

    const int32_t stride = HybridAstar::closedStride(cfg.closed_cell_size, snap_resolution);
    const double effective_cell = stride * snap_resolution;
    const double straight_len = kin_model.kin.vmax * kin_model.kin.atomic_time;
    // A straight move leaves a square cell of side s from any position only if longer than its diagonal.
    CHECK(straight_len > std::sqrt(2.0) * effective_cell,
          "straight primitive length vmax*atomic_time (" + std::to_string(straight_len) +
          " m) must exceed sqrt(2) * effective closed cell (" +
          std::to_string(std::sqrt(2.0) * effective_cell) + " m = " + std::to_string(stride) +
          " map cells of " + std::to_string(snap_resolution) +
          " m); lower closed_cell_size or lengthen the primitive.");

    // Fast collision tests decide only if the distance field's limit exceeds what they compare against.
    const double edt_limit = snapshot->edt_.getMaxDist();
    const double r_out = kin_model.motion->body().r_out;
    double c_max = 0.0;
    for (const Primitive & p : kin_model.prim_lib->primitives()) {
        for (double c : p.sample_clearance) { c_max = std::max(c_max, c); }
    }
    const double need = std::max(r_out + std::sqrt(2.0) * snap_resolution, c_max);
    CHECK(edt_limit > need,
          "distance-field limit edt_max_dist (" + std::to_string(edt_limit) +
          " m) must exceed " + std::to_string(need) +
          " m (the larger of r_out + sqrt(2)*cell and the largest primitive sample clearance), "
          "otherwise the fast collision tests cannot decide; raise edt_max_dist.");

    CHECK(cost_policy.clearance_radius_ > r_out,
          "cost_policy.clearance_radius (" + std::to_string(cost_policy.clearance_radius_) +
          " m) must exceed the robot's circumscribed radius r_out (" + std::to_string(r_out) +
          " m), otherwise the clearance cost ends before the footprint can reach an obstacle");

    // SE(2) distance shared between the state-space and the heuristic. Its weight `rmin^2` matches the
    // control cost, so the unweighted straight-line estimate never exceeds the motion cost to the exact goal pose.
    auto se2_dist = std::make_shared<const PointsWeightedEuclideanSE2>(
        kin_model.kin.rmin * kin_model.kin.rmin);

    // Aliasing shared_ptrs into the snapshot: the checker and downstream consumers share the
    // snapshot's control block, so the snapshot lives exactly as long as any aliased reference.
    auto grid_ptr = std::shared_ptr<const OccupancyGrid>(snapshot, &snapshot->grid_);
    auto edt_ptr  = std::shared_ptr<const EDT>(snapshot, &snapshot->edt_);

    auto checker = std::make_shared<const CollisionChecker>(
        grid_ptr, edt_ptr, kin_model.motion->body());

    auto space = std::make_shared<const DiffDriveStateSpace>(
        grid_meta, se2_dist);

    auto costmap = std::make_shared<const CostMap>(
        buildCostMap(*snapshot, cost_policy, kin_model.motion->body().r_in, cfg.costmap_weight));

    // Weighted-Euclidean child (kinematic distance, obstacle-blind).
    auto distance_h = std::make_shared<const DistanceHeuristic>(se2_dist, heur_cfg.weight);

    std::shared_ptr<const Heuristic> heuristic;
    if (heur_cfg.use_obstacle_heuristic) {
        // Obstacle-aware child: full Dijkstra cost-to-goal over the grid. It must use the SAME
        // costmap_weight the search charges in g (cfg.costmap_weight), distinct from heur_cfg.weight.
        auto obstacle_h = std::make_shared<const ObstacleHeuristic>(
            grid_ptr, costmap, cfg.costmap_weight, heur_cfg.weight);
        heuristic = std::make_shared<const MaxHeuristic>(
            std::vector<std::shared_ptr<const Heuristic>>{distance_h, obstacle_h});
    } else {
        heuristic = distance_h;
    }

    // Snapshot-aware MotionContext: the cached primitive library inside kin_model.prim_lib is
    // body-frame and stays valid across snapshots, but the planner's exact motion checks and goal
    // connection work in world coordinates and need the SLAM grid's real origin.
    KinematicModel runtime_kin = kin_model;
    runtime_kin.motion = std::make_shared<const MotionContext>(
        kin_model.motion->body(), space, grid_meta);

    return std::make_unique<HybridAstar>(
        std::move(runtime_kin), checker, space, heuristic, costmap, grid_meta, cfg);
}
