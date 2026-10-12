/**
 * @file motion_context.cpp
 * @author Seif ElShafey
 * @brief Implementation of MotionContext.
 * @version 0.1
 * @date 2026-05-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cmath>
#include "core/diagnostics/check.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/map/rasterize.hpp"

std::vector<SE2State> MotionContext::sampleTrajectory(const SE2State & start, const Control & u) const
{
    std::vector<SE2State> interp_poses;
    int n_intervals;

    double arc_len = std::abs(u.v)*u.dt;
    double rot = std::abs(u.omega) * u.dt;
    // Inscribed disks should overlap (distance less than r_in), 1/2 factor of safety
    if(u.omega== 0){
        n_intervals = std::ceil(arc_len / (body_.r_in / 2));
    }

    // Rotation only. r_out * dtheta should be inscribed within the overlapping r_in disks
    else if(u.v == 0){
        n_intervals = std::ceil(rot / (body_.r_in / (2*body_.r_out)));
    }

    else{
        n_intervals = std::max({2.0, std::ceil(arc_len / (body_.r_in / 2)),
                            std::ceil((rot / (body_.r_in / (2*body_.r_out))))});
    }

    n_intervals = std::max(1, n_intervals);
    interp_poses.reserve(n_intervals+1);

    for(int i = 0; i<=n_intervals; i++){
        double time = (static_cast<double>(i) / n_intervals) * u.dt;
        interp_poses.push_back(space_->propagate(start, {u.v, u.omega, time}));
    }
    return interp_poses;
}

MotionContext::SweptResult MotionContext::computeExactCover(const SE2State & start, const Control & u) const
{
    CHECK(std::isfinite(u.dt) && u.dt >= 0.0, "computeExactCover: u.dt must be finite and >= 0");
    std::vector<GridCoords> cells;
    visitExactCover(start, u, [&](GridCoords c) { cells.push_back(c); return true; });
    std::vector<SE2State> poses = sampleTrajectory(start, u);
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());

    std::vector<GridCoords> sample_cells;
    sample_cells.reserve(poses.size());
    for (const auto & p : poses) sample_cells.push_back(grid_meta_.worldToGrid(p.x, p.y));
    return SweptResult{std::move(cells), std::move(poses), std::move(sample_cells)};
}
