#pragma once

// Test-only geometric oracle for the cells a robot footprint touches while executing a control. It does
// not use rasterizePolygon or computeExactCover. The motion is sampled at poses spaced at most h/4 of
// footprint travel apart, and a cell counts as touched only when its square and the footprint polygon
// overlap with positive extent (> 1e-9 m) on every separating axis (x, y, polygon edge normals).
// Boundary-only contact does not count, so the oracle under-approximates the true swept set: every cell
// it reports is genuinely touched. Cells are indexed in the frame of the supplied poses, with cell (i, j)
// the square [i*h, (i+1)*h] x [j*h, (j+1)*h].

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/motion/kinematics/diff_drive.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/control.hpp"
#include "core/types/state.hpp"

namespace swept_oracle {

constexpr double kMinOverlap = 1e-9;

using Cell = std::pair<int, int>;

// Overlap length of the cell square [i*h,(i+1)*h] x [j*h,(j+1)*h] and the polygon on one unit axis.
inline bool overlapsOnAxis(const std::vector<Eigen::Vector2d> & poly, int i, int j, double h,
                           const Eigen::Vector2d & ax, double min_overlap){
    double pmin = 1e300, pmax = -1e300, smin = 1e300, smax = -1e300;
    for (const auto & p : poly) { const double d = p.dot(ax); pmin = std::min(pmin, d); pmax = std::max(pmax, d); }
    for (int di = 0; di < 2; ++di)
        for (int dj = 0; dj < 2; ++dj) {
            const double d = Eigen::Vector2d((i + di) * h, (j + dj) * h).dot(ax);
            smin = std::min(smin, d); smax = std::max(smax, d);
        }
    return std::min(pmax, smax) - std::max(pmin, smin) > min_overlap;
}

inline void touchedCells(const RobotParams & body, const SE2State & p, double h, std::set<Cell> & out,
                         double min_overlap = kMinOverlap){
    std::vector<Eigen::Vector2d> poly;
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    for (const auto & v : body.vertices)
        poly.emplace_back(p.x + c * v.x() - s * v.y(), p.y + s * v.x() + c * v.y());

    std::vector<Eigen::Vector2d> axes = {Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 1)};
    for (std::size_t n = 0; n < poly.size(); ++n) {
        const Eigen::Vector2d e = poly[(n + 1) % poly.size()] - poly[n];
        axes.emplace_back(Eigen::Vector2d(-e.y(), e.x()).normalized());
    }

    double minx = 1e300, maxx = -1e300, miny = 1e300, maxy = -1e300;
    for (const auto & q : poly) { minx = std::min(minx, q.x()); maxx = std::max(maxx, q.x());
                                  miny = std::min(miny, q.y()); maxy = std::max(maxy, q.y()); }
    const int i0 = static_cast<int>(std::floor(minx / h)) - 1, i1 = static_cast<int>(std::floor(maxx / h)) + 1;
    const int j0 = static_cast<int>(std::floor(miny / h)) - 1, j1 = static_cast<int>(std::floor(maxy / h)) + 1;
    for (int i = i0; i <= i1; ++i)
        for (int j = j0; j <= j1; ++j) {
            bool all = true;
            for (const auto & ax : axes) if (!overlapsOnAxis(poly, i, j, h, ax, min_overlap)) { all = false; break; }
            if (all) out.insert({i, j});
        }
}


/// Cells touched by the footprint while executing `u` from `start` (world frame, cells of size `h`).
/// With `start_only`, only the start pose is used. `closed` counts boundary contact as touching (overlap
/// >= -1e-9) instead of requiring positive overlap; `step_div` sets the pose spacing to h/step_div.
inline std::set<Cell> motionTouchedCells(const RobotParams & body, const DiffDriveStateSpace & space, double h,
                                         const SE2State & start, const Control & u, bool start_only = false,
                                         bool closed = false, double step_div = 4.0){
    const int J = start_only ? 0 :
        std::max(1, static_cast<int>(std::ceil((std::abs(u.v) + body.r_out * std::abs(u.omega)) * u.dt / (h / step_div))));
    std::set<Cell> out;
    for (int j = 0; j <= J; ++j) {
        const SE2State p = (j == 0) ? start : space.propagate(start, Control{u.v, u.omega, u.dt * j / J});
        touchedCells(body, p, h, out, closed ? -kMinOverlap : kMinOverlap);
    }
    return out;
}

}  // namespace swept_oracle
