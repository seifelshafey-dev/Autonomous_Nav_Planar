/**
 * @file build_diff_prim_lib.cpp
 * @author Seif ElShafey
 * @brief Implementation of DiffDrivePrimLib — builds the per-control, per-heading-bin motion primitive library.
 * @version 0.2
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

#include "core/diagnostics/check.hpp"
#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/map/rasterize.hpp"
#include "core/types/math/wrap_angle.hpp"

DiffDrivePrimLib::DiffDrivePrimLib(MotionContext motion, DiffDriveParams kin)
    : motion_(std::move(motion)),
      kin_(std::move(kin))
{
    CHECK(kin_.rmin > 0 && kin_.vmax > 0 && kin_.atomic_time > 0 && kin_.n_headings >= 1,
          "DiffDriveParams must be positive with at least one heading bin");
    CHECK(motion_.gridMeta().resolution > 0, "grid resolution must be positive");
    CHECK(motion_.body().r_out > 0, "robot r_out must be positive");

    const auto t0 = std::chrono::steady_clock::now();
    const auto controls = buildControlSet();
    primitives_.reserve(controls.size());
    for (const auto & u : controls) {
        primitives_.push_back(buildPrimitive(u));
    }
    build_time_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::vector<Control> DiffDrivePrimLib::buildControlSet() const {
    std::vector<Control> control_set;
    // Defined by arcs of radii scaled by the minimum turning radius of the robot.
    const std::vector<double> radius = {0, 0.5*kin_.rmin, kin_.rmin, 2*kin_.rmin, 4*kin_.rmin};

    Control straight;
    straight.v = kin_.vmax; straight.omega = 0.0; straight.dt = kin_.atomic_time;
    control_set.push_back(straight);

    Control straight_rev;
    straight_rev.v = -kin_.vmax; straight_rev.omega = 0.0; straight_rev.dt = kin_.atomic_time;
    control_set.push_back(straight_rev);

    for (const auto & x : radius) {
        double velocity     = kin_.vmax * x / (x + kin_.rmin);
        double rot_velocity = kin_.vmax / (x + kin_.rmin);

        Control left_cntrl;
        left_cntrl.v = velocity; left_cntrl.omega = rot_velocity; left_cntrl.dt = kin_.atomic_time;
        Control right_cntrl;
        right_cntrl.v = velocity; right_cntrl.omega = -rot_velocity; right_cntrl.dt = kin_.atomic_time;

        control_set.push_back(left_cntrl);
        control_set.push_back(right_cntrl);
    }
    return control_set;
}

Primitive DiffDrivePrimLib::buildPrimitive(const Control & u) const
{
    Primitive prim;
    prim.control = u;

    auto cost_fcn = makeControlCost(kin_);
    prim.endpoint_offset = motion_.space().propagate(SE2State{0, 0, 0}, u);
    prim.cost = cost_fcn(u);

    prim.swept_cells.resize(kin_.n_headings);
    prim.sample_cells.resize(kin_.n_headings);

    // Cells are anchored at the body origin: a local context with grid origin (0, 0) gives body-frame
    // cells that are integer-translated by the robot's map cell at expansion time.
    const auto & map_meta = motion_.gridMeta();
    GridMetaData body_meta{
        map_meta.cols,
        map_meta.rows,
        map_meta.resolution,
        Eigen::Vector2f::Zero()
    };
    MotionContext body_motion{motion_.body(), motion_.spacePtr(), std::move(body_meta)};

    const double delta = (2.0 * std::numbers::pi) / kin_.n_headings;
    for (int k = 0; k < kin_.n_headings; ++k) {
        const double theta_k = -std::numbers::pi + k * delta;
        prim.swept_cells[k] = buildCover(u, k);
        const auto poses = body_motion.sampleTrajectory(SE2State{0.0, 0.0, theta_k + 0.5 * delta}, u);
        auto & sc = prim.sample_cells[k];
        sc.reserve(poses.size());
        for (const auto & p : poses) sc.push_back(body_motion.gridMeta().worldToGrid(p.x, p.y));
    }

    // Per-sample clearance; rho is rotation-invariant so heading 0 suffices.
    const auto poses0 = body_motion.sampleTrajectory(SE2State{0.0, 0.0, 0.0}, u);
    const double r_out = motion_.body().r_out;
    const double h = motion_.gridMeta().resolution;
    const double n_int = static_cast<double>(poses0.size() - 1);
    const double sigma = std::abs(u.v) * u.dt / n_int;
    const double s4 = std::sin(delta / 4.0);
    prim.sample_clearance.reserve(poses0.size());
    for (const auto & p : poses0) {
        const double rho = std::hypot(p.x, p.y);
        const double need = r_out + 2.0 * rho * s4 + 2.0 * std::numbers::sqrt2 * h + 0.5 * sigma;
        CHECK(need >= r_out, "sample clearance must be at least r_out");
        prim.sample_clearance.push_back(need);
    }

    return prim;
}

std::vector<GridCoords> DiffDrivePrimLib::buildCover(const Control & u, int k) const
{
    const double h = motion_.gridMeta().resolution;
    const double r_out = motion_.body().r_out;
    const double delta = (2.0 * std::numbers::pi) / kin_.n_headings;
    const double theta_k = -std::numbers::pi + k * delta;
    const double arc = std::abs(u.v) * u.dt;

    const int M = std::max(1, static_cast<int>(std::ceil((arc + r_out) * delta / h)));
    const int n = std::max(1, static_cast<int>(std::ceil((std::abs(u.v) + r_out * std::abs(u.omega)) * u.dt / h)));
    const double m_theta = (arc + r_out) * delta / (2.0 * M);
    const double m_s = (std::abs(u.v) + r_out * std::abs(u.omega)) * u.dt / (2.0 * n);
    CHECK(m_theta <= h / 2 + 1e-12, "heading margin exceeds half a cell");
    CHECK(m_s <= h / 2 + 1e-12, "arc margin exceeds half a cell");
    const double margin = m_theta + m_s;

    std::vector<GridCoords> cells;
    for (int m = 0; m <= M; ++m) {
        const double theta = theta_k + m * delta / M;
        for (int j = 0; j <= n; ++j) {
            const SE2State pose = motion_.space().propagate(
                SE2State{0.0, 0.0, theta}, Control{u.v, u.omega, u.dt * j / n});
            const auto geo = transformGeometry(motion_.body(), pose, Vec2::Zero());
            for (const auto & c : rasterizePolygon(geo.half_planes, geo.aabb, h, margin)) {
                cells.push_back(c);
                cells.push_back({c.i + 1, c.j});
                cells.push_back({c.i, c.j + 1});
                cells.push_back({c.i + 1, c.j + 1});
            }
        }
    }
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    return cells;
}

int16_t DiffDrivePrimLib::headingBin(double theta) const noexcept {
    const double bin_size = (2.0 * std::numbers::pi) / kin_.n_headings;
    const double wrapped  = wrapToPi(theta);

    int bin = static_cast<int>(std::floor((wrapped + std::numbers::pi) / bin_size));
    if (bin == kin_.n_headings) bin = 0;

    return static_cast<int16_t>(bin);
}
