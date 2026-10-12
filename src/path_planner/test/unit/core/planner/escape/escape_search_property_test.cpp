#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <vector>
#include <Eigen/Core>

#include "core/planner/escape/escape_search.hpp"
#include "core/collision_checker/collision_checker.hpp"
#include "core/map/edt.hpp"
#include "core/map/grid.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/motion_context.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/robot/robot_params.hpp"

/*
Escape move is invariant under shifting the map origin and the start pose together by a multiple of
0.125 m (exact in float and double): same outcome, same segment count, same direction and duration.
*/

namespace {

constexpr double H = 0.05;
constexpr int N = 60;

std::optional<Path> run(const std::vector<int8_t> & cells, const Eigen::Vector2f & origin, const SE2State & start) {
    DiffDriveParams k;
    k.rmin = 0.3; k.vmax = 0.5; k.atomic_time = 0.6; k.n_headings = 16;
    GridMetaData meta(N, N, H, origin);
    auto grid = std::make_shared<const OccupancyGrid>(meta, cells);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid, 5.0));
    auto body = buildRobotParams({{0.3, 0.3}, {-0.3, 0.3}, {-0.3, -0.3}, {0.3, -0.3}});
    CollisionChecker checker(grid, edt, body);
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(k.rmin * k.rmin);
    auto space = std::make_shared<const DiffDriveStateSpace>(meta, dist);
    MotionContext motion(body, space, meta);
    return findEscape(start, checker, motion, k, EscapeConfig{}, 0.0);
}

}  // namespace

RC_GTEST_PROP(EscapeSearchProperty, resultInvariantUnderOriginShift, ()) {
    const unsigned seed = *rc::gen::arbitrary<unsigned>();
    const int a = *rc::gen::inRange(-80, 81);
    const int b = *rc::gen::inRange(-80, 81);
    const int sx = *rc::gen::inRange(0, 1001);
    const int sy = *rc::gen::inRange(0, 1001);
    const int sth = *rc::gen::inRange(0, 3600);
    const bool block_start = *rc::gen::inRange(0, 4) != 0;  // three quarters of the cases

    std::mt19937 rng(seed);
    std::bernoulli_distribution occ(0.002);
    std::vector<int8_t> cells(static_cast<size_t>(N) * N);
    for (auto & c : cells) c = occ(rng) ? 100 : 0;

    // Middle third of the 3 m map: [1.0, 2.0] m.
    const SE2State s0{1.0 + sx / 1000.0, 1.0 + sy / 1000.0, -std::numbers::pi + sth * (2.0 * std::numbers::pi / 3600.0)};
    if (block_start) {
        // One occupied cell 0.33 m ahead of the start: inside the start gate's margin.
        const int ci = static_cast<int>(std::floor((s0.x + 0.33 * std::cos(s0.theta)) / H));
        const int cj = static_cast<int>(std::floor((s0.y + 0.33 * std::sin(s0.theta)) / H));
        cells[static_cast<size_t>(cj) * N + ci] = 100;
    }
    const double tx = a * 0.125, ty = b * 0.125;
    const SE2State s1{s0.x + tx, s0.y + ty, s0.theta};

    const auto p0 = run(cells, Eigen::Vector2f::Zero(), s0);
    const auto p1 = run(cells, Eigen::Vector2f(static_cast<float>(tx), static_cast<float>(ty)), s1);

    RC_CLASSIFY(block_start, "blocked start requested");
    RC_CLASSIFY(p0.has_value() && p0->segments.size() == 1, "one segment");
    RC_CLASSIFY(!p0.has_value(), "no escape");
    RC_ASSERT(p0.has_value() == p1.has_value());
    if (!p0) return;
    RC_ASSERT(p0->segments.size() == p1->segments.size());
    if (p0->segments.size() == 1) {
        RC_ASSERT((p0->segments[0].u.v < 0.0) == (p1->segments[0].u.v < 0.0));
        RC_ASSERT(std::abs(p0->segments[0].u.dt - p1->segments[0].u.dt) <= 1e-9);
    }
}
