#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <random>
#include <set>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/map/grid.hpp"
#include "core/map/rasterize.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/state.hpp"
#include "core/types/grid_coords.hpp"

/*
Agreement of the swept-cell cache with the collision checker's own scan. For a primitive u, a heading in
bin k and a start pose inside grid cell c (grid origin may be non-zero), every cell that
`CollisionChecker::checkState` scans at a pose of `sampleTrajectory(start, u)` (transformGeometry with the
grid origin, then rasterizePolygon with margin 0) must be a member of `swept_cells[k] + c`.
*/

namespace {

constexpr double PI = std::numbers::pi;

struct Config {
    const char * kin_name;
    DiffDriveParams kin;
    double h;
    RobotParams body;
    GridMetaData meta{1, 1, 1.0, Eigen::Vector2f::Zero()};   // checker-side grid (possibly non-zero origin)
    std::unique_ptr<MotionContext> ctx;
    std::unique_ptr<DiffDrivePrimLib> lib;
};

RobotParams makeBody(int which){
    switch (which) {
        case 0:
            return buildRobotParams({Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
        case 1:
            return buildRobotParams({Vec2(0.5, 0.2), Vec2(-0.1, 0.2), Vec2(-0.1, -0.15), Vec2(0.5, -0.15)});
        default:
            return buildRobotParams({Vec2(0.4, 0.0), Vec2(0.0, 0.12), Vec2(-0.2, 0.05), Vec2(-0.2, -0.05), Vec2(0.0, -0.12)});
    }
}

const std::vector<std::unique_ptr<Config>> & configs(){
    static const auto all = [] {
        std::vector<std::unique_ptr<Config>> v;
        for (int body = 0; body < 3; ++body)
            for (int slow = 0; slow < 2; ++slow)
                for (int N : {16, 32, 72})
                    for (double h : {0.05, 0.1}) {
                        auto c = std::make_unique<Config>();
                        c->kin_name = slow ? "SLOW" : "NOMINAL";
                        c->kin.rmin = 0.3; c->kin.vmax = 0.5;
                        c->kin.atomic_time = slow ? 2.4 * h : 0.6;   // slow: straight reach = 1.2 h
                        c->kin.n_headings = N;
                        c->h = h;
                        c->body = makeBody(body);
                        const bool shifted = (N == 32 && h == 0.1);
                        const GridMetaData body_meta{100, 100, h, Eigen::Vector2f::Zero()};
                        c->meta = GridMetaData{100, 100, h,
                                               shifted ? Eigen::Vector2f(0.37f, -0.21f) : Eigen::Vector2f::Zero()};
                        auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(c->kin.rmin * c->kin.rmin);
                        auto space = std::make_shared<const DiffDriveStateSpace>(body_meta, dist);
                        c->ctx = std::make_unique<MotionContext>(c->body, space, body_meta);
                        c->lib = std::make_unique<DiffDrivePrimLib>(MotionContext{c->body, space, body_meta}, c->kin);
                        v.push_back(std::move(c));
                    }
        return v;
    }();
    return all;
}

struct Miss { std::size_t pose; GridCoords cell; };

// Returns the first scanned cell not in swept_cells[k] + c, if any.
std::vector<Miss> evalCase(const Config & cfg, std::size_t prim_idx, int k, double t, double a, double b,
                           int ci, int cj, SE2State & start_out){
    const Primitive & prim = cfg.lib->primitives()[prim_idx];
    const double delta = 2.0 * PI / cfg.kin.n_headings;
    const double theta = -PI + (k + t) * delta;
    const SE2State start{cfg.meta.origin.x() + (ci + a) * cfg.h, cfg.meta.origin.y() + (cj + b) * cfg.h, theta};
    start_out = start;
    const GridCoords c = cfg.meta.worldToGrid(start.x, start.y);

    std::set<std::pair<int, int>> cover;
    for (const auto & g : prim.swept_cells[k]) cover.insert({g.i + c.i, g.j + c.j});

    std::vector<Miss> miss;
    const auto poses = cfg.ctx->sampleTrajectory(start, prim.control);
    for (std::size_t i = 0; i < poses.size(); ++i) {
        const auto geo = transformGeometry(cfg.body, poses[i], cfg.meta.origin.cast<double>());
        for (const auto & g : rasterizePolygon(geo.half_planes, geo.aabb, cfg.meta.resolution))
            if (!cover.count({g.i, g.j})) miss.push_back({i, g});
    }
    return miss;
}

}  // namespace

RC_GTEST_PROP(AgreementCover, everyScannedCellIsInTheCover, ()){
    const auto & all = configs();
    const auto & cfg = *all[*rc::gen::inRange<std::size_t>(0, all.size())];
    const std::size_t pi = *rc::gen::inRange<std::size_t>(0, cfg.lib->primitives().size());
    const int k = *rc::gen::inRange(0, cfg.kin.n_headings);
    const double t = *rc::gen::inRange(0, 1000) / 1000.0;
    const double a = *rc::gen::inRange(0, 1000) / 1000.0;
    const double b = *rc::gen::inRange(0, 1000) / 1000.0;
    const int ci = *rc::gen::inRange(-50, 50), cj = *rc::gen::inRange(-50, 50);
    SE2State s{};
    const auto miss = evalCase(cfg, pi, k, t, a, b, ci, cj, s);
    RC_LOG() << cfg.kin_name << " N=" << cfg.kin.n_headings << " h=" << cfg.h << " prim=" << pi << " k=" << k
             << " theta=" << s.theta << " a=" << a << " b=" << b << " misses=" << miss.size() << std::endl;
    RC_ASSERT(miss.empty());
}

TEST(AgreementCoverMeasurement, seededSampleHasNoMiss){
    std::mt19937_64 rng(20261006);
    long cases = 0, with_miss = 0;
    int reported = 0;
    constexpr int kPerConfig = 700;
    for (const auto & cp : configs()) {
        const Config & cfg = *cp;
        std::uniform_int_distribution<std::size_t> pd(0, cfg.lib->primitives().size() - 1);
        std::uniform_int_distribution<int> kd(0, cfg.kin.n_headings - 1), cd(-50, 50);
        std::uniform_real_distribution<double> ud(0.0, 1.0);
        for (int n = 0; n < kPerConfig; ++n) {
            const std::size_t prim = pd(rng);
            const int k = kd(rng);
            const double t = ud(rng), a = ud(rng), b = ud(rng);
            SE2State s{};
            const auto miss = evalCase(cfg, prim, k, t, a, b, cd(rng), cd(rng), s);
            ++cases;
            if (miss.empty()) continue;
            ++with_miss;
            if (reported++ < 5) {
                const Control & u = cfg.lib->primitives()[prim].control;
                ADD_FAILURE() << cfg.kin_name << " N=" << cfg.kin.n_headings << " h=" << cfg.h << " origin=("
                              << cfg.meta.origin.x() << "," << cfg.meta.origin.y() << ") prim=" << prim
                              << " u=(" << u.v << "," << u.omega << "," << u.dt << ") k=" << k
                              << " theta=" << s.theta << " offset=(" << a << "," << b << ") pose_index="
                              << miss.front().pose << " missing_cell=(" << miss.front().cell.i << ","
                              << miss.front().cell.j << ")";
            }
        }
    }
    std::printf("agreement-cover cases=%ld cases_with_miss=%ld\n", cases, with_miss);
    EXPECT_GE(cases, 20000);
    EXPECT_EQ(with_miss, 0);
}
