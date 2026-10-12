#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <numbers>
#include <random>
#include <set>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/planner/primitives/control_primitives.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "core/types/grid_coords.hpp"
#include "../../../../support/swept_oracle.hpp"

/*
Cover property of the per-heading-bin swept-cell cache. For a primitive u, a heading theta in bin k and
a continuous start pose (a, b, theta) inside the grid cell (0, 0), every cell the footprint touches while
executing u must appear in prim.swept_cells[k] (cells are body-frame offsets from the start cell, so
they compare directly).

The oracle (support/swept_oracle.hpp) is independent of rasterizePolygon and computeExactCover and
under-approximates the true swept set, so every reported miss is genuine.
*/

namespace {

using swept_oracle::Cell;
constexpr double PI = std::numbers::pi;

struct Config {
    const char * body_name;
    int cases;  // measurement cases drawn for this configuration
    const char * kin_name;
    DiffDriveParams kin;
    double h;
    RobotParams body;
    std::shared_ptr<const DiffDriveStateSpace> space;
    std::unique_ptr<DiffDrivePrimLib> lib;
};

RobotParams makeBody(int which){
    switch (which) {
        case 0:  // 0.2 x 0.2 square: r_in = 0.1, r_out = sqrt(0.02) ~ 0.1414.
            return buildRobotParams({Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
        case 1:  // off-centre rectangle (origin near one end)
            return buildRobotParams({Vec2(0.5, 0.2), Vec2(-0.1, 0.2), Vec2(-0.1, -0.15), Vec2(0.5, -0.15)});
        default:  // pentagon
            return buildRobotParams({Vec2(0.4, 0.0), Vec2(0.0, 0.12), Vec2(-0.2, 0.05), Vec2(-0.2, -0.05), Vec2(0.0, -0.12)});
    }
}

const std::vector<std::unique_ptr<Config>> & configs(){
    static const auto all = [] {
        std::vector<std::unique_ptr<Config>> v;
        auto add = [&v](int body, const char * body_name, int cases, const char * kin_name,
                        double vmax, double dt, int N, double h) {
            auto c = std::make_unique<Config>();
            c->body_name = body_name; c->cases = cases; c->kin_name = kin_name;
            c->kin.rmin = 0.3; c->kin.vmax = vmax; c->kin.atomic_time = dt;
            c->kin.n_headings = N;
            c->h = h;
            c->body = makeBody(body);
            GridMetaData meta{100, 100, h, Eigen::Vector2f::Zero()};
            auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(c->kin.rmin * c->kin.rmin);
            c->space = std::make_shared<const DiffDriveStateSpace>(meta, dist);
            c->lib = std::make_unique<DiffDrivePrimLib>(MotionContext{c->body, c->space, meta}, c->kin);
            v.push_back(std::move(c));
        };
        const std::pair<const char *, std::pair<double, double>> kins[] = {
            {"SHIPPED", {3.0, 1.0}}, {"SHORT", {0.5, 0.6}}};
        for (const auto & [name, vt] : kins)
            for (int N : {8, 16, 32, 64})
                for (double h : {0.05, 0.1}) add(0, "square", 5000, name, vt.first, vt.second, N, h);
        for (int body : {1, 2})
            for (int N : {8, 32})
                for (double h : {0.05, 0.1}) add(body, body == 1 ? "offcentre_rect" : "pentagon", 1000, "SHORT", 0.5, 0.6, N, h);
        return v;
    }();
    return all;
}

struct CaseResult {
    std::set<Cell> oracle;
    std::vector<Cell> miss;
    int k = 0;
    Control control{};
    double theta = 0;
};

// Evaluate one case. start_only restricts the oracle to the start pose (j = 0); k_override, when
// non-negative, selects the cache bin directly instead of deriving it from theta.
CaseResult evalCase(const Config & cfg, std::size_t prim_idx, double theta, double a, double b,
                    bool start_only = false, int k_override = -1){
    const Primitive & prim = cfg.lib->primitives()[prim_idx];
    const Control & u = prim.control;
    CaseResult r;
    r.control = u; r.theta = theta;
    r.k = (k_override >= 0) ? k_override : cfg.lib->headingBin(theta);

    r.oracle = swept_oracle::motionTouchedCells(cfg.body, *cfg.space, cfg.h, SE2State{a, b, theta}, u, start_only);

    std::set<Cell> cached;
    for (const auto & g : prim.swept_cells[r.k]) cached.insert({g.i, g.j});
    for (const auto & c : r.oracle) if (!cached.count(c)) r.miss.push_back(c);
    return r;
}

double thetaFor(const Config & cfg, int k_hint_unused, double t, int k){
    (void)k_hint_unused;
    const double delta = 2.0 * PI / cfg.kin.n_headings;
    return -PI + (k + t) * delta;
}

}  // namespace

RC_GTEST_PROP(SweptCover, everyTouchedCellIsCached, ()){
    const auto & all = configs();
    const auto & cfg = *all[*rc::gen::inRange<std::size_t>(0, all.size())];
    const std::size_t pi = *rc::gen::inRange<std::size_t>(0, cfg.lib->primitives().size());
    const int k = *rc::gen::inRange(0, cfg.kin.n_headings);
    const double t = *rc::gen::inRange(0, 1000) / 1000.0;
    const double a = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const double b = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const double theta = thetaFor(cfg, 0, t, k);
    const auto r = evalCase(cfg, pi, theta, a, b);
    RC_LOG() << cfg.body_name << " " << cfg.kin_name << " N=" << cfg.kin.n_headings << " h=" << cfg.h << " prim=" << pi
             << " v=" << r.control.v << " w=" << r.control.omega << " dt=" << r.control.dt
             << " k=" << r.k << " theta=" << theta << " a=" << a << " b=" << b
             << " oracle=" << r.oracle.size() << " miss=" << r.miss.size() << std::endl;
    RC_ASSERT(r.miss.empty());
}

TEST(SweptCoverMeasurement, reportsMissesOverSeededSample){
    const auto t0 = std::chrono::steady_clock::now();
    std::mt19937_64 rng(20261005);
    long total_cases = 0, total_miss_cases = 0;

    struct Worst { const Config * cfg = nullptr; std::size_t prim = 0; double a = 0, b = 0; CaseResult r; } worst;

    for (const auto & cp : configs()) {
        const Config & cfg = *cp;
        const std::size_t P = cfg.lib->primitives().size();
        std::uniform_int_distribution<std::size_t> pd(0, P - 1);
        std::uniform_real_distribution<double> ud(0.0, 1.0);
        long cases = 0, with_miss = 0; std::size_t max_miss = 0; double frac_sum = 0;
        for (int n = 0; n < cfg.cases; ++n) {
            const std::size_t prim = pd(rng);
            const double t = ud(rng), a = ud(rng) * cfg.h, b = ud(rng) * cfg.h;
            const int kg = std::min(cfg.kin.n_headings - 1, static_cast<int>(ud(rng) * cfg.kin.n_headings));
            const double theta = thetaFor(cfg, 0, t, kg);
            auto r = evalCase(cfg, prim, theta, a, b);
            ++cases;
            if (!r.miss.empty()) ++with_miss;
            frac_sum += r.oracle.empty() ? 0.0 : double(r.miss.size()) / double(r.oracle.size());
            max_miss = std::max(max_miss, r.miss.size());
            if (r.miss.size() > worst.r.miss.size()) worst = {&cfg, prim, a, b, std::move(r)};
        }
        total_cases += cases; total_miss_cases += with_miss;
        std::printf("%s %s N=%d h=%.2f cases=%ld cases_with_miss=%ld max_missed_cells=%zu mean_missed_fraction=%.4f\n",
                    cfg.body_name, cfg.kin_name, cfg.kin.n_headings, cfg.h, cases, with_miss, max_miss, frac_sum / cases);
    }

    if (worst.cfg) {
        const Config & cfg = *worst.cfg;
        const auto & cached = cfg.lib->primitives()[worst.prim].swept_cells[worst.r.k];
        int far_d = -1; Cell far_c{0, 0};
        for (const auto & m : worst.r.miss) {
            int best = 1 << 30;
            for (const auto & g : cached) best = std::min(best, std::max(std::abs(g.i - m.first), std::abs(g.j - m.second)));
            if (best > far_d) { far_d = best; far_c = m; }
        }
        std::printf("WORST %s %s N=%d h=%.2f control(v=%.4f w=%.4f dt=%.2f) k=%d theta=%.6f a=%.6f b=%.6f "
                    "oracle=%zu miss=%zu farthest_miss=(%d,%d) chebyshev_to_cache=%d\n",
                    cfg.body_name, cfg.kin_name, cfg.kin.n_headings, cfg.h, worst.r.control.v, worst.r.control.omega,
                    worst.r.control.dt, worst.r.k, worst.r.theta, worst.a, worst.b,
                    worst.r.oracle.size(), worst.r.miss.size(), far_c.first, far_c.second, far_d);
    }
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("measurement wall time %.2f s, total cases %ld\n", secs, total_cases);
    EXPECT_EQ(total_miss_cases, 0);
}

// Oracle sanity: at theta exactly theta_k, offset (0, 0), the start pose alone is the pose the cache
// rasterizes, so its oracle cells must all be cached (straight and pivot primitives).
TEST(SweptCoverOracle, startPoseAtBinAnchorIsCached){
    for (const auto & cp : configs()) {
        const Config & cfg = *cp;
        const auto & prims = cfg.lib->primitives();
        for (std::size_t pi = 0; pi < prims.size(); ++pi) {
            const bool straight = prims[pi].control.omega == 0.0;
            const bool pivot = prims[pi].control.v == 0.0;
            if (!straight && !pivot) continue;
            for (int k = 0; k < cfg.kin.n_headings; ++k) {
                const double theta = -PI + k * (2.0 * PI / cfg.kin.n_headings);
                const auto r = evalCase(cfg, pi, theta, 0.0, 0.0, /*start_only=*/true, k);
                EXPECT_FALSE(r.oracle.empty());
                EXPECT_TRUE(r.miss.empty()) << cfg.kin_name << " N=" << cfg.kin.n_headings << " h=" << cfg.h
                                            << " prim=" << pi << " k=" << k << " miss=" << r.miss.size();
            }
        }
    }
}
