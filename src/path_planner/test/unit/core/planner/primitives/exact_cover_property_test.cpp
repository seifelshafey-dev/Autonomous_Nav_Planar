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
#include "core/planner/primitives/control_primitives.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/robot/robot_params.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"
#include "core/types/control.hpp"
#include "../../../../support/swept_oracle.hpp"

/*
Exact-cover property of MotionContext::computeExactCover. For a primitive control u from the library
control set, a continuous start pose (any heading, any in-cell offset) and a grid origin (zero or not),
every cell an independent geometric oracle reports as touched while executing u must appear in `cells`.
The oracle (support/swept_oracle.hpp) under-approximates the true swept set, so every miss is genuine.
*/

namespace {

using swept_oracle::Cell;
constexpr double PI = std::numbers::pi;

struct Config {
    const char * body_name;
    const char * kin_name;
    DiffDriveParams kin;
    double h;
    RobotParams body;
    std::shared_ptr<const DiffDriveStateSpace> space;
    std::vector<Control> controls;
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
        const char * names[] = {"square", "offcentre_rect", "pentagon"};
        const std::pair<const char *, std::pair<double, double>> kins[] = {
            {"SHIPPED", {3.0, 1.0}}, {"SHORT", {0.5, 0.6}}};
        for (int body = 0; body < 3; ++body)
            for (const auto & [kname, vt] : kins)
                for (double h : {0.05, 0.1}) {
                    auto c = std::make_unique<Config>();
                    c->body_name = names[body]; c->kin_name = kname;
                    c->kin.rmin = 0.3; c->kin.vmax = vt.first; c->kin.atomic_time = vt.second;
                    c->kin.n_headings = 16;
                    c->h = h;
                    c->body = makeBody(body);
                    GridMetaData meta{100, 100, h, Eigen::Vector2f::Zero()};
                    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(c->kin.rmin * c->kin.rmin);
                    c->space = std::make_shared<const DiffDriveStateSpace>(meta, dist);
                    DiffDrivePrimLib lib(MotionContext{c->body, c->space, meta}, c->kin);
                    for (const auto & p : lib.primitives()) c->controls.push_back(p.control);
                    v.push_back(std::move(c));
                }
        return v;
    }();
    return all;
}

// Misses of computeExactCover against the oracle for one case. `origin_kind` 0 -> origin (0, 0),
// otherwise a non-zero origin that is not a multiple of h.
std::vector<Cell> misses(const Config & cfg, std::size_t ci, double theta, double a, double b, int origin_kind){
    const Eigen::Vector2f origin = origin_kind == 0 ? Eigen::Vector2f(0.f, 0.f) : Eigen::Vector2f(0.37f, -0.21f);
    MotionContext ctx(cfg.body, cfg.space, GridMetaData{100, 100, cfg.h, origin});
    // Map-frame start; the oracle works in the origin-relative frame, exactly as the context does.
    const SE2State start{a + origin.x(), b + origin.y(), theta};
    const SE2State rel{start.x - static_cast<double>(origin.x()), start.y - static_cast<double>(origin.y()), theta};
    const Control & u = cfg.controls[ci];
    const auto got = ctx.computeExactCover(start, u).cells;
    std::set<Cell> have;
    for (const auto & g : got) have.insert({g.i, g.j});
    std::vector<Cell> out;
    for (const auto & c : swept_oracle::motionTouchedCells(cfg.body, *cfg.space, cfg.h, rel, u))
        if (!have.count(c)) out.push_back(c);
    return out;
}

// True if visitExactCover and computeExactCover.cells disagree as sets.
bool visitDiffersFromCompute(const Config & cfg, std::size_t ci, double theta, double a, double b, int origin_kind){
    const Eigen::Vector2f origin = origin_kind == 0 ? Eigen::Vector2f(0.f, 0.f) : Eigen::Vector2f(0.37f, -0.21f);
    MotionContext ctx(cfg.body, cfg.space, GridMetaData{100, 100, cfg.h, origin});
    const SE2State start{a + origin.x(), b + origin.y(), theta};
    std::set<Cell> have, visited;
    for (const auto & g : ctx.computeExactCover(start, cfg.controls[ci]).cells) have.insert({g.i, g.j});
    ctx.visitExactCover(start, cfg.controls[ci], [&](GridCoords g){ visited.insert({g.i, g.j}); return true; });
    return have != visited;
}

}  // namespace

RC_GTEST_PROP(ExactCover, visitMatchesComputeAsSet, ()){
    const auto & all = configs();
    const auto & cfg = *all[*rc::gen::inRange<std::size_t>(0, all.size())];
    const std::size_t ci = *rc::gen::inRange<std::size_t>(0, cfg.controls.size());
    const double theta = -PI + *rc::gen::inRange(0, 100000) / 100000.0 * 2.0 * PI;
    const double a = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const double b = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const int ok = *rc::gen::inRange(0, 2);
    RC_LOG() << cfg.body_name << " " << cfg.kin_name << " h=" << cfg.h << " ctrl=" << ci
             << " theta=" << theta << " a=" << a << " b=" << b << " origin_kind=" << ok << std::endl;
    RC_ASSERT(!visitDiffersFromCompute(cfg, ci, theta, a, b, ok));
}

RC_GTEST_PROP(ExactCover, everyTouchedCellIsCovered, ()){
    const auto & all = configs();
    const auto & cfg = *all[*rc::gen::inRange<std::size_t>(0, all.size())];
    const std::size_t ci = *rc::gen::inRange<std::size_t>(0, cfg.controls.size());
    const double theta = -PI + *rc::gen::inRange(0, 100000) / 100000.0 * 2.0 * PI;
    const double a = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const double b = *rc::gen::inRange(0, 1000) / 1000.0 * cfg.h;
    const int ok = *rc::gen::inRange(0, 2);
    RC_LOG() << cfg.body_name << " " << cfg.kin_name << " h=" << cfg.h << " ctrl=" << ci
             << " theta=" << theta << " a=" << a << " b=" << b << " origin_kind=" << ok << std::endl;
    RC_ASSERT(misses(cfg, ci, theta, a, b, ok).empty());
}

TEST(ExactCoverMeasurement, noMissesOverSeededSample){
    std::mt19937_64 rng(20261006);
    std::uniform_real_distribution<double> ud(0.0, 1.0);
    constexpr int kCasesPerConfig = 1000;
    long total = 0, bad = 0;
    for (const auto & cp : configs()) {
        const Config & cfg = *cp;
        std::uniform_int_distribution<std::size_t> cd(0, cfg.controls.size() - 1);
        for (int n = 0; n < kCasesPerConfig; ++n) {
            const std::size_t ci = cd(rng);
            const double theta = -PI + ud(rng) * 2.0 * PI, a = ud(rng) * cfg.h, b = ud(rng) * cfg.h;
            const int ok = n % 2;
            const auto m = misses(cfg, ci, theta, a, b, ok);
            ++total;
            if (!m.empty()) {
                ++bad;
                ADD_FAILURE() << cfg.body_name << " " << cfg.kin_name << " h=" << cfg.h << " ctrl=" << ci
                              << " theta=" << theta << " a=" << a << " b=" << b << " origin_kind=" << ok
                              << " first_miss=(" << m[0].first << "," << m[0].second << ") misses=" << m.size();
            }
        }
    }
    std::printf("exact-cover cases=%ld cases_with_miss=%ld\n", total, bad);
    EXPECT_GE(total, 10000);
    EXPECT_EQ(bad, 0);
}
