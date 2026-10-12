#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
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

/*
DiffDrivePrimLib builds, at construction, one Primitive per control in buildControlSet(). Tests cover
the three surfaces:
  - buildControlSet (observed via primitives()[i].control): 2 straights + 5 arc-radii x {L,R} = 12
    controls, dt == atomic_time, arc velocities per v = vmax*x/(x+rmin), omega = +-vmax/(x+rmin).
  - buildPrimitive's own logic: endpoint_offset = propagate({0,0,0}, u), cost = makeControlCost(kin)(u),
    and swept_cells / sample_cells each sized n_headings with every bin populated. The dedup/overlap
    inside the cell lists is computeExactCover's contract (tested there) and is NOT re-asserted here.
  - headingBin: floor((wrapToPi(theta)+pi)/bin_size) with the +pi-seam clamp.

Collaborators are real and controlled: a small square body via buildRobotParams and a
real DiffDriveStateSpace. The body is small so the per-bin rasterization is cheap.
*/

namespace {

constexpr double PI = std::numbers::pi;

DiffDriveParams makeKin(){
    DiffDriveParams kin;
    kin.rmin = 0.5; kin.vmax = 1.0; kin.atomic_time = 0.4; kin.n_headings = 8;
    return kin;
}

RobotParams makeBody(){
    // 0.2 x 0.2 square, CCW, contains the origin -> r_in = 0.1, r_out = sqrt(0.02).
    return buildRobotParams({Vec2(0.1, 0.1), Vec2(-0.1, 0.1), Vec2(-0.1, -0.1), Vec2(0.1, -0.1)});
}

std::shared_ptr<const DiffDriveStateSpace> makeSpace(const DiffDriveParams & kin){
    auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
    return std::make_shared<const DiffDriveStateSpace>(
        GridMetaData{100, 100, 0.1, Eigen::Vector2f::Zero()}, dist);
}

DiffDrivePrimLib makeLib(const DiffDriveParams & kin){
    MotionContext motion{makeBody(), makeSpace(kin),
                         GridMetaData{100, 100, 0.1, Eigen::Vector2f::Zero()}};
    return DiffDrivePrimLib{std::move(motion), kin};
}

}  // namespace

// ---------------------------------------------------------------------------
// Control set (observed through the built primitives).
// ---------------------------------------------------------------------------

TEST(ControlSet, hasTwelveControls){
    EXPECT_EQ(makeLib(makeKin()).primitives().size(), 12u);
}

TEST(ControlSet, exactlyTwoStraightsAtPlusMinusVmax){
    auto lib = makeLib(makeKin());
    int zero_omega = 0;
    bool fwd = false, rev = false;
    for (const auto & p : lib.primitives()){
        if (p.control.omega == 0.0){
            ++zero_omega;
            if (p.control.v == 1.0) fwd = true;
            if (p.control.v == -1.0) rev = true;
        }
    }
    EXPECT_EQ(zero_omega, 2);
    EXPECT_TRUE(fwd);
    EXPECT_TRUE(rev);
}

TEST(ControlSet, pivotInPlaceIsPresent){
    // x = 0 arc: v = 0, |omega| = vmax/rmin = 2.0.
    auto lib = makeLib(makeKin());
    int pivots = 0;
    for (const auto & p : lib.primitives())
        if (p.control.v == 0.0 && std::abs(p.control.omega) == 2.0) ++pivots;
    EXPECT_EQ(pivots, 2);  // left and right pivot
}

TEST(ControlSet, midRadiusArcMatchesHandComputedVelocities){
    // x = rmin: v = vmax*rmin/(2*rmin) = 0.5, |omega| = vmax/(2*rmin) = 1.0.
    auto lib = makeLib(makeKin());
    int matches = 0;
    for (const auto & p : lib.primitives())
        if (std::abs(p.control.v - 0.5) < 1e-12 && std::abs(std::abs(p.control.omega) - 1.0) < 1e-12)
            ++matches;
    EXPECT_EQ(matches, 2);  // left and right
}

TEST(ControlSet, everyControlSharesAtomicTime){
    auto lib = makeLib(makeKin());
    for (const auto & p : lib.primitives())
        EXPECT_DOUBLE_EQ(p.control.dt, 0.4);
}

TEST(ControlSet, isMirrorSymmetricInOmega){
    // For every control there is one with the same (v, dt) and negated omega (left<->right; a
    // straight is its own mirror since -0 == 0).
    auto lib = makeLib(makeKin());
    const auto & prims = lib.primitives();
    for (const auto & p : prims){
        bool mirrored = false;
        for (const auto & q : prims){
            if (q.control.v == p.control.v && q.control.dt == p.control.dt &&
                q.control.omega == -p.control.omega){
                mirrored = true;
                break;
            }
        }
        EXPECT_TRUE(mirrored) << "no omega-mirror for v=" << p.control.v
                              << " omega=" << p.control.omega;
    }
}

// ---------------------------------------------------------------------------
// buildPrimitive: wiring + per-bin structure.
// ---------------------------------------------------------------------------

TEST(BuildPrimitive, threadsEndpointAndCostFromCollaborators){
    auto kin = makeKin();
    auto lib = makeLib(kin);
    auto space = makeSpace(kin);
    auto cost_fcn = makeControlCost(kin);
    for (const auto & p : lib.primitives()){
        SE2State expected = space->propagate(SE2State{0, 0, 0}, p.control);
        EXPECT_NEAR(p.endpoint_offset.x, expected.x, 1e-12);
        EXPECT_NEAR(p.endpoint_offset.y, expected.y, 1e-12);
        EXPECT_NEAR(p.endpoint_offset.theta, expected.theta, 1e-12);
        EXPECT_NEAR(p.cost, cost_fcn(p.control), 1e-12);
    }
}

TEST(BuildPrimitive, forwardStraightEndpointAndCostHandComputed){
    auto lib = makeLib(makeKin());
    // v = 1, omega = 0, dt = 0.4 -> endpoint (0.4, 0, 0); cost = sqrt(1 + 0)*0.4 = 0.4.
    const Primitive * straight = nullptr;
    for (const auto & p : lib.primitives())
        if (p.control.omega == 0.0 && p.control.v > 0.0){ straight = &p; break; }
    ASSERT_NE(straight, nullptr);
    EXPECT_NEAR(straight->endpoint_offset.x, 0.4, 1e-12);
    EXPECT_NEAR(straight->endpoint_offset.y, 0.0, 1e-12);
    EXPECT_NEAR(straight->endpoint_offset.theta, 0.0, 1e-12);
    EXPECT_NEAR(straight->cost, 0.4, 1e-12);
}

TEST(BuildPrimitive, everyPrimitiveHasFullPerBinCellData){
    auto lib = makeLib(makeKin());  // n_headings = 8
    ASSERT_FALSE(lib.primitives().empty());
    for (const auto & p : lib.primitives()){
        ASSERT_EQ(p.swept_cells.size(), 8u);
        ASSERT_EQ(p.sample_cells.size(), 8u);
        for (int k = 0; k < 8; ++k){
            EXPECT_FALSE(p.swept_cells[k].empty())  << "empty swept bin " << k;
            EXPECT_FALSE(p.sample_cells[k].empty()) << "empty sample bin " << k;
        }
    }
}

// ---------------------------------------------------------------------------
// headingBin: seams and bin centers (n_headings = 8, bin_size = pi/4).
// ---------------------------------------------------------------------------

TEST(HeadingBin, seamsAnchorsAndCentersForEightBins){
    auto lib = makeLib(makeKin());
    EXPECT_EQ(lib.nHeadings(), 8);

    // Exact values that are float-robust (0, +-pi, 2pi map to exact ratios).
    EXPECT_EQ(lib.headingBin(-PI), 0);      // bin 0 anchored at -pi
    EXPECT_EQ(lib.headingBin(PI), 0);       // +pi seam clamps n_headings -> 0
    EXPECT_EQ(lib.headingBin(0.0), 4);      // 0 sits at the start of bin 4
    EXPECT_EQ(lib.headingBin(2 * PI), 4);   // periodic with 0

    // Bin centers (mid-interval, away from float-fragile edges).
    EXPECT_EQ(lib.headingBin(-PI + PI / 8), 0);  // center of bin 0
    EXPECT_EQ(lib.headingBin(-PI / 8), 3);       // center of bin 3
    EXPECT_EQ(lib.headingBin(PI - PI / 8), 7);   // center of bin 7
}

// ---------------------------------------------------------------------------
// buildTimeMs: finite, non-negative, fixed after construction (copies carry the same value).
// ---------------------------------------------------------------------------

TEST(BuildTime, finiteNonNegativeAndCopiedUnchanged){
    auto lib = makeLib(makeKin());
    EXPECT_TRUE(std::isfinite(lib.buildTimeMs()));
    EXPECT_GE(lib.buildTimeMs(), 0.0);
    const DiffDrivePrimLib copy = lib;
    EXPECT_EQ(copy.buildTimeMs(), lib.buildTimeMs());
}

// ---------------------------------------------------------------------------
// Cover cache structure and per-sample clearance, over several heading-bin counts.
// ---------------------------------------------------------------------------

namespace {
const double kH = static_cast<double>(0.1f);  // grid resolution as stored (GridMetaData holds a float)

MotionContext bodyFrameContext(const DiffDriveParams & kin){
    return MotionContext{makeBody(), makeSpace(kin), GridMetaData{100, 100, kH, Eigen::Vector2f::Zero()}};
}
DiffDriveParams kinWith(int n){ auto k = makeKin(); k.n_headings = n; return k; }
}  // namespace

TEST(SampleClearance, sizeMatchesSampleCellsInEveryBin){
    for (int n : {8, 16, 32}){
        auto lib = makeLib(kinWith(n));
        for (const auto & p : lib.primitives())
            for (int k = 0; k < n; ++k)
                EXPECT_EQ(p.sample_clearance.size(), p.sample_cells[k].size()) << "N=" << n << " k=" << k;
    }
}

TEST(SampleClearance, pivotsHaveRoutPlusTwoSqrt2H){
    const double r_out = makeBody().r_out;
    for (int n : {8, 16, 32}){
        auto lib = makeLib(kinWith(n));
        int pivots = 0;
        for (const auto & p : lib.primitives()){
            if (p.control.v != 0.0) continue;
            ++pivots;
            for (double c : p.sample_clearance) EXPECT_NEAR(c, r_out + 2.0 * std::sqrt(2.0) * kH, 1e-12);
        }
        EXPECT_EQ(pivots, 2);  // the radius-0 left and right arcs
    }
}

TEST(SampleClearance, boundedBelowAndNonDecreasingInDistanceFromStart){
    const double floor_c = makeBody().r_out + 2.0 * std::sqrt(2.0) * kH;
    for (int n : {8, 16, 32}){
        const auto kin = kinWith(n);
        auto lib = makeLib(kin);
        const auto ctx = bodyFrameContext(kin);
        for (const auto & p : lib.primitives()){
            const auto poses = ctx.sampleTrajectory(SE2State{0, 0, 0}, p.control);
            ASSERT_EQ(poses.size(), p.sample_clearance.size());
            for (std::size_t a = 0; a < poses.size(); ++a){
                EXPECT_GE(p.sample_clearance[a], floor_c - 1e-12);
                for (std::size_t b = 0; b < poses.size(); ++b){
                    const double ra = std::hypot(poses[a].x, poses[a].y), rb = std::hypot(poses[b].x, poses[b].y);
                    if (ra <= rb) EXPECT_LE(p.sample_clearance[a], p.sample_clearance[b] + 1e-12);
                }
            }
        }
    }
}

TEST(SweptCover, listsAreSortedUniqueAndContainStartCell){
    for (int n : {8, 16, 32}){
        auto lib = makeLib(kinWith(n));
        for (const auto & p : lib.primitives())
            for (int k = 0; k < n; ++k){
                const auto & v = p.swept_cells[k];
                EXPECT_TRUE(std::is_sorted(v.begin(), v.end())) << "N=" << n << " k=" << k;
                EXPECT_TRUE(std::adjacent_find(v.begin(), v.end()) == v.end()) << "N=" << n << " k=" << k;
                EXPECT_TRUE(std::binary_search(v.begin(), v.end(), GridCoords{0, 0})) << "N=" << n << " k=" << k;
            }
    }
}

TEST(SweptCover, containsSingleHeadingFootprintOfBinAnchor){
    for (int n : {8, 16, 32}){
        const auto kin = kinWith(n);
        auto lib = makeLib(kin);
        const auto ctx = bodyFrameContext(kin);
        for (const auto & p : lib.primitives())
            for (int k = 0; k < n; ++k){
                const double theta_k = -PI + k * (2.0 * PI / n);
                for (const auto & c : ctx.computeExactCover(SE2State{0, 0, theta_k}, p.control).cells)
                    EXPECT_TRUE(std::binary_search(p.swept_cells[k].begin(), p.swept_cells[k].end(), c))
                        << "N=" << n << " k=" << k << " cell (" << c.i << "," << c.j << ")";
            }
    }
}

TEST(SweptCover, everyCachedCellIsWithinReachOfTheMotion){
    // Upper bound on how far a covered cell can lie from the start cell origin: the arc length, the
    // circumscribing radius, one build margin (<= h), the rasterizer's half diagonal and the one-cell
    // dilation (sqrt2*h). A cover returning far too many cells violates it.
    const double r_out = makeBody().r_out;
    for (int n : {8, 16, 32}){
        auto lib = makeLib(kinWith(n));
        for (const auto & p : lib.primitives()){
            const double R = std::abs(p.control.v) * p.control.dt + r_out + kH + std::sqrt(2.0) / 2.0 * kH
                             + std::sqrt(2.0) * kH;
            for (int k = 0; k < n; ++k)
                for (const auto & c : p.swept_cells[k]){
                    const double dx = std::max({c.i * kH, -(c.i + 1) * kH, 0.0});
                    const double dy = std::max({c.j * kH, -(c.j + 1) * kH, 0.0});
                    EXPECT_LE(std::hypot(dx, dy), R) << "N=" << n << " k=" << k << " cell (" << c.i << "," << c.j << ")";
                }
        }
    }
}

TEST(DiffDrivePrimLibChecks, invalidKinematicsThrow){
    const auto good = makeKin();
    MotionContext motion{makeBody(), makeSpace(good), GridMetaData{100, 100, kH, Eigen::Vector2f::Zero()}};
    auto bad = [&](auto mutate){ auto k = good; mutate(k); return k; };
    EXPECT_THROW(DiffDrivePrimLib(motion, bad([](auto & k){ k.rmin = 0.0; })), std::invalid_argument);
    EXPECT_THROW(DiffDrivePrimLib(motion, bad([](auto & k){ k.vmax = 0.0; })), std::invalid_argument);
    EXPECT_THROW(DiffDrivePrimLib(motion, bad([](auto & k){ k.atomic_time = 0.0; })), std::invalid_argument);
    EXPECT_THROW(DiffDrivePrimLib(motion, bad([](auto & k){ k.n_headings = 0; })), std::invalid_argument);
}
