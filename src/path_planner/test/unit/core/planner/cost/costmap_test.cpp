#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/cost/costmap.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/config/config_loader.hpp"
#include "core/map/map_provider.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"

/*
CostMap is a value object: costAt(col,row) looks up cost_[row*cols+col], returning the unknown cost u when
out of grid. buildCostMap composes two terms over the grid:
    clearance = (edt < radius) ? max_cost * exp(-decay * edt) : 0       // keyed on EDT only
    cost      = clearance + (grid == -1 ? u : 0)                        // unknown penalty is ADDITIVE
with u = (factor * (1 + w * b) - 1) / w (0 when w == 0), b the dearest free-space clearance cost.
It reads the raw (unclamped) EDT field, and CHECKs EDT/grid congruence + policy/r_in/w ranges.

Tests use a real OccupancyGrid + EDT, built small and controlled. Clearance values that go
through exp() are compared with a small absolute tolerance because Eigen's vectorized exp may differ
from libm by ~1 ULP; exact-by-construction values (0, exp(0)=1) use FLOAT_EQ.
*/

namespace {

GridMetaData metaRow(int cols){
    return GridMetaData{cols, 1, 1.0, Eigen::Vector2f::Zero()};
}

// One-row snapshot: cell index == column, so cost layout is trivial to reason about.
MapSnapshot snapRow(std::vector<int8_t> occ, std::vector<float> edt, double clamp = 5.0){
    GridMetaData m = metaRow(static_cast<int>(occ.size()));
    return MapSnapshot{OccupancyGrid(m, std::move(occ)), EDT(m, std::move(edt), clamp)};
}

// Test grids have resolution 1.0, so with r_in = 0.5 the smallest free-space distance value is
// d_min = max(0, 0.5 - 0.7071) = 0 and the dearest free-space clearance cost is b = max_cost = 100.
// With w = 1 and factor = 1.0 the unknown cost is u = (1 * (1 + 100) - 1) / 1 = 100.
constexpr double kRIn = 0.5;
constexpr double kW   = 1.0;
constexpr float  kU   = 100.0f;

// radius = 1.0, decay = 1.0, max_cost = 100.0, unknown_cost_factor = 1.0
CostPolicy policy(){ return CostPolicy{1.0f, 1.0f, 100.0f, 1.0f}; }

// Closed-form unknown cost from the documented formula.
double expectedU(const CostPolicy & p, double r_in, double h, double w){
    if (w == 0.0) return 0.0;
    const double d_min = std::max(0.0, r_in - std::sqrt(2.0) / 2.0 * h);
    const double b = d_min < p.clearance_radius_ ? p.max_clearance_cost_ * std::exp(-p.clearance_decay_ * d_min) : 0.0;
    return (p.unknown_cost_factor_ * (1.0 + w * b) - 1.0) / w;
}

// Resolve the SHIPPED config/cost_policy/cost_policy.yaml relative to this test's source location,
// so the clearance-band gradient test validates the real config the planner ships with (editing the
// YAML is what flips the test). __FILE__ is the compile-time absolute path:
//   <path_planner>/test/unit/core/planner/cost/costmap_test.cpp  -> up 5 dirs to <path_planner>.
std::string shippedCostPolicyPath(){
    namespace fs = std::filesystem;
    const fs::path root = fs::path(__FILE__)
        .parent_path()   // .../test/unit/core/planner/cost
        .parent_path()   // .../test/unit/core/planner
        .parent_path()   // .../test/unit/core
        .parent_path()   // .../test/unit
        .parent_path()   // .../test
        .parent_path();  // .../path_planner
    return (root / "config" / "cost_policy" / "cost_policy.yaml").string();
}

}  // namespace

// ---------------------------------------------------------------------------
// CostMap value object: costAt lookup, out-of-grid policy, getters.
// ---------------------------------------------------------------------------

TEST(CostMapObject, costAtIsRowMajorLookup){
    // 2x2, cost_[row*cols+col]
    CostMap cm(GridMetaData{2, 2, 1.0, Eigen::Vector2f::Zero()},
               std::vector<float>{1.f, 2.f, 3.f, 4.f}, 9.f);
    EXPECT_FLOAT_EQ(cm.costAt(0, 0), 1.f);  // idx 0
    EXPECT_FLOAT_EQ(cm.costAt(1, 0), 2.f);  // idx 1
    EXPECT_FLOAT_EQ(cm.costAt(0, 1), 3.f);  // idx 2
    EXPECT_FLOAT_EQ(cm.costAt(1, 1), 4.f);  // idx 3
}

TEST(CostMapObject, outOfGridReturnsUnknownCostOnEveryEdge){
    CostMap cm(GridMetaData{2, 2, 1.0, Eigen::Vector2f::Zero()},
               std::vector<float>{1.f, 2.f, 3.f, 4.f}, 9.f);
    EXPECT_FLOAT_EQ(cm.costAt(-1, 0), 9.f);  // col < 0
    EXPECT_FLOAT_EQ(cm.costAt(2, 0), 9.f);   // col >= cols
    EXPECT_FLOAT_EQ(cm.costAt(0, -1), 9.f);  // row < 0
    EXPECT_FLOAT_EQ(cm.costAt(0, 2), 9.f);   // row >= rows
}

// ---------------------------------------------------------------------------
// buildCostMap: clearance inflation (keyed on EDT).
// ---------------------------------------------------------------------------

TEST(BuildCostMap, clearanceInflationAcrossTheBand){
    // edt = {0.0, 0.5, 1.0, 2.0}, all FREE. radius = 1.0, decay = 1.0, max = 100.
    CostMap cm = buildCostMap(snapRow({0, 0, 0, 0}, {0.0f, 0.5f, 1.0f, 2.0f}), policy(), kRIn, kW);
    EXPECT_FLOAT_EQ(cm.costAt(0, 0), 100.f);                    // edt 0   -> max * exp(0)
    EXPECT_NEAR(cm.costAt(1, 0), 100.f * std::exp(-0.5f), 1e-3f);  // edt 0.5 -> ~60.6531
    EXPECT_FLOAT_EQ(cm.costAt(2, 0), 0.f);                     // edt 1.0 == radius (not < radius)
    EXPECT_FLOAT_EQ(cm.costAt(3, 0), 0.f);                     // edt 2.0 -> outside band
}

TEST(BuildCostMap, occupiedCellGetsMaxClearanceAndNoUnknown){
    // Occupied cell (raw 100, EDT 0 since it is the obstacle): max_cost, no unknown add.
    CostMap cm = buildCostMap(snapRow({100}, {0.0f}), policy(), kRIn, kW);
    EXPECT_FLOAT_EQ(cm.costAt(0, 0), 100.f);
}

TEST(BuildCostMap, decayShapesTheInflation){
    // edt 0.25, decay 1.0 -> 100 * exp(-0.25) ~ 77.8801
    CostMap cm = buildCostMap(snapRow({0}, {0.25f}), policy(), kRIn, kW);
    EXPECT_NEAR(cm.costAt(0, 0), 100.f * std::exp(-0.25f), 1e-3f);
}

// ---------------------------------------------------------------------------
// buildCostMap: unknown penalty (additive, keyed on grid == -1).
// ---------------------------------------------------------------------------

TEST(BuildCostMap, unknownPenaltyOutOfBandIsJustUnknownCost){
    // Unknown cell, edt 2.0 (outside band): clearance 0 + u = 100.
    CostMap cm = buildCostMap(snapRow({-1}, {2.0f}), policy(), kRIn, kW);
    EXPECT_FLOAT_EQ(cm.costAt(0, 0), kU);
}

TEST(BuildCostMap, unknownPenaltyIsAddedOnTopOfClearance){
    // Unknown cell inside the band, edt 0.5: clearance + unknown, NOT one or the other.
    CostMap cm = buildCostMap(snapRow({-1}, {0.5f}), policy(), kRIn, kW);
    EXPECT_NEAR(cm.costAt(0, 0), 100.f * std::exp(-0.5f) + kU, 1e-3f);  // ~160.6531
}

// ---------------------------------------------------------------------------
// buildCostMap: reads the RAW (unclamped) EDT field.
// ---------------------------------------------------------------------------

TEST(BuildCostMap, usesRawUnclampedEdtNotDistanceAt){
    // radius (10) > clamp (5). A far cell has raw edt 100.
    //   raw:     100 < 10 ? no  -> cost 0     (correct: the cell is genuinely far)
    //   clamped:   5 < 10 ? yes -> cost > 0   (spurious)
    // A result of 0 proves the raw field is used.
    CostPolicy p{10.0f, 1.0f, 100.0f, 1.0f};
    CostMap cm = buildCostMap(snapRow({0}, {100.0f}, /*clamp=*/5.0), p, kRIn, kW);
    EXPECT_FLOAT_EQ(cm.costAt(0, 0), 0.f);
}

// ---------------------------------------------------------------------------
// buildCostMap: input validation (always-on CHECK).
// ---------------------------------------------------------------------------

TEST(BuildCostMap, throwsWhenEdtSizeDoesNotMatchGrid){
    // Grid is 2x2 (n=4); the EDT is internally consistent but belongs to a different-sized
    // grid (3x1, 3 entries). buildCostMap must reject the pair rather than over-read.
    GridMetaData m{2, 2, 1.0, Eigen::Vector2f::Zero()};
    OccupancyGrid g(m, std::vector<int8_t>(4, 0));
    GridMetaData em{3, 1, 1.0, Eigen::Vector2f::Zero()};
    EDT e(em, std::vector<float>(3, 1.0f), 5.0);
    MapSnapshot s{std::move(g), std::move(e)};
    EXPECT_THROW(buildCostMap(s, policy(), kRIn, kW), std::invalid_argument);
}

TEST(BuildCostMap, throwsOnInvalidParams){
    MapSnapshot s = snapRow({0}, {0.5f});
    EXPECT_THROW(buildCostMap(s, CostPolicy{-1.f, 1.f, 100.f, 1.f}, kRIn, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, CostPolicy{1.f, -1.f, 100.f, 1.f}, kRIn, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, CostPolicy{1.f, 1.f, -100.f, 1.f}, kRIn, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, CostPolicy{1.f, 1.f, 100.f, 0.99f}, kRIn, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, CostPolicy{1.f, 1.f, 100.f, std::nanf("")}, kRIn, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, policy(), 0.0, kW), std::invalid_argument);
    EXPECT_THROW(buildCostMap(s, policy(), kRIn, -0.1), std::invalid_argument);
}

TEST(BuildCostMap, acceptsZeroClearanceParams){
    // Zero clearance settings are a legitimate "no inflation" configuration; with b = 0 and factor 1, u = 0.
    MapSnapshot s = snapRow({-1}, {0.5f});
    CostMap cm = buildCostMap(s, CostPolicy{0.f, 0.f, 0.f, 1.f}, kRIn, kW);
    EXPECT_FLOAT_EQ(cm.unknownCost(), 0.f);
}

// ---------------------------------------------------------------------------
// buildCostMap: unknown cost derived from factor, r_in, resolution, clearance policy and w.
// ---------------------------------------------------------------------------

TEST(BuildCostMap, unknownCostFollowsFormulaForWeightOneAndHalf){
    // Resolution 0.05, r_in 0.3 -> d_min = 0.3 - 0.035355 = 0.264645; b = 30 * exp(-4.5 * d_min) ~ 9.118.
    const CostPolicy p{0.7f, 4.5f, 30.0f, 1.03f};
    GridMetaData m{1, 1, 0.05, Eigen::Vector2f::Zero()};
    for (double w : {1.0, 0.5}) {
        MapSnapshot s{OccupancyGrid(m, std::vector<int8_t>{-1}), EDT(m, std::vector<float>{5.0f}, 5.0)};
        const CostMap cm = buildCostMap(s, p, 0.3, w);
        EXPECT_NEAR(cm.unknownCost(), expectedU(p, 0.3, 0.05, w), 1e-4);
        EXPECT_FLOAT_EQ(cm.costAt(-1, 0), cm.unknownCost());  // out-of-grid value
    }
    EXPECT_NEAR(expectedU(p, 0.3, 0.05, 1.0), 9.422, 2e-3);  // pins the closed form itself
}

TEST(BuildCostMap, zeroCostmapWeightGivesZeroUnknownCost){
    const CostMap cm = buildCostMap(snapRow({-1}, {0.5f}), CostPolicy{1.f, 1.f, 100.f, 3.f}, kRIn, 0.0);
    EXPECT_FLOAT_EQ(cm.unknownCost(), 0.f);
}

TEST(BuildCostMap, factorOneGivesUnknownCostEqualToDearestFreeCost){
    // d_min = 0 here, so b = 100 and u == b for any w.
    for (double w : {0.5, 1.0, 4.0}) {
        const CostMap cm = buildCostMap(snapRow({-1}, {0.5f}), policy(), kRIn, w);
        EXPECT_NEAR(cm.unknownCost(), 100.0, 1e-3);
    }
}

// ---------------------------------------------------------------------------
// Shipped cost-policy sizing: the soft clearance band must actually bias the planned centreline
// outward across the pure-pursuit tracking budget. The centreline is already held at r_out by the
// footprint's own outer radius; to protect the executed (corner-cut) path the soft cost has to keep
// falling from r_out (0.424) out to r_out + e_track (0.694). That requires (a) the inflation radius
// to bracket the whole band and (b) a decay gentle enough to leave a real gradient inside it.
// Loads the SHIPPED YAML, so it FAILS if the radius does not bracket the band and PASSES on the
// sized 0.70/4.5 config.
// ---------------------------------------------------------------------------

TEST(CostMapClearance, hasOutwardGradientAcrossTrackingBand){
    const CostPolicy pol = loadCostPolicy(shippedCostPolicyPath());

    const double r_out   = 0.42426;  // 0.6 m box footprint: r_out = 0.3 * sqrt(2)
    const double e_track = 0.27;     // pure-pursuit tracking budget (corrected corner-cut model)

    // (a) Band-containment: both edges of [r_out, r_out + e_track] = [0.424, 0.694] must lie inside
    // the inflation radius, else clearance cost is 0 beyond it and there is nothing to push against.
    // This is the assertion that fails unless the shipped radius brackets the 0.694 outer edge.
    ASSERT_LT(r_out + e_track, static_cast<double>(pol.clearance_radius_));

    // (b) Query the ACTUAL CostMap clearance behaviour (Eigen SIMD exp path), not a re-derived
    // formula: two FREE cells whose raw EDT equals the band edges (clamp 5.0 >> both, so raw passes
    // through). buildCostMap keys clearance on EDT alone.
    const CostMap cm = buildCostMap(
        snapRow({0, 0}, {static_cast<float>(r_out), static_cast<float>(r_out + e_track)}), pol, kRIn, kW);
    const double c_in  = cm.costAt(0, 0);  // clearance cost at edt = r_out            (inner edge)
    const double c_out = cm.costAt(1, 0);  // clearance cost at edt = r_out + e_track  (outer edge)

    // Both edges are inside the band -> nonzero clearance cost with a meaningful outward drop.
    // Ratio = exp(decay * e_track); for decay = 4.5 and e_track = 0.27 that is exp(1.215) ~= 3.37,
    // comfortably above the 1.5 threshold. At decay = 15 there is no gradient here (cost ~0 by
    // 0.3 m), which is exactly what the retune fixes.
    EXPECT_GT(c_in, 0.0);
    EXPECT_GT(c_out, 0.0);
    EXPECT_GT(c_in / c_out, 1.5);
}
