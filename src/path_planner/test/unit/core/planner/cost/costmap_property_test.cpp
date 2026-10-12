#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/cost/costmap.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/map/map_provider.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"

/*
Three properties for buildCostMap (neither re-derives the exp formula):

1. clearanceIsMonotoneNonIncreasingInEDT: on an all-FREE grid (so the unknown term never fires), a cell
   that is closer to an obstacle never costs less than a cell farther away. This is a structural
   consequence of exp-decay plus the band cutoff to 0 -- it catches a flipped comparison (< vs >) or a
   sign error on the decay, neither of which a single hand-computed value would localize.

2. unknownPenaltyIsExactlyAdditive: holding EDT and policy fixed, flipping a cell FREE -> UNKNOWN raises
   its cost by exactly the map's unknownCost(). The clearance term is identical in both builds and cancels, so the
   difference isolates the unknown term and pins the "+=" semantic (vs. replace / max).

3. unknownCostBoundsFreeSpaceCost: for random valid policies, r_in, resolution and w > 0, every FREE cell
   whose distance value is at least d_min = max(0, r_in - (sqrt2/2) h) costs no more than the unknown cost.
*/

namespace {

GridMetaData metaRow(int cols){
    return GridMetaData{cols, 1, 1.0, Eigen::Vector2f::Zero()};
}

CostPolicy genPolicy(){
    return CostPolicy{
        static_cast<float>(*rc::gen::inRange(0, 31)) / 10.0f,   // radius 0.0 .. 3.0
        static_cast<float>(*rc::gen::inRange(0, 31)) / 10.0f,   // decay  0.0 .. 3.0
        static_cast<float>(*rc::gen::inRange(0, 101)),          // max    0 .. 100
        1.0f + static_cast<float>(*rc::gen::inRange(0, 101)) / 20.0f};  // factor 1 .. 6
}

}  // namespace

RC_GTEST_PROP(CostMapProp, clearanceIsMonotoneNonIncreasingInEDT, ()){
    const int cols = *rc::gen::inRange(2, 12);
    std::vector<int8_t> occ(cols, 0);  // all FREE: unknown term never fires
    std::vector<float> edt(cols);
    for (auto& e : edt) e = static_cast<float>(*rc::gen::inRange(0, 41)) / 10.0f;  // 0.0 .. 4.0

    CostPolicy p = genPolicy();
    CostMap cm = buildCostMap(
        MapSnapshot{OccupancyGrid(metaRow(cols), occ), EDT(metaRow(cols), edt, 5.0)}, p, 0.5, 1.0);

    // Order cells by EDT ascending; cost must be non-increasing along that order.
    std::vector<int> order(cols);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b){ return edt[a] < edt[b]; });

    for (std::size_t k = 1; k < order.size(); ++k){
        float closer  = cm.costAt(order[k - 1], 0);  // smaller EDT
        float farther = cm.costAt(order[k], 0);      // larger (or equal) EDT
        RC_ASSERT(closer >= farther - 1e-3f);
    }
}

RC_GTEST_PROP(CostMapProp, unknownPenaltyIsExactlyAdditive, ()){
    const float edt = static_cast<float>(*rc::gen::inRange(0, 41)) / 10.0f;  // 0.0 .. 4.0
    CostPolicy p = genPolicy();
    GridMetaData m = metaRow(1);

    float cost_free = buildCostMap(
        MapSnapshot{OccupancyGrid(m, std::vector<int8_t>{0}),
                    EDT(m, std::vector<float>{edt}, 5.0)}, p, 0.5, 1.0).costAt(0, 0);
    CostMap cm_unknown = buildCostMap(
        MapSnapshot{OccupancyGrid(m, std::vector<int8_t>{-1}),
                    EDT(m, std::vector<float>{edt}, 5.0)}, p, 0.5, 1.0);

    RC_ASSERT(std::abs((cm_unknown.costAt(0, 0) - cost_free) - cm_unknown.unknownCost()) < 1e-3f);
}

RC_GTEST_PROP(CostMapProp, unknownCostBoundsFreeSpaceCost, ()){
    const CostPolicy p = genPolicy();
    const double h    = static_cast<double>(*rc::gen::inRange(1, 21)) / 20.0;        // 0.05 .. 1.0
    const double r_in = static_cast<double>(*rc::gen::inRange(1, 31)) / 20.0;        // 0.05 .. 1.5
    const double w    = static_cast<double>(*rc::gen::inRange(1, 41)) / 10.0;        // 0.1 .. 4.0
    const double d_min = std::max(0.0, r_in - std::sqrt(2.0) / 2.0 * h);

    const int cols = *rc::gen::inRange(1, 12);
    GridMetaData m{cols, 1, h, Eigen::Vector2f::Zero()};
    std::vector<float> edt(cols);
    for (auto& e : edt) e = static_cast<float>(d_min + static_cast<double>(*rc::gen::inRange(0, 41)) / 10.0);
    CostMap cm = buildCostMap(
        MapSnapshot{OccupancyGrid(m, std::vector<int8_t>(cols, 0)), EDT(m, edt, 100.0)}, p, r_in, w);

    // 1e-3 relative slack covers float storage of u and Eigen's vectorized exp.
    for (int c = 0; c < cols; ++c)
        RC_ASSERT(cm.costAt(c, 0) <= cm.unknownCost() * (1.0f + 1e-3f) + 1e-3f);
}
