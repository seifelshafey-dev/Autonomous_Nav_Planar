#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/planner/hybrid_astar/obstacle_heuristic.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/map/map_provider.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/cost/cost_policy.hpp"
#include "core/types/grid_coords.hpp"
#include "core/types/state.hpp"

/*
Properties of the obstacle field on random grids, with sound oracles independent of the Dijkstra code:

1. consistencyAndEuclidLowerBound: every reachable cell satisfies the Dijkstra relaxation invariant
   field(a) <= field(b) + edge(b,a) against each free neighbour (an optimal-substructure check that
   does not re-derive the field), and field(cell) >= straight-line distance to the goal (obstacles and
   cost can only lengthen a path), and field >= 0 with the goal cell exactly 0.
2. bellmanOptimalityWithCostmap: the STRONGER tightness half — every non-goal reachable cell c
   satisfies field(c) == min over free neighbours nb of field(nb) + step(c,nb)*(1 + cw*costAt(c)).
   The <= consistency check in (1) only proves the field is a valid lower bound; equality proves it is
   the OPTIMAL (least-cost) field, which is the invariant Dial's bucket queue must preserve bit-for-bit
   versus a binary-heap Dijkstra. Run over a NONZERO costmap so cost-weighted edges are exercised
   (the value the bucket index is computed from), not just bare geometric steps. The oracle is the
   Bellman equation itself, derived from the cost model in obstacle_heuristic.cpp — not from the field.

The costmap is zeroed in (1) so edge costs are the bare geometric step (resolution /
sqrt(2)*resolution), keeping the consistency oracle exact; (2) supplies its own clearance-inflated map.
*/

namespace {

constexpr double CELL = 0.1;
constexpr int    C    = 12;
constexpr int    R    = 12;

std::shared_ptr<MapSnapshot> snapshot(std::vector<int8_t> occ){
    GridMetaData m{C, R, CELL, Eigen::Vector2f::Zero()};
    OccupancyGrid g(m, std::move(occ));
    EDT e = buildEDTGrid(g, 5.0);
    return std::make_shared<MapSnapshot>(MapSnapshot{std::move(g), std::move(e)});
}

ObstacleHeuristic makeH(std::shared_ptr<MapSnapshot> snap){
    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto cm   = std::make_shared<const CostMap>(buildCostMap(*snap, CostPolicy{0.0f, 0.0f, 0.0f, 1.0f}, 0.1, 1.0));
    return ObstacleHeuristic(grid, std::move(cm), 1.0, 1.0);
}

// Random ~20%-occupied grid with a guaranteed-free goal cell; returns the heuristic (prepared) plus
// the goal cell.
struct Fixture { std::shared_ptr<MapSnapshot> snap; int gi, gj; };

Fixture makeFixture(){
    std::vector<int8_t> occ(static_cast<std::size_t>(C) * R, 0);
    for (auto & c : occ) { c = (*rc::gen::inRange(0, 5) == 0) ? static_cast<int8_t>(100)
                                                              : static_cast<int8_t>(0); }
    const int gi = *rc::gen::inRange(0, C);
    const int gj = *rc::gen::inRange(0, R);
    occ[static_cast<std::size_t>(gj) * C + gi] = 0;  // goal cell free
    return Fixture{snapshot(std::move(occ)), gi, gj};
}

bool reachable(double v){ return v < ObstacleHeuristic::UNREACHABLE; }

// Cost-aware fixture for the Bellman-optimality property: a clearance-inflated costmap (nonzero around
// obstacles) so the field's edge weights are genuinely cost-dependent. Returns the heuristic together
// with the SAME costmap object it was built from, so the test can recompute the exact edge cost the
// Dijkstra charged. costmap_weight is fixed at 1.0 to match the heuristic.
struct CostFixture {
    std::shared_ptr<MapSnapshot> snap;
    std::shared_ptr<const CostMap> cm;
    ObstacleHeuristic h;
    int gi, gj;
    static constexpr double kCostmapWeight = 1.0;
};

CostFixture makeCostFixture(){
    std::vector<int8_t> occ(static_cast<std::size_t>(C) * R, 0);
    for (auto & c : occ) { c = (*rc::gen::inRange(0, 6) == 0) ? static_cast<int8_t>(100)
                                                              : static_cast<int8_t>(0); }
    const int gi = *rc::gen::inRange(0, C);
    const int gj = *rc::gen::inRange(0, R);
    occ[static_cast<std::size_t>(gj) * C + gi] = 0;  // goal cell free
    auto snap = snapshot(std::move(occ));
    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    // Nonzero clearance inflation -> per-cell costs vary near obstacles, exercising cost-weighted edges.
    auto cm = std::make_shared<const CostMap>(
        buildCostMap(*snap, CostPolicy{0.3f, 5.0f, 20.0f, 1.0f}, 0.1, CostFixture::kCostmapWeight));
    ObstacleHeuristic h(grid, cm, CostFixture::kCostmapWeight, 1.0);
    return CostFixture{snap, cm, std::move(h), gi, gj};
}

// Verbatim port of the eager full build (8-connected Dial's Dijkstra, whole grid), used as the bitwise
// reference for the on-demand field. Returns row-major costs, UNREACHABLE for never-settled cells.
std::vector<double> fullBuild(const OccupancyGrid & grid, const CostMap & cm, double cw, int gi, int gj){
    const GridMetaData meta = grid.getMetadata();
    const int cols = meta.cols, rows = meta.rows;
    const double res = meta.resolution, diag = std::sqrt(2.0) * res;
    const std::size_t n = static_cast<std::size_t>(cols) * rows;
    constexpr double BLOCKED = -1.0;
    std::vector<double> mul(n);
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i)
            mul[meta.index(GridCoords{i, j})] = (grid.findCellStatus(GridCoords{i, j}) == CellState::OCCUPIED)
                ? BLOCKED : 1.0 + cw * static_cast<double>(cm.costAt(i, j));
    std::vector<double> cost(n, ObstacleHeuristic::UNREACHABLE);
    double max_mul = 1.0;
    for (const double m : mul) { if (m > max_mul) { max_mul = m; } }
    const std::size_t nb_k = static_cast<std::size_t>(std::ceil((diag * max_mul) / res)) + 2;
    std::vector<std::vector<int32_t>> buckets(nb_k);
    std::vector<char> settled(n, 0);
    const int32_t g_idx = static_cast<int32_t>(meta.index(GridCoords{gi, gj}));
    cost[static_cast<std::size_t>(g_idx)] = 0.0;
    buckets[0].push_back(g_idx);
    struct Nb { int di, dj, didx; bool diag; };
    const std::array<Nb, 8> NB = {{
        {1, 0, 1, false}, {-1, 0, -1, false}, {0, 1, cols, false}, {0, -1, -cols, false},
        {1, 1, cols + 1, true}, {1, -1, -cols + 1, true}, {-1, 1, cols - 1, true}, {-1, -1, -cols - 1, true}}};
    std::size_t remaining = n, b = 0, empty = 0;
    while (remaining > 0 && empty < nb_k) {
        auto & cur = buckets[b % nb_k];
        if (cur.empty()) { ++b; ++empty; continue; }
        empty = 0;
        while (!cur.empty()) {
            const int32_t cidx = cur.back(); cur.pop_back();
            const std::size_t cz = static_cast<std::size_t>(cidx);
            if (settled[cz]) { continue; }
            settled[cz] = 1; --remaining;
            const double d = cost[cz];
            const int ci = cidx % cols, cj = cidx / cols;
            for (const Nb & nb : NB) {
                const int ni = ci + nb.di, nj = cj + nb.dj;
                if (ni < 0 || ni >= cols || nj < 0 || nj >= rows) { continue; }
                const std::size_t nidx = static_cast<std::size_t>(cidx + nb.didx);
                if (settled[nidx]) { continue; }
                const double m = mul[nidx];
                if (m < 0.0) { continue; }
                const double nd = d + (nb.diag ? diag : res) * m;
                if (nd < cost[nidx]) {
                    cost[nidx] = nd;
                    buckets[static_cast<std::size_t>(nd / res) % nb_k].push_back(static_cast<int32_t>(nidx));
                }
            }
        }
        ++b;
    }
    return cost;
}

// Random grid + costmap + goal, plus the eager reference field and a fresh (prepared) heuristic.
struct LazyFixture {
    std::vector<double> ref;
    ObstacleHeuristic   h;
    int gi, gj;
};

LazyFixture makeLazyFixture(double weight = 1.0){
    std::vector<int8_t> occ(static_cast<std::size_t>(C) * R, 0);
    for (auto & c : occ) { c = (*rc::gen::inRange(0, 5) == 0) ? static_cast<int8_t>(100)
                                                              : static_cast<int8_t>(0); }
    const int gi = *rc::gen::inRange(0, C);
    const int gj = *rc::gen::inRange(0, R);
    occ[static_cast<std::size_t>(gj) * C + gi] = 0;
    auto snap = snapshot(std::move(occ));
    auto grid = std::shared_ptr<const OccupancyGrid>(snap, &snap->grid_);
    auto cm   = std::make_shared<const CostMap>(buildCostMap(*snap, CostPolicy{0.3f, 5.0f, 20.0f, 1.0f}, 0.1, 1.0));
    std::vector<double> ref = fullBuild(*grid, *cm, 1.0, gi, gj);
    ObstacleHeuristic h(grid, cm, 1.0, weight);
    h.prepare(SE2State{(gi + 0.5) * CELL, (gj + 0.5) * CELL, 0.0});
    return LazyFixture{std::move(ref), std::move(h), gi, gj};
}

}  // namespace

// On-demand field == eager full build, bitwise, for every cell in a random query order.
RC_GTEST_PROP(ObstacleHeuristicProp, lazyFieldAtBitwiseEqualsFullBuildAnyOrder, ()){
    LazyFixture f = makeLazyFixture();
    std::vector<int> order(static_cast<std::size_t>(C) * R);
    for (std::size_t k = 0; k < order.size(); ++k) { order[k] = static_cast<int>(k); }
    const auto keys = *rc::gen::container<std::vector<int>>(order.size(), rc::gen::arbitrary<int>());
    std::sort(order.begin(), order.end(), [&](int a, int b){
        return keys[static_cast<std::size_t>(a)] < keys[static_cast<std::size_t>(b)]; });
    for (const int idx : order) {
        const double got = f.h.fieldAt({idx % C, idx / C});
        RC_ASSERT(got == f.ref[static_cast<std::size_t>(idx)]);
    }
}

// operator() on a random subset of cell centres == weight * reference (sentinel clamped to UNREACHABLE).
RC_GTEST_PROP(ObstacleHeuristicProp, lazyOperatorBitwiseEqualsWeightedFullBuild, ()){
    const double w = *rc::gen::element(1.0, 1.5);
    LazyFixture f = makeLazyFixture(w);
    const int count = *rc::gen::inRange(1, C * R + 1);
    for (int k = 0; k < count; ++k) {
        const int i = *rc::gen::inRange(0, C), j = *rc::gen::inRange(0, R);
        const double ref = f.ref[static_cast<std::size_t>(j) * C + i];
        const double expected = (ref >= ObstacleHeuristic::UNREACHABLE) ? w * ObstacleHeuristic::UNREACHABLE
                                                                        : w * ref;
        RC_ASSERT(f.h(SE2State{(i + 0.5) * CELL, (j + 0.5) * CELL, 0.0}) == expected);
    }
}

RC_GTEST_PROP(ObstacleHeuristicProp, consistencyAndEuclidLowerBound, ()){
    Fixture f = makeFixture();
    auto h = makeH(f.snap);
    h.prepare(SE2State{(f.gi + 0.5) * CELL, (f.gj + 0.5) * CELL, 0.0});

    RC_ASSERT(h.fieldAt({f.gi, f.gj}) == 0.0);  // goal is the zero of the field

    const double ortho = CELL, diag = std::sqrt(2.0) * CELL;
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (int j = 0; j < R; ++j) {
        for (int i = 0; i < C; ++i) {
            const double fa = h.fieldAt({i, j});
            if (!reachable(fa)) { continue; }

            RC_ASSERT(fa >= 0.0);
            const double e = std::sqrt(double((i - f.gi) * (i - f.gi) + (j - f.gj) * (j - f.gj))) * CELL;
            RC_ASSERT(fa >= e - 1e-6);  // obstacles / cost only lengthen the path

            for (int k = 0; k < 8; ++k) {
                const double fb = h.fieldAt({i + di[k], j + dj[k]});
                if (!reachable(fb)) { continue; }
                const double step = (di[k] != 0 && dj[k] != 0) ? diag : ortho;
                RC_ASSERT(fa <= fb + step + 1e-6);  // relaxation invariant (zero costmap -> edge = step)
            }
        }
    }
}

RC_GTEST_PROP(ObstacleHeuristicProp, bellmanOptimalityWithCostmap, ()){
    // Tightness half of the optimality argument, over a NONZERO costmap: every non-goal reachable cell
    // equals the Bellman minimum over its free neighbours. This is the invariant Dial's bucket queue
    // must reproduce exactly; the consistency (<=) check elsewhere does not pin it.
    CostFixture f = makeCostFixture();
    f.h.prepare(SE2State{(f.gi + 0.5) * CELL, (f.gj + 0.5) * CELL, 0.0});

    const double ortho = CELL, diag = std::sqrt(2.0) * CELL;
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (int j = 0; j < R; ++j) {
        for (int i = 0; i < C; ++i) {
            if (i == f.gi && j == f.gj) { continue; }  // goal is the field root (0), not a relaxation
            const double fc = f.h.fieldAt({i, j});
            if (!reachable(fc)) { continue; }

            // Edge cost INTO cell c from any neighbour charges the ENTERED cell's per-cell cost
            // (costmap.costAt(col=i, row=j)) times the geometric step — exactly the cost model the
            // Dijkstra applies (obstacle_heuristic.cpp). So the additive cost factor is the same for
            // every neighbour of c; only the step (ortho vs diag) and the neighbour's field differ.
            const double rate = CostFixture::kCostmapWeight * static_cast<double>(f.cm->costAt(i, j));

            double best = ObstacleHeuristic::UNREACHABLE;
            for (int k = 0; k < 8; ++k) {
                const double fb = f.h.fieldAt({i + di[k], j + dj[k]});
                if (!reachable(fb)) { continue; }
                const double step = (di[k] != 0 && dj[k] != 0) ? diag : ortho;
                const double via  = fb + step * (1.0 + rate);
                if (via < best) { best = via; }
            }

            // A reachable non-goal cell must have at least one reachable neighbour it was relaxed from.
            RC_ASSERT(reachable(best));
            // Bellman equality: the field IS the least-cost combination, not merely a lower bound.
            RC_ASSERT(std::abs(fc - best) <= 1e-6);
        }
    }
}
