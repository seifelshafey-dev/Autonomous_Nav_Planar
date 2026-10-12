#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <algorithm>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <numbers>
#include <cmath>

#include "core/collision_checker/collision_checker.hpp"
#include "core/map/grid.hpp"
#include "core/map/edt.hpp"
#include "core/robot/robot_params.hpp"
#include "core/types/grid_coords.hpp"
#include "core/motion/motion_context.hpp"
#include "core/motion/world_robot_geometry.hpp"
#include "core/motion/kinematics/diff_drive.hpp"
#include "core/motion/distance_functions.hpp"
#include "core/planner/primitives/build_diff_prim_lib.hpp"
#include "core/robot/diff_drive_parameters.hpp"
#include "../../../support/swept_oracle.hpp"

/*
Property: fastAcceptReject is a monotonic threshold classifier in EDT distance. As the distance to the
nearest obstacle grows, the verdict never becomes *less* safe -- it moves COLLISION -> UNCERTAIN ->
VALID and never backwards. This is a consequence of the two-threshold rule (not a restatement of it),
so it catches a swapped r_in/r_out or a flipped comparison. Oracle: rank COLLISION<UNCERTAIN<VALID.
*/

namespace {
int rank(Validity v){
    switch (v){
        case Validity::COLLISION:  return 0;
        case Validity::UNCERTAIN:  return 1;
        case Validity::VALID:      return 2;
    }
    return -1;
}
RobotParams discRobot(double r_in, double r_out){
    RobotParams p; p.r_in = r_in; p.r_out = r_out; return p;
}
}  // namespace

RC_GTEST_PROP(CollisionCheckerProp, fastAcceptRejectIsMonotonicInDistance, ()){
    const double r_in  = *rc::gen::inRange(1, 101) / 100.0;          // 0.01 .. 1.0
    const double r_out = r_in + *rc::gen::inRange(1, 101) / 100.0;   // strictly > r_in

    double da = *rc::gen::inRange(0, 301) / 100.0;   // 0.0 .. 3.0
    double db = *rc::gen::inRange(0, 301) / 100.0;
    if (da > db) std::swap(da, db);                  // da <= db

    auto grid = std::make_shared<const OccupancyGrid>(
        GridMetaData{2, 1, 1.0, Eigen::Vector2f::Zero()}, std::vector<int8_t>(2, 0));
    auto edt = std::make_shared<const EDT>(
        GridMetaData{2, 1, 1.0, Eigen::Vector2f::Zero()},
        std::vector<float>{static_cast<float>(da), static_cast<float>(db)}, 10.0);

    CollisionChecker c(grid, edt, discRobot(r_in, r_out));

    Validity lo = c.fastAcceptReject(GridCoords{0, 0});  // distance da
    Validity hi = c.fastAcceptReject(GridCoords{1, 0});  // distance db (>= da)
    RC_ASSERT(rank(hi) >= rank(lo));
}

/*
Property: with a real primitive library, the distance-field shortcuts of checkTrajectory agree with the
cells and the continuous geometry. On random occupancy maps (obstacle blocks placed near the node):
  - FAST_ACCEPT  =>  the footprint, executing the control from a random continuous pose in that cell with
                     a heading in bin k, touches no OCCUPIED cell;
  - FAST_REJECT  =>  the same motion touches an OCCUPIED cell
  (geometric test oracle, support/swept_oracle.hpp). Note that FAST_ACCEPT does not imply that the cached
  cover is free of OCCUPIED cells: the cover is deliberately larger than what the sample clearance certifies.
The robot is a 1 m square so that the reject rule (edt + (clearance - r_out) < r_in) is reachable.
*/
namespace {
constexpr int kGridN = 60;
constexpr double kRes = 0.1;

struct LibFixture {
    RobotParams body;
    std::shared_ptr<const DiffDriveStateSpace> space;
    std::unique_ptr<DiffDrivePrimLib> lib;
    GridMetaData meta{kGridN, kGridN, kRes, Eigen::Vector2f::Zero()};
    LibFixture(){
        body = buildRobotParams({Vec2(0.5, 0.5), Vec2(-0.5, 0.5), Vec2(-0.5, -0.5), Vec2(0.5, -0.5)});
        DiffDriveParams kin; kin.rmin = 0.3; kin.vmax = 0.5; kin.atomic_time = 0.6; kin.n_headings = 16;
        auto dist = std::make_shared<const PointsWeightedEuclideanSE2>(kin.rmin * kin.rmin);
        space = std::make_shared<const DiffDriveStateSpace>(meta, dist);
        lib = std::make_unique<DiffDrivePrimLib>(MotionContext{body, space, meta}, kin);
    }
};
const LibFixture & fixture(){ static const LibFixture f; return f; }

struct World {
    std::shared_ptr<const OccupancyGrid> grid;
    std::shared_ptr<const EDT> edt;
};
World worldFrom(std::vector<int8_t> occ){
    const auto & f = fixture();
    auto grid = std::make_shared<const OccupancyGrid>(f.meta, std::move(occ));
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid));
    return {grid, edt};
}
}  // namespace

RC_GTEST_PROP(CollisionCheckerProp, fastVerdictsAgreeWithCellsAndGeometry, ()){
    const auto & f = fixture();
    const GridCoords c{*rc::gen::inRange(10, 50), *rc::gen::inRange(10, 50)};
    const double theta = *rc::gen::inRange(-3140, 3141) / 1000.0;
    const int k = f.lib->headingBin(theta);
    const auto & prims = f.lib->primitives();
    const auto & prim = prims[*rc::gen::inRange<std::size_t>(0, prims.size())];

    // Half of the obstacles sit near the node; the other half sit at the largest sample clearance
    // (+-2h) from a sample centre, where the accept/reject decision is borderline.
    const auto & clr = prim.sample_clearance;
    const double max_clr = *std::max_element(clr.begin(), clr.end());
    std::vector<int8_t> occ(kGridN * kGridN, 0);
    const int blocks = *rc::gen::inRange(0, 6);
    for (int n = 0; n < blocks; ++n){
        int bi, bj;
        if (*rc::gen::inRange(0, 2) == 0){
            bi = c.i + *rc::gen::inRange(-8, 9); bj = c.j + *rc::gen::inRange(-8, 9);
        } else {
            const auto & sc = prim.sample_cells[k][*rc::gen::inRange<std::size_t>(0, prim.sample_cells[k].size())];
            const double dist = max_clr + *rc::gen::inRange(-200, 201) / 100.0 * kRes;
            const double ang = *rc::gen::inRange(0, 6283) / 1000.0;
            bi = c.i + sc.i + static_cast<int>(std::lround(dist / kRes * std::cos(ang)));
            bj = c.j + sc.j + static_cast<int>(std::lround(dist / kRes * std::sin(ang)));
        }
        const int bw = *rc::gen::inRange(1, 5), bh = *rc::gen::inRange(1, 5);
        for (int i = bi; i < bi + bw; ++i)
            for (int j = bj; j < bj + bh; ++j)
                if (i >= 0 && i < kGridN && j >= 0 && j < kGridN) occ[j * kGridN + i] = 100;
    }
    const World w = worldFrom(occ);
    CollisionChecker checker(w.grid, w.edt, f.body);

    const TrajectoryVerdict v = checker.checkTrajectory(prim.swept_cells[k], prim.sample_cells[k],
                                                        prim.sample_clearance, c);
    RC_TAG(v == TrajectoryVerdict::FAST_ACCEPT ? "FAST_ACCEPT" : v == TrajectoryVerdict::FAST_REJECT ? "FAST_REJECT"
           : v == TrajectoryVerdict::SCAN_CLEAR ? "SCAN_CLEAR" : "SCAN_BLOCKED");

    if (v == TrajectoryVerdict::FAST_ACCEPT || v == TrajectoryVerdict::FAST_REJECT){
        const bool accept = (v == TrajectoryVerdict::FAST_ACCEPT);
        const double a = *rc::gen::inRange(0, 1000) / 1000.0 * kRes;
        const double b = *rc::gen::inRange(0, 1000) / 1000.0 * kRes;
        const double th = -std::numbers::pi + (k + *rc::gen::inRange(0, 1000) / 1000.0) * (2.0 * std::numbers::pi / 16);
        // Accept uses the strict-under-approximation-free oracle: boundary contact counts and poses are
        // twice as dense. Reject uses positive-overlap contact so a reported hit is a genuine touch.
        const auto touched = swept_oracle::motionTouchedCells(
            f.body, *f.space, kRes, SE2State{c.i * kRes + a, c.j * kRes + b, th}, prim.control,
            /*start_only=*/false, /*closed=*/accept, /*step_div=*/accept ? 8.0 : 4.0);
        bool hit = false;
        for (const auto & t : touched)
            if (w.grid->findCellStatus(GridCoords{t.first, t.second}) == CellState::OCCUPIED) { hit = true; break; }
        RC_ASSERT(hit == !accept);
    }
}

// Deterministic companion: guarantees the reject branch is exercised (an obstacle on the robot centre).
TEST(CollisionCheckerFastPath, obstacleOnSampleFastRejectsEveryPrimitive){
    const auto & f = fixture();
    const GridCoords c{30, 30};
    std::vector<int8_t> occ(kGridN * kGridN, 0);
    occ[30 * kGridN + 30] = 100;
    const World w = worldFrom(occ);
    CollisionChecker checker(w.grid, w.edt, f.body);
    for (const auto & prim : f.lib->primitives())
        EXPECT_EQ(checker.checkTrajectory(prim.swept_cells[0], prim.sample_cells[0], prim.sample_clearance, c),
                  TrajectoryVerdict::FAST_REJECT);
}

/*
Properties of the start gate over random convex body, pose, resolution, grid origin and occupancy:
  - startClear is monotone in the extra margin: clear at m1 implies clear at every m0 <= m1;
  - startClear(p) implies checkState(p).
*/
RC_GTEST_PROP(CollisionCheckerProp, startClearIsMonotoneAndImpliesCheckState, ()){
    const float h = *rc::gen::element(0.05f, 0.1f);
    const int n = *rc::gen::inRange(3, 8);
    const double R = *rc::gen::inRange(10, 51) / 100.0;
    const double phase = *rc::gen::inRange(0, 1000) / 1000.0 * 2.0 * M_PI;
    std::vector<Vec2> v;
    for (int k = 0; k < n; ++k){
        const double u = *rc::gen::inRange(0, 301) / 1000.0;
        const double ang = phase + 2.0 * M_PI * (k + u) / n;
        v.emplace_back(R * std::cos(ang), R * std::sin(ang));
    }
    const RobotParams body = buildRobotParams(v);

    const int N = 60;
    const Eigen::Vector2f origin(0.37f, -0.21f);
    std::vector<int8_t> occ(N * N, 0);
    const int blocks = *rc::gen::inRange(0, 7);
    for (int b = 0; b < blocks; ++b){
        const int i0 = *rc::gen::inRange(0, N - 1), j0 = *rc::gen::inRange(0, N - 1);
        const int w = *rc::gen::inRange(1, 4), hh = *rc::gen::inRange(1, 4);
        for (int j = j0; j < std::min(N, j0 + hh); ++j)
            for (int i = i0; i < std::min(N, i0 + w); ++i) occ[j * N + i] = 100;
    }
    auto grid = std::make_shared<const OccupancyGrid>(GridMetaData{N, N, h, origin}, occ);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid));
    CollisionChecker c(grid, edt, body);

    const SE2State start{origin.x() + *rc::gen::inRange(100, 900) / 1000.0 * N * h,
                         origin.y() + *rc::gen::inRange(100, 900) / 1000.0 * N * h,
                         *rc::gen::inRange(-31415, 31416) / 10000.0};

    const bool clear0 = c.startClear(start, 0.0);
    RC_TAG(clear0 ? "clear" : "blocked");
    RC_ASSERT(c.startClear(start) == clear0);
    if (clear0) RC_ASSERT(c.checkState(start));

    const double m1 = *rc::gen::inRange(0, 3001) / 1000.0 * h;   // 0 .. 3h
    const double m0 = *rc::gen::inRange(0, 1001) / 1000.0 * m1;              // 0 .. m1
    if (c.startClear(start, m1)) RC_ASSERT(c.startClear(start, m0));
}

/*
Property: startClear brackets its guarantees. With h the grid resolution, m = (sqrt(2)/2 + 1/2)*h +
extra_margin, q an OCCUPIED cell centre in the body frame (origin + (index + 0.5)*h per axis, moved by
R(-theta)), n_k the outward unit normal and d_k the offset of edge k, s = max_k (n_k . q - d_k) the
signed distance outside the worst edge line, and dist the straight-line distance from q to the footprint
polygon (0 when s <= 0), two implications hold for every case, with band = 1e-6 m:
  (A) if some OCCUPIED centre has dist <= m - band, startClear is false;
  (B) if startClear is false, some OCCUPIED centre has s <= m + band.
A centre farther than m from the footprint but still within m outside every edge line (a sharp corner
tip) may give either answer. Grids are 40..80 cells per side with a non-zero origin and poses that can
hang off the grid; both the 1 m square and a non-square, off-centre rectangle are used.
*/
RC_GTEST_PROP(CollisionCheckerProp, startClearBracketsItsGuarantees, ()){
    const bool square = *rc::gen::arbitrary<bool>();
    const RobotParams body = square
        ? buildRobotParams({Vec2(0.5, 0.5), Vec2(-0.5, 0.5), Vec2(-0.5, -0.5), Vec2(0.5, -0.5)})
        : buildRobotParams({Vec2(0.30, 0.15), Vec2(-0.20, 0.15), Vec2(-0.20, -0.15), Vec2(0.30, -0.15)});

    const float h = *rc::gen::element(0.03f, 0.05f, 0.1f);
    const int cols = *rc::gen::inRange(40, 81), rows = *rc::gen::inRange(40, 81);
    const Eigen::Vector2f origin(*rc::gen::inRange(0, 13201) / 1000.0f - 7.3f,
                                 *rc::gen::inRange(0, 13201) / 1000.0f - 7.3f);
    GridMetaData meta{cols, rows, h, origin};

    std::vector<int8_t> occ(cols * rows, 0);
    std::vector<GridCoords> occupied;
    const int n_occ = *rc::gen::inRange(1, 13);
    for (int n = 0; n < n_occ; ++n){
        const GridCoords c{*rc::gen::inRange(0, cols), *rc::gen::inRange(0, rows)};
        occ[c.j * cols + c.i] = 100;
        occupied.push_back(c);
    }
    const int n_unk = *rc::gen::inRange(0, 6);
    for (int n = 0; n < n_unk; ++n){
        const int i = *rc::gen::inRange(0, cols), j = *rc::gen::inRange(0, rows);
        if (occ[j * cols + i] == 0) occ[j * cols + i] = -1;
    }

    const double res = meta.resolution, ox = meta.origin.x(), oy = meta.origin.y();
    double px, py;
    if (*rc::gen::inRange(0, 2) == 0){   // half of the poses sit near an occupied cell
        const GridCoords c = occupied[*rc::gen::inRange<std::size_t>(0, occupied.size())];
        px = ox + (c.i + 0.5) * res + *rc::gen::inRange(-800, 801) / 1000.0;
        py = oy + (c.j + 0.5) * res + *rc::gen::inRange(-800, 801) / 1000.0;
    } else {
        px = ox - 1.0 + *rc::gen::inRange(0, 1001) / 1000.0 * (cols * res + 2.0);
        py = oy - 1.0 + *rc::gen::inRange(0, 1001) / 1000.0 * (rows * res + 2.0);
    }
    const double theta = *rc::gen::inRange(-31415, 31416) / 10000.0;
    const double extra = *rc::gen::element(0.0, 0.5 * res, 0.07, 0.3);

    auto grid = std::make_shared<const OccupancyGrid>(meta, occ);
    auto edt = std::make_shared<const EDT>(buildEDTGrid(*grid));
    CollisionChecker checker(grid, edt, body);

    // Outward unit normals n_k and offsets d_k of the edges, orientation from the signed area.
    const auto & v = body.vertices;
    const std::size_t nv = v.size();
    double area2 = 0.0;
    for (std::size_t k = 0; k < nv; ++k){
        const auto & a = v[k]; const auto & b = v[(k + 1) % nv];
        area2 += a.x() * b.y() - b.x() * a.y();
    }
    const double sgn = area2 > 0.0 ? 1.0 : -1.0;   // CCW: outward normal of (dx, dy) is (dy, -dx)
    std::vector<double> nx(nv), ny(nv), d(nv);
    for (std::size_t k = 0; k < nv; ++k){
        const auto & a = v[k]; const auto & b = v[(k + 1) % nv];
        const double dx = b.x() - a.x(), dy = b.y() - a.y(), len = std::hypot(dx, dy);
        nx[k] = sgn * dy / len; ny[k] = -sgn * dx / len;
        d[k] = nx[k] * a.x() + ny[k] * a.y();
    }

    const double m = (std::sqrt(2.0) / 2.0 + 0.5) * res + extra;
    const double ct = std::cos(theta), st = std::sin(theta), band = 1e-6;
    bool near = false, edge_in = false;
    for (int j = 0; j < rows; ++j)
        for (int i = 0; i < cols; ++i){
            if (occ[j * cols + i] != 100) continue;
            const double cx = ox + (i + 0.5) * res - px, cy = oy + (j + 0.5) * res - py;
            const double qx = ct * cx + st * cy, qy = -st * cx + ct * cy;   // R(-theta) * (c - p)
            double s = -1e300;
            for (std::size_t k = 0; k < nv; ++k) s = std::max(s, nx[k] * qx + ny[k] * qy - d[k]);
            double dist = 0.0;
            if (s > 0.0){
                dist = 1e300;
                for (std::size_t k = 0; k < nv; ++k){
                    const auto & a = v[k]; const auto & b = v[(k + 1) % nv];
                    const double ex = b.x() - a.x(), ey = b.y() - a.y();
                    const double t = std::clamp(((qx - a.x()) * ex + (qy - a.y()) * ey) / (ex * ex + ey * ey), 0.0, 1.0);
                    dist = std::min(dist, std::hypot(qx - (a.x() + t * ex), qy - (a.y() + t * ey)));
                }
            }
            if (dist <= m - band) near = true;
            if (s <= m + band) edge_in = true;
        }

    RC_CLASSIFY(near, "near");
    RC_CLASSIFY(!near && edge_in, "corner band");
    RC_CLASSIFY(!edge_in, "far");
    const bool result = checker.startClear(SE2State{px, py, theta}, extra);
    if (near) RC_ASSERT(!result);
    if (!result) RC_ASSERT(edge_in);
}
