#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/map/rasterize.hpp"
#include "core/types/geometry.hpp"

/*
Property-based tests for rasterize.cpp (RapidCheck).

rasterizePolygon: coverage / no-false-negative. Every point inside the true (un-inflated) polygon
must land in a returned cell -- this verifies the clearance inflation is large enough to never miss
a cell the polygon touches. Oracle = independent point-in-polygon sampling.
*/

namespace {
int cellIndex(double v, double cs){ return static_cast<int>(std::floor(v / cs)); }
}

// Coverage: the rasterized footprint contains the cell of every interior point of the polygon.
RC_GTEST_PROP(RasterizeProp, footprintCoversPolygonInterior, ()) {
    const double cs = *rc::gen::inRange(10, 201) / 100.0; // 0.1 .. 2.0
    const double x0 = *rc::gen::inRange(-50, 51) / 10.0;  // -5 .. 5
    const double y0 = *rc::gen::inRange(-50, 51) / 10.0;
    const double w  = *rc::gen::inRange(1, 51) / 10.0;    // 0.1 .. 5.0
    const double h  = *rc::gen::inRange(1, 51) / 10.0;
    const double x1 = x0 + w, y1 = y0 + h;

    std::vector<HalfPlane> hp = {{Vec2(1, 0), x1}, {Vec2(-1, 0), -x0},
                                 {Vec2(0, 1), y1}, {Vec2(0, -1), -y0}};
    std::vector<GridCoords> cells = rasterizePolygon(hp, AABB{Vec2(x0, y0), Vec2(x1, y1)}, cs);

    std::set<std::pair<int, int>> cset;
    for (const auto & c : cells) cset.insert({c.i, c.j});

    // Sample the closed rectangle on a 5x5 fraction grid.
    for (int a = 0; a <= 4; ++a)
        for (int b = 0; b <= 4; ++b){
            double px = x0 + (a / 4.0) * w;
            double py = y0 + (b / 4.0) * h;
            RC_ASSERT(cset.count({cellIndex(px, cs), cellIndex(py, cs)}) == 1);
        }
}

// Margin: a convex polygon inscribed in a circle. result(m) contains result(0), and contains the cell of
// every point within m of the polygon (boundary points pushed outward along their edge normal).
RC_GTEST_PROP(RasterizeProp, marginResultIsMonotoneAndCoversOutwardPoints, ()) {
    const double cs = *rc::gen::inRange(5, 51) / 100.0;           // 0.05 .. 0.5
    const double m  = *rc::gen::inRange(0, 1001) / 1000.0 * 2.0 * cs;  // [0, 2*cs]
    const double cx = *rc::gen::inRange(-50, 51) / 10.0;
    const double cy = *rc::gen::inRange(-50, 51) / 10.0;
    const double R  = *rc::gen::inRange(2, 31) / 10.0;            // 0.2 .. 3.0
    const int n = *rc::gen::inRange(3, 8);
    const double phase = *rc::gen::inRange(0, 1000) / 1000.0 * 2.0 * M_PI;

    std::vector<Vec2> v;
    for (int k = 0; k < n; ++k){
        const double u = *rc::gen::inRange(0, 301) / 1000.0;      // jitter < 0.3 of a sector: gaps < pi
        const double ang = phase + 2.0 * M_PI * (k + u) / n;
        v.emplace_back(cx + R * std::cos(ang), cy + R * std::sin(ang));
    }
    std::vector<HalfPlane> hps;
    std::vector<Vec2> normals;
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (int k = 0; k < n; ++k){
        const Vec2 e = v[(k + 1) % n] - v[k];
        const Vec2 nrm = Vec2(e.y(), -e.x()).normalized();         // outward for CCW order
        normals.push_back(nrm);
        hps.push_back({nrm, nrm.dot(v[k])});
        xmin = std::min(xmin, v[k].x()); xmax = std::max(xmax, v[k].x());
        ymin = std::min(ymin, v[k].y()); ymax = std::max(ymax, v[k].y());
    }
    const AABB box{Vec2(xmin, ymin), Vec2(xmax, ymax)};

    std::set<std::pair<int, int>> base, widened;
    for (const auto & c : rasterizePolygon(hps, box, cs, 0.0)) base.insert({c.i, c.j});
    for (const auto & c : rasterizePolygon(hps, box, cs, m)) widened.insert({c.i, c.j});
    for (const auto & c : base) RC_ASSERT(widened.count(c) == 1);

    for (int k = 0; k < n; ++k){
        const int sk = *rc::gen::inRange(0, 1001);
        const int dk = *rc::gen::inRange(0, 1001);
        const Vec2 p = v[k] + (sk / 1000.0) * (v[(k + 1) % n] - v[k]) + (dk / 1000.0 * m) * normals[k];
        RC_ASSERT(widened.count({cellIndex(p.x(), cs), cellIndex(p.y(), cs)}) == 1);
    }
}

// visitPolygonCells visits exactly rasterizePolygon's cells, in the same order.
RC_GTEST_PROP(RasterizeProp, visitSequenceEqualsRasterizePolygon, ()) {
    const double cs = *rc::gen::inRange(5, 51) / 100.0;
    const double m  = *rc::gen::inRange(0, 1001) / 1000.0 * 2.0 * cs;
    const double cx = *rc::gen::inRange(-50, 51) / 10.0;
    const double cy = *rc::gen::inRange(-50, 51) / 10.0;
    const double R  = *rc::gen::inRange(2, 31) / 10.0;
    const int n = *rc::gen::inRange(3, 8);
    const double phase = *rc::gen::inRange(0, 1000) / 1000.0 * 2.0 * M_PI;

    std::vector<Vec2> v;
    for (int k = 0; k < n; ++k){
        const double u = *rc::gen::inRange(0, 301) / 1000.0;
        const double ang = phase + 2.0 * M_PI * (k + u) / n;
        v.emplace_back(cx + R * std::cos(ang), cy + R * std::sin(ang));
    }
    std::vector<HalfPlane> hps;
    double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;
    for (int k = 0; k < n; ++k){
        const Vec2 e = v[(k + 1) % n] - v[k];
        const Vec2 nrm = Vec2(e.y(), -e.x()).normalized();
        hps.push_back({nrm, nrm.dot(v[k])});
        xmin = std::min(xmin, v[k].x()); xmax = std::max(xmax, v[k].x());
        ymin = std::min(ymin, v[k].y()); ymax = std::max(ymax, v[k].y());
    }
    const AABB box{Vec2(xmin, ymin), Vec2(xmax, ymax)};

    std::vector<GridCoords> visited;
    const bool ret = visitPolygonCells(hps, box, cs, m, [&](GridCoords c){ visited.push_back(c); return true; });
    RC_ASSERT(ret);
    RC_ASSERT(visited == rasterizePolygon(hps, box, cs, m));
}
