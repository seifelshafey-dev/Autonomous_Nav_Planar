#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <Eigen/Core>

#include "core/map/grid.hpp"

/*
Property-based tests for the GridMetaData world<->grid transforms (RapidCheck).

Generators are bounded so the float-valued transforms keep precision: indices well below 2^24
(beyond which an int index isn't representable as float), resolution and origin in modest ranges.
*/

namespace {
// Random valid metadata (cols/rows/resolution strictly positive, per the GridMetaData invariant).
GridMetaData randomMeta(){
    int cols   = *rc::gen::inRange(1, 201);
    int rows   = *rc::gen::inRange(1, 201);
    double res = *rc::gen::inRange(1, 501) / 100.0;     // 0.01 .. 5.0
    float ox   = *rc::gen::inRange(-5000, 5001) / 100.0f; // -50 .. 50
    float oy   = *rc::gen::inRange(-5000, 5001) / 100.0f;
    return GridMetaData(cols, rows, res, Eigen::Vector2f{ox, oy});
}
}

// Symmetry: a cell -> its center -> back to the cell is exact. This exercises the floor math and,
// crucially, sign handling for negative indices (floor, not truncation toward zero).
RC_GTEST_PROP(GridTransformProp, worldToGridInvertsGridToWorld, ()) {
    GridMetaData meta = randomMeta();
    GridCoords gc{*rc::gen::inRange(-50000, 50001), *rc::gen::inRange(-50000, 50001)};

    Eigen::Vector2f center = meta.gridToWorld(gc);
    GridCoords round_trip = meta.worldToGrid(center.x(), center.y());

    RC_ASSERT(round_trip == gc);
}

// Snap bound: a world point -> its cell -> that cell's center moves the point by at most res/2 per
// axis. The reverse round-trip can't be identity (continuous->discrete loses sub-cell position);
// this is the correct invariant for that direction.
RC_GTEST_PROP(GridTransformProp, gridToWorldSnapsWithinHalfResolution, ()) {
    GridMetaData meta = randomMeta();
    const double res = meta.resolution;
    const float ox = meta.origin.x();
    const float oy = meta.origin.y();

    // Point drawn from within the grid extent: [origin, origin + n*res).
    double fx = *rc::gen::inRange(0, 1000000) / 1000000.0; // [0, 1)
    double fy = *rc::gen::inRange(0, 1000000) / 1000000.0;
    double x = ox + fx * meta.cols * res;
    double y = oy + fy * meta.rows * res;

    Eigen::Vector2f center = meta.gridToWorld(meta.worldToGrid(x, y));

    RC_ASSERT(std::abs(x - center.x()) <= res / 2.0 + 1e-3);
    RC_ASSERT(std::abs(y - center.y()) <= res / 2.0 + 1e-3);
}
