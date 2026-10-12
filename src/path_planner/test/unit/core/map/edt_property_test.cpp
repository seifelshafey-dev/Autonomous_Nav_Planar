#include <gtest/gtest.h>
#include <rapidcheck/gtest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include "core/map/edt.hpp"
#include "core/map/grid.hpp"

/*
Property-based test for buildEDTGrid (RapidCheck + OpenCV).

Headline property: distanceAt(c,r) == min(true nearest-obstacle Euclidean distance * resolution,
max_dist). A brute-force O(cells*obstacles) oracle is the independent ground truth; comparing against
it validates the OpenCV integration AND the read-time clamp in one shot. Grids are kept small so the
oracle is cheap, and a min/max range on max_dist ensures some cells actually clamp.
*/

RC_GTEST_PROP(BuildEDTProp, matchesBruteForceNearestObstacleClamped, ()) {
    const int cols = *rc::gen::inRange(2, 13);
    const int rows = *rc::gen::inRange(2, 13);
    const double res = *rc::gen::inRange(10, 201) / 100.0;   // 0.1 .. 2.0
    const double max_dist = *rc::gen::inRange(1, 2001) / 100.0; // 0.01 .. 20.0 (forces clamping sometimes)

    std::vector<int8_t> data(static_cast<std::size_t>(rows) * cols);
    std::vector<std::pair<int, int>> obstacles; // (col, row)
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c){
            bool is_obstacle = (*rc::gen::inRange(0, 4) == 0); // ~25% obstacle density
            data[r * cols + c] = is_obstacle ? 100 : 0;
            if (is_obstacle) obstacles.emplace_back(c, r);
        }

    RC_PRE(!obstacles.empty()); // the no-obstacle case is covered deterministically

    OccupancyGrid grid(GridMetaData(cols, rows, res, Eigen::Vector2f{0, 0}), data);
    EDT edt = buildEDTGrid(grid, max_dist);

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c){
            double best = std::numeric_limits<double>::infinity();
            for (const auto & ob : obstacles){
                double dc = c - ob.first;
                double dr = r - ob.second;
                best = std::min(best, std::sqrt(dc * dc + dr * dr));
            }
            double expected = std::min(best * res, max_dist);
            RC_ASSERT(std::abs(edt.distanceAt(c, r) - expected) <= 1e-3);
        }
}
