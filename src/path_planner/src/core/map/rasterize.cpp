/**
 * @file rasterize.cpp
 * @author Seif ElShafey
 * @brief Implementation of the declared functions in rasterization.hpp
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "core/map/rasterize.hpp"
#include "core/map/grid.hpp"
#include "core/diagnostics/check.hpp"

// O(N*N) time complexity, but N is limited by the size of the robot vs the size of the grid. Usually small.
// Move forward with scan-fill if too slow.
std::vector<GridCoords> rasterizePolygon(const std::vector<HalfPlane> & world_half_planes,
                                         const AABB & world_aabb,
                                         double cell_size,
                                         double margin){
    std::vector<GridCoords> robot_footprint;
    visitPolygonCells(world_half_planes, world_aabb, cell_size, margin, [&](const GridCoords & c){
        robot_footprint.push_back(c);
        return true;
    });
    return robot_footprint;
}
