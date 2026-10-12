/**
 * @file rasterize.hpp
 * @author Seif ElShafey
 * @brief Defines a vector-based 2D body (robot) or line segment into the grid cells they occupy/traverse on a 2D map.
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <vector>
#include "core/types/grid_coords.hpp"
#include "core/types/geometry.hpp"
#include "core/diagnostics/check.hpp"


/**
 * @brief Visit, one at a time, the cells covered by a convex body given as half-planes, including a
 *        safety margin.
 *
 * Visits exactly the cells `rasterizePolygon` returns for the same arguments, each exactly once, in the
 * same order (first index ascending, second index ascending within it), without building a list. The
 * visit can stop the walk early.
 *
 * A cell is visited only if its centre passes every half-plane test with the plane pushed out by half a
 * cell diagonal plus `margin`, and it overlaps the bounding box grown by the same amount (indices from
 * `floor((min - c) / cell_size)` to `floor((max + c) / cell_size)` inclusive, `c` being half a cell diagonal plus `margin`).
 * Cell `(i, j)` has its centre at `((i + 0.5) * cell_size, (j + 0.5) * cell_size)` in the input frame.
 *
 * @tparam Visit Callable taking a `GridCoords` and returning `bool`.
 * @param world_half_planes Convex half-planes of the body, in the caller's frame (body-frame or
 *        origin-relative).
 * @param world_aabb Axis-aligned bounding box of the body in the same frame as the half-planes (m).
 * @param cell_size Grid resolution (m per cell); must be finite and > 0.
 * @param margin Extra clearance around the body (m); must be finite and >= 0.
 * @param visit Called once per covered cell with its index in the same frame as the input geometry.
 *        Return true to continue, false to stop. An exception thrown by `visit` propagates.
 * @return false if `visit` returned false and the walk stopped early; true otherwise (including when no
 *         cell is covered).
 *
 * @note A non-finite or non-positive `cell_size`, or a negative or non-finite `margin`, throws
 *       `std::invalid_argument` before any cell is visited.
 */
template <typename Visit>
bool visitPolygonCells(const std::vector<HalfPlane> & world_half_planes,
                       const AABB & world_aabb,
                       double cell_size,
                       double margin,
                       Visit && visit){
    CHECK(std::isfinite(cell_size) && cell_size > 0.0, "polygon rasterization: cell_size must be finite and > 0");
    CHECK(std::isfinite(margin) && margin >= 0.0, "polygon rasterization: margin must be finite and >= 0");
    const double clearance = (0.5 * std::sqrt(2.0) * cell_size) + margin;
    const int i_min = static_cast<int>(std::floor((world_aabb.min.x() - clearance) / cell_size));
    const int j_min = static_cast<int>(std::floor((world_aabb.min.y() - clearance) / cell_size));
    const int i_max = static_cast<int>(std::floor((world_aabb.max.x() + clearance) / cell_size));
    const int j_max = static_cast<int>(std::floor((world_aabb.max.y() + clearance) / cell_size));

    for(int i = i_min; i <= i_max; i++){
        const double px = (i + 0.5) * cell_size;
        for(int j = j_min; j <= j_max; j++){
            const double py = (j + 0.5) * cell_size;
            bool inside = true;
            for(const auto & hp : world_half_planes){
                if((hp.normal.x() * px + hp.normal.y() * py) > hp.offset + clearance){
                    inside = false;
                    break;
                }
            }
            if(inside && !visit(GridCoords{i, j})) return false;
        }
    }
    return true;
}

/**
 * @brief Cells covered by a convex body, given as half-planes, including a safety margin.
 *
 * A cell is returned if its centre passes every half-plane test with the plane pushed out by half a cell
 * diagonal plus `margin`, and it overlaps the bounding box grown by the same amount (indices from
 * `floor((min - c) / cell_size)` to `floor((max + c) / cell_size)` inclusive, `c` being half a cell diagonal plus `margin`). When
 * `world_aabb` encloses the body, this includes every cell whose area touches the body, also a cell the
 * body only clips at a corner.
 *
 * @param world_half_planes Convex half-planes of the body, in the caller's frame (body-frame or
 *        origin-relative).
 * @param world_aabb Axis-aligned bounding box of the body in the same frame as the half-planes (m).
 * @param cell_size Grid resolution (m per cell); must be > 0.
 * @param margin Extra clearance around the body (m); must be finite and >= 0. Default 0.
 * @return Cells covering the body, indexed in the same frame as the input geometry. When `world_aabb`
 *         encloses the body, the result contains every cell that any point within `margin` metres of
 *         the body lies in.
 *
 * @note The rasterizer is frame-agnostic: it floors `x / cell_size` in whatever frame the input is in.
 *       To obtain map-grid cells (as used by `OccupancyGrid::findCellStatus`), pass geometry already
 *       translated by `-cell_origin`, e.g. via `transformGeometry` with the map origin.
 * @note A non-finite or non-positive `cell_size`, or a negative or non-finite `margin`, throws
 *       `std::invalid_argument`.
 */
std::vector<GridCoords> rasterizePolygon(const std::vector<HalfPlane> & world_half_planes,
                                         const AABB & world_aabb,
                                         double cell_size,
                                         double margin = 0.0);
