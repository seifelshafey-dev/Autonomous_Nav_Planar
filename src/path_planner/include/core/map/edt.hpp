/**
 * @file edt.hpp
 * @author Seif ElShafey
 * @brief  Interface for a grid denoting the EDT of the config-space grid cells.
 * @version 0.1
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <vector>
#include <algorithm>
#include "core/map/grid.hpp"
#include "core/diagnostics/check.hpp"

/**
 * @brief Interface for a 2D grid wherein each cell value determines its world distance from the nearest obstacle
 @note The dimensions of the EDT are exactly the same as the config-space occupancy grid.
 @note The maximum distance threshold is only used for clamping large numerical computations. 
 */

class EDT{
    GridMetaData meta_;
    std::vector<float> distance_field; // Unsigned euclidean distance from nearest neighbor
    double maximum_distance; // Clamping threshold for EDT cells in meters. If EDT > maximum_distance, the cell's EDT value is clamped to the max value.

    public:
        /**
         * @brief Construct an EDT from a precomputed distance field and the grid metadata it belongs to.
         *
         * @param m_ Grid metadata; must match the dimensions of `df_`.
         * @param df_ Row-major distance field in meters; size `meta.rows * meta.cols`.
         * @param max_dist Clamping threshold in meters, must be > 0 (infinity allowed); cells farther than this are reported as `max_dist`.
         * @throws std::invalid_argument (via CHECK) if `df_` does not hold exactly `rows * cols` entries.
         * @throws std::invalid_argument (via CHECK) if `max_dist` is not > 0 (NaN included).
         */
        EDT(GridMetaData m_, std::vector<float> df_, double max_dist) : meta_(std::move(m_)), distance_field(std::move(df_)), maximum_distance(max_dist) {
            CHECK(distance_field.size() == static_cast<std::size_t>(meta_.rows) * static_cast<std::size_t>(meta_.cols),
                  "EDT distance field size must equal rows * cols");
            CHECK(maximum_distance > 0.0, "EDT clamp distance must be positive");
        }

        /**
         * @brief Euclidean distance (m) from the cell's centre to the nearest obstacle cell's centre, clamped to the maximum distance.
         * @param col Column index of the cell; must satisfy `0 <= col < cols`.
         * @param row Row index of the cell; must satisfy `0 <= row < rows`.
         * @return Distance in meters, at most `getMaxDist()`; 0 for an obstacle cell. Indices outside the grid are not checked.
         */
        inline float distanceAt(int col, int row) const {
            return std::min(distance_field[row * meta_.cols + col], static_cast<float>(maximum_distance));
        }

        inline const std::vector<float>& getData() const {return distance_field;} ///< Getter for the raw (unclamped) EDT distance field; use distanceAt for clamped values. @return Row-major distances (m), `rows * cols` entries.

        inline const GridMetaData& getMetadata() const { return meta_;} ///< Getter for the EDT grid's properties (dimensions and resolution) @return Reference valid for the lifetime of this EDT.


        inline double getMaxDist() const{return maximum_distance;} ///< Getter for EDT's distance threshold for clamping @return Clamping threshold (m).
        /**
         * @brief Set the distance threshold used for clamping.
         * @param max New clamping threshold (m); must be > 0 (infinity allowed). Applies to `distanceAt`.
         * @throws std::invalid_argument (via CHECK) if `max` is not > 0 (NaN included).
         */
        inline void setMaxDist(double max) {
            CHECK(max > 0.0, "EDT clamp distance must be positive");
            maximum_distance = max;
        }
};

/**
 * @brief Factory function to construct an EDT grid from an occupancy grid. Called at each occupancy grid update.
 @note Requires OpenCV for EDT compute.
 @note A cell is an obstacle if its raw value is >= the grid's `occupied_threshold`; unknown cells are not obstacles.
 * 
 * @param cs_grid Configuration space occupancy grid returned by the SLAM node.
 * @param edt_clamp Clamping threshold (m), must be > 0, stored in the result; if the grid has no obstacle, every cell is `edt_clamp`.
 * @throws std::invalid_argument (via CHECK) if `edt_clamp` is not > 0 (NaN included).
 * @return EDT Full grid of the same dimensions as the occupancy grid denoting the distance between each cell and its respective closest obstacle.
 */

EDT buildEDTGrid(const OccupancyGrid & cs_grid, double edt_clamp = 5.0);