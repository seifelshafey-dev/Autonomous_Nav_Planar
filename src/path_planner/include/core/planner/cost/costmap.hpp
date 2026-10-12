/**
 * @file costmap.hpp
 * @author Seif ElShafey
 * @brief Interface for a spatial costmap defined by obstacle clearance and cell occupation status for planner costs.
 * @version 0.1
 * @date 2026-05-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <vector>
#include "core/planner/cost/cost_policy.hpp"
#include "core/map/map_provider.hpp"


/**
 * @brief Spatial cell->cost mapping over the grid. Each cell's cost combines three contributions:
 *        occupancy (an occupied cell takes the maximum clearance cost), a smooth EDT-based inflation
 *        cost that decays with distance from obstacles, and a penalty for unknown cells.
 */

class CostMap{

    public:
        /**
         * @brief Construct a cost map from grid metadata and per-cell costs.
         * @param m_ Grid metadata.
         * @param c_ Row-major per-cell cost, `rows * cols` entries.
         * @param unknown_cost Per-metre cost of unmapped territory, returned by `unknownCost()` and by `costAt` outside the grid.
         */
        CostMap(GridMetaData m_, std::vector<float> c_, float unknown_cost)
            : meta_(std::move(m_)), cost_(std::move(c_)), unknown_cost_(unknown_cost) {}

        /**
         * @brief Cost at (col, row). Out-of-grid cells return `unknownCost()`, so the planner can
         *        expand past the grid extent with the unknown-space penalty.
         *
         * @param col Column index of the cell (may be outside the grid).
         * @param row Row index of the cell (may be outside the grid).
         * @return The stored cost of an in-grid cell; `unknownCost()` outside the grid.
         */
        inline float costAt(int col, int row) const {
            if (col < 0 || col >= meta_.cols || row < 0 || row >= meta_.rows) {
                return unknown_cost_;
            }
            return cost_[meta_.index(GridCoords{col, row})];
        }

        inline const std::vector<float>& getData() const {return cost_;} ///< Getter for the CostMap data vector @return Row-major per-cell costs, `rows * cols` entries.

        inline const GridMetaData& getMetadata() const { return meta_;} ///< Getter for the CostMap's properties (dimensions and resolution) @return Reference valid for the lifetime of this cost map.

        inline float unknownCost() const noexcept {return unknown_cost_;} ///< Per-metre cost `u` of unmapped territory (in-grid unknown and out-of-grid alike); see buildCostMap(). @return The per-metre unknown cost `u`.

    private:
        GridMetaData meta_;
        std::vector<float> cost_;
        float unknown_cost_;    ///< Per-metre cost of unmapped territory.
};

/**
 * @brief Builds the per-cell traversal cost map for a map snapshot.
 *
 * Known cells cost `max_clearance_cost * exp(-clearance_decay * d)` when their distance-map value
 * `d` (m) is below `clearance_radius`, else 0. Unknown in-grid cells add a further per-metre cost `u`
 * on top, and cells outside the grid cost `u`.
 *
 * `u` is chosen so that a metre of unknown space is never cheaper, under the search's per-metre
 * charge `1 + costmap_weight * cost`, than the dearest metre of known free space the robot can stand
 * on. With `d_min = max(0, r_in - (sqrt(2)/2) * h)` (h = grid resolution) and
 * `b = (d_min < clearance_radius) ? max_clearance_cost * exp(-clearance_decay * d_min) : 0`:
 * `u = (unknown_cost_factor * (1 + costmap_weight * b) - 1) / costmap_weight` for `costmap_weight > 0`,
 * and `u = 0` for `costmap_weight == 0`.
 *
 * @param map_ Snapshot whose grid and distance map define the cells (distance map in m).
 * @param p_ Clearance settings (non-negative) and `unknown_cost_factor` (>= 1).
 * @param r_in Inscribed radius of the robot footprint (m); positive and finite.
 * @param costmap_weight Weight on the cost map in the search's per-metre charge; non-negative and finite.
 * @return The cost map; its `unknownCost()` is `u`.
 * @throws std::invalid_argument (via CHECK) if an argument violates the ranges above.
 */
CostMap buildCostMap(const MapSnapshot& map_, const CostPolicy & p_, double r_in, double costmap_weight);