/**
 * @file grid.hpp
 * @author Seif ElShafey
 * @brief Defines a 2D config-space occupancy grid with its appropriate methods
 * @version 0.3
 * @date 2026-05-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include <optional>
#include <cmath>

#include "core/types/grid_coords.hpp"
#include "core/diagnostics/check.hpp"

/**
 * @brief Interface holding all grid representation properties, excluding the values of each cell.
 Provides easy indexing, world<->cell transform, and sufficient description of the occupancy grid structure.
 @note If a cell has >=50% chance of being occupied, it's marked as an obstacle due to safety concerns.
 */
struct GridMetaData{
    float world_width; ///< World extent along x in meters: `cols * resolution` rounded up to a whole meter. Positive, except 0 in the metadata of a moved-from grid.
    float world_height; ///< World extent along y in meters: `rows * resolution` rounded up to a whole meter. Positive, except 0 in the metadata of a moved-from grid.
    float resolution; ///< The world-to-cell size interpretation of the map - cell-size defined in meters/cell
    int rows; ///< Number of rows (cells along y) of the occupancy grid 2D matrix representation. Positive, except 0 in the metadata of a moved-from grid.
    int cols; ///< Number of columns (cells along x) of the occupancy grid 2D matrix representation. Positive, except 0 in the metadata of a moved-from grid.
    Eigen::Vector2f origin; ///< The world origin as defined by SLAM mapped to the corner of cell (0,0) with the smallest x and y. For each SLAM map update, the origin may shift such that the mapping of state->cell changes.
    int8_t occupied_threshold = 50; ///< The acceptable probability for which a cell is defined as occupied. If we're >= 50% sure a cell is occupied, we mark it as occupied.

/**
 * @brief Construct a new GridMetaData object defined by the dimensions and granularity of the occupancy map.
 *
 * @param c The width of the occupancy map in number of cells; must be > 0.
 * @param r The height of the occupancy map in number of cells; must be > 0.
 * @param cell_size Defines the resolution/granularity of the grid in meters per cell; must be > 0 and finite.
 * @param orgn World position (m, finite) of the corner of cell (0, 0) with the smallest x and y.
 * @throws std::invalid_argument (via CHECK) if `c`, `r` or `cell_size` is not positive (NaN included), or if `cell_size` (as a float) or `orgn` is not finite.
 */
    GridMetaData(int c, int r, double cell_size, const Eigen::Vector2f & orgn) : resolution(static_cast<float>(cell_size)),
    rows(r), cols(c), origin(orgn){
        CHECK(cols > 0 && rows > 0 && resolution > 0, "grid dimensions and resolution must be positive");
        CHECK(std::isfinite(resolution) && std::isfinite(origin.x()) && std::isfinite(origin.y()), "grid resolution and origin must be finite");
        world_height = std::ceil(rows * resolution);
        world_width = std::ceil(cols * resolution);
    }

/**
 * @brief Whether `gc` indexes a cell inside the grid.
 * @param gc Grid cell indices (column i, row j).
 * @return true if `0 <= gc.i < cols` and `0 <= gc.j < rows`.
 */
inline bool inBounds(const GridCoords gc) const{
    return(gc.i >= 0 && gc.i < cols && gc.j >= 0 && gc.j < rows);
}

/**
 * @brief Row-major linear index of cell `gc` into a `rows * cols` buffer.
 * @param gc Grid cell indices (column i, row j). Caller ensures `inBounds(gc)`.
 * @return std::size_t Offset `j * cols + i`.
 */
inline std::size_t index(const GridCoords & gc) const{
    return static_cast<std::size_t>(gc.j) * cols + gc.i;
}

/**
 * @brief Snap a continuous planar position to the grid cell that contains it.
 *
 * Cell membership is floor-based: cell (i, j) covers `[i*res + origin.x, (i+1)*res + origin.x)` × `[j*res + origin.y, (j+1)*res + origin.y)`.
 * The inverse mapping (`gridToWorld`) returns the cell's center — the canonical representative
 * point used by the rasterizer, EDT, and costmap throughout this codebase.
 *
 * @param x Position along +x axis in meters.
 * @param y Position along +y axis in meters.
 * @return GridCoords Indices (i, j) of the cell containing (x, y).
 */
    GridCoords worldToGrid(double x, double y) const{
        GridCoords gc{static_cast<int>(std::floor((x - origin.x()) / resolution)),
                      static_cast<int>(std::floor((y - origin.y()) / resolution))};
        return gc;
    }

/**
 * @brief Convert a grid cell index to its center position in world coordinates (meters).
 *
 * @note Cell centers are the canonical representative point for grid cells throughout the codebase:
 *       the rasterizer tests half-plane membership at cell centers, and the EDT at cell (i, j) stores
 *       distance from that cell's center. Every point inside a cell therefore lies within h·√2/2 of
 *       its center (half the cell diagonal, for resolution h) — the worst-case position snap error
 *       the footprint inflation must absorb.
 *
 * @param gc Grid cell indices (column i, row j).
 * @return Eigen::Vector2f World-frame center of cell (gc.i, gc.j) in meters.
 */

    Eigen::Vector2f gridToWorld(const GridCoords & gc) const{
        return Eigen::Vector2f{(gc.i + 0.5f) * resolution + origin.x(),
                               (gc.j + 0.5f) * resolution + origin.y()};
    }

};

/**
 * @brief Classification of a grid cell: FREE, OCCUPIED, or UNKNOWN (no occupancy information).
 */
enum CellState {
    FREE, ///< Known cell whose raw value is below `occupied_threshold`.
    OCCUPIED, ///< Cell whose raw value is at least `occupied_threshold`.
    UNKNOWN ///< Raw value -1, or a cell outside the grid.
};

/**
 * @brief Interface for building a 2D occupany grid from nav_msg::OccupancyGrids for raw C++
 representation.
 */

class OccupancyGrid {
public:
/**
 * @brief Construct a grid from its metadata and, optionally, its cell values.
 * @param meta_data Grid dimensions, resolution and origin.
 * @param data_ Row-major raw occupancy values (index `j * cols + i`), `rows * cols` entries; -1 marks
 *        an unknown cell. If empty (`std::nullopt`), every cell is unknown.
 * @throws std::invalid_argument (via CHECK) if `data_` does not hold exactly `rows * cols` entries.
 */
  OccupancyGrid(const GridMetaData & meta_data, std::optional<std::vector<int8_t>> data_) :
                meta_(meta_data), cells_(data_ ? std::move(*data_) :
            std::vector<int8_t>(static_cast<std::size_t>(meta_.rows) * meta_.cols, -1)){
    CHECK(cells_.size() == static_cast<std::size_t>(meta_.rows) * static_cast<std::size_t>(meta_.cols),
          "occupancy data size must equal rows * cols");
  }

/**
 * @brief Copy constructor.
 * @param other Grid to copy.
 */
  OccupancyGrid(const OccupancyGrid& other) = default;
/**
 * @brief Copy assignment.
 * @param other Grid to copy.
 * @return This grid.
 */
  OccupancyGrid& operator=(const OccupancyGrid& other) = default;
/**
 * @brief Move constructor.
 * @param other Grid to move from. Afterwards it is empty: `rows() == cols() == 0`, `getData()` is
 *        empty, `at()` returns -1 and `findCellStatus()` returns UNKNOWN for every cell, `rawAccess()`
 *        returns -1 for every index, and `setData` accepts only an empty vector. `resolution()` and
 *        `origin()` keep their values. It may be assigned to or destroyed.
 */
  OccupancyGrid(OccupancyGrid&& other) noexcept : meta_(other.meta_), cells_(std::move(other.cells_)) {
    other.clear();
  }
/**
 * @brief Move assignment.
 * @param other Grid to move from; left empty as described for the move constructor. Assigning a
 *        grid to itself leaves it unchanged.
 * @return This grid.
 */
  OccupancyGrid& operator=(OccupancyGrid&& other) noexcept {
    if(this != &other) {
      meta_ = other.meta_;
      cells_ = std::move(other.cells_);
      other.clear();
    }
    return *this;
  }

/**
 * @brief Returns the raw cell value corresponding to the occupational probability of the cell
 * 
 * @param gc Grid cell indices (column i, row j).
 * @return int8_t Probability of the cell being occupied [-1, 100]. -1 means the cell's occupation is unknown.
 *         A cell outside the grid also returns -1.
 */

  inline int8_t at(const GridCoords & gc) const {
    if(!meta_.inBounds(gc)) {return -1;}
    return cells_[meta_.index(gc)];
  }

/**
 * @brief Defines whether a chosen grid cell is free, occupied, or unknown (not yet searched).
 * 
 * @param gc Grid cell indices (column i, row j).
 * @return CellState UNKNOWN if the raw value is -1 or the cell is outside the grid; OCCUPIED if the raw
 *         value is >= `occupied_threshold`; FREE otherwise.
 */

  inline CellState findCellStatus(const GridCoords & gc) const {
    CellState status;
    auto raw_value = at(gc);
    if(raw_value == -1) {status = CellState::UNKNOWN;}
    else
    {
        status = (raw_value >= meta_.occupied_threshold) ? CellState::OCCUPIED : CellState::FREE;
    }
    return status;
  }

/**
 * @brief Raw occupancy values, row-major (index `j * cols + i`), `rows * cols` entries.
 * @return Reference to this grid's own vector; it stays valid for the lifetime of the grid and always
 *         reflects the current contents (empty for a moved-from grid). Element pointers and iterators are invalidated by `setData`
 *         and by assigning to or moving from this grid.
 */
  inline const std::vector<int8_t>& getData() const {return cells_;}
/**
 * @brief Replace the raw occupancy values.
 * @param data_ Row-major raw occupancy values, `rows * cols` entries; its contents are moved from.
 * @throws std::invalid_argument (via CHECK) if `data_` does not hold exactly `rows * cols` entries.
 */
  inline void setData(std::vector<int8_t> & data_) {
    CHECK(data_.size() == static_cast<std::size_t>(meta_.rows) * static_cast<std::size_t>(meta_.cols),
          "occupancy data size must equal rows * cols");
    cells_ = std::move(data_);
  }

/**
 * @brief Raw occupancy value at a linear row-major index.
 * @param i Linear index `row * cols + col`.
 * @return The raw value, or -1 if `i` is outside `[0, rows * cols)`.
 */
  inline const int8_t rawAccess(int i) const {return (i >= 0 && static_cast<std::size_t>(i) < cells_.size()) ? cells_[i] : -1;}

/**
 * @brief Metadata of this grid.
 * @return Reference to this grid's own metadata; valid for the lifetime of the grid. Its values
 *         change when the grid is assigned to or moved from.
 */
  inline const GridMetaData& getMetadata() const { return meta_; }

  inline int rows() const { return meta_.rows; } ///< Number of rows in the grid. @return Row count.

  inline int cols() const { return meta_.cols; } ///< Number of columns in the grid. @return Column count.

  inline const Eigen::Vector2f origin() const {return meta_.origin;} ///< World-frame position of the corner of cell (0,0) with the smallest x and y, in meters. @return Origin (m), as in `GridMetaData::origin`.

  inline float resolution() const { return meta_.resolution; } ///< Grid cell size in meters per cell. @return Resolution (m).

  private:
  // Leaves a moved-from grid empty: zero extent, no cells.
  void clear() noexcept {
    meta_.rows = 0;
    meta_.cols = 0;
    meta_.world_width = 0.0f;
    meta_.world_height = 0.0f;
    cells_.clear();
  }

  GridMetaData meta_;
  std::vector<int8_t> cells_;
};

