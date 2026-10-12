/**
 * @file grid_coords.hpp
 * @author Seif ElShafey
 * @brief Defines an interface for grid accessing.
 * @version 0.1
 * @date 2026-05-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <compare>

/**
 * @brief Grid cell position expressed as integer indices.
 * Comparators are defined so cells can be sorted and compared for position checks.
 * @note Ordered as (col, row).
 */

struct GridCoords{
    int i; ///< index for x-position in grid resolution (column entry)
    int j; ///< index for y-position in grid resolution (row entry)
    /**
     * @brief Ordering compares indices relative to the grid origin (usually (0,0)):
     *        x indices increase to the right, y indices increase upwards.
     * @return Lexicographic ordering of (i, j): `i` is compared first, then `j`.
     */
    auto operator <=>(const GridCoords&) const = default;
    /**
     * @brief Equal cells represent the same position in space.
     * @return True if both `i` and `j` are equal.
     */
    bool operator ==(const GridCoords&) const = default;
    /**
     * @brief Component-wise sum of two cell indices.
     * @param other Offset to add (cells).
     * @return Cell with indices `(i + other.i, j + other.j)`.
     */
    GridCoords operator +(const GridCoords & other) const {return GridCoords{this->i + other.i, this->j + other.j};}
};