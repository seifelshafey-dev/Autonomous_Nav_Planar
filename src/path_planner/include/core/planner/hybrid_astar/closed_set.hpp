/**
 * @file closed_set.hpp
 * @author Seif ElShafey
 * @brief Defines the interface for the A* closed set, as well as the node key used for hashing based on discretized grid state.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>
#include <cassert>
#include "absl/container/flat_hash_map.h"

/// @brief Packed `(grid_i, grid_j, heading_bin)` triple used as the closed-set hash key.
using ClosedKey = int64_t;

constexpr int I_BITS = 24;    ///< Number of bits reserved for `grid_i` inside `ClosedKey`.
constexpr int J_BITS = 24;    ///< Number of bits reserved for `grid_j` inside `ClosedKey`.
constexpr int K_BITS = 16;    ///< Number of bits reserved for `heading_bin` inside `ClosedKey`.
static_assert(I_BITS + J_BITS + K_BITS == 64);

constexpr int32_t I_MAX = int32_t{1} << I_BITS; ///< Total addressable range along the `i` axis after packing.
constexpr int32_t J_MAX = int32_t{1} << J_BITS; ///< Total addressable range along the `j` axis after packing.
constexpr int32_t K_MAX = int32_t{1} << K_BITS; ///< Total number of distinct heading bins addressable.

/// @brief Offset added to a signed `grid_i` before packing; supported range is `[-I_OFFSET, I_OFFSET)`.
constexpr int32_t I_OFFSET = I_MAX / 2;
/// @brief Offset added to a signed `grid_j` before packing; supported range is `[-J_OFFSET, J_OFFSET)`.
constexpr int32_t J_OFFSET = J_MAX / 2;

/**
 * @brief Pack a `(grid_i, grid_j, heading_bin)` triple into a single `ClosedKey`.
 *
 * @param i Grid column; supported range `[-I_OFFSET, I_OFFSET)`.
 * @param j Grid row; supported range `[-J_OFFSET, J_OFFSET)`.
 * @param k Heading bin; supported range `[0, K_MAX)`.
 * @return ClosedKey Packed key suitable as a hash-map index.
 * @note `i` and `j` may be negative or fall outside the current SLAM grid extent — the offset
 *       lets the planner expand into cells that the map has not yet covered without corrupting
 *       the key. At 0.05 m resolution the supported signed range exceeds ±400 km.
 */
inline ClosedKey packClosedKey(int32_t i, int32_t j, int32_t k) noexcept {
    const uint32_t ui = static_cast<uint32_t>(i + I_OFFSET);
    const uint32_t uj = static_cast<uint32_t>(j + J_OFFSET);
    assert(ui < static_cast<uint32_t>(I_MAX) && "grid i out of packing range");
    assert(uj < static_cast<uint32_t>(J_MAX) && "grid j out of packing range");
    assert(k >= 0 && k < K_MAX && "heading bin out of packing range");

    return (static_cast<uint64_t>(ui) << (J_BITS + K_BITS))
         | (static_cast<uint64_t>(uj) << K_BITS)
         |  static_cast<uint64_t>(static_cast<uint16_t>(k));
}

/**
 * @brief Closed-set payload: the best `g`-cost seen for a `ClosedKey` and the node-pool index of
 *        the corresponding `Node`.
 */
struct ClosedEntry{
    double g_cost;  ///< Lowest `g` value observed for this closed-set key.
    int32_t node_idx; ///< Index of the corresponding node in the planner's node pool.
};

/// @brief Closed set keyed by packed `(grid_i, grid_j, heading_bin)`.
using ClosedSet = absl::flat_hash_map<ClosedKey, ClosedEntry>;
