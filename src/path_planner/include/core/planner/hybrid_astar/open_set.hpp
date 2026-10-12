/**
 * @file open_set.hpp
 * @author Seif ElShafey
 * @brief Defines the interface for the open set in A* graph search.
 * @version 0.1
 * @date 2026-06-02
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <queue>
#include <cstdint>
#include <vector>

/**
 * @brief Entry pushed onto the open set: the index of a node in the planner's node pool and its
 *        `f = g + h` priority value.
 */
struct OpenEntry{
    int32_t idx; ///< Refers to the node index in the node pool.
    double f; ///< Refers to the f(i) cost of the node (g+h).
    double h; ///< Heuristic cost-to-goal; the secondary ordering key, breaking f-ties toward the goal.
    /// True when the motion from the node's parent has not yet been confirmed collision-free over its
    /// exact motion. Not part of the ordering (see `OpenEntryGreater`).
    bool pending_exact = false;
};

/**
 * @brief Comparator that orders `OpenEntry` values by ascending `f`, then ascending `h`, then
 *        ascending `idx`, suitable as the comparison functor for `std::priority_queue` (which is a
 *        max-heap by default).
 *
 * On equal `f` the entry with the smaller `h` (closer to goal) is returned first; on equal `f` and
 * `h` the entry with the smaller `idx` (the earlier-generated node) is returned first. For entries
 * with distinct `idx` this is a strict total order, so the pop order is fully determined by the
 * entries' values and does not depend on insertion history. The secondary keys never reorder
 * entries with different `f`.
 */
struct OpenEntryGreater{
    /**
     * @brief Compare two open-set entries by `f`, then `h`, then `idx`.
     * @param a First entry.
     * @param b Second entry.
     * @return true when `a` should be returned later than `b`: `a.f > b.f`, or on equal `f`
     *         `a.h > b.h`, or on equal `f` and `h` `a.idx > b.idx`.
     */
    bool operator()(const OpenEntry & a, const OpenEntry & b) const noexcept{
        return a.f != b.f ? a.f > b.f : a.h != b.h ? a.h > b.h : a.idx > b.idx;
    }
};

/// @brief Min-heap priority queue of `OpenEntry`; pops in ascending `f`, then `h`, then `idx`
///        (see `OpenEntryGreater`), independent of insertion order.
using OpenSet = std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater>;
