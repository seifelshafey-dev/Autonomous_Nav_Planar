/**
 * @file search_observer.hpp
 * @author Seif ElShafey
 * @brief Planner-agnostic observation seam for streaming search progress (expansion events) to an
 *        external consumer such as a visualizer.
 * @version 0.1
 * @date 2026-06-17
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>
#include "core/types/state.hpp"

/**
 * @brief A single node-expansion event reported by a planner, in the order the planner expands nodes.
 *
 * This is a planner-agnostic snapshot: it carries only the data a consumer needs to reconstruct the
 * search tree in true expansion order. `id` and `parent_id` are opaque integer handles assigned by
 * the planner; `parent_id` refers to the `id` of a previously reported expansion (or is negative for
 * the root). Consumers must not assume any meaning for these handles beyond linkage.
 */
struct SearchExpansion {
    int32_t id;        ///< Opaque handle identifying this expanded node, unique within one plan() call.
    int32_t parent_id; ///< `id` of the parent expansion; negative if this node has no parent (root).
    SE2State state;    ///< Continuous SE(2) pose of the expanded node.
    double g;          ///< Cost-so-far from start to this node.
    double f;          ///< Total estimated cost (g + heuristic) used to order the node in the search.
};

/**
 * @brief Callback sink for search-progress events.
 *
 * Pass a pointer to a concrete observer into the observer-aware `Planner::plan` overload to receive
 * one `onExpand` call per node the planner expands, in expansion order. A planner reports nothing if
 * no observer is supplied. Implementations should keep `onExpand` cheap: it runs inside the search
 * hot loop.
 */
class SearchObserver {
public:
    virtual ~SearchObserver() = default;

    /**
     * @brief Invoked once per node expansion, in the order the planner expands nodes.
     * @param e The expansion event (handle, parent handle, pose, and g/f costs).
     */
    virtual void onExpand(const SearchExpansion& e) = 0;
};
