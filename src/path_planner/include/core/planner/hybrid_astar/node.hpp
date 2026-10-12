/**
 * @file node.hpp
 * @author Seif ElShafey
 * @brief Defines the interface for each node in the implicit hybrid A* graph.
 * @version 0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #pragma once

 #include <cstdint>
 #include "core/types/state.hpp"
 #include "core/types/grid_coords.hpp"

/**
 * @brief One node in the hybrid A* search graph.
 *
 * A `Node` couples the continuous SE(2) state with the discrete (grid, heading-bin) key used for
 * the closed set, the costs reported by the heuristic and edge functions, and back-references to
 * the parent node and the primitive applied to reach it. Path reconstruction walks `parent_idx`
 * pointers from the goal back to the start.
 */
struct Node{
    SE2State state; ///< Continuous robot state defined by the node in [m,m,rad]
    double g_cost; ///< True traversal cost of reaching the node based on path distance, clearance, and unknown cell traversal.
    double h_cost; ///< Heuristic estimate of the remaining cost to the goal. Used to order the search.
    int32_t parent_idx; ///< Index of the parent node. A parent node is defined as the node i such that applying primitive j to node i would result in reaching the given node.
    int16_t prim_idx; ///< Index of the control input in the primitive library applied from the parent state to reach the current node.
    int16_t heading_bin; ///< Binned heading calculation required for cached swept footprint calculation.
    GridCoords grid_coords; ///< Derived grid position from state. Helps with cached lookup.
 };