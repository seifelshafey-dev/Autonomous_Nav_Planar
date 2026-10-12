/**
 * @file search_trace.hpp
 * @author Seif ElShafey
 * @brief Renderer-agnostic snapshot of one hybrid A* query, for non-ROS visualization.
 * @version 0.1
 * @date 2026-06-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/map/grid.hpp"
#include "core/planner/path.hpp"
#include "core/planner/search_observer.hpp"
#include "core/types/geometry.hpp"
#include "core/types/state.hpp"

/**
 * @brief One expanded node in the rendered search tree, in true expansion (pop) order.
 *
 * Built from the planner's `SearchObserver` event stream, so the sequence is the order the planner
 * actually popped and expanded nodes — not node-generation order. `parent` is an index into the
 * enclosing `SearchTrace::nodes` vector (the position of this node's parent expansion), or negative
 * for the root. These are local renderer indices; the planner's opaque handles are remapped away.
 */
struct TraceNode {
    double x;        ///< World x of the expanded pose [m].
    double y;        ///< World y of the expanded pose [m].
    double theta;    ///< Heading of the expanded pose [rad].
    double g;        ///< Cost-so-far from start to this node.
    double h;        ///< Heuristic remainder (f - g) at expansion time.
    int32_t parent;  ///< Index into `SearchTrace::nodes` of the parent expansion; negative for root.
};

/**
 * @brief A self-contained snapshot of a single planning query, built entirely from core types.
 *
 * This is the data contract between the planner and any renderer: the JSON writer below is one sink
 * (matplotlib); a future ROS layer can consume the same struct to publish MarkerArray / OccupancyGrid
 * / nav_msgs::Path without the planner depending on either renderer.
 */
struct SearchTrace {
    GridMetaData meta;                  ///< Grid dimensions / resolution / origin.
    std::vector<int8_t> occupancy;      ///< Row-major occupancy (-1 unknown, >=50 occupied); size rows*cols.
    std::vector<float>  costmap;        ///< Row-major costmap cells; same size (may be empty).
    SE2State start{};                   ///< Query start pose.
    SE2State goal{};                    ///< Query goal pose.
    std::vector<Vec2> footprint;        ///< Body-frame robot polygon vertices (for footprint rendering).
    std::vector<TraceNode> nodes;       ///< Expanded nodes in true pop order; `parent` forms the search tree.
    bool found = false;                 ///< Whether plan() returned a path.
    Path path{};                        ///< Result path; valid when found.
};

/**
 * @brief Collects `SearchObserver` expansion events into an ordered stream for the viz dump.
 *
 * A viz-local sink: pass a `shared_ptr<TraceRecorder>` into the observer-aware `Planner::plan`
 * overload, then read `events` afterward. Events arrive in true expansion (pop) order; the recorder
 * does no remapping itself — the dump converts the planner's opaque `id`/`parent_id` handles into
 * local `TraceNode::parent` indices. `onExpand` just appends, so it stays cheap inside the hot loop.
 */
class TraceRecorder : public SearchObserver {
public:
    void onExpand(const SearchExpansion & e) override { events.push_back(e); }

    std::vector<SearchExpansion> events;  ///< Expansion events in the order the planner popped them.
};

/**
 * @brief Build the renderer's pop-ordered node list from a planner expansion-event stream.
 *
 * Preserves the event order (true expansion order) and rewrites each event's opaque planner handle
 * into a local index: `TraceNode::parent` becomes the position of the parent's expansion within the
 * returned vector (or -1 for the root / any parent that was never itself reported). `h` is recovered
 * as `f - g`.
 *
 * @param events Expansion events as collected by a `TraceRecorder`, in pop order.
 * @return std::vector<TraceNode> Nodes in the same order, with `parent` as a self-referential index.
 */
std::vector<TraceNode> buildTraceNodes(const std::vector<SearchExpansion> & events);

/**
 * @brief Serialize a SearchTrace to JSON at `out_path`.
 *
 * @param trace Snapshot to serialize.
 * @param out_path Destination file path.
 */
void writeSearchTraceJson(const SearchTrace & trace, const std::string & out_path);
