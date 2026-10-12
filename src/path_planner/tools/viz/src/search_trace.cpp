/**
 * @file search_trace.cpp
 * @author Seif ElShafey
 * @brief JSON + binary side-car serializer for SearchTrace (one rendering sink; ROS would add another).
 * @version 0.2
 * @date 2026-06-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "search_trace.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

std::vector<TraceNode> buildTraceNodes(const std::vector<SearchExpansion> & events){
    // plot_search.py treats each node's "parent" as a direct index into the nodes array
    // (parent[parent[i]]). The planner's id/parent_id are opaque node-pool handles that are neither
    // contiguous nor sorted relative to the pop order, so we first map every reported handle to its
    // position in this pop-ordered stream, then resolve each parent_id through that map.
    std::unordered_map<int32_t, int32_t> id_to_pos;
    id_to_pos.reserve(events.size());
    for (std::size_t i = 0; i < events.size(); ++i) {
        id_to_pos.emplace(events[i].id, static_cast<int32_t>(i));
    }

    std::vector<TraceNode> nodes;
    nodes.reserve(events.size());
    for (const auto & e : events) {
        // Root parents are negative; any parent never itself reported collapses to -1 (no edge),
        // which plot_search.py's tree segments already skip (parent >= 0 guard).
        int32_t parent_pos = -1;
        if (e.parent_id >= 0) {
            auto it = id_to_pos.find(e.parent_id);
            if (it != id_to_pos.end()) { parent_pos = it->second; }
        }
        nodes.push_back(TraceNode{
            .x = e.state.x, .y = e.state.y, .theta = e.state.theta,
            .g = e.g, .h = e.f - e.g, .parent = parent_pos,
        });
    }
    return nodes;
}

namespace {

// Append a contiguous little-endian array to the binary stream and record its descriptor. The host is
// assumed little-endian (x86/ARM); the JSON `binary` block documents this so the reader matches.
nlohmann::json appendArray(std::ofstream & bin, std::uint64_t & offset, const char * name,
                           const char * dtype, std::uint64_t count,
                           const void * data, std::uint64_t elem_bytes) {
    const std::uint64_t nbytes = count * elem_bytes;
    bin.write(static_cast<const char *>(data), static_cast<std::streamsize>(nbytes));
    nlohmann::json desc = {
        {"name", name}, {"dtype", dtype}, {"shape", {count}}, {"offset", offset},
    };
    offset += nbytes;
    return desc;
}

}  // namespace

void writeSearchTraceJson(const SearchTrace & t, const std::string & out_path){
    using json = nlohmann::json;

    // The bulk arrays (the 1.68M-cell occupancy/costmap and the per-node columns) go to a raw
    // little-endian side-car next to the JSON: <stem>.bin. The JSON keeps only small metadata plus a
    // `binary` descriptor mapping each array name -> {dtype, shape, byte offset}. This keeps the
    // renderer-agnostic SearchTrace contract intact (the struct is unchanged) while cutting the JSON
    // from tens of MB of text to a few KB. The descriptor's `file` is the side-car's basename so a
    // moved bundle stays portable (the reader resolves it against the JSON's own directory).
    const std::filesystem::path json_path(out_path);
    const std::filesystem::path bin_path =
        std::filesystem::path(json_path).replace_extension(".bin");

    std::ofstream bin(bin_path, std::ios::binary);
    std::uint64_t offset = 0;
    json arrays = json::array();

    // Grid arrays (row-major, rows*cols). Occupancy is int8 in [-1,100]; costmap is float32.
    arrays.push_back(appendArray(bin, offset, "occupancy", "int8", t.occupancy.size(),
                                 t.occupancy.data(), sizeof(int8_t)));
    arrays.push_back(appendArray(bin, offset, "costmap", "float32", t.costmap.size(),
                                 t.costmap.data(), sizeof(float)));

    // Node columns (SoA), each length n_nodes. float32 is ample for viz coordinates; `parent` is int32
    // (an index into the node columns, or -1 for the root). The result path keeps double precision in
    // the JSON below — only the dense per-expansion stream is narrowed.
    const std::uint64_t n = t.nodes.size();
    std::vector<float> col(n);
    std::vector<int32_t> parent(n);
    auto emit_float_col = [&](const char * name, double TraceNode::* member) {
        for (std::uint64_t i = 0; i < n; ++i) col[i] = static_cast<float>(t.nodes[i].*member);
        arrays.push_back(appendArray(bin, offset, name, "float32", n, col.data(), sizeof(float)));
    };
    emit_float_col("x", &TraceNode::x);
    emit_float_col("y", &TraceNode::y);
    emit_float_col("theta", &TraceNode::theta);
    emit_float_col("g", &TraceNode::g);
    emit_float_col("h", &TraceNode::h);
    for (std::uint64_t i = 0; i < n; ++i) parent[i] = t.nodes[i].parent;
    arrays.push_back(appendArray(bin, offset, "parent", "int32", n, parent.data(), sizeof(int32_t)));
    bin.close();

    json j;
    j["meta"] = {
        {"cols", t.meta.cols},
        {"rows", t.meta.rows},
        {"resolution", t.meta.resolution},
        {"origin", {t.meta.origin.x(), t.meta.origin.y()}},
    };

    // Descriptor for the bulk arrays. `n_nodes` is published so the reader can size columns without
    // re-deriving from the shape, and `endianness` documents the raw layout.
    j["binary"] = {
        {"file", bin_path.filename().string()},
        {"endianness", "little"},
        {"n_nodes", n},
        {"arrays", std::move(arrays)},
    };

    j["start"] = {t.start.x, t.start.y, t.start.theta};
    j["goal"]  = {t.goal.x, t.goal.y, t.goal.theta};

    json fp = json::array();
    for (const auto & v : t.footprint) fp.push_back({v.x(), v.y()});
    j["footprint"] = std::move(fp);

    j["found"] = t.found;
    if (t.found) {
        json pts = json::array();
        for (const auto & p : t.path.node_points) pts.push_back({p.x, p.y, p.theta});
        json samples = json::array();
        for (const auto & seg : t.path.segments)
            for (const auto & s : seg.samples_poses) samples.push_back({s.x, s.y, s.theta});
        j["path"] = {
            {"total_cost", t.path.total_cost},
            {"node_points", std::move(pts)},
            {"samples", std::move(samples)},
        };
    }

    // Compact dump (no indent) — the JSON is now metadata-only, so readability is not the concern.
    std::ofstream out(json_path);
    out << j.dump();
}
