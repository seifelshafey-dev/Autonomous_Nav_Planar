/**
 * @file search_viz_smoke.cpp
 * @author Seif ElShafey
 * @brief CI smoke test for the trace-dump pipeline: load the seed scenario, build the planner, plan,
 *        serialize a SearchTrace to a JSON metadata file + a binary side-car, then reparse and assert
 *        the JSON is well-formed, carries the required metadata keys, and that the `binary` descriptor
 *        + side-car .bin file are consistent with the advertised grid geometry. Exits non-zero on any
 *        failure (drives ctest pass/fail).
 * @version 0.2
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "search_trace.hpp"

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_loader.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/path.hpp"

namespace {

// Resolve the trace output path: argv[1] if given, else a temp file. The dump pipeline only needs a
// writable destination; CI passes a build-dir path so the artifact is sandboxed and cleanable.
std::string outputPath(int argc, char ** argv) {
    return (argc > 1) ? argv[1] : std::string("search_viz_smoke_trace.json");
}

}  // namespace

int main(int argc, char ** argv) {
    const std::string scenario_path =
        std::string(TEST_SCENARIO_CORPUS_DIR) + "/straight_corridor.yaml";
    const std::string out_path = outputPath(argc, argv);

    // --- Run the exact dump pipeline the tool runs. ---
    const ScenarioSpec spec = loadScenario(scenario_path);
    BuiltScenario built = buildScenario(spec);
    auto path = built.planner->plan(spec.start, spec.goal).path;
    const CostMap costmap = buildCostMap(*built.snapshot, spec.cost_policy, built.robot.r_in, spec.a_star.costmap_weight);

    SearchTrace trace{
        .meta = spec.meta,
        .occupancy = spec.occupancy,
        .costmap = costmap.getData(),
        .start = spec.start,
        .goal = spec.goal,
        .footprint = built.robot.vertices,
        .nodes = {},  // node pool is HybridAstar-internal; not required for the well-formedness check
        .found = path.has_value(),
        .path = path.value_or(Path{}),
    };
    writeSearchTraceJson(trace, out_path);

    // --- Reparse and validate. ---
    std::ifstream in(out_path);
    if (!in) {
        std::fprintf(stderr, "smoke: could not reopen dumped trace at %s\n", out_path.c_str());
        return 1;
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception & e) {
        std::fprintf(stderr, "smoke: dumped trace is not well-formed JSON: %s\n", e.what());
        return 1;
    }

    // Required metadata keys. The bulk arrays (occupancy/costmap/node columns) now live in the binary
    // side-car described by the `binary` block, not as JSON arrays. `path` is present only when found.
    const char * required[] = {"meta", "binary", "start", "goal", "footprint", "found"};
    for (const char * key : required) {
        if (!j.contains(key)) {
            std::fprintf(stderr, "smoke: dumped trace missing required key '%s'\n", key);
            return 1;
        }
    }

    const auto & meta = j.at("meta");
    if (!meta.contains("cols") || !meta.contains("rows") || !meta.contains("resolution")) {
        std::fprintf(stderr, "smoke: meta block missing cols/rows/resolution\n");
        return 1;
    }
    const std::size_t expected_cells =
        meta.at("cols").get<std::size_t>() * meta.at("rows").get<std::size_t>();

    // The `binary` descriptor must name a side-car, document little-endian layout, carry n_nodes, and
    // list each array's {name, dtype, shape, offset}. Validate the occupancy array's shape against the
    // grid geometry and that the .bin file exists and is at least as large as the highest array tail.
    const auto & bin = j.at("binary");
    if (!bin.contains("file") || !bin.contains("arrays") || !bin.contains("n_nodes") ||
        bin.value("endianness", std::string()) != "little") {
        std::fprintf(stderr, "smoke: binary block missing file/arrays/n_nodes or not little-endian\n");
        return 1;
    }

    auto dtype_size = [](const std::string & dt) -> std::uint64_t {
        if (dt == "int8") return 1;
        if (dt == "int32" || dt == "float32") return 4;
        return 0;  // unknown dtype
    };

    bool saw_occupancy = false;
    std::uint64_t max_tail = 0;
    for (const auto & arr : bin.at("arrays")) {
        const std::string name = arr.at("name").get<std::string>();
        const std::string dtype = arr.at("dtype").get<std::string>();
        const std::uint64_t esize = dtype_size(dtype);
        if (esize == 0) {
            std::fprintf(stderr, "smoke: array '%s' has unknown dtype '%s'\n", name.c_str(), dtype.c_str());
            return 1;
        }
        const std::uint64_t count = arr.at("shape").at(0).get<std::uint64_t>();
        const std::uint64_t off = arr.at("offset").get<std::uint64_t>();
        max_tail = std::max(max_tail, off + count * esize);
        if (name == "occupancy") {
            saw_occupancy = true;
            if (dtype != "int8" || count != expected_cells) {
                std::fprintf(stderr, "smoke: occupancy array dtype/shape mismatch (%s, %llu vs %zu)\n",
                             dtype.c_str(), static_cast<unsigned long long>(count), expected_cells);
                return 1;
            }
        }
    }
    if (!saw_occupancy) {
        std::fprintf(stderr, "smoke: binary descriptor has no 'occupancy' array\n");
        return 1;
    }

    // Resolve the side-car relative to the JSON's directory (matches the renderer) and check its size.
    const std::filesystem::path bin_path =
        std::filesystem::path(out_path).parent_path() / bin.at("file").get<std::string>();
    std::error_code ec;
    const std::uintmax_t bin_size = std::filesystem::file_size(bin_path, ec);
    if (ec) {
        std::fprintf(stderr, "smoke: side-car .bin missing at %s\n", bin_path.string().c_str());
        return 1;
    }
    if (bin_size < max_tail) {
        std::fprintf(stderr, "smoke: side-car .bin too small: %llu < %llu (max array tail)\n",
                     static_cast<unsigned long long>(bin_size),
                     static_cast<unsigned long long>(max_tail));
        return 1;
    }

    if (!j.at("start").is_array() || j.at("start").size() != 3 ||
        !j.at("goal").is_array() || j.at("goal").size() != 3) {
        std::fprintf(stderr, "smoke: start/goal must each be a 3-element [x,y,theta] array\n");
        return 1;
    }

    // When a plan was found, the serialized trace must carry the `path` block.
    if (j.at("found").get<bool>() && !j.contains("path")) {
        std::fprintf(stderr, "smoke: found=true but no 'path' block in trace\n");
        return 1;
    }

    std::printf("smoke: OK — trace at %s (%zu occupancy cells, %llu nodes, found=%s, side-car %llu B)\n",
                out_path.c_str(), expected_cells,
                static_cast<unsigned long long>(bin.at("n_nodes").get<std::uint64_t>()),
                j.at("found").get<bool>() ? "true" : "false",
                static_cast<unsigned long long>(bin_size));
    return 0;
}
