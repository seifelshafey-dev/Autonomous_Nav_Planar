/**
 * @file search_viz_dump.cpp
 * @author Seif ElShafey
 * @brief Loads a scenario from an explicit YAML path (argv[1]), runs the hybrid A* planner, and
 *        dumps a SearchTrace to JSON for the matplotlib visualizer (tools/viz/plot_search.py).
 *        Non-ROS, inspection-only.
 * @version 0.2
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "search_trace.hpp"

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_loader.hpp"
#include "core/scenario/scenario_planner.hpp"
#include "core/planner/cost/costmap.hpp"
#include "core/planner/path.hpp"

int main(int argc, char ** argv){
    // Scenario source is a required, explicit YAML path (argv[1]). The tool references no corpus.
    if (argc < 2) {
        std::cerr << "usage: search_viz_dump <scenario.yaml> [out.json]\n";
        return 1;
    }
    const std::string scenario_path = argv[1];

    // Resolve scenario inputs from YAML, then instantiate the runnable planner + map snapshot via the
    // production factory chain. `built` aliases the snapshot's grid/EDT, so it must outlive the plan.
    const ScenarioSpec spec = loadScenario(scenario_path);
    BuiltScenario built = buildScenario(spec);

    // Default output is per-scenario: tools/viz/output/<stem>.json, where <stem> is the scenario file
    // name without extension (falling back to spec.name if the path carries no usable stem). This lets
    // dumps for different scenarios coexist instead of overwriting one fixed file; plot_search.py then
    // derives <stem>.png next to it. The output dir is gitignored project space; pass argv[2] to
    // override with an explicit path. Default is relative, so run from the package root.
    const std::filesystem::path out_dir = "tools/viz/output";
    std::string stem = std::filesystem::path(scenario_path).stem().string();
    if (stem.empty()) {
        stem = spec.name.empty() ? std::string("search_trace") : spec.name;
    }
    const std::string out_path =
        (argc > 2) ? argv[2] : (out_dir / (stem + ".json")).string();

    // Create the output directory if needed so the default path is writable on a fresh checkout.
    if (argc <= 2) {
        std::filesystem::create_directories(out_dir);
    }

    // Capture the true expansion (pop) order via the planner-agnostic SearchObserver seam: the
    // recorder receives one event per expanded node, in order, with no dynamic_cast or planner-type
    // knowledge. That yields the order nodes were actually expanded, not the node-generation order a
    // nodePool walk would reconstruct.
    auto recorder = std::make_shared<TraceRecorder>();
    auto path = built.planner->plan(spec.start, spec.goal, recorder).path;

    // Costmap is rendering data (heatmap overlay), not scenario input — recompute it for the trace.
    const CostMap costmap = buildCostMap(*built.snapshot, spec.cost_policy, built.robot.r_in, spec.a_star.costmap_weight);

    SearchTrace trace{
        .meta = spec.meta,
        .occupancy = spec.occupancy,
        .costmap = costmap.getData(),
        .start = spec.start,
        .goal = spec.goal,
        .footprint = built.robot.vertices,
        .nodes = buildTraceNodes(recorder->events),
        .found = path.has_value(),
        .path = path.value_or(Path{}),
    };
    writeSearchTraceJson(trace, out_path);

    std::cout << "wrote " << out_path << ": " << trace.nodes.size() << " nodes, "
              << (trace.found ? "path found" : "no path") << "\n";
    return 0;
}
