# Contributing

Issues and pull requests are welcome.

## Reporting a bug or proposing a change

Open an issue. For a bug, include:

- the scenario file, or the map and the start and goal poses;
- the outcome you got and the outcome you expected;
- the commit you built.

For a change, describe what you want and why before you write it.

## Before a pull request

1. Build the workspace (see [getting started](docs/guide/getting-started.md)).
2. Run the fast tests: `colcon test --packages-select planner --ctest-args -L fast`.
3. Add or update a test with any code change. The [testing guide](docs/guide/testing.md) says which kind to write.
4. Keep the pull request to one topic.

I review every pull request. CI must be green before I merge.

## Conventions

1. Validate every config field when it is loaded, with `CHECK` (`src/path_planner/include/core/diagnostics/check.hpp`). A bad value then fails at start-up with a message naming the field, not deep in a run. `CHECK` is compiled into every build type, Release included, and throws `std::invalid_argument` on failure. Example: `CHECK(p.lookahead_gain > 0.0, "controller.lookahead_gain must be positive")` in `src/path_planner/src/core/config/config_loader.cpp`.

2. Do not hand-set a number that depends on another setting. A value that follows from the robot, the grid or another setting is computed in code. A relation between two settings is a `CHECK` where both are known. A real choice stays in config as a ratio or a count of cells. The reason: change the robot and a hand-set number goes quietly wrong. Examples:
   - computed: the follower's lookahead is `lookahead_gain * (r_in / 2)`, worked out from the footprint (`src/path_planner/src/ros/controller/controller_node.cpp`);
   - checked: the straight primitive must be longer than `sqrt(2)` times the closed-set cell (`src/path_planner/src/core/planner/hybrid_astar/build_hybrid_astar.cpp`);
   - kept as a ratio: the escape step is `step_fraction` times the grid resolution, and `slack_cells` is a count of cells (`src/path_planner/src/core/config/config_loader.cpp`).

3. Comments on public symbols are API documentation. They say what the caller passes, what comes back (including the failure value), the constraints, and the units and frames. They carry no history and no notes on how the code works inside. Example: `HybridAstar::closedStride` in `src/path_planner/include/core/planner/hybrid_astar/search.hpp`.

4. C++20. Every header starts with `#pragma once`. A signature in the header and in the source match exactly, `const` included. Return by value, never a reference to a local.

5. Keep the core free of ROS. Nothing under `src/path_planner/include/core` or `src/path_planner/src/core` includes a ROS header, so the planner builds without ROS. The nodes in `include/ros` and `src/ros` call the core.

## Commit subjects

Use `type(scope): subject`, in lower case and the imperative:

- `fix(search): stop expanding a state already closed`
- `test(collision): cover the start gate on a map border`
- `docs(readme): add the known limitations`

## Licence

Contributions are licensed under Apache-2.0, the licence of the project. See [LICENSE](LICENSE).
