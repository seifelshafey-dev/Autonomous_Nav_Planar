# planner

A hybrid A* path planner for differential-drive robots, plus the ROS 2 nodes that run it. Given a start pose, a goal pose and an occupancy grid, the library returns a drivable path or the reason there is none.

The package is called `planner` in every `colcon` and `ros2` command, although its folder is `src/path_planner`.

## Layout

`include/core` and `src/core` hold the planner. They build without ROS. `include/ros` and `src/ros` are the ROS 2 nodes; they call the core and add no planning logic of their own.

## Using the library

This loads a scenario file, builds the planner for it and runs one plan. The scenario format is described in the [scenario guide](../../docs/guide/scenarios.md).

```cpp
#include <iostream>

#include "core/scenario/scenario_loader.hpp"
#include "core/scenario/scenario_planner.hpp"

int main(int argc, char ** argv) {
    if (argc < 2) {
        std::cerr << "usage: plan_example <scenario.yaml>\n";
        return 2;
    }
    const ScenarioSpec spec = loadScenario(argv[1]);
    BuiltScenario built = buildScenario(spec);
    const PlanResult result = built.planner->plan(spec.start, spec.goal);
    if (result.outcome == PlanOutcome::Found) {
        std::cout << "found, cost " << result.path->total_cost << ", "
                  << result.path->segments.size() << " segments\n";
        return 0;
    }
    std::cout << "no path, outcome " << static_cast<int>(result.outcome) << "\n";
    return 1;
}
```

`loadScenario` throws `std::invalid_argument` on a bad file. Keep `built` alive for as long as you use the planner: the planner reads the map snapshot that `built` owns.

`plan` returns a `PlanResult`. Its `outcome` is one of:

- `Found`: a path was found.
- `StartBlocked`: the start pose does not have the required clearance.
- `GoalBlocked`: the goal pose is in collision.
- `NoPath`: the search ran out of states without reaching the goal.
- `BudgetExhausted`: the expansion cap or the time limit was reached.
- `Stopped`: the `std::stop_token` passed to the four-argument `plan` was set.

`path` is set only for `Found`. A start that already satisfies the goal is a `Found` with a zero-length path.

If you already have an occupancy grid and a configuration in memory, call `buildHybridAstar` (`core/planner/hybrid_astar/build_hybrid_astar.hpp`) instead of going through a scenario file.

## ROS nodes

| Node | Class | What it does |
|---|---|---|
| Map provider | `ROSMapProvider` (`map_server`) | Turns each occupancy grid on `/map` into an immutable snapshot with a distance field. |
| Planner server | `PlannerNode` (`planner_node`) | Serves the `plan_path` action (`PlanPath`), one search per request, on the latest snapshot. |
| Navigator | `Navigator` | Owns the goal; decides when to plan, follow, replan or clear a blocked start. |
| Follower | `ControllerNode` (`controller_node`) | Pure pursuit along the planned path. |
| Scan guard | `ScanSafetyGuard` (`scan_safety_guard`) | Sits between the follower and the drive and zeroes the command when the lidar sees something inside the stopping zone. |

The map provider, the planner server and the navigator run in one executable, `planning_container`. The message and action definitions are in [`path_planner_msgs`](../path_planner_msgs/README.md). Topics and parameters are listed in the [ROS reference](../../docs/architecture/ros.md).

Two launch files start them:

| Launch file | What it starts | Arguments |
|---|---|---|
| `planning_system.launch.py` | SLAM Toolbox, the lifecycle manager, `planning_container`, the follower and the scan guard | `slam_mode` (`mapping` or `localization`, default `mapping`), `map_file` (pose graph to load in localization mode, default empty), `use_sim_time` (default `false`), `slam_transform_publish_period` (default `0.02` s; `0.0` leaves `map` to `odom` to another node) |
| `planning_container.launch.py` | `planning_container` only | none |

```bash
ros2 launch planner planning_system.launch.py slam_mode:=mapping
ros2 launch planner planning_container.launch.py
```

`planning_system.launch.py` expects a `/scan` topic and a robot that takes `/diff_drive_controller/cmd_vel_unstamped`. The [reference robot](../robot_bringup/README.md) provides both.

## Configuration

One folder per area under `config/`. The YAML comments are the reference for each key; see also the [configuration reference](../../docs/architecture/config.md).

| Folder | Controls |
|---|---|
| `controller` | Pure-pursuit gains for the follower. |
| `cost_policy` | Soft costmap penalties: clearance and unknown space. |
| `lifecycle_manager` | Which nodes are brought up, and the bond timeout. |
| `navigation` | Navigator timing, stall handling and the escape move. |
| `planner` | Search budget, goal tolerances and the goal shot (`a_star`), and the heuristic (`heuristic`). |
| `robot` | Footprint polygon and the differential-drive limits. |
| `safety` | Stopping-zone and debounce settings for the scan guard. |
| `slam` | SLAM Toolbox parameters. |

A value that follows from the robot or the grid is computed in code and is not written in these files. A relation between two settings is checked when the files are loaded.

## Building

```bash
colcon build --packages-select path_planner_msgs planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

| Option | Default | Effect |
|---|---|---|
| `BUILD_TESTING` | `OFF` in a plain CMake build; a `colcon` build turns it on | Build the unit, property and integration tests. |
| `BUILD_TOOLS` | `OFF` | Build `search_viz_dump`, `plan_benchmark` and `swept_cover_dump`; see the [tools page](tools/viz/README.md). |
| `PLANNER_BUILD_PROPERTY_TESTS` | `ON` | Build the RapidCheck property tests. Turn it off to build the tests without fetching RapidCheck. |

The ROS layer needs GCC 12 or newer, and configuring with an older GCC fails with a message saying so. The core alone builds with GCC 11.

Apache-2.0, see [LICENSE](../../LICENSE).
