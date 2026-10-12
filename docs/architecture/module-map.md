# Module map

Every tracked file under `src/path_planner/{include,src,tools,launch,config}` is listed once. Behaviour is in [core.md](core.md) and [ros.md](ros.md). Paths are relative to `src/path_planner` unless they start with `src/`.

The package is called `planner` in `colcon` and `ros2` commands. `include/core` and `src/core` build without ROS; `include/ros` and `src/ros` are the nodes and call the core.

## Core headers: `include/core/`

```
types/
  state.hpp                      SE2State { x, y, θ }
  control.hpp                    Control { v, ω, dt }
  geometry.hpp                   Vec2, Mat2, Rot2, HalfPlane, AABB
  grid_coords.hpp                GridCoords { i, j } cell index
  state_space_interface.hpp      Abstract State_Space<State>: propagate, interpolate
  math/wrap_angle.hpp            wrapToPi, the angle wrap to [-π, π)
robot/
  robot_params.hpp               RobotParams { vertices, half_planes, r_in, r_out }; polygon builder and validator
  diff_drive_parameters.hpp      DiffDriveParams { rmin, vmax, atomic_time, n_headings }
motion/
  kinematic_model.hpp            KinematicModel: state space, geometry, grid cell size and primitive library together
  motion_context.hpp             MotionContext: sampleTrajectory, computeExactCover, visitExactCover (early-stopping cell visitor)
  world_robot_geometry.hpp       Robot polygon in the world frame
  distance_functions.hpp         Weighted-Euclidean SE(2) distance and the control cost functor
  kinematics/diff_drive.hpp      DiffDriveStateSpace, the differential-drive State_Space
map/
  grid.hpp                       OccupancyGrid and GridMetaData (resolution, origin)
  edt.hpp                        EDT: Euclidean distance transform of the grid
  rasterize.hpp                  Filled-polygon rasterization; visitPolygonCells (visitor form with early stop)
  map_provider.hpp               MapSnapshot (grid and EDT) and the abstract map provider
  map_signature.hpp              MapSignature, hashOccupancy, makeSignature: change detection by hash
collision_checker/
  collision_checker.hpp          CollisionChecker: checkCell, checkCells, checkState, checkTrajectory, startClear, gridMeta
config/
  config_loader.hpp              YAML loaders and the typed parameter structs
diagnostics/
  check.hpp                      CHECK, the always-on input validation macro
planner/
  planner_interface.hpp          Abstract planner; PlanOutcome and PlanResult; plan overload that takes an observer
  search_observer.hpp            SearchObserver and SearchExpansion: per-expansion progress events
  path.hpp                       Path: control-applied segments and the poses between them
  primitives/
    control_primitives.hpp       Primitive { control, endpoint_offset, cost, swept_cells, ... }
    build_diff_prim_lib.hpp      DiffDrivePrimLib: control set and per-heading-bin swept cells
  cost/
    costmap.hpp                  CostMap: cost of travel from clearance, occupancy and unknown space
    cost_policy.hpp              CostPolicy: clearance and unknown-space settings
  hybrid_astar/
    config.hpp                   AStarConfig
    heuristics.hpp               HeuristicConfig and the cost-to-goal estimators
    obstacle_heuristic.hpp       ObstacleHeuristic: obstacle-aware Dijkstra field
    node.hpp                     Node of the implicit search graph
    open_set.hpp                 Open set: priority queue with a total order on f, h and node index
    closed_set.hpp               Closed set and its (grid_i, grid_j, heading_bin) key
    search.hpp                   The hybrid A* expansion loop
    build_hybrid_astar.hpp       Factory that assembles a planner for one map snapshot
  escape/
    escape_search.hpp            findEscape: straight move out of a blocked start to a pose that passes the start gate
  validation/
    path_validity.hpp            Collision re-check of a planned path
navigation/
  forward_clock.hpp              ForwardClock: turns a clock that can step backward into event times that never decrease
  navigation_checks.hpp          checkRemainingPath (cursor and remaining-path scan), atRest (two-pose rest test)
  navigation_mode.hpp            NavigationMode: events in, actions out; NavState, BlockedReason, NavigationConfig
control/
  pure_pursuit.hpp               PurePursuit follower, PurePursuitParams, TrajectoryPoint, MotionMode
  stop_zone.hpp                  motionBlocked and StopZoneParams: scan returns inside the area swept while stopping
scenario/
  scenario.hpp                   ScenarioSpec (resolved inputs) and the Expectations of a scenario
  scenario_loader.hpp            loadScenario (YAML and optional grayscale image), discoverScenarioFiles
  scenario_planner.hpp           buildScenario: ScenarioSpec to a runnable planner
```

## Core sources: `src/core/`

One `.cpp` per header that needs one, in the same sub-directory as the header.

```
collision_checker/collision_checker.cpp
config/config_loader.cpp
control/pure_pursuit.cpp
control/stop_zone.cpp
map/edt.cpp
map/map_signature.cpp
map/rasterize.cpp
motion/kinematic_model.cpp
motion/motion_context.cpp
motion/world_robot_geometry.cpp
navigation/navigation_mode.cpp
planner/cost/costmap.cpp
planner/escape/escape_search.cpp
planner/hybrid_astar/build_hybrid_astar.cpp
planner/hybrid_astar/obstacle_heuristic.cpp
planner/hybrid_astar/search.cpp
planner/primitives/build_diff_prim_lib.cpp
planner/validation/path_validity.cpp
robot/robot_params.cpp
scenario/scenario_loader.cpp
scenario/scenario_planner.cpp
```

## ROS layer: `include/ros/` and `src/ros/`

```
include/ros/map_provider/map_provider_server.hpp      ROSMapProvider: lifecycle node that turns the SLAM map into a MapSnapshot
include/ros/planner_server/planner_node.hpp           PlannerNode: lifecycle node, PlanPath action server
include/ros/planner_server/plan_conversions.hpp       pathToTrajectory: Path to PlanTrajectory
include/ros/planner_server/search_marker_observer.hpp SearchObserver that buffers expansions for the search_markers topic
include/ros/navigator/navigator.hpp                   Navigator: owns the goal, plans, replans, escapes
include/ros/controller/controller_node.hpp            ControllerNode: plan to velocity commands with PurePursuit
include/ros/safety/scan_safety_guard.hpp              ScanSafetyGuard: scan-based stop filter on the command stream
src/ros/map_provider/map_provider_server.cpp
src/ros/planner_server/planner_node.cpp
src/ros/planner_server/plan_conversions.cpp
src/ros/navigator/navigator.cpp
src/ros/controller/controller_node.cpp
src/ros/safety/scan_safety_guard.cpp
src/ros/app/planning_container_main.cpp               Executable: map provider, planner node and navigator in one process
src/ros/app/controller_main.cpp                       Executable: ControllerNode
src/ros/app/scan_safety_guard_main.cpp                Executable: ScanSafetyGuard
```

## Launch files: `launch/`

```
launch/planning_container.launch.py   Starts the planning_container executable with the config paths
launch/planning_system.launch.py      Starts SLAM, the lifecycle manager, planning_container, the controller and the scan guard
```

## Config files: `config/`

Keys, bounds and defaults are in [config.md](config.md).

```
config/robot/robot_params.yaml                      Footprint polygon and differential-drive limits
config/planner/planner.yaml                         Search settings and heuristic
config/cost_policy/cost_policy.yaml                 Clearance and unknown-space cost
config/controller/controller.yaml                   Pure-pursuit follower
config/safety/scan_safety_guard.yaml                Scan guard stop model
config/navigation/navigator.yaml                    Navigator timers, retries and escape move
config/lifecycle_manager/lifecycle_manager.yaml     nav2_lifecycle_manager settings
config/slam/mapper_params.yaml                      slam_toolbox settings for the reference robot
```

## Tools: `tools/`

Built with `-DBUILD_TOOLS=ON`. They need no ROS. Usage is in `tools/viz/README.md`.

```
tools/CMakeLists.txt                  Targets: viz_search_trace, search_viz_dump, plan_benchmark and the five figure dumps below
tools/plan_benchmark.cpp              Times the planner on a scenario; prints one key=value line per configuration
tools/viz/README.md                   How to run the tools
tools/viz/scenario_studio.py          new, pick, run, preview: build a scenario from a map image and run it
tools/viz/_scenario/__init__.py       Package marker
tools/viz/_scenario/decode.py         Decodes a grayscale map image to occupied, free and unknown cells
tools/viz/_scenario/pick.py           Start and goal picker, pixel to world pose
tools/viz/_scenario/mapmeta.py        Reads resolution and origin from the nav2 map YAML beside the image
tools/viz/_scenario/scenario_yaml.py  Writes a scenario YAML and rewrites the start and goal of an existing one
tools/viz/src/search_trace.hpp        SearchTrace: renderer-neutral record of one search
tools/viz/src/search_trace.cpp        Writes a SearchTrace as JSON with a binary side-car
tools/viz/src/search_viz_dump.cpp     Runs a scenario and writes the search trace; read by plot_search.py (figure: docs/media/search.png, shown in README.md)
tools/viz/plot_search.py              Draws a search trace: map, expanded states, path, footprint; optional animation
tools/viz/plot_plan_vs_executed.py    Draws a planned path against the path the robot drove, on a trace's map
tools/viz/src/swept_cover_dump.cpp    Writes the cached and exact swept cells of one motion as JSON on stdout; figure docs/media/swept_cover.png in docs/design/swept-cover.md
tools/viz/plot_swept_cover.py         Draws that figure
tools/viz/src/primitives_dump.cpp     Writes the robot's motions and reached states as JSON on stdout; figure docs/media/primitives.png in docs/design/primitives.md
tools/viz/plot_primitives.py          Draws that figure
tools/viz/src/cost_field_dump.cpp     Writes the cost of travel and the obstacle-aware cost to goal over a scenario map as JSON on stdout; figure docs/media/cost_field.png in docs/design/cost-and-heuristic.md
tools/viz/plot_cost_field.py          Draws that figure
tools/viz/src/escape_move_dump.cpp    Writes the escape move for a robot standing in a wall as JSON on stdout; the lower panel of docs/media/navigation.png in docs/design/navigation.md
tools/viz/plot_navigation.py          Draws that figure
tools/viz/src/stop_zone_dump.cpp      Writes the region where one scan return blocks a command, for three commands, as JSON on stdout; figure docs/media/stop_zone.png in docs/design/safety-guard.md
tools/viz/plot_stop_zone.py           Draws that figure
```

## Tests: `test/`

Registered in `test/CMakeLists.txt`; labels are in [environment.md](environment.md). The unit tree mirrors `src/core`. Counts are files.

```
test/CMakeLists.txt                                   Test targets and labels
test/support/swept_oracle.hpp                         Brute-force swept-cell oracle shared by the cover tests
test/unit/core/collision_checker/                     2 files
test/unit/core/config/                                1 file
test/unit/core/control/                               4 files: pure pursuit and stop zone, examples and properties
test/unit/core/map/                                   8 files
test/unit/core/motion/                                7 files
test/unit/core/motion/kinematics/                     2 files
test/unit/core/navigation/                            4 files: forward_clock_test.cpp, forward_clock_property_test.cpp, navigation_checks_test.cpp, navigation_mode_test.cpp
test/unit/core/planner/cost/                          2 files
test/unit/core/planner/escape/                        2 files
test/unit/core/planner/hybrid_astar/                  9 files
test/unit/core/planner/primitives/                    5 files: includes the swept-cover property tests
test/unit/core/planner/validation/                    2 files
test/unit/core/robot/                                 1 file
test/unit/core/scenario/                              1 file
test/unit/core/types/                                 1 file
test/unit/core/types/math/                            2 files
test/unit/ros/navigator_test.cpp                      Navigator in one process against a scripted plan server
test/unit/ros/planner_lifecycle_test.cpp              PlannerNode and ROSMapProvider configure and cleanup
test/integration/CMakeLists.txt                       Registers the two integration tests with the labels integration and fast
test/integration/scenario_plan_test.cpp               Runs every scenario file in test/integration/scenarios
test/integration/shipped_config_test.cpp              Loads the shipped config files and plans around a wall
test/integration/scenarios/                           11 scenario YAML files (one in the e2e_demo folder)
test/integration/ros/lifecycle_bringup_test.py        Launch test: the two lifecycle nodes reach active
test/integration/ros/scan_safety_guard_test.py        Launch test: scan guard
test/tools/viz/                                       Trace-dump smoke test
```

## Other packages

`src/path_planner_msgs`: the `PlanPath` action and the `PlanTrajectory` and `PlanProgress` messages.

`src/robot_bringup`: the reference robot (URDF, Gazebo worlds, launch files, EKF and controller config); see [reference-robot.md](reference-robot.md).
