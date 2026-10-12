# Getting started

This page takes you from a clone to a built workspace, a first plan and a first change. Paths are relative to the repository root unless a step says otherwise.

## Build

### In the container (recommended)

The container is the environment CI uses. It carries ROS 2 Humble, GCC 12 and every library.

```bash
docker build -f docker/Dockerfile --target ci -t planner-ci .
docker compose -f docker/compose.yaml run --rm dev \
  bash -c 'colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=ON'
```

Sources are mounted from your checkout. Build output lives in Docker volumes, so nothing lands in the tree. The build takes about five minutes.

### On the host

This needs Ubuntu 22.04 with ROS 2 Humble installed. The package list below is taken from the `ci` stage of `docker/Dockerfile`, so that file is the authority if the two ever differ.

```bash
# toolchain
sudo apt install build-essential cmake g++-12 gcc-12 git python3-colcon-common-extensions

# libraries
sudo apt install libabsl-dev libcurl4-openssl-dev libeigen3-dev libgtest-dev libopencv-dev \
  libyaml-cpp-dev nlohmann-json3-dev python3-matplotlib python3-numpy python3-opencv

# ROS packages
sudo apt install ros-humble-ament-lint-auto ros-humble-ament-lint-common ros-humble-bondcpp \
  ros-humble-controller-manager ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
  ros-humble-lifecycle-msgs ros-humble-nav2-lifecycle-manager ros-humble-robot-localization \
  ros-humble-robot-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-slam-toolbox ros-humble-tf2-geometry-msgs ros-humble-tf2-ros ros-humble-topic-tools \
  ros-humble-xacro
```

Then build:

```bash
source /opt/ros/humble/setup.bash
CC=gcc-12 CXX=g++-12 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=ON
source install/setup.bash
```

The ROS layer needs GCC 12 or newer. The simulator needs more packages: the `sim` stage of the Dockerfile adds Gazebo Fortress bindings (`ros-humble-ign-ros2-control`, `ros-humble-ros-gz-bridge`, `ros-humble-ros-gz-interfaces`, `ros-humble-ros-gz-sim`), `ros-humble-rviz2`, `libgl1-mesa-dri` and `mesa-utils`.

## Core only, no ROS

The planner core builds without ROS. Use a shell where ROS is not sourced:

```bash
cmake -S src/path_planner -B build-core -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPLANNER_BUILD_PROPERTY_TESTS=OFF
cmake --build build-core -j2
ctest --test-dir build-core -L fast
```

This builds with GCC 11. Turning the property tests on (`-DPLANNER_BUILD_PROPERTY_TESTS=ON`, the default) fetches RapidCheck from GitHub, so it needs the network.

If ROS is sourced, CMake finds `rclcpp`, `rclcpp_lifecycle` and `nav_msgs` and tries to build the nodes as well. That step needs GCC 12 or newer and the `path_planner_msgs` package, and it stops with an error otherwise. Open a clean shell to avoid it.

## First plan

Build with `-DBUILD_TOOLS=ON`, then run the visualiser pair from `src/path_planner`. The first command searches a scenario and writes a trace; the second draws it.

```bash
cd src/path_planner
../../build/planner/tools/search_viz_dump test/integration/scenarios/known_door_preferred.yaml
python3 tools/viz/plot_search.py tools/viz/output/known_door_preferred.json
```

In the container, the same two commands are in the root [README](../../README.md#plan-a-path). The picture lands next to the trace as `tools/viz/output/known_door_preferred.png`. If a display is available a window opens as well; close it to get the shell back.

How to read it:

- The grey image is the map. The green dot is the start and the purple-pink dot is the goal.
- The coloured dots are the states the search expanded, coloured by the order it expanded them.
- The red line is the path. The outlined boxes are the robot footprint along it.
- The shading near walls is the clearance cost, the penalty for passing close to an obstacle.

All options for the tools are on the [tools page](../../src/path_planner/tools/viz/README.md).

## First change: add a scenario to the test corpus

Every `.yaml` file directly inside `src/path_planner/test/integration/scenarios/` becomes one test case. The folder is read when the test runs, so adding a file needs no CMake reconfigure and no rebuild. Files in sub-folders are not read.

The case is part of the `scenario_plan_test` executable. In ctest it is one test named `scenario_plan_test`, labelled `integration` and `fast`. Inside it, GoogleTest names each case after the file name: `ScenarioCorpus/ScenarioPlanTest.PlansCorrectly/<file stem>`. For each file the test plans, then checks the path (collision free, reaches the goal, costs add up, a second plan gives the identical path). The `expectations` block adds the author's checks: `expect_found` says whether a path should exist, and the optional `cost_max`, `expansions_max` and `unknown_poses_max` are upper limits.

The walk below moves the goal of `straight_corridor.yaml` to a free spot higher up the map.

1. Copy the file:

   ```bash
   cd src/path_planner/test/integration/scenarios
   cp straight_corridor.yaml high_goal.yaml
   ```

2. In `high_goal.yaml`, change `name: straight_corridor` to `name: high_goal`.
3. Change the goal line from `goal:  [7.0, 4.0, 0.0]` to `goal:  [7.0, 6.0, 0.0]`. The grid is 8 m by 8 m, and the only obstacle is the block at x 3.2 to 4.8 m and y 2.0 to 4.0 m, so (7, 6) is open ground. `expect_found: true` stays correct.
4. Run the scenario test, from the repository root.

   In the container:

   ```bash
   docker compose -f docker/compose.yaml run --rm dev \
     bash -c 'build/planner/test/integration/scenario_plan_test --gtest_filter="*high_goal"'
   ```

   On the host:

   ```bash
   build/planner/test/integration/scenario_plan_test --gtest_filter='*high_goal'
   ```

   To run the whole corpus through ctest instead, use `ctest --test-dir build/planner -R scenario_plan_test --output-on-failure`.

The test needs the workspace built first, once. The format of every field is in the [scenario guide](scenarios.md).

## Where things live

- Planner core: `src/path_planner/include/core` and `src/path_planner/src/core`. No ROS. See the [planner README](../../src/path_planner/README.md).
- ROS nodes: `src/path_planner/include/ros` and `src/path_planner/src/ros`.
- Configuration: `src/path_planner/config`, one folder per area. See the [configuration reference](../architecture/config.md).
- Launch files: `src/path_planner/launch` and `src/robot_bringup/launch`.
- Tests: `src/path_planner/test`, laid out like the source. See the [testing guide](testing.md).
- Tools: `src/path_planner/tools`. See the [tools page](../../src/path_planner/tools/viz/README.md).
- Messages and the `PlanPath` action: `src/path_planner_msgs`. See its [README](../../src/path_planner_msgs/README.md).
- Example robot, worlds and SLAM bring-up: `src/robot_bringup`. See its [README](../../src/robot_bringup/README.md).

The per-file list is in the [module map](../architecture/module-map.md).

## Troubleshooting

- If a conda environment is active, the Python tools may pick up a Python without numpy or matplotlib. Deactivate it, or call `/usr/bin/python3`.
- On a machine with little memory, keep `-j2`. The property tests are heavy to compile.
