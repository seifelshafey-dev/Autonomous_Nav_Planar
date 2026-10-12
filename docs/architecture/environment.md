# Build and test environment

The reference environment is the container image built from `docker/Dockerfile`; CI uses it. A host build works too if it meets the requirements below. Step-by-step instructions are in [getting started](../guide/getting-started.md), and the guide to running and writing tests is [testing](../guide/testing.md).

## Image

`docker/Dockerfile` has three stages. The image holds dependencies only. Sources are mounted at `/ws/src`, and the container runs as user `dev` (uid 1000), who owns `/ws`.

- `base`: `ros:humble-ros-base-jammy`, pinned by digest `sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19` (resolved 2026-10-05).
- `ci`: GCC 12 (`CC=gcc-12 CXX=g++-12`, `MAKEFLAGS=-j2`) and every apt dependency of the three packages. The install is three layers, from least to most likely to change: toolchain and build tools, system libraries, ROS packages. The entry point sources `/opt/ros/humble/setup.bash`. Full `libopencv-dev` is installed: the sources use only core, imgproc and imgcodecs, but Debian's OpenCV CMake config fails when any module library is missing, so per-module packages would need a source change and a build test first.
- `sim`: `ci` plus Gazebo Fortress (`ros-humble-ros-gz-*`, `ros-humble-ign-ros2-control`), `ros-humble-rviz2`, `libgl1-mesa-dri` and `mesa-utils`, with `LIBGL_ALWAYS_SOFTWARE=1` and `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`. The Gazebo packages are `exec_depend`s of `robot_bringup` but are installed in `sim` only, so `ci` does not satisfy every `package.xml` entry.

The simulator renders in software. `docker/compose.yaml` passes no GPU device and sets no `DISPLAY`.

## Version locking

ROS packages come from the dated snapshot `http://snapshots.ros.org/humble/2026-09-14/ubuntu`. Its signing key has to be fetched by full fingerprint (`4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`); the key in the base image does not verify the snapshot. Ubuntu packages are locked by the base digest.

`docker/apt-lock.txt` lists every package installed in the `sim` image with its exact version. `docker/check-lock.sh <image>` exits 1 and prints the offending lines when the image holds a package or version that is not in the lock. The lock comes from `sim`, a superset of `ci`, so lock entries missing from `ci` are expected.

To update, change the snapshot date (and the digest if wanted) in the Dockerfile, rebuild `sim`, and regenerate the lock with the command in the header of `docker/Dockerfile`.

## Container commands

Run from the repository root.

```
docker build -f docker/Dockerfile --target ci -t planner-ci .
docker build -f docker/Dockerfile --target sim -t planner-sim .
docker compose -f docker/compose.yaml run --rm dev bash -c 'colcon build --executor sequential'
docker compose -f docker/compose.yaml run --rm dev bash -c 'colcon test --packages-select path_planner_msgs planner --executor sequential && colcon test-result --verbose'
docker compose -f docker/compose.yaml run --rm sim bash -c 'source install/setup.bash && ros2 launch robot_bringup sim_bringup.launch.py'
```

`compose.yaml` defines two services: `dev` (image `planner-ci`, target `ci`) and `sim` (image `planner-sim`, target `sim`). Both mount `src/` from the host at `/ws/src` as it is, so edits to the source tree show up inside a running container at once. Build outputs live in the named volumes `ws-build`, `ws-install` and `ws-log` (`/ws/build`, `/ws/install`, `/ws/log`), never in the host tree. Build the workspace before launching the simulator.

## Building on the host

Ubuntu 22.04 with ROS 2 Humble. The package list is the `ci` stage of the Dockerfile; [getting started](../guide/getting-started.md) has it as `apt` lines.

The ROS layer needs GCC 12 or newer, because it uses `std::atomic<std::shared_ptr>`. `src/path_planner/CMakeLists.txt` stops at configure time with a message when the ROS layer is configured with GNU older than 12. Select the compiler explicitly:

```
source /opt/ros/humble/setup.bash
CC=gcc-12 CXX=g++-12 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The ROS-free core builds alone with plain CMake and the default compiler (GCC 11 is enough), in a shell where ROS is not sourced:

```
cmake -S src/path_planner -B build-core -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPLANNER_BUILD_PROPERTY_TESTS=OFF
cmake --build build-core -j2
ctest --test-dir build-core -L fast
```

CMake options of `src/path_planner/CMakeLists.txt` and `src/path_planner/test/CMakeLists.txt`:

| Option | Default | Effect |
|---|---|---|
| `BUILD_TESTING` | OFF | Builds the tests. |
| `BUILD_TOOLS` | OFF | Builds the tools in `tools/` (see [module-map.md](module-map.md)). |
| `PLANNER_BUILD_PROPERTY_TESTS` | ON | Builds the RapidCheck property tests. RapidCheck is fetched with `FetchContent`, so this needs git and network access. |

## Running tests

```
colcon test --executor sequential && colcon test-result --verbose
colcon test --executor sequential --packages-select planner
colcon test --packages-select planner --ctest-args -L fast
ctest --test-dir build/planner -R <name> --output-on-failure
```

Run `colcon build` first. A bare `ctest` needs `install/setup.bash` sourced: the tests that start ROS nodes need the built message package on the library path, and `colcon test` sets that up while `ctest` does not.

Labels (set in `src/path_planner/test/CMakeLists.txt`, `test/integration/CMakeLists.txt`, `test/tools/viz/CMakeLists.txt`, the root `CMakeLists.txt` and `src/robot_bringup/CMakeLists.txt`):

| Label | Tests |
|---|---|
| `fast` | Example unit tests, the two integration tests, `planner_lifecycle_test`, `navigator_test`. Runs with `-DPLANNER_BUILD_PROPERTY_TESTS=OFF`. |
| `full` | Property tests, `viz_trace_dump_smoke`, `lifecycle_bringup_test`, `scan_safety_guard_test`. |
| `unit` | Example tests of single modules (with `fast`), plus the two ROS tests that run in one process. |
| `prop` | RapidCheck property tests (with `full`). |
| `integration` | `scenario_plan_test` and `shipped_config_test` (with `fast`). |
| `tools` | `viz_trace_dump_smoke` (with `full`). |
| `integration_safety` | `scan_safety_guard_test`. |
| `integration_closedloop` | `controller_tracking_test` in `robot_bringup`. |
| `integration_pipeline` | `planning_control_pipeline_test` in `robot_bringup`. |

Each `planner` test carries exactly one of `fast` and `full`. The two `robot_bringup` tests with their own label carry neither, and `test_mock_bringup` has no label.

## Container lanes and CI

Two workflows under `.github/workflows/`:

- `image.yml` runs when `docker/` or the workflow itself changes, and on manual dispatch. It builds target `ci` and pushes `ghcr.io/<repo>/planner-ci` with two tags: the hash of `docker/Dockerfile` and `latest`. Then it runs `docker/check-lock.sh` on the pushed image when `docker/apt-lock.txt` exists.
- `ci.yml` pulls the image tagged with the current Dockerfile hash. It retries up to 30 times at 60 s intervals, so a push that also changes `docker/` waits for `image.yml`; if the image never appears the job fails and names `image.yml`. The job mounts `src/` into the container with `docker run -v`. Runs of the same ref cancel each other.

Lanes of `ci.yml`:

| Lane | Runs on | Builds | Tests |
|---|---|---|---|
| `fast` | push to `main`; draft pull request | `path_planner_msgs`, `planner`; Release; `-DPLANNER_BUILD_PROPERTY_TESTS=OFF` | `ctest -L fast` |
| `full`, Release | pull request marked ready, and each later push to it | `path_planner_msgs`, `planner`, `robot_bringup`; Release; property tests on | every test |
| `full`, Debug | pull request marked ready, and each later push to it | `path_planner_msgs`, `planner`; property tests off; `-DCMAKE_CXX_FLAGS_DEBUG="-O2 -g"` | `ctest -L unit` |

A change that touches only `docs/` or Markdown files starts no job.

The Debug leg is optimised with assertions on (`NDEBUG` undefined), so death tests run, which they do not in Release. A plain unoptimised Debug build works locally and is slower.

Tests that start ROS nodes must not run beside a simulator, or beside each other, on one machine: they share one ROS graph, and topic and action names are global to it. `navigator_test` publishes `/clock`. `navigator_test` and four of the five launch tests (all but `test_mock_bringup`) are `RUN_SERIAL`, so ctest does not run them together with other tests.
