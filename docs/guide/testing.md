# Testing

Tests live in `src/path_planner/test` and `src/robot_bringup/test`. The commands below run from the repository root; in the container, wrap them as `docker compose -f docker/compose.yaml run --rm dev bash -c '...'`.

## Run tests

```bash
colcon test --executor sequential && colcon test-result --verbose                   # everything
colcon test --executor sequential --packages-select planner                         # one package
colcon test --packages-select planner --ctest-args -L fast                          # one label
ctest --test-dir build/planner -R <name> --output-on-failure                        # one test by name
```

`colcon build` has to run first. For a bare `ctest` line on the host, source `install/setup.bash` too: the tests that start ROS nodes need the built message package on the library path, and `colcon test` sets that up for you while `ctest` does not. Add `--output-on-failure` to a ctest line to see the output of a failing test.

## Labels

Every `planner` test carries exactly one of `fast` or `full`. The other labels describe what a test is.

- `fast`: the example tests, the scenario tests and the two ROS tests that run inside one process. No RapidCheck needed, so it works with `-DPLANNER_BUILD_PROPERTY_TESTS=OFF`. This is the lane to run before a pull request.
- `unit`: the example tests of single modules under `test/unit`. The property tests are not `unit`.
- `integration`: the scenario tests and the shipped-config test (`test/integration`). They run the whole planner on a map.
- `full`: the property tests, the trace-dump smoke test and the launch tests.
- `prop`: the property tests (RapidCheck).
- `tools`: the smoke test of the visualiser's trace output (`viz_trace_dump_smoke`).
- `integration_safety`: the scan guard launch test.
- `integration_closedloop` and `integration_pipeline`: the two launch tests in `robot_bringup`. They have no `fast` or `full` label.

Five tests start ROS nodes through `launch_testing`: `lifecycle_bringup_test` and `scan_safety_guard_test` in `planner`, and `test_mock_bringup`, `controller_tracking_test` and `planning_control_pipeline_test` in `robot_bringup`. All except the mock bring-up test run one at a time (`RUN_SERIAL`), because they share one ROS graph and starve each other of CPU. `navigator_test` and `planner_lifecycle_test` start nodes inside the test process; they are `fast`.

## Three kinds of test

- Example tests (GoogleTest): fixed inputs, expected outputs. Use these for behaviour you can state with a few numbers.
- Property tests (RapidCheck): random inputs checked against a rule. I use them wherever the code claims a bound, for example that cached footprint cells cover every cell the footprint can touch.
- Scenario tests: a map, a start and a goal, and what should come out. See the [scenario guide](scenarios.md).

## Add a test

The test tree mirrors the source tree. A test of `src/core/map/edt.cpp` goes in `test/unit/core/map/edt_test.cpp`. Register it in `src/path_planner/test/CMakeLists.txt`.

An example test:

```cmake
add_planner_test(edt_test unit/core/map/edt_test.cpp)
```

A property test. Several sources can share one executable:

```cmake
add_planner_property_test(stop_zone_property_test unit/core/control/stop_zone_property_test.cpp)
```

A scenario test is a YAML file in `test/integration/scenarios`; it needs no CMake line. The first walk in [getting started](getting-started.md) shows how.

When a property test fails, RapidCheck prints a line of the form `Use RC_PARAMS="reproduce=..."` to replay the failing case. Set that variable and run the test again:

```bash
RC_PARAMS="reproduce=<string from the failure>" ctest --test-dir build/planner -R props_core --output-on-failure
```

To try more random cases than the default:

```bash
RC_PARAMS="max_success=20000" ctest --test-dir build/planner -R props_core --output-on-failure
```

## CI

The workflow is `.github/workflows/ci.yml`. It runs in the container image and builds with `colcon`.

On a push, one job (`fast`) builds `path_planner_msgs` and `planner` in Release with `-DPLANNER_BUILD_PROPERTY_TESTS=OFF`, then runs `colcon test --packages-select planner --ctest-args -L fast`.

On a pull request, the `full` job runs twice, once per build type:

- Release: builds `path_planner_msgs`, `planner` and `robot_bringup` with the property tests on, and runs every test in them.
- Debug: builds `path_planner_msgs` and `planner` with the property tests off and `-DCMAKE_CXX_FLAGS_DEBUG="-O2 -g"`, and runs the tests labelled `unit`.

A newer run on the same branch cancels the older one.

## Two things that look like failures

- Death tests, the ones that expect an abort from `assert`, run only when `NDEBUG` is not defined. The test source skips them under `NDEBUG` (only `closed_set_test.cpp` has such tests now). The grid metadata checks are `CHECK`s and are tested with `EXPECT_THROW` in every build. A Release build defines `NDEBUG`, so these tests only count in a Debug build. The Debug leg of CI uses `-O2 -g` so they run at a useful speed. `CHECK` is not affected: it throws in every build type.
- Timing tests can fail on a machine whose clock jumps, as WSL's does. `scan_safety_guard_test` measures the jump (`wall_clock_step`). If the output stops matching during a window where the clock stepped by more than 0.02 s, it throws that window away and tries again, up to three windows. If the clock was steady, it fails at once and prints `wall_clock_step` along with the feed timing. If all three windows saw a step, it fails and prints the steps. A failure that prints a `wall_clock_step` near 0.5 s points at the clock or the test's own scan feed, not at the guard.
