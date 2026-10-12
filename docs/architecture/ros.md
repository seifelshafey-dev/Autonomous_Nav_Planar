# ROS layer of the planner package

The `planner` package wraps the ROS-free core in five nodes. The core is described in [core.md](core.md), the YAML keys the nodes read in [config.md](config.md), the source tree in [module-map.md](module-map.md), and the build and container setup in [environment.md](environment.md). The robot model, simulator, SLAM and EKF are not part of the planner package; they are in [reference-robot.md](reference-robot.md).

Topic names below are relative to the node namespace. The launch files run everything in the root namespace, so `plan` is `/plan`.

## Node and topic map

| Node (class) | Subscribes | Publishes | Action / service | Frames |
|---|---|---|---|---|
| `map_server` (`ROSMapProvider`, lifecycle) | `map_topic` (`nav_msgs/OccupancyGrid`, default `/map`) | none | none | none |
| `planner_node` (`PlannerNode`, lifecycle) | none | `search_markers` (`visualization_msgs/MarkerArray`) | serves `plan_path` (`path_planner_msgs/PlanPath`) | TF `planning_frame` <- `robot_frame` |
| `navigator` (`Navigator`) | `goal_pose` (`geometry_msgs/PoseStamped`), `plan_progress` (`path_planner_msgs/PlanProgress`) | `plan` (`path_planner_msgs/PlanTrajectory`), `plan_path` (`nav_msgs/Path`) | client of `plan_path` | TF `planning_frame` <- `robot_frame` |
| `controller_node` (`ControllerNode`) | `plan` (`PlanTrajectory`) | `cmd_vel` (`geometry_msgs/Twist`), `plan_progress` (`PlanProgress`) | none | TF `planning_frame` -> `odom_frame`, `odom_frame` <- `robot_frame` |
| `scan_safety_guard` (`ScanSafetyGuard`) | `scan` (`sensor_msgs/LaserScan`), `cmd_vel_in` (`Twist`) | `cmd_vel` (`Twist`) | none | TF `robot_frame` <- scan frame |

Executables, all installed by the `planner` package:

| Executable | Hosts | Executor |
|---|---|---|
| `planning_container` | `map_server`, `planner_node`, `navigator` in one process | `MultiThreadedExecutor` |
| `controller` | `controller_node` | single-threaded `rclcpp::spin` |
| `scan_safety_guard` | `scan_safety_guard` | single-threaded `rclcpp::spin` |

Node names are fixed in the constructors (`map_server`, `planner_node`, `navigator`, `controller_node`, `scan_safety_guard`). Assigning a node name from the launch file would rename every node in `planning_container`, so the launch files do not.

Command chain when launched with `planning_system.launch.py`:

```
controller_node  cmd_vel -> /cmd_vel_raw
scan_safety_guard  cmd_vel_in = /cmd_vel_raw,  cmd_vel -> /diff_drive_controller/cmd_vel_unstamped
```

The sink name is the input of the reference robot's `diff_drive_controller` with `use_stamped_vel: false` (see [reference-robot.md](reference-robot.md)). A different robot needs its own remap in the launch file.

All timers of these nodes are node-clock timers (`rclcpp::create_timer` with `get_clock()`), so they follow `use_sim_time`. The guard has no timer.

## Map provider

`ROSMapProvider` (`include/ros/map_provider/map_provider_server.hpp`) turns each `nav_msgs/OccupancyGrid` into an `OccupancyGrid` and an EDT and stores both as one immutable `MapSnapshot` behind an `std::atomic<std::shared_ptr<const MapSnapshot>>`. `snapshot()` returns that pointer and can be called from any thread. It is null before the first valid map and after cleanup.

The planner node and the navigator hold a handle to the provider (`bindMapProvider`), so they read the snapshot in process. No occupancy data goes between them over DDS.

| Parameter | Type | Default | Unit | Meaning |
|---|---|---|---|---|
| `map_topic` | string | `/map` | | Occupancy grid topic. Subscribed with `QoS(1).transient_local()`. |
| `edt_max_dist` | double | 5.0 | m | Clamp of the distance field. |

Bad input:

- `on_configure` returns `FAILURE` with an error log if `edt_max_dist` is not positive or is NaN.
- `onMap` ignores a message and keeps the previous snapshot, with a warning throttled to once per 5 s, when width or height is zero or above `INT_MAX`, `resolution` is not positive and finite, `origin.position.x` or `.y` is not finite after conversion to `float`, or `data.size()` is not width * height.
- A map with the same size, resolution, origin and cell hash as the last stored one (`makeSignature`, a 64-bit hash of the cells) leaves the snapshot pointer unchanged and builds no new EDT.

Lifecycle: `on_activate` creates the subscription and the heartbeat bond (period 1 s, timeout 4 s). `on_deactivate` releases both and keeps the snapshot. `on_cleanup` drops the snapshot.

## Planner node

`PlannerNode` (`include/ros/planner_server/planner_node.hpp`) is a lifecycle node that serves `plan_path`. It holds no path, watches no map and publishes no replan request.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `planning_frame` | string | `map` | Frame of goals, paths and markers. |
| `robot_frame` | string | `base_link` | Robot base frame. |
| `robot_params_path` | string | empty | Footprint file. |
| `diff_drive_params_path` | string | empty | File with the `diff_drive` section. |
| `astar_config_path` | string | empty | Search configuration. |
| `heuristic_config_path` | string | empty | Heuristic configuration. |
| `cost_policy_path` | string | empty | Cost policy. |

The five file paths must name existing files; the launch files pass installed ones. Keys in each file are in [config.md](config.md).

Request handling:

- A goal whose `header.frame_id` differs from `planning_frame` (an empty frame included) is rejected in `handleGoal` with a warning.
- The start is the latest `planning_frame` <- `robot_frame` transform (`tf2::TimePointZero`). If the lookup throws, the node logs an error and aborts the goal with `STOPPED`.
- A search runs on a `std::jthread` with a `stop_source`. One goal is active; a newer accepted goal stops the running search first and the older goal ends aborted with `STOPPED`. A client cancel ends the goal canceled with `STOPPED`.
- The search runs on the newest snapshot at the time the goal is accepted. A map update during the search does not affect it.
- If no snapshot exists, the node logs "no map snapshot available" and aborts with `STOPPED`.
- A snapshot whose resolution differs from the one the motion model was built at is rejected with an error log, and the previous planner keeps serving.
- The goal ends `succeed` only with `FOUND`. Every other outcome ends it aborted, carrying the outcome code.

Motion model: it is built when the first snapshot is available, by a 0.2 s timer started on activation, which logs "motion model built in <ms> ms on the first map". A request that arrives earlier builds it itself. `on_cleanup` discards it; the bound provider is kept, so configure, cleanup, configure, activate works.

Progress: while a search runs, a ~0.066 s timer publishes the expansions since the last publish as one `POINTS` marker (namespace `search`, in `planning_frame`) on `search_markers` (depth 10), and `PlanPath` feedback (`expansions`, `best_f`) once at least one expansion exists. Nothing is published when no search runs.

Lifecycle failures:

- `on_configure` lets a missing or malformed file raise a YAML exception, or an out-of-range value raise `std::invalid_argument`; the transition then fails.
- `on_activate` fails with `std::invalid_argument` if `bindMapProvider` was not called.
- `on_deactivate` and `on_cleanup` stop a running search, join the worker and break the bond.

## Navigator

`Navigator` (`include/ros/navigator/navigator.hpp`) is a plain node. It owns the navigation goal. The decision logic is `NavigationMode` ([core.md](core.md), Navigation mode; behaviour in [../design/navigation.md](../design/navigation.md)). The node feeds it events and carries out its actions. One timer of period `tick_period_sec` drives it.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `planning_frame` | string | `map` | Frame of goals and paths. |
| `robot_frame` | string | `base_link` | Robot base frame. |
| `robot_params_path`, `diff_drive_params_path`, `astar_config_path`, `controller_params_path`, `navigator_params_path` | string | empty | Required. An empty value fails the constructor (`CHECK`, `std::invalid_argument`). |

Keys of `navigator.yaml` are in [config.md](config.md) (`navigator.yaml`), and the values derived from them in the section "Values computed in code". Two relations are checked in the constructor and fail it with a message: `v_cruise` must not exceed `vmax`, and `stall_timeout` must exceed the follower's final-approach time plus final-turn time at speed scale 1 (and the approach time at `escape_speed_scale`). `stop_distance` is `v_cruise * (max_plan_time_sec + tick_period_sec)` and `plan_reply_timeout` is `max_plan_time_sec + stall_timeout`.

Goals:

- A goal whose frame is not `planning_frame` (an empty frame included) is ignored with a warning.
- A newer goal replaces the current one.
- A goal that arrives before the first map is held with the log line "goal held until the first map arrives". On the first map the navigator builds its motion model, then plans the newest held goal once. Earlier held goals are discarded. A plan request before the model exists fails a `CHECK`.

Plan requests and replies:

- A request goes to the `plan_path` action server. If the server is not ready or there is no goal, the navigator logs a warning and treats it as "no plan now" (`PlanOutcome::BudgetExhausted`), so the retry rule applies.
- A result code maps to one `PlanOutcome`. An aborted result with outcome `STOPPED` also counts as no plan now. A canceled one stays `Stopped`.
- A found path is checked from its first pose against the newest snapshot with `checkRemainingPath`. If it is blocked there, the navigator logs "planned path rejected: blocked <d> m along the path on the newest map" and treats the reply as no plan now. A path planned on an older map that the newest map blocks therefore stops the robot until the retry.
- A snapshot whose resolution differs from the motion model's is ignored with an error log.

Path checks while driving:

- On each new snapshot the remaining driven path is checked against it (log line "driven path blocked <d> m ahead by a map update"). If TF is unavailable the check is retried on the next tick, with a warning throttled to 5 s.
- While the mode is `Planning` and a plan is still driven, the check also runs on every tick, with the log line "driven path blocked <d> m ahead while replanning", throttled to 2 s.
- The stored driven path is cleared when nothing is driven.
- A stall replan logs "no progress along the path for <stall_timeout> s; replanning".

Escape: after a start-blocked reply the navigator waits for the follower to be idle and the robot at rest, then calls `findEscape` and publishes the result on `plan` with `speed_scale = escape_speed_scale`. The wait ends with no escape found after `stall_timeout`. The escape path is not path-checked. Algorithm: [core.md](core.md).

Outputs:

- `plan` (`PlanTrajectory`) and `plan_path` (`nav_msgs/Path`) are both `QoS(1).transient_local()`. `plan_path` carries the path of whatever was last published on `plan`.
- A stop is a `plan` with no poses and `plan_id` 0.
- A published plan with no poses counts as arrived at once, because the follower has nothing to report.
- Each state change logs "navigation state <state>, reason <reason>". There is no status topic.

Event time: the times given to `NavigationMode` come from `ForwardClock` (`include/core/navigation/forward_clock.hpp`) fed with the node clock.

- The value is seconds counted from the first event: the sum of the forward differences of the node clock.
- It never decreases and follows `use_sim_time`.
- A backward step of the node clock adds nothing.
- A forward step is passed through in full and can fire a stall, a reply timeout or a retry early, or end an escape wait early with no escape found.
- Message header stamps use the node clock directly.

All callbacks share one mutually exclusive callback group.

## Controller node

`ControllerNode` (`include/ros/controller/controller_node.hpp`) follows `plan` with the ROS-free `PurePursuit` law (`include/core/control/pure_pursuit.hpp`).

| Parameter | Type | Default | Unit | Meaning |
|---|---|---|---|---|
| `ctrl_rate` | double | 20.0 | Hz | Control rate. Must be positive (`CHECK`). |
| `planning_frame` | string | `map` | | Frame of the incoming plan. |
| `odom_frame` | string | `odom` | | Frame the plan is tracked in. |
| `robot_frame` | string | `base_link` | | Robot base frame. |
| `controller_params_path` | string | empty | | `controller.yaml`. |
| `robot_params_path` | string | empty | | Footprint and `diff_drive` file. |

The lookahead is `L = lookahead_gain * (r_in / 2)`. The constructor fails a `CHECK` unless `r_in / 2 < L < 2 * rmin`, and unless `v_cruise <= vmax`.

Tracking:

- `onPlan` transforms every pose of the plan from `planning_frame` into `odom_frame` once, with the transform at the stamp of the plan's path header (`path.header.stamp`) if TF has it and the latest otherwise. The transform is not refreshed until the next `plan`.
- Each tick looks up `odom_frame` <- `robot_frame` (latest), calls `PurePursuit::step` and publishes the `Twist` on `cmd_vel`.
- The follower is rebuilt on every plan, so a new plan resets the cursor.
- The mode of a pose (`FWD`, `REV`, `PIVOT`) and its speed come from `mode[]` and `speed[]`. `speed_scale` multiplies the cruise speed and the pivot rate.

`mode`, `speed` and `path.poses` of unequal length fail a `CHECK` in `onPlan`. The exception is not caught and no stop is published.

Stop cases, each publishing a zero `Twist`:

- No active plan: every tick.
- A plan with no poses: the follower is cleared.
- A plan with `speed_scale` outside (0, 1]: logged as an error, treated as a stop.
- A failed `planning_frame` -> `odom_frame` lookup in `onPlan`, or a failed `odom_frame` <- `robot_frame` lookup in a tick: warning throttled to 2 s.
- The follower reports done (`PurePursuit::done()`): the zero command is published until the next plan.

`cmd_vel` is published every tick, so a downstream node that gates on command arrival (the guard) sees a steady stream. Its tuning keys are in [config.md](config.md); the law, the goal-alignment turn and the end-of-path ramp are in [core.md](core.md).

## Scan safety guard

`ScanSafetyGuard` (`include/ros/safety/scan_safety_guard.hpp`) passes a velocity command on or replaces it with zero. It needs the scan, the footprint and the static transform from the scan frame to `robot_frame`. It does not use `map` or `odom`. Why the stop zone has the shape it has: [../design/safety-guard.md](../design/safety-guard.md).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `robot_frame` | string | `base_link` | Frame the scan is transformed into. |
| `robot_params_path` | string | empty | Footprint file. |
| `guard_params_path` | string | empty | `scan_safety_guard.yaml`: `linear_decel`, `angular_decel`, `forward_pad`, `side_pad`, `stop_count`, `clear_count`, `stale_scan_timeout`, `tf_lookup_timeout`. |

Behaviour:

- `scan` uses `SensorDataQoS` (best effort). Each message is transformed into `robot_frame` once, with the transform at the scan stamp if available within `tf_lookup_timeout` and the latest otherwise, projected to the plane. Returns that are not finite or outside `[range_min, range_max]` are dropped. If the lookup fails, the previous scan stays cached and a warning is logged, throttled to 2 s.
- The node publishes only when a command arrives on `cmd_vel_in`. A command is blocked when a cached return lies in the stop zone of its `(v, omega)` (`motionBlocked`, `include/core/control/stop_zone.hpp`). A zero command is never blocked.
- Reaction time is `scan_age + stop_count * cmd_period`. `scan_age` is the larger of the age by the scan's header stamp and the time since it arrived, clamped to `[0, stale_scan_timeout]`. `cmd_period` is the gap between the receive times of the last two commands, and counts as `stale_scan_timeout` until two commands have arrived or when the gap is outside `(0, stale_scan_timeout]`.
- Debounce: the hold engages after `stop_count` consecutive blocked commands and releases after `clear_count` consecutive clear ones. While held, every command is replaced by a zero `Twist`.
- Stale scan: if no scan has arrived, or the scan age exceeds `stale_scan_timeout`, the command is replaced by zero at once, the hold is set and both streaks reset. A backward step of the node clock makes the age infinite, which counts as stale.
- A command with a non-finite linear or angular velocity is replaced by zero at once and counts as blocked.
- Logs: a warning when the hold starts, giving the distance of the nearest return from the robot centre; an info line "released the hold" when it ends; a warning "scan stale or never received; fail-safe stop." throttled to 2 s.

The guard is a reactive backstop. It is not a hard guarantee on tracking error; [limits.md](limits.md) lists that as not built.

## Plan outcome, plan identity and follower progress

`path_planner_msgs` (`src/path_planner_msgs`):

| Type | Fields |
|---|---|
| `PlanPath` goal | `geometry_msgs/PoseStamped goal`. The start is the robot's current pose. |
| `PlanPath` result | `uint8 outcome`, `PlanTrajectory trajectory`, `uint32 expansions`, `float64 cost`. |
| `PlanPath` feedback | `uint32 expansions`, `float64 best_f`. |
| `PlanTrajectory` | `header`, `nav_msgs/Path path`, `int8[] mode`, `float64[] speed`, `uint32 plan_id`, `float64 speed_scale`. |
| `PlanProgress` | `header`, `uint32 plan_id`, `uint32 cursor`, `uint32 size`, `bool arrived`. |

Outcome:

- `outcome` is one of `FOUND` (0), `START_BLOCKED` (1), `GOAL_BLOCKED` (2), `NO_PATH` (3), `BUDGET_EXHAUSTED` (4), `STOPPED` (5), mirroring the core `PlanOutcome`. There is no `success` field and the goal has no timeout.
- `trajectory`, `expansions` and `cost` are filled only for `FOUND`. They are zero or empty otherwise.
- `STOPPED` also covers a missing transform, a missing map, a superseded goal and a canceled goal.

Trajectory:

- `pathToTrajectory` (`include/ros/planner_server/plan_conversions.hpp`) flattens each segment's sample poses into `path.poses`. Each pose gets its segment's `mode` and `speed` (the segment's `v`, m/s, signed). `mode` is `PIVOT` (2) when `|v| < 1e-6` and `|omega| > 1e-6`, `REV` (1) when `v < 0`, otherwise `FWD` (0).
- `mode`, `speed` and `path.poses` have equal length. `pathToTrajectory` and `ControllerNode` both `CHECK` it.
- `pathToTrajectory` sets `plan_id` 0 and `speed_scale` 1.0, and the planner node returns it as is.
- The navigator assigns `plan_id` (increasing, 0 only for the empty stop plan) and `speed_scale` in (0, 1]: 1.0 for a goal plan, `escape_speed_scale` for the clearing move.

Progress:

- `ControllerNode` publishes `plan_progress` when `plan_id`, `cursor` or `arrived` changes and every 0.2 s regardless. `plan_id` is the plan it follows (0 when idle), `cursor` the tracked pose index, `size` the number of poses (0 when idle), `arrived` true once the follower is done.
- The navigator hands a report to `NavigationMode` only when its `plan_id` is non-zero and equals the plan being driven.

## Launch files

`launch/` of the `planner` package. The simulator and the robot are started by [reference-robot.md](reference-robot.md).

| File | Starts |
|---|---|
| `planning_container.launch.py` | `planning_container` only, with `map_topic` `/map`, `edt_max_dist` 5.0 and the seven file paths. No lifecycle manager: `map_server` and `planner_node` stay unconfigured until something configures and activates them. |
| `planning_system.launch.py` | The nodes below. |

`planning_system.launch.py` arguments:

| Argument | Default | Meaning |
|---|---|---|
| `slam_mode` | `mapping` | `mapping`, or `localization` to load `map_file`. |
| `map_file` | empty | Serialized pose graph, no extension. Ignored in mapping mode. |
| `use_sim_time` | `false` | Applied to every node of this launch through one `SetParameter`. |
| `slam_transform_publish_period` | `0.02` | s, `transform_publish_period` of `slam_toolbox`. `0.0` stops its `map` -> `odom` broadcast so another node can own that transform. |

Nodes it starts:

| Node | Executable | Parameters passed | Remaps |
|---|---|---|---|
| `slam_toolbox` | `map_and_localization_slam_toolbox_node` | `config/slam/mapper_params.yaml`, `mode`, `map_file_name`, `transform_publish_period` | none |
| `lifecycle_manager` | `nav2_lifecycle_manager` | `config/lifecycle_manager/lifecycle_manager.yaml` | none |
| container | `planning_container` | `map_topic` `/map`, `edt_max_dist` 5.0, `planning_frame` `map`, `robot_frame` `base_link`, `robot_params_path`, `diff_drive_params_path`, `astar_config_path`, `heuristic_config_path`, `cost_policy_path`, `controller_params_path`, `navigator_params_path` | none |
| `controller_node` | `controller` | `ctrl_rate` 20.0, `planning_frame` `map`, `robot_frame` `base_link`, `controller_params_path`, `robot_params_path` | `/cmd_vel` -> `/cmd_vel_raw` |
| `scan_safety_guard` | `scan_safety_guard` | `robot_frame` `base_link`, `robot_params_path`, `guard_params_path` | `scan` -> `/scan`, `cmd_vel_in` -> `/cmd_vel_raw`, `cmd_vel` -> `/diff_drive_controller/cmd_vel_unstamped` |

`lifecycle_manager.yaml` sets `autostart: true`, `node_names: [map_server, planner_node]`, `bond_timeout: 4.0` and `attempt_respawn_reconnection: true`. The manager configures and activates the two nodes in that order. Each opens a bond with a 1 s heartbeat that the manager monitors.

`mapper_params.yaml` is keyed by the `/**` wildcard so that the `transform_publish_period` override from the launch file takes effect.

The launch does not start a robot or a source of `scan` and `odom` -> `base_link`.

To save a map, call `/slam_toolbox/serialize_map` (`slam_toolbox/srv/SerializePoseGraph`) and start later with `slam_mode:=localization map_file:=<saved graph>`. The saved map is loaded with the robot at the pose where the mapping run started (`map_start_at_dock: true` in `mapper_params.yaml`), so start the robot there; in the reference simulator it always spawns at that pose, the world origin. To start from another pose, or to load a map into a running localization-mode node, call `/slam_toolbox/deserialize_map` (`slam_toolbox/srv/DeserializePoseGraph`) with the file name, `match_type` 3 (localize at pose) and the robot's pose on the map.

## Reference robot

The optional `robot_bringup` package, which supplies the robot, the simulator, an EKF and a combined launch, is in [reference-robot.md](reference-robot.md).
