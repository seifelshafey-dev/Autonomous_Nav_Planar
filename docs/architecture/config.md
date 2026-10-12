# Configuration reference

The YAML files live in `src/path_planner/config/<area>/` and install to `share/planner/config/<area>/`. The comments in each file are the per-key reference: what a key means and why the shipped value is what it is. This page lists units, the bounds the loaders enforce, defaults, values the code computes, and the relations between settings. Every value is checked when it is loaded (`CHECK` in `src/path_planner/src/core/config/config_loader.cpp`, in `src/core/robot/robot_params.cpp` and in `src/core/scenario/scenario_loader.cpp`), and a bad value stops start-up with a message that names the key. A key marked "required" has no default: a missing key makes the loader throw.

## Files

Eight files are shipped. Six are read by `config_loader`, one by `nav2_lifecycle_manager` and one by `slam_toolbox`.

### `src/path_planner/config/robot/robot_params.yaml`

Read by `loadRobotParams` and `loadDiffDriveParams`. Several nodes read the same file.

| Key | Unit | Allowed | Default |
|---|---|---|---|
| `vertices` | m, body frame | at least 3 vertices; strictly convex with counter-clockwise order (every cross product of consecutive edges `> 1e-9`); origin strictly inside the polygon | required |
| `diff_drive.rmin` | m | `> 0` | required |
| `diff_drive.vmax` | m/s | `> 0` | required |
| `diff_drive.atomic_time` | s | `> 0` | 0.6 |
| `diff_drive.n_headings` | count | `>= 1`, at most 32768 (not checked) | 16 |

### `src/path_planner/config/planner/planner.yaml`

Read by `loadAStarConfig` (section `a_star`) and `loadHeuristicConfig` (section `heuristic`). Every key is optional and falls back to the default in the table.

| Key | Unit | Allowed | Default |
|---|---|---|---|
| `a_star.goal_pos_tol` | m | `> 0` | 0.10 |
| `a_star.goal_heading_tol` | rad | `> 0` | 0.10 |
| `a_star.max_expansions` | count | `> 0` | 200000 |
| `a_star.max_plan_time_sec` | s | `> 0` | 5.0 |
| `a_star.enable_shot` | bool | any | true |
| `a_star.shot_max_distance` | m | `>= 0` | 5.0 |
| `a_star.shot_allow_unknown` | bool | any | false |
| `a_star.closed_cell_size` | m | `> 0` | 0.15 |
| `a_star.node_pool_reserve` | count | `>= 0` | 200000 |
| `a_star.closed_set_reserve` | count | `>= 0` | 200000 |
| `a_star.costmap_weight` | unitless | `>= 0` | 1.0 |
| `heuristic.weight` | unitless | `> 0` | 1.0 |
| `heuristic.use_obstacle_heuristic` | bool | any | true |

`heuristic.weight` 1.0 is the unscaled estimate and a larger value is a greedier search. The cheapest path is not guaranteed at any value. The shipped file sets `shot_allow_unknown: true`; the code default is false. The behaviour of each search setting is in [core.md](core.md).

### `src/path_planner/config/cost_policy/cost_policy.yaml`

Read by `loadCostPolicy`. All four keys are required. The key `unknown_cost` is rejected with a message that points to `unknown_cost_factor`.

| Key | Unit | Allowed |
|---|---|---|
| `clearance_radius` | m | `>= 0` |
| `clearance_decay` | 1/m | `>= 0` |
| `max_clearance_cost` | cost per metre | `>= 0` |
| `unknown_cost_factor` | unitless | finite, `>= 1` |

The cost of a metre of unknown or out-of-map space is computed from `unknown_cost_factor` in `buildCostMap`.

### `src/path_planner/config/controller/controller.yaml`

Read by `loadControllerParams`. All six keys are required.

| Key | Unit | Allowed |
|---|---|---|
| `lookahead_gain` | unitless | `> 0` |
| `v_cruise` | m/s | `> 0` |
| `a_lat_max` | m/s² | `> 0` |
| `omega_pivot` | rad/s | `> 0` |
| `goal_pos_tol` | m | `> 0` |
| `goal_heading_tol` | rad | `> 0` |

### `src/path_planner/config/safety/scan_safety_guard.yaml`

Read by `loadScanGuardParams`. All eight keys are required.

| Key | Unit | Allowed |
|---|---|---|
| `linear_decel` | m/s² | finite, `> 0` |
| `angular_decel` | rad/s² | finite, `> 0` |
| `forward_pad` | m | finite, `>= 0` |
| `side_pad` | m | finite, `> 0` |
| `stop_count` | count | `>= 1` |
| `clear_count` | count | `>= 1` |
| `stale_scan_timeout` | s | `> 0` |
| `tf_lookup_timeout` | s | `> 0` |

### `src/path_planner/config/navigation/navigator.yaml`

Read by `loadNavigatorParams`, section `navigator`. All nine keys are required.

| Key | Unit | Allowed |
|---|---|---|
| `tick_period_sec` | s | finite, `> 0` |
| `stall_timeout` | s | finite, `> 0` and `> tick_period_sec` |
| `blocked_retry_period` | s | finite, `> 0` |
| `max_clearing_attempts` | count | `>= 1` |
| `max_stall_replans` | count per goal | `>= 0` |
| `escape_speed_scale` | fraction of full speed | finite, in `(0, 1]` |
| `escape.step_fraction` | fraction of the grid resolution | finite, in `(0, 1)` |
| `escape.slack_cells` | cells | `>= 0` |
| `escape.max_travel_factor` | footprint outer radii | finite, `> 0` |

The `escape.*` keys are `EscapeConfig`; the escape move is described in [core.md](core.md).

### Files not read by `config_loader`

| File | Read by | Notes |
|---|---|---|
| `src/path_planner/config/lifecycle_manager/lifecycle_manager.yaml` | `nav2_lifecycle_manager` | Manages `map_server` and `planner_node`. |
| `src/path_planner/config/slam/mapper_params.yaml` | `slam_toolbox` | Part of the reference robot set-up, see [reference-robot.md](reference-robot.md). `mode` and `map_file_name` come from the `slam_mode` and `map_file` launch arguments. The file is keyed by the `/**` wildcard instead of the node name, so that launch-time overrides win. |

## Values computed in code

These have no config key. The navigator computes them in its constructor (`src/path_planner/src/ros/navigator/navigator.cpp`) and when it first sees a map.

| Name | Formula | Meaning |
|---|---|---|
| `stop_distance` (m) | `v_cruise · (max_plan_time_sec + tick_period_sec)` | Distance covered while a replan runs plus one tick. A blocked path nearer than this stops the robot. `v_cruise` is from the controller file, `max_plan_time_sec` from the planner file, `tick_period_sec` from the navigator file. |
| `plan_reply_timeout` (s) | `max_plan_time_sec + stall_timeout` | How long a plan request may stay unanswered before it is abandoned. |
| `stop_short` (m) | `controller.goal_pos_tol` | Passed to `findEscape`. |
| rest position tolerance (m) | `δ / 4`, with `δ = escape.step_fraction · h` | Largest position change between two ticks that still counts as at rest. `h` is the grid resolution in metres. |
| rest heading tolerance (rad) | `δ / (4 · r_out)` | Same for heading. `r_out` is the circumscribed radius of the footprint. |

With the shipped files and `h = 0.05` m, the rest tolerances are 0.005 m and 0.0118 rad.

The controller node computes the carrot distance `L = lookahead_gain · (r_in / 2)`, where `r_in` is the inscribed radius of the footprint. The pure-pursuit follower uses `L` as its lookahead.

`closed_cell_size` is rounded by the planner to a whole number of map cells, at least one (`HybridAstar::closedStride`). The effective cell is that count times the map resolution.

## Relations checked between settings

| Relation | Message (abridged) | Checked in |
|---|---|---|
| `controller.v_cruise <= diff_drive.vmax` | "must not exceed diff-drive vmax, the speed the planner's motions are built at" | `Navigator` constructor and `ControllerNode` constructor |
| `r_in/2 < L < 2·rmin`, with `L = lookahead_gain · r_in/2` | "derived lookahead L must satisfy r_in/2 < L < 2*rmin (adjust controller.lookahead_gain)" | `ControllerNode` constructor |
| `stall_timeout > crawl(1) + goalAlignTimeBound(omega_pivot, goal_heading_tol)` | "stall_timeout must exceed the follower's final approach time plus its final turn" | `Navigator` constructor |
| `stall_timeout > crawl(escape_speed_scale)` | "stall_timeout must exceed the follower's final approach time" | `Navigator` constructor |
| `stall_timeout > tick_period_sec` | "navigator.stall_timeout must be greater than navigator.tick_period_sec" | `loadNavigatorParams` |
| `vmax · atomic_time > √2 · effective closed cell` | "straight primitive length vmax*atomic_time must exceed sqrt(2) * effective closed cell; lower closed_cell_size or lengthen the primitive" | `buildHybridAstar` |
| `edt_max_dist > max(r_out + √2 · h, c_max)`, with `c_max` the largest primitive sample clearance | "distance-field limit edt_max_dist must exceed ...; raise edt_max_dist" | `buildHybridAstar` |
| `cost_policy.clearance_radius > r_out` | "cost_policy.clearance_radius must exceed the robot's circumscribed radius r_out" | `buildHybridAstar` |
| kinematic model cell size equals the snapshot resolution (within 1e-6 m) | "KinematicModel cell size does not match the map snapshot grid resolution" | `buildHybridAstar` |

The stall relations use the time the follower needs to finish a plan: `crawl(s) = (L / (s · v_cruise)) · max(ln(L / goal_pos_tol), spacing / L, ln 3)`, with `spacing = r_in / 2`. `goalAlignTimeBound` is the time to turn onto the goal heading at `omega_pivot`: with `h_sat = 2.5 · goal_heading_tol`, it is `(π − h_sat)/omega_pivot + (h_sat/omega_pivot) · ln 2.5` when `h_sat < π`, and `(h_sat/omega_pivot) · ln(π / goal_heading_tol)` otherwise. With the shipped files the two stall bounds are about 4.47 s (scale 1) and 2.71 s (scale 0.5) against `stall_timeout` 5.0 s.

## Node parameters outside the YAML files

These are ROS parameters set by the launch files. Node behaviour is in [ros.md](ros.md).

| Node | Parameter | Type and default | Notes |
|---|---|---|---|
| map provider | `map_topic` | string, `/map` | Occupancy grid topic. |
| map provider | `edt_max_dist` | double, 5.0 m | Distance-field limit; must be `> 0` or configuring fails. Also bound by the relation above. |
| planner node | `robot_params_path`, `diff_drive_params_path`, `astar_config_path`, `heuristic_config_path`, `cost_policy_path` | string, empty | Paths to the config files. |
| planner node, navigator, controller, scan guard | `planning_frame`, `robot_frame` | string, `map` and `base_link` | The scan guard has `robot_frame` only. |
| navigator | `robot_params_path`, `diff_drive_params_path`, `astar_config_path`, `controller_params_path`, `navigator_params_path` | string, required | A path that is empty stops start-up. |
| controller | `controller_params_path`, `robot_params_path` | string, empty | Paths to the config files. |
| controller | `ctrl_rate` | double, 20.0 Hz | Must be `> 0`. |
| controller | `odom_frame` | string, `odom` | Frame the follower tracks in. |
| scan guard | `robot_params_path`, `guard_params_path` | string, empty | Paths to the config files. |

`planning_system.launch.py` also takes the launch arguments `slam_mode` (default `mapping`), `map_file` (empty), `use_sim_time` (false) and `slam_transform_publish_period` (0.02 s).
