# path_planner_msgs

The action and messages the planner stack talks in: `PlanPath` to ask for a path, `PlanTrajectory` to carry one, `PlanProgress` to report how far the follower has got.

## PlanPath (action)

A client sends a goal pose and gets back an outcome and, if a path was found, the trajectory.

| Section | Field | Type | Meaning |
|---|---|---|---|
| Goal | `goal` | `geometry_msgs/PoseStamped` | Goal pose. The server plans from the robot's current pose and accepts only a goal in its planning frame (`map` by default). |
| Result | `outcome` | `uint8` | One of the codes below. |
| Result | `trajectory` | `PlanTrajectory` | The path; populated only when `outcome` is `FOUND`. |
| Result | `expansions` | `uint32` | Number of states the search expanded. Set when a path was found; `0` otherwise. |
| Result | `cost` | `float64` | Total cost of the path: weighted SE(2) arc length plus the costmap penalty integral. Lower is better. `0.0` unless `outcome` is `FOUND`. |
| Feedback | `expansions` | `uint32` | States expanded so far. |
| Feedback | `best_f` | `float64` | `f` value (cost so far plus the heuristic) of the most recently expanded state. |

Outcome codes:

| Constant | Value |
|---|---|
| `FOUND` | 0 |
| `START_BLOCKED` | 1 |
| `GOAL_BLOCKED` | 2 |
| `NO_PATH` | 3 |
| `BUDGET_EXHAUSTED` | 4 |
| `STOPPED` | 5 |

## PlanTrajectory (message)

A path plus the motion intent for each pose, which a bare `nav_msgs/Path` cannot carry.

| Field | Type | Unit or range | Meaning |
|---|---|---|---|
| `header` | `std_msgs/Header` | | Stamp and frame of the plan. |
| `path` | `nav_msgs/Path` | | Sample poses ordered from start to goal; this is what RViz reads. |
| `mode` | `int8[]` | `FWD`, `REV` or `PIVOT` | Motion mode at each pose. |
| `speed` | `float64[]` | m/s, signed | Reference speed `v` at each pose. |
| `plan_id` | `uint32` | | Identity of the plan, assigned by its publisher and increasing; `0` for the empty stop plan. |
| `speed_scale` | `float64` | (0, 1] | Multiplies the follower's cruise speed and pivot rate for this plan. |

Mode constants:

| Constant | Value |
|---|---|
| `FWD` | 0 |
| `REV` | 1 |
| `PIVOT` | 2 |

Rules for a publisher:

- `mode`, `speed` and `path.poses` must have the same length. Entry `i` of `mode` and `speed` describes pose `i`.
- `plan_id` increases from one plan to the next. `0` is reserved for the empty stop plan.
- `speed_scale` is in (0, 1].

The planner server fills in `mode` and `speed` from the controls of the path and leaves `plan_id` at `0` and `speed_scale` at `1.0`; whoever publishes the plan to the follower assigns the id.

## PlanProgress (message)

Follower progress, published at 5 Hz and whenever a field changes.

| Field | Type | Unit or range | Meaning |
|---|---|---|---|
| `header` | `std_msgs/Header` | | Stamp of the report. |
| `plan_id` | `uint32` | | `plan_id` of the plan being followed; `0` when idle. |
| `cursor` | `uint32` | `[0, size)` | Index of the trajectory pose being tracked. |
| `size` | `uint32` | | Number of poses in the plan; `0` when idle. |
| `arrived` | `bool` | | The final pose has been reached within the goal tolerances. |

Apache-2.0, see [LICENSE](../../LICENSE).
