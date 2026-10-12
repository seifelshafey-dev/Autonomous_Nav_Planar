# Overview

This project is a hybrid A* planner for differential-drive robots in continuous SE(2), a ROS 2 stack around it (map provider, planner action server, navigator, pure-pursuit follower, scan guard), and a reference robot package that the planner does not depend on. This page is the map of the architecture pages. The front page is the [README](../../README.md).

## How a goal becomes motion

1. SLAM publishes an occupancy grid on `/map`.
2. `ROSMapProvider` (`map_server`) turns each grid into an immutable `MapSnapshot` with a distance field.
3. A goal arrives on `goal_pose`. `Navigator` decides to plan and sends a `plan_path` action goal.
4. `PlannerNode` (`planner_node`) runs one search on the newest snapshot and returns the path.
5. `Navigator` checks the path against the newest map and publishes it on `plan` (and on `plan_path` as a `nav_msgs/Path`).
6. `ControllerNode` (`controller_node`) tracks `plan` with pure pursuit in the odometry frame, publishes `cmd_vel` and reports progress on `plan_progress`.
7. `ScanSafetyGuard` (`scan_safety_guard`) reads `cmd_vel_in` and `scan` and publishes `cmd_vel`, zero when the lidar sees something inside the stopping zone.

```
/map -> map_server -> snapshot
goal_pose -> navigator -> plan_path -> planner_node
navigator -> plan -> controller_node -> cmd_vel_in -> scan_safety_guard -> cmd_vel
controller_node -> plan_progress -> navigator
```

## Design decisions

- State stays continuous and is never snapped to a grid. Cells are used only as lookups (the visited-set key, the per-heading footprint cache, collision queries, the cost map, the obstacle-aware estimate). The escape move is the one place that walks a straight line in fixed steps. [core.md](core.md)
- Motions are control primitives `(v, ω, dt)`, precomputed in the robot frame. [core.md](core.md), [primitives](../design/primitives.md)
- The cached swept cells are a cover: a clear scan proves the motion clear, and an occupied cell triggers an exact check, not a rejection. [core.md](core.md), [swept cover](../design/swept-cover.md)
- Collision checking runs in three steps: distance-field accept or reject, cover scan, exact check. [core.md](core.md)
- Cost is `sqrt(v² + rmin²·ω²)·dt` plus a clearance cost. The estimate is the larger of an obstacle-aware field and a weighted straight-line distance. The cheapest path is not guaranteed. [core.md](core.md), [cost and heuristic](../design/cost-and-heuristic.md)
- The visited set uses a cell that is a whole number of map cells (0.15 m in the shipped settings). [core.md](core.md), [primitives](../design/primitives.md)
- Unknown space can be crossed, at a price. [limits.md](limits.md)
- A blocked start or goal is refused. A blocked start is cleared outside the planner by a short straight move. [core.md](core.md), [navigation](../design/navigation.md)
- The core has no ROS dependency. [core.md](core.md), [module-map.md](module-map.md)
- The map is shared as an unchanging snapshot; a new map is a new snapshot and a new planner. [core.md](core.md), [ros.md](ros.md)
- Navigation logic is a pure state machine (events in, actions out) inside a thin node, and its event times never decrease. [ros.md](ros.md), [navigation](../design/navigation.md)
- The follower tracks in the odometry frame. [ros.md](ros.md)
- The scan guard stops on what the robot would hit before it can stop, and does not use the map. [ros.md](ros.md), [safety guard](../design/safety-guard.md)
- A value that follows from another setting is computed in code, not written in a config file. [config.md](config.md)
- Attaching an observer never changes a search result. [core.md](core.md)

## Pages

| Page | Content |
|---|---|
| [core.md](core.md) | The ROS-free planner: types, primitives, collision checking, search, heuristics, gates, navigation mode, following. |
| [ros.md](ros.md) | The five nodes, their topics, actions, parameters and launch files. |
| [reference-robot.md](reference-robot.md) | The example robot, simulator, SLAM and EKF. |
| [config.md](config.md) | The YAML files, values computed in code, and the checks between settings. |
| [module-map.md](module-map.md) | Every source file and what it holds. |
| [environment.md](environment.md) | The container image, host build, test lanes and CI. |
| [limits.md](limits.md) | What the software does not guarantee and does not include. |

The derivations behind the bounds are in [../design/README.md](../design/README.md). Getting started, scenarios and testing are in [../guide/](../guide/getting-started.md).
