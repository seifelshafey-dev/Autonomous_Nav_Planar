# Limits

What the software as built does not guarantee, and what it does not include. The reasoning behind each limit is in the [design pages](../design/README.md).

## Known limits

- Cheapest path. A returned path is not guaranteed to be the cheapest. The obstacle-aware estimate can exceed the true cost, and when `enable_shot` is set the first goal shot that fits is returned (`HybridAstar::shotAtGoal`), even if the search's own path would cost less. Set `enable_shot: false` to drop the shot. See [cost-and-heuristic.md](../design/cost-and-heuristic.md).
- Unknown space is crossable. The search expands through UNKNOWN cells at a price (`CostMap::costAt`), and `shot_allow_unknown` is `true` in the shipped `planner.yaml`. A path can cross space the map has not seen. Set `shot_allow_unknown: false` to stop the shot from doing so.
- Off-map goal. For a goal outside the grid `ObstacleHeuristic` returns 0, so only the straight-line estimate guides the search.
- New obstacles reach the map late. Observation, not code: in the simulator with the shipped SLAM settings (`mapper_params.yaml`, `minimum_travel_distance` 0.2 m), a lidar-visible obstacle entered the map only after the robot had driven 0.24 to 0.85 m, and not while it stood still. The planner can plan through an obstacle the lidar already sees until the map has it.
- Guard and planner disagree near walls. The planner uses the footprint as it is. The guard grows it by `side_pad`. The guard's zone is the footprint swept over the stopping motion and grown by `side_pad`. A path whose zone reaches a wall the planner accepted ends in a guard hold, and after `max_stall_replans` replans in `Blocked(Stalled)`.
- A return inside the padded footprint holds the robot whatever the direction. `motionBlocked` tests the present pose first, so it reports every non-zero command as blocked, reverse included. The hold engages after `stop_count` consecutive blocked commands. Zero commands count as clear and release it after `clear_count`, so after a release the first `stop_count - 1` blocked commands pass. An escape move out of that position is held.
- Stop zone from commanded speed. `motionBlocked` takes the commanded `(v, omega)`, not the measured velocity. After a switch from driving to pivoting the robot can still be rolling while the zone is sized for a turn.
- Guard input. The guard sees one scan plane. Returns nearer than the sensor's minimum range are dropped.
- No tracking guarantee. Nothing bounds the distance between the robot and the planned path. `ScanSafetyGuard` is a best-effort backstop.
- Forward clock steps. `ForwardClock` removes backward steps of the navigator's clock but passes forward steps through. A forward step fires the stall, reply-timeout, retry or escape-wait rule early. A fired stall rule uses one of `max_stall_replans`.
- Map values below -1. `OccupancyGrid::findCellStatus` reads only -1 as UNKNOWN. Any other value below `occupied_threshold`, including values below -1, is FREE.
- Escape move. `findEscape` moves straight at constant heading. A robot wedged at an angle can get no escape and the navigator reports `Blocked(NoEscape)`.
- Stall detection. Progress is the follower's path index. A robot creeping inside one sample interval looks stalled. `Blocked(Stalled)` is left only by a new goal.
- Malformed plan message. A plan whose mode, speed and pose arrays differ in length throws out of the follower's callback. The planner never sends one; only another publisher on the topic can.
- Primitive sets. For arcs and pivots nothing checks at build time that the motion leaves its duplicate cell or heading bin. A motion that does not is dropped as a duplicate of its parent. Only the straight primitive is checked (`buildHybridAstar`: its length against `sqrt(2)` times the effective duplicate cell). See [primitives.md](../design/primitives.md).
- Simulator and SLAM, observation. In one headless run in `phase2_demo.sdf` (one wall and a ground plane), SLAM's pose stopped following the robot after about 6 s of the second goal and was 3 m off at the end, while wheel odometry stayed within 0.03 m. In `cluttered_demo.sdf` localisation held, but the robot covered about 2 m in 70 s with several guard holds and stall replans beside one pillar. Test in `cluttered_demo.sdf` or `tb4_warehouse`.
- Hardware. Everything has been run on ROS 2 Humble and in simulation. Nothing has run on a physical robot.

## Not built

- Reeds-Shepp goal shot. The shot is pivot, straight, pivot.
- A distance field that updates in place. A new map builds a new snapshot and a new planner.
- Non-convex footprints. `buildRobotParams` takes a convex polygon.
- A behaviour-tree layer above the planner.
- A hard runtime guarantee on tracking error, by a swept-footprint guard or by inflating the footprint.
- A replan because a newly mapped area offers a shorter route. The navigator replans when the driven path is blocked or stalled.
- Projection of an off-map goal onto the grid for the obstacle-aware estimate.
- A live scan layer over the SLAM map.
- Odometry input to the scan guard.
