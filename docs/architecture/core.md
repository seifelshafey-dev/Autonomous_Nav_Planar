# Planner core

Reference for the code under `src/path_planner/include/core` and `src/path_planner/src/core`. It has no ROS dependency. It says what each part does, what a caller can rely on, and in which unit, frame and rounding mode. The reasoning behind the rules is in the [design pages](../design/README.md). The ROS nodes are in [ros.md](ros.md), the YAML keys in [config.md](config.md), the file listing in [module-map.md](module-map.md).

The planner is a hybrid A* search in continuous SE(2). It is not a state lattice: states are never snapped. Cells are used as lookups: the closed-set key, the per-heading-bin footprint cache, collision queries, the cost map and the obstacle-aware estimate. The escape move is the one place that walks in fixed steps. `CHECK(cond, msg)` is compiled into every build type and throws `std::invalid_argument` with file and line (`core/diagnostics/check.hpp`). `assert` is used only for the closed-key packing range (`packClosedKey`) and is absent when `NDEBUG` is defined.

## 1. Types and conventions

| Item | Rule |
|---|---|
| Frames | Planner poses, map cells and `CollisionChecker` inputs are in the map frame: the frame of the occupancy grid. Primitive data (`endpoint_offset`, `swept_cells`, `sample_cells`) is in the body frame: origin at the robot reference point, x forward. |
| Units | Metres, radians, seconds. Cells are integers; a metre value is never used as a cell index without `worldToGrid`. |
| `SE2State` | `{x, y, theta}`: position (m) and heading (rad, counter-clockwise from +x). |
| `Control` | `{v, omega, dt}`: `v` in m/s (signed, negative is reverse), `omega` in rad/s (positive is counter-clockwise), `dt` in s. Both rates are in the body frame. |
| `GridCoords` | `{i, j}`: column (x) and row (y). Ordered by `i`, then `j`. `operator+` adds component-wise. |
| `CellState` | `FREE`, `OCCUPIED`, `UNKNOWN`. |

**World to cell.** `GridMetaData::worldToGrid(x, y)` returns `(floor((x - ox) / h), floor((y - oy) / h))`, with `h` the resolution and `(ox, oy)` the origin, the world position of the corner of cell (0, 0) with the smallest x and y. Cell `(i, j)` covers `[ox + i·h, ox + (i+1)·h)` in x and likewise in y. Every snap in the planner is a floor, never a round. `gridToWorld` returns the cell centre `((i + 0.5)·h + ox, (j + 0.5)·h + oy)` as floats. Cell centres are the representative point for the rasterizer and the distance field.

**Heading range.** `wrapToPi` is `std::remainder(angle, 2π)`. Its result lies in the closed range `[-π, π]`: `wrapToPi(π) == π` and `wrapToPi(-π) == -π`. A non-finite input gives NaN. `DiffDriveStateSpace::propagate` returns `theta + omega·dt` unwrapped. `DiffDriveStateSpace::interpolate` wraps. So `SE2State::theta` can be outside `[-π, π]`; wrap it before comparing. **Heading bins.** With `N = n_headings` and `Δ = 2π/N`, bin `k` covers the half-open range `[-π + k·Δ, -π + (k+1)·Δ)`, so the bins span `[-π, π)`. A heading within rounding of a bin edge can land in the bin below it; the cover of each bin includes both of its edges, so this does not affect the collision check. `DiffDrivePrimLib::headingBin(theta)` returns `floor((wrapToPi(theta) + π)/Δ)` and maps the value `N` (reached for a wrapped heading of π, or within rounding of it) to bin 0. The result is an `int16_t` in `[0, N)`. The index fits only when `n_headings` is at most 32768; nothing rejects a larger value.

**Propagation.** `DiffDriveStateSpace::propagate(a, u)`: with `ω = u.omega`, if `ω == 0` exactly then `dx = v·dt, dy = 0`; otherwise `R = v/ω`, `dx = R·sin(ω·dt)`, `dy = R·(1 − cos(ω·dt))`. The offset `(dx, dy)` is rotated by `a.theta` and added to `(a.x, a.y)`; the heading becomes `a.theta + ω·dt`. With `v == 0` this is a pivot. `DiffDriveStateSpace::inBounds` is true only strictly inside the grid extent. **Footprint.** `buildRobotParams(vertices)` takes a body-centred polygon that is strictly convex (consecutive-edge cross product above `1e-9`), wound counter-clockwise, and contains the origin strictly inside; it throws `std::invalid_argument` otherwise. It stores one outward `HalfPlane {normal, offset}` per edge, `r_in` (the smallest offset: the largest origin-centred circle inside the polygon) and `r_out` (the largest vertex norm: the smallest origin-centred circle containing it). **Rasterizing.** `visitPolygonCells` / `rasterizePolygon(half_planes, aabb, h, margin)` return the cells whose centre passes every half-plane test with the plane pushed out by `√2/2·h + margin`, inside the bounding box grown by the same amount (indices from `floor((min − c)/h)` to `floor((max + c)/h)` inclusive). `h` is finite and positive, `margin` finite and `>= 0` (`CHECK`). The frame is whatever frame the geometry is in; `transformGeometry(body, state, cell_origin)` subtracts `cell_origin` so that the cells match the map grid.

## 2. Map types

**`GridMetaData(cols, rows, cell_size, origin)`** rejects with `CHECK`: a non-positive column or row count, a non-positive or NaN resolution, a resolution (as a float) or an origin component that is not finite. `world_width` and `world_height` are `cols·h` and `rows·h` rounded up to a whole metre. `inBounds(gc)` is `0 <= i < cols` and `0 <= j < rows`; `index(gc)` is `j·cols + i`, and the caller ensures `inBounds`. The resolution is a `float`. `occupied_threshold` is 50.

**`OccupancyGrid(meta, data)`** holds its metadata by value. `data` is row-major, `rows·cols` entries of `int8_t`; `std::nullopt` makes every cell `-1`. A different size throws (`CHECK`), also in `setData`. `at(gc)` returns the raw value, and `-1` outside the grid. `findCellStatus(gc)` is `UNKNOWN` for raw `-1` or outside the grid, `OCCUPIED` for raw `>= occupied_threshold`, `FREE` otherwise. A moved-from grid is empty: 0 rows and columns, no cells, resolution and origin kept; `at` returns `-1`, `findCellStatus` returns `UNKNOWN`, `rawAccess` returns `-1`, and `setData` accepts only an empty vector.

**`EDT`** is a row-major distance field with the same extent as the grid. The constructor checks that the field has `rows·cols` entries and that the clamp distance is positive (`CHECK`; infinity is accepted and means no clamp). `distanceAt(col, row)` returns `min(field, clamp)` in metres and does not check the indices. `getData()` is the unclamped field. `setMaxDist` rejects a non-positive or NaN value. `buildEDTGrid(grid, edt_clamp = 5.0)` marks a cell as an obstacle when its raw value is `>= occupied_threshold` (UNKNOWN is not an obstacle), runs an exact Euclidean transform (OpenCV `distanceTransform`, `DIST_L2`, `DIST_MASK_PRECISE`) and scales by the resolution. A value is the distance from the cell centre to the nearest obstacle cell centre. With no obstacle, every cell holds the clamp value. `edt_clamp` must be positive.

**`MapSnapshot {grid_, edt_}`** bundles the two for one map version. `IMapProvider::snapshot()` returns `std::shared_ptr<const MapSnapshot>`. A snapshot is immutable; a new map is a new snapshot. `buildHybridAstar` keeps the snapshot alive through aliasing `shared_ptr`s held by the collision checker and the obstacle heuristic, so it lives as long as the planner. Cells, distance field and cost map are not updated in place: a changed snapshot needs a new planner. The primitive library is body-frame and is reused across snapshots.

## 3. Primitive library

`buildKinematicModel(body, kin, cell_size)` returns a `KinematicModel {motion, prim_lib, kin}`. Its `motion` uses a 1 x 1 stub grid at origin (0, 0); `buildHybridAstar` replaces it with a `MotionContext` built on the snapshot's metadata. `cell_size` must equal the snapshot resolution (checked in `buildHybridAstar`).

`DiffDrivePrimLib` builds everything in its constructor. It `CHECK`s `rmin`, `vmax`, `atomic_time > 0`, `n_headings >= 1`, and a positive resolution and `r_out`. The control set (`buildControlSet`) has 12 controls, all with `dt = atomic_time`: the straight `±vmax`, and for each `x` in `{0, 0.5, 1, 2, 4}·rmin` a left and a right arc with `v = vmax·x/(x + rmin)` and `|omega| = vmax/(x + rmin)`. `x = 0` is a pivot at `vmax/rmin`. See [primitives.md](../design/primitives.md).

A `Primitive` holds, for one control:

| Field | Content |
|---|---|
| `control` | The `(v, omega, dt)`. |
| `endpoint_offset` | Body-frame `propagate({0, 0, 0}, control)`. The same for every bin. |
| `cost` | `sqrt(v² + rmin²·omega²)·dt` in metres (`makeControlCost`), no map term. A pivot at `vmax/rmin` costs the same as a straight at `vmax`. |
| `swept_cells[k]` | Body-frame cells, sorted and unique, that cover the motion from any pose in the node's cell with heading in bin `k` (section 4). |
| `sample_cells[k]` | Body-frame cell of the robot centre at each `sampleTrajectory` pose, starting at the mid-bin heading `-π + (k + 0.5)·Δ`, floored with `worldToGrid` at origin (0, 0). |
| `sample_clearance` | One obstacle distance per sample, in metres, the same for every bin. |

For sample `i`, with `ρ_i` the distance of the sample from the start point and `σ = |v|·dt / n_s` the centre path length between consecutive samples (`n_s` is the number of `sampleTrajectory` intervals): `sample_clearance[i] = r_out + 2·ρ_i·sin(Δ/4) + 2·√2·h + σ/2`. Every entry is at least `r_out` (`CHECK`). The terms are explained in [swept-cover.md](../design/swept-cover.md).

`swept_cells[k]` is built by `buildCover` from `M = max(1, ceil((|v|·dt + r_out)·Δ/h))` heading samples `θ_k + m·Δ/M`, `m = 0..M`, and `n = max(1, ceil((|v| + r_out·|ω|)·dt/h))` time intervals (`n + 1` poses each). Each pose is rasterized with margin `m_θ + m_s`, with `m_θ = (|v|·dt + r_out)·Δ/(2M)` and `m_s = (|v| + r_out·|ω|)·dt/(2n)`; both are checked to be at most `h/2` (`CHECK`). Every resulting cell `(i, j)` is also added as `(i+1, j)`, `(i, j+1)` and `(i+1, j+1)`, for the position of the robot inside its cell. `buildTimeMs()` is the wall-clock time of the whole build.

`MotionContext::sampleTrajectory(start, u)` returns equally spaced poses in time, `start` and the end point included, with `n` intervals: for `omega == 0`, `ceil(|v|·dt / (r_in/2))`; for `v == 0`, `ceil(|omega|·dt / (r_in/(2·r_out)))`; otherwise the largest of those two and 2; at least 1.

## 4. Collision checking

`CollisionChecker(grid, edt, params)` answers against one snapshot. OCCUPIED cells collide. UNKNOWN cells and cells outside the grid collide only where `reject_unknown` is set (`checkCell`, `checkCells`).

**The swept-cell cache is a cover.** `swept_cells[k]` contains every cell the footprint can touch while executing the primitive from any heading in bin `k` and any position in the node's cell (floor snapping for both). So a clear scan proves the motion clear. An occupied cell in the cover does not prove a collision. This is checked by `swept_cover_property_test`; the construction and the bound are in [swept-cover.md](../design/swept-cover.md).

`checkTrajectory(swept_body, samples_body, sample_clearance, robot_cell)` returns a `TrajectoryVerdict`. Body-frame cells are added to `robot_cell`. The steps, in order:

1. **Distance-field pass.** Samples are taken in order. With `d` the distance-field value at the sample cell and `c` its `sample_clearance`: if `d + (c − r_out) < r_in` the verdict is `FAST_REJECT`, and the motion collides. If `d > c` the sample is clear and the next one is taken. A sample outside the grid, or one that is neither clear nor rejecting, ends the pass and goes to step 2 (later samples are not looked at). If every sample is clear and there was at least one, the verdict is `FAST_ACCEPT`: the motion is collision-free.
2. **Cover scan.** Every cell of `swept_body` is checked for OCCUPIED. None: `SCAN_CLEAR`, collision-free when `swept_body` is a cover. One occupied: `SCAN_BLOCKED`.
3. **Exact check.** Not part of `checkTrajectory`. The search runs it on a `SCAN_BLOCKED` motion (section 5).

`sample_clearance.size()` must equal `samples_body.size()` and every entry must be at least `r_out`; otherwise a `CHECK` throws, entries being checked in order up to the first sample that ends the pass. The distance field must have a clamp above the clearances it is compared with: `buildHybridAstar` requires `edt_max_dist > max(r_out + √2·h, largest sample_clearance)`.

**Single pose.** `fastAcceptReject(cell)`: `VALID` iff `d > r_out + √2·h`; `COLLISION` iff `d + √2/2·h < r_in`; otherwise `UNCERTAIN`; a cell outside the grid is `UNCERTAIN`. `checkState(pose)` uses it, then for `UNCERTAIN` rasterizes the footprint (margin 0) and fails on any OCCUPIED cell; UNKNOWN and outside cells pass. `checkCell` and `checkCells` are plain per-cell scans with no distance field. **Exact cover.** `MotionContext::computeExactCover(start, u)` returns the map-frame cells the footprint can touch while executing `u` from the continuous pose `start`, sorted and unique, with the sampled poses and their centre cells. It needs `u.dt` finite and `>= 0` (`CHECK`). With `reach = (|v| + r_out·|ω|)·dt` and `n = max(1, ceil(reach/h))`, the cells are the union of the footprint at the `n + 1` poses `dt·j/n` with margin `reach/(2n)`, and the footprint at every `sampleTrajectory` pose with margin 0. `visitExactCover(start, u, visit)` walks the same set without building a list, may visit a cell twice, and stops when `visit` returns false. `HybridAstar::exactMotionClear` calls it with `checkCell`. No collision-checker function takes an exempt set.

## 5. Search

`HybridAstar` (`core/planner/hybrid_astar/search.hpp`) is built by `buildHybridAstar(kin_model, cfg, heur_cfg, cost_policy, snapshot)`. A call to `plan` resets the open set, closed set, node pool and counters, so one instance serves many queries.

**`plan` order.** Down to the loop, the first match wins. Inside the loop, each iteration tests the loop condition first (open set not empty and `max_expansions` not reached); a failed condition ends the search with `NoPath` (open set empty) or `BudgetExhausted` (cap), even when `stop` is pending or the deadline has passed. The `stop` and deadline rows apply only on iterations that pass the condition.

| Step | Outcome |
|---|---|
| `stop` already requested | `Stopped` |
| `!startClear(start)` | `StartBlocked` |
| `!checkState(goal)` | `GoalBlocked` |
| `heuristic->prepare(goal)`; deadline passed | `BudgetExhausted` |
| `stop` requested | `Stopped` |
| loop: `stop` requested (checked on the first iteration and every 1024th) | `Stopped` |
| loop: deadline passed (same cadence) | `BudgetExhausted` |
| loop: popped node meets the goal, or the goal shot passes | `Found` |
| open set empty | `NoPath` |
| `max_expansions` nodes expanded | `BudgetExhausted` |

The deadline is `max_plan_time_sec` from the entry to `plan`, steady clock, and covers the heuristic preparation. A call can run slightly past it. `max_expansions` counts nodes expanded; a stale or exactly rejected pop does not count. `lastPlanStats()` is zero for the first three outcomes.

**Open order.** Ascending `f = g + h`, then ascending `h`, then ascending node index (`OpenEntryGreater`). With distinct indices this is a strict total order, so the pop order depends on the entries' values only. **Pop.** In order:

1. Skip a stale entry: the closed key exists with `g <= curr.g`. A node that meets the goal is never stale.
2. If the entry is unconfirmed (`OpenEntry::pending_exact`), run `exactMotionClear(parent state, primitive control, false)`. A rejected node is dropped: no closed-set entry, no expansion count, no observer event (`exact_reject`).
3. Insert the node in the closed set (a goal node does not overwrite a cheaper entry), count the expansion, and call the observer.
4. Goal test, then the goal shot, then generation.

**Goal test.** `atGoal`: `dx² + dy² <= goal_pos_tol²` and `|wrapToPi(goal.theta − theta)| <= goal_heading_tol`, both bounds inclusive. A start that already satisfies it returns `Found` with a path of no segments. **Generation.** For each primitive of the expanded node, in this order:

1. Propagate; `g_new = g + costmap_weight·mean(costmap over sample_cells[k] translated by the node cell)·|v|·dt + primitive.cost`. A pivot has `|v| = 0` and so no clearance term. See [cost-and-heuristic.md](../design/cost-and-heuristic.md).
2. Duplicate test: if the closed set holds the child's key with `g <= g_new`, drop it (`duplicate_skipped`). A child that meets the goal is never dropped.
3. `checkTrajectory` with the node's bin `k` and cell. `FAST_REJECT` is final. `SCAN_BLOCKED` is pushed unconfirmed, because the cover does not prove a collision. `FAST_ACCEPT` (distance field) and `SCAN_CLEAR` (cover) are final accepts; neither is checked again.
4. `h_new = heuristic(child)`; push with `f = g_new + h_new`.

**Closed set.** The key (`packClosedKey`) packs `(i, j, k)` into 64 bits: `i` and `j` 24 bits each with an offset of `2^23`, `k` 16 bits. The supported range is `i, j` in `[-2^23, 2^23)` and `k` in `[0, 65536)`; outside it the key is not checked in an `NDEBUG` build and trips an `assert` in a debug build. The key uses `i' = floorDiv(i, stride)` and `j' = floorDiv(j, stride)`, with `floorDiv` rounding toward negative infinity (`HybridAstar::keyOf`), and the heading bin of the state (`headingBin`). `stride = HybridAstar::closedStride(closed_cell_size, h) = max(1, lround(closed_cell_size / h))` map cells, so the dedup cell is `stride·h` metres. The key is for pruning only; the node keeps its map-resolution cell for collision and cost. `buildHybridAstar` throws `std::invalid_argument` (`CHECK`) unless:

- the kinematic model cell size equals the snapshot resolution within `1e-6`;
- `vmax·atomic_time > √2·stride·h`;
- the distance-field clamp exceeds `max(r_out + √2·h, largest sample_clearance)`;
- `cost_policy.clearance_radius_ > r_out`.

No build-time check covers arcs and pivots against the dedup cell.

**Goal shot.** `shotAtGoal(current)` runs when `enable_shot` is set and the node is within `shot_max_distance` of the goal. The shot is a pivot to the bearing of the goal, a straight at `vmax`, and a pivot to the goal heading, each omitted when below `1e-9` (m or rad). First a pre-test on the `sampleTrajectory` centre cells of the straight: it fails on `fastAcceptReject == COLLISION` or on `!checkCell(cell, reject_unknown)`. Then `exactMotionClear` on each segment, in ascending `reach`. `reject_unknown` is `!shot_allow_unknown`, so UNKNOWN blocks unless configured. The path is built only if every segment passes. A returned path is the node chain followed by the shot segments.

**Path output.** `reconstructPath` walks `parent_idx` from the goal node, resamples every segment with `sampleTrajectory` from the continuous parent state, and sets each `seg.cost = child.g − parent.g`. `Path` has `node_points.size() == segments.size() + 1`, the first being the start pose, and `total_cost` the sum of segment costs. **Counters.** `PlanStats` counts `expansions`, `fast_accept`, `fast_reject`, `scan_clear`, `scan_blocked`, `duplicate_skipped`, `exact_accept`, `exact_reject`. For a plan with `P` primitives, `fast_accept + fast_reject + scan_clear + scan_blocked + duplicate_skipped == P·(expansions − (path returned by the loop ? 1 : 0))`, and `exact_accept + exact_reject <= scan_blocked`. They are reset at every `plan` and reading them does not change the result. **Observers and stop tokens.** `plan(start, goal, observer, stop)` calls `observer->onExpand(SearchExpansion)` once per node taken for expansion, in pop order, after step 3 of the pop (so the goal node is reported). `id` and `parent_id` are opaque handles. A null observer reports nothing. The observer is held for the call only. Attaching one does not change the path or the counters (`observerDoesNotChangePathOrStats`). A `stop_token` ends the search with `Stopped` at the checks in the table above and changes nothing else.

## 6. Heuristics

`Heuristic` has two phases. `prepare(goal)` binds the goal and is called once per `plan`, before the loop. `operator()(state)` returns the weighted estimate and is called per generated node. Each heuristic applies its own weight, so `f = g + h` needs no further scaling. `prepare` is `const` and the heuristics keep mutable state: one object is not safe to call from two threads.

| Heuristic | Estimate |
|---|---|
| `DistanceHeuristic` | `weight·sqrt(dx² + dy² + rmin²·dθ²)`, with `dθ` the wrapped heading difference (`PointsWeightedEuclideanSE2`). |
| `ObstacleHeuristic` | `weight` times the least cost from the state's cell to the goal cell over the grid. |
| `MaxHeuristic` | The larger of its children. `buildHybridAstar` uses `max(DistanceHeuristic, ObstacleHeuristic)` when `use_obstacle_heuristic` is set, else `DistanceHeuristic` alone. |

**`ObstacleHeuristic`.** The field is an 8-connected shortest-path cost from the goal cell. A step costs `h` (axis) or `√2·h` (diagonal) times `1 + costmap_weight·cost` of the cell the step reaches when expanding from the goal, which is the cell the path leaves when driven toward the goal. OCCUPIED cells are impassable. UNKNOWN cells are passable at their cost-map cost. The robot is a point and heading is ignored. `costmap_weight` must be the search's `AStarConfig::costmap_weight`, not `HeuristicConfig::weight`. `prepare` only selects the goal cell: the field is grown on demand as cells are queried, and a value does not depend on the order of queries. A `prepare` whose goal is in the same cell as the previous call keeps the field; a new goal cell discards it.

| Case | `operator()` returns |
|---|---|
| Goal off the grid or on an OCCUPIED cell | 0 for every state |
| State off the grid | 0 |
| State on an OCCUPIED cell | `weight·UNREACHABLE` |
| State in a cell with no route to the goal | `weight·UNREACHABLE` |
| Otherwise | `weight` times the routed cost |

`fieldAt(cell)` gives the unweighted cost, and `UNREACHABLE` for an off-grid cell, an obstacle, an unrooted field or a cell with no route. `UNREACHABLE` is `3.4028234663852886e38`, finite so that `g + h` stays finite.

The planner makes no admissibility or optimality claim for either estimate or for `HeuristicConfig::weight`. The limits are in [cost-and-heuristic.md](../design/cost-and-heuristic.md).

## 7. Start and goal gates, and the escape move

Both gates are hard. `plan` returns `StartBlocked` unless `CollisionChecker::startClear(start)` holds, and `GoalBlocked` unless `checkState(goal)` holds.

`startClear(start, extra_margin = 0)`: with `m = (√2/2 + 1/2)·h + extra_margin`, an OCCUPIED cell centre within distance `m` of the footprint at `start` always gives false. A false result always has an OCCUPIED cell centre within `m` outside every footprint edge line. A centre next to a corner, beyond `m` from the footprint but inside every edge line pushed out by `m`, may give either answer. UNKNOWN and out-of-grid cells never fail it. `extra_margin` is finite and `>= 0` (`CHECK`). A pose that passes also passes `checkState`, and a larger margin can only turn true into false. The cells come from `visitPolygonCells` with margin `h/2 + extra_margin`. Enforced by `startClearBracketsItsGuarantees`. Why this distance makes the first pose of every primitive clear is in [swept-cover.md](../design/swept-cover.md).

**Escape move.** `findEscape(start, checker, motion, kin, config, stop_short)` (`core/planner/escape/escape_search.hpp`) is not a search. It is one straight move at constant heading, reverse or forward, in steps of `δ = step_fraction·h`, from a start that fails the gate to a pose that passes it. All poses are in the map frame; every cell test takes resolution and origin from `checker.gridMeta()`, and `motion` supplies the footprint and kinematics only.

- `EscapeConfig`: `step_fraction` in (0, 1), `slack_cells >= 0`, `max_travel_factor` finite and `> 0`; `stop_short` finite and `>= 0` (metres). All `CHECK`ed. Per direction at most `floor(max_travel_factor·r_out/δ)` steps are tried.
- A start that already passes `startClear(start, 0)` returns a path of no segments and the single node `start`.
- A step from `parent` to `child` is allowed iff every OCCUPIED cell in contact at the child is in contact at the parent and its penetration did not grow (`pen(child) <= pen(parent) + 1e-9`). Contact uses margin `δ/2`: for edge `(n_e, d_e)` and cell centre `c`, `pen_e = (|n_e.x| + |n_e.y|)·h/2 − (n_e·c − d_e)`, `pen = min_e pen_e`, and the cell is in contact iff every `pen_e >= −δ/2` and `c` lies within `h/2 + δ/2` of the footprint's bounding box on both axes. UNKNOWN and out-of-grid cells never block.
- Per direction the walk stops at the first pose `k` such that `startClear(pose, 0)` holds for the poses `k − e .. k`, with `e = ceil(stop_short/δ − 1e-9)`, and `startClear(pose k − e, slack_cells·h)` holds. If the walk is blocked or capped first, the first `k` that satisfies the `slack = 0` form is used.
- Choice between directions: one that reached the slack form, then the one with fewer steps, then reverse. Neither: `std::nullopt`.
- Result: one segment `{±vmax, 0, k·δ/vmax}`, cost `sqrt(v² + rmin²·omega²)·dt`, end pose heading wrapped.

## 8. Navigation mode and navigation checks

`NavigationMode` (`core/navigation/navigation_mode.hpp`) is the state logic that decides when to plan, follow, escape, stop or give up. It owns no clock, threads or I/O and holds no paths. `step(event)` returns the actions to carry out, in order. The caller carries a path alongside the event and action it belongs to. Event stamps (s) must not decrease; `ForwardClock::advance(raw)` makes a source clock that can step backward usable: it returns the sum of the forward differences, `0` on the first call, and a backward step adds zero (readings 0, 1, 2, −298, −297 return 0, 1, 2, 2, 3). The constructor `CHECK`s `NavigationConfig`: `stop_distance`, `stall_timeout`, `blocked_retry_period`, `plan_reply_timeout > 0`, `max_clearing_attempts >= 1`, `max_stall_replans >= 0`. The ROS side is in [ros.md](ros.md); the transitions with their reasons are in [navigation.md](../design/navigation.md).

| Event | Meaning |
|---|---|
| `GoalReceived` | New goal. In any state: cancel an outstanding request, request a plan, `Planning`. Resets the clearing and stall counts. |
| `PlanReply{request_id, outcome}` | Answer to `RequestPlan`; a reply with another id is ignored. |
| `EscapeResult{found}` | The escape search finished. |
| `MapChanged{path_valid, blocked_distance}` | The driven path was re-checked. |
| `Progress{plan_id, cursor, arrived}` | From the follower, for the driven plan. |
| `Tick` | Time passes. |

| Action | Meaning |
|---|---|
| `RequestPlan{request_id}` | Ask the planner; the reply carries the id. |
| `CancelPlan` | Cancel the request. |
| `RunEscape` | Run `findEscape` from the current pose. |
| `PublishPlan{plan_id, kind}` | Publish the path that came with the event, `Goal` or `Escape`. |
| `PublishStop` | Publish the empty plan. |
| `Report{state, reason}` | The state or reason changed. |

States: `Idle`, `Planning`, `Following`, `Clearing`, `Blocked` with `BlockedReason` `NoPathToGoal`, `NoEscape` or `Stalled`. Rules, as `NavigationMode::step` implements them:

- **Planning.** `Found`: publish, `Following`. `StartBlocked`: stop; if fewer than `max_clearing_attempts` escape moves have been published for this goal, `RunEscape` and `Clearing`, else `Blocked(NoEscape)`. `Stopped`: stop, `Idle`. `GoalBlocked`, `NoPath`, `BudgetExhausted`: stop, `Blocked(NoPathToGoal)`. A request with no reply after `plan_reply_timeout` (checked on `Tick`) is cancelled, the robot stopped, `Blocked(NoPathToGoal)`; a late reply is ignored. While a driven path remains, a `MapChanged` with `path_valid == false` and `blocked_distance < stop_distance` stops the robot and sends no second request. Arrival on the driven path of the current goal cancels the request and goes `Idle`.
- **Following.** `Progress` with `arrived`: `Idle`. A changed `cursor` restarts the stall timer and nothing else. `MapChanged` with an invalid path: stop first if `blocked_distance < stop_distance`, then request, `Planning`. `Tick` with no cursor movement for `stall_timeout`: stop; if `max_stall_replans` replans have been used for this goal, `Blocked(Stalled)`, else request, `Planning`. The stall count resets only on a new goal and on arrival.
- **Clearing.** `EscapeResult{found}`: publish the escape and count it; not found: `Blocked(NoEscape)`; the count resets on a new goal and on a found plan, and a search that finds nothing does not count. Arrival: request, `Planning`. No cursor movement for `stall_timeout`: stop, `Blocked(NoEscape)`.
- **Blocked.** On `Tick` or `MapChanged`, once `blocked_retry_period` has passed since the last request and a goal stands: request, `Planning`. `Blocked(Stalled)` is not retried; only a new goal leaves it.

**Navigation checks** (`core/navigation/navigation_checks.hpp`). `checkRemainingPath(samples, robot_x, robot_y, cursor_prev, checker)` returns `{valid, blocked_distance, cursor}`. The cursor is the sample at or after `cursor_prev` nearest to the robot in position (lowest index on a tie; heading ignored). Samples from the cursor on are tested with `checkState` in order, and `blocked_distance` is the path length (m) from the cursor sample to the first blocked sample. An empty path is `{true, 0, 0}`; `cursor_prev >= samples.size()` throws (`CHECK`). `atRest(a, b, position_tolerance, heading_tolerance)` is true iff the Euclidean distance is at most the position tolerance (m) and `|wrapToPi(b.theta − a.theta)|` at most the heading tolerance (rad), bounds inclusive; both tolerances finite and `>= 0` (`CHECK`).

## 9. Planner interface

`Planner` (`core/planner/planner_interface.hpp`) is the calling contract. `plan(start, goal)` takes poses in the map frame (m, m, rad) and returns `PlanResult {PlanOutcome outcome; std::optional<Path> path;}`. `path` is set if and only if `outcome == Found`. The second overload, `plan(start, goal, std::shared_ptr<SearchObserver>, std::stop_token = {})`, has the same results. The base class ignores the observer and token and calls the two-argument form; `HybridAstar` handles both.

| `PlanOutcome` | Meaning |
|---|---|
| `Found` | A path was found. A zero-length path (the start meets the goal) is `Found`. |
| `StartBlocked` | The start fails `startClear`. |
| `GoalBlocked` | The goal pose fails `checkState`. |
| `NoPath` | The open set ran out before the goal was reached. |
| `BudgetExhausted` | `max_expansions` or `max_plan_time_sec` was reached. |
| `Stopped` | The stop token was set. A planner that ignores the token never returns it. |

`Path` holds `segments` (a `Control`, its `samples_poses`, its `cost`), `node_points` (the pose at each segment boundary) and `total_cost`. `buildHybridAstar` returns a non-null `std::unique_ptr<Planner>` or throws `std::invalid_argument`; it builds a new planner for each snapshot.

## 10. Path following

`PurePursuit` (`core/control/pure_pursuit.hpp`) is the follower law. It has no ROS, TF or I/O. `ControllerNode` ([ros.md](ros.md)) calls it; the geometry and the reasons are in [navigation.md](../design/navigation.md).

**Inputs.** `PurePursuit(params, trajectory, dt, speed_scale = 1)` takes `TrajectoryPoint {SE2State pose; MotionMode mode; double speed;}` in order from start to goal, and `step(pose)` takes the robot pose in the same frame (m, m, rad); the node uses `odom_frame`. `mode` is `Forward`, `Reverse` or `Pivot`. `speed` is not used by the law: the direction comes from `mode`, the magnitude from the rules below. `dt` is copied into the returned `Control`.

**Constructor `CHECK`s** (throw `std::invalid_argument`). Every `PurePursuitParams` field (`lookahead`, `v_cruise`, `a_lat_max`, `omega_pivot`, `goal_pos_tol`, `goal_heading_tol`) is `> 0`. `dt > 0`. The trajectory is non-empty. `speed_scale` is finite and in `(0, 1]`. `loadControllerParams` checks the six `controller.yaml` keys (`lookahead_gain` in place of `lookahead`) are `> 0`. `ControllerNode` sets `L = lookahead_gain·(r_in/2)` and fails unless `r_in/2 < L < 2·rmin` and `v_cruise <= vmax`; it also needs `ctrl_rate > 0`.

**Cursor.** One index, never decreasing until `reset()`. Leading `Pivot` points whose heading is within `goal_heading_tol` are skipped, as are leading drive points within 1e-3 m of the robot (never onto the last point). Then it moves to the nearest point in position at or after itself.

**Drive.** The carrot is the first point from the cursor at least `L` from the robot (Euclidean), else the last point. It is expressed in the travel frame, which is the body frame for `Forward` and the body frame turned by π for `Reverse`. With carrot `(x_c, y_c)` there, `κ = 2·y_c/(x_c² + y_c²)`. The speed magnitude is `min(speed_scale·v_cruise, sqrt(a_lat_max/|κ|), ramp)` and the command is `v = ±speed`, `ω = κ·speed` (sign of `v` by mode; `ω` is not clamped to `omega_pivot`). `ramp = speed_scale·v_cruise·remaining/L` when `remaining < L`, where `remaining` is the distance from the robot to the point after the cursor plus the path length beyond it. A carrot at the robot position gives zero.

**Pivot points.** `v = 0`, `ω = ±speed_scale·omega_pivot` toward the point's heading by the shorter way.

**Path end.** The goal position test is `|robot − last pose| <= goal_pos_tol`. If the carrot is the last point, the last point has the same mode as the current one, and it lies beside or behind the robot (`x_c <= 0`) while outside the tolerance, the command is zero. The follower does not turn back; the navigator's stall rule ([section 8](#8-navigation-mode-and-navigation-checks)) handles it.

**Final alignment.** Inside the position tolerance, with `e` the signed shortest heading error to the last pose: `|e| < goal_heading_tol` latches `done()` and returns zero. Otherwise `ω = clamp(k·e, ±speed_scale·omega_pivot)` with `k = omega_pivot/(2.5·goal_heading_tol)`. A latched follower returns zero until the robot is inside the position tolerance with `|e| > 1.5·goal_heading_tol`; leaving the position tolerance does not reopen it.

**`goalAlignTimeBound(omega_pivot, goal_heading_tol)`** (static) is the time in s the alignment needs to bring a π error inside the tolerance at speed scale 1, for a robot that turns at exactly the commanded rate. Arguments finite, `omega_pivot > 0`, `0 < goal_heading_tol < π` (`CHECK`). `Navigator` adds it to the time to creep the last lookahead and requires `stall_timeout` to exceed the sum (`CHECK`), so a final turn is not taken for a stall.

**Stop zone.** `motionBlocked(robot, points_body, v, omega, reaction_time, StopZoneParams)` (`core/control/stop_zone.hpp`) is true if any scan return (body frame, m) lies in the area the footprint sweeps while stopping from `(v, omega)`. `StopZoneParams` holds `linear_decel`, `angular_decel` (`> 0`), `forward_pad` (`>= 0`) and `side_pad` (`> 0`). Non-finite arguments, a negative `reaction_time` and out-of-range parameters throw. It returns false for `v = ω = 0` or no points. The scan guard calls it; geometry and bounds are in [safety-guard.md](../design/safety-guard.md).
