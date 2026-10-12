# Scenario guide

A scenario is one file that holds everything the planner needs for a single run: the grid, the occupancy (a grayscale map or inline rectangles), the robot footprint, the differential-drive limits, the search and cost settings, and a start and goal. It can also carry an `expectations` block that the tests check. `loadScenario` and `buildScenario` (`core/scenario/`) read it. The visualiser and the integration tests both run on it.

Every command on this page runs from `src/path_planner`.

There are two places for scenarios, and they are separate trees:

- `scenarios/` is the local studio workspace, with one folder per map. Git ignores it, because maps are large binaries and the outputs are scratch. `scenario_studio.py` writes and runs here.
- `test/integration/scenarios/` is the test corpus. It is tracked in git. Put a scenario here when you want the build to assert on it.

## Tooling environment

Run the Python tools with the system interpreter. A conda Python often lacks `numpy`, `cv2` and `matplotlib`:

```bash
/usr/bin/python3 tools/viz/scenario_studio.py ...
```

`search_viz_dump` is built only with `-DBUILD_TOOLS=ON`. `scenario_studio.py run` looks for the binary under `build-gcc12/` and `build/` inside `src/path_planner`. A `colcon` build puts it elsewhere (`../../build/planner/tools/search_viz_dump` on a host, `/ws/build/planner/tools/search_viz_dump` in the container), so pass `--bin <path>`.

## Bundle layout

```
scenarios/
  <name>/                 one folder per map
    <name>.pgm            the map (input)
    <name>.yaml           the scenario: grid, image: <name>.pgm, robot, diff_drive,
                          heuristic, cost_policy, start, goal
    output/               everything generated: preview PNG, trace JSON, path PNG
```

A scenario file and its map live together. The file names its map relative to its own folder (`image: <name>.pgm`), and the loader resolves that path against the folder of the YAML file, not the working directory. A bundle works from anywhere. Everything generated goes to that bundle's `output/`.

## scenario_studio.py

Map commands take a map image. Scenario commands take a scenario file; the map is read from `input.image.path`.

```bash
# new: copy a map into a fresh bundle, click a start and a goal, write the YAML, run and show it.
python3 tools/viz/scenario_studio.py new path/to/foo.pgm [--name NAME] [--bin PATH]

# pick: click a new start and goal for an existing scenario. The file is patched in place.
#   --out writes the result to a new copy instead; --run runs it afterwards.
python3 tools/viz/scenario_studio.py pick scenarios/depot/depot.yaml [--out NEW.yaml] [--run] [--bin PATH]

# run: run search_viz_dump on a scenario and render the trace into the bundle's output/.
python3 tools/viz/scenario_studio.py run scenarios/depot/depot.yaml [--bin PATH]

# preview: decode a map without a window; print a histogram, the OCCUPIED/FREE/UNKNOWN counts and a
#   point-robot connectivity check; write a colour preview PNG.
python3 tools/viz/scenario_studio.py preview scenarios/depot/depot.pgm [--out PNG]
       [--occupied-thresh F] [--free-thresh F]
```

`new` takes the grid resolution, origin and thresholds from a nav2 `<map>.yaml` next to the map. Without one it uses 0.05 m per cell, origin `[0, 0]`, and thresholds 0.65 and 0.196. It writes a 0.2 m square robot, `rmin: 0.5`, `vmax: 1.0`, `atomic_time: 0.3`, `n_headings: 16`, and `expect_found: true`. For a map with more than 1,000,000 cells it also writes an `a_star` block with a larger budget (30 s, 800000 expansions). To pick a pose you click for the position and drag for the heading, start first, then goal.

`pick` needs the scenario to give its map as `image:` with a `path:` key, as in the schema below. `preview` writes `<map stem>_decoded.png` into the `output/` folder next to the map unless you pass `--out`; without `--occupied-thresh` and `--free-thresh` it reads the thresholds from the nav2 file next to the map, or uses the defaults.

`run` calls `search_viz_dump`, which writes `output/<scenario file stem>.json`, then calls `plot_search.py` to draw `output/<scenario file stem>.png` beside it. The plot uses the TkAgg window when a display exists and falls back to saving only when it does not.

A typical first pass is `new <map.pgm>` to create and run a bundle, then `pick <scenario.yaml> --run` to try other poses.

## Map image format

The `image:` is a grayscale PNG or PGM, one pixel per cell. Standard nav2 map sets work: use the `<map>.pgm` and copy the resolution and origin from its `<map>.yaml`. Each pixel is turned into a darkness fraction:

```
p = (255 - px) / 255          darker pixel, more occupied
p >  occupied_thresh  ->  OCCUPIED
p <  free_thresh      ->  FREE
otherwise             ->  UNKNOWN
```

`occupied_thresh` defaults to 0.65 and `free_thresh` to 0.196. Both must be in `[0, 1]`, and `free_thresh` must be below `occupied_thresh`. The image is flipped vertically on load: its top row becomes the grid's highest row, and row 0 is the bottom. The image width and height must equal `input.grid.cols` and `input.grid.rows`.

`input.image` cannot be combined with the inline `input.obstacles` or `input.unknown` lists. Use one occupancy source.

## Scenario YAML schema

There are three top-level keys: `name`, `input` and `expectations`. `name` is optional and defaults to the file name without its extension. `input` holds the planner inputs. `expectations` is the test oracle, and the visualiser ignores it. `test/integration/scenarios/straight_corridor.yaml` is the annotated reference file.

```yaml
name: my_scenario

input:
  grid:
    cols: 604              # cells in x; required; must equal the image width
    rows: 307              # cells in y; required; must equal the image height
    resolution: 0.05       # m per cell; required; positive
    origin: [0.0, 0.0]     # world position of cell (0, 0), bottom-left; optional, default [0, 0]

  image:                   # use this, or obstacles / unknown below, not both
    path: my_scenario.pgm  # relative to this YAML file, or absolute
    occupied_thresh: 0.65  # optional
    free_thresh: 0.196     # optional

  robot:
    vertices: [[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]   # convex, counter-clockwise, contains the origin

  diff_drive:
    rmin: 0.5              # required; minimum turning radius, m
    vmax: 1.0              # required; m/s
    atomic_time: 0.3       # optional, default 0.6; seconds per primitive
    n_headings: 16         # optional, default 16

  a_star:                  # the whole block is optional; each key falls back to its default
    max_plan_time_sec: 30.0   # default 5.0
    max_expansions: 800000    # default 200000
    node_pool_reserve: 800000    # default 200000
    closed_set_reserve: 800000   # default 200000
    # also: goal_pos_tol (0.10), goal_heading_tol (0.10), enable_shot (true),
    # shot_max_distance (5.0), shot_allow_unknown (false), closed_cell_size (0.15),
    # costmap_weight (1.0)

  heuristic:               # optional
    weight: 1.0            # positive; 1.0 is admissible
    use_obstacle_heuristic: true

  cost_policy:             # required, and all four keys are required
    clearance_radius: 0.5      # non-negative
    clearance_decay: 15.0      # non-negative
    max_clearance_cost: 30.0   # non-negative
    unknown_cost_factor: 1.0   # at least 1

  edt_clamp: 5.0           # optional, default 5.0; positive; distance-field limit in metres

  start: [0, 0, 0]         # world frame [x, y, theta]
  goal:  [0, 0, 0]

expectations:              # optional; read by the tests only
  expect_found: true       # default false when the block or the key is missing
  cost_max: 12.0           # optional
  expansions_max: 50000    # optional
  unknown_poses_max: 0     # optional; at least 0
```

Without an image, the grid starts all free and the loader paints rectangles onto it:

```yaml
  obstacles:
    - rect: [64, 40, 95, 79]   # [c0, r0, c1, r1], inclusive cell indices (column, row)
      value: 100               # optional, default 100 (occupied)
  unknown:
    - rect: [8, 8, 24, 24]     # value defaults to -1 (unknown)
```

Row 0 is the bottom of the grid. A rectangle with its corners in the wrong order is swapped, and one that runs past the border is cut to the grid. Obstacles are painted first and unknown rectangles after, so an unknown rectangle overwrites an obstacle where they overlap. A `value` is the raw cell value: -1 is unknown, 0 to 49 is free, 50 to 100 is occupied.

Always write the image as a mapping with a `path:` key, as above. `scenario_studio.py pick` needs it.

The meaning of the `expectations` keys:

- `expect_found`: whether a path should exist. A scenario that leaves it out expects no path.
- `cost_max`: an upper limit on the total path cost.
- `expansions_max`: an upper limit on the size of the node pool, the states generated during the search.
- `unknown_poses_max`: an upper limit on the sampled path poses that fall in unknown cells or outside the grid.

Set these from the geometry you drew and what you intend, not from what the planner printed. A bound copied from the planner's own output proves nothing.

`loadScenario` checks the file and throws `std::invalid_argument` on a problem. The message ends with `<file>:<field> <why>` and begins with the source location of the check, so it points at the field. `buildScenario` runs a second round of checks. It rejects a footprint that is not convex, not counter-clockwise or does not contain the origin. It also rejects a setup where the straight primitive, `vmax * atomic_time`, is not longer than `sqrt(2)` times the closed-set cell. That cell is `closed_cell_size` rounded to a whole number of map cells (at least one), times the resolution. The default is 0.15 m, so with a 0.05 m grid the primitive must be longer than about 0.212 m.

## Promoting a scenario into the test corpus

The scenario test reads every `.yaml` and `.yml` file directly inside `test/integration/scenarios/` each time it runs. Sub-folders are not read. A new file is picked up with no CMake change and no rebuild, and it becomes one test case named after the file name (non-alphanumeric characters become `_`). To promote a studio scenario:

1. Copy the scenario YAML into `test/integration/scenarios/`. Keep it small and deterministic. Prefer inline `obstacles` and `unknown` rectangles to a large map file.
2. If it has to stay image-backed, copy the map next to it and point `image:` at that copy.
3. Add `expectations` so the case checks something: `expect_found`, and `cost_max` or `expansions_max` if you want regression limits.
4. Track the files:

   ```bash
   git add test/integration/scenarios/my_scenario.yaml
   git add test/integration/scenarios/my_scenario.pgm   # only if image-backed
   ```

The trace files that `search_viz_dump` writes never feed the tests. The scenario test loads each YAML itself and plans in-process. To run it, see [testing](testing.md).
