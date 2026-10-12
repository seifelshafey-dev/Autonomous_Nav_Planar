# Planner tools

Command-line tools for looking at what the planner does, timing it and redrawing its figures, with no ROS needed. Run every command here from `src/path_planner` after a build with `-DBUILD_TOOLS=ON`. Inside the container the binaries are under `/ws/build/planner/tools/`.

Python tools need numpy, matplotlib and OpenCV (`cv2`).

## Render a search

`search_viz_dump <scenario.yaml> [out.json]` runs the planner on a scenario and writes the search as a JSON file with a binary side-car. `plot_search.py` draws it: the map, the states the search expanded, the path and the robot footprint along it.

```bash
/ws/build/planner/tools/search_viz_dump test/integration/scenarios/known_door_preferred.yaml
python3 tools/viz/plot_search.py tools/viz/output/known_door_preferred.json
```

The first command writes `tools/viz/output/known_door_preferred.json`; the second saves `known_door_preferred.png` beside it. Use the default output path: with an explicit `out.json`, the folder must already exist.

`plot_search.py` options:

- `--save <png>`: write the static picture somewhere else.
- `--animate`: replay the search in the order states were expanded.
- `--save-anim <file>`: save the animation; `.mp4` uses ffmpeg if it is installed and falls back to a GIF, any other extension writes a GIF.
- `--hold-frames <n>`: extra frames that repeat the final picture, path included, at the end of the animation (default 50, which is 2 s at 25 fps); 0 disables. A saved GIF that loops then rests on the finished search before it restarts.
- `--frames <n>`: animation frames (default 150).
- `--max-anim-nodes <n>`: cap on the states drawn in the animation (default 6000; 0 means no cap).
- `--no-tree`, `--no-costmap`: leave out the parent links or the costmap heatmap.

## Build and run scenarios

`scenario_studio.py` turns a map image into a scenario, lets you click the start and goal, runs the search and shows the result. The scenario file format is in the [scenario guide](../../../../docs/guide/scenarios.md).

```bash
python3 tools/viz/scenario_studio.py new <map.pgm>
```

| Command | What it does |
|---|---|
| `new <map>` | Makes a scenario folder from a grayscale map image, asks for start and goal, writes the YAML and runs it. |
| `pick <scenario.yaml>` | Picks a new start and goal for an existing scenario. `--out` writes a copy; `--run` runs it afterwards. |
| `run <scenario.yaml>` | Runs `search_viz_dump` on the scenario and renders the result into the scenario's `output/` folder. |
| `preview <map>` | Decodes a map image without a window and writes a colour preview with cell counts. |

`--bin <path>` on `new`, `pick --run` and `run` names the `search_viz_dump` binary to use.

## Compare a plan with the driven path

`plot_plan_vs_executed.py` draws a planned path and the trajectory the robot actually drove on the scenario map, so tracking error shows as the gap between the two lines.

```bash
python3 tools/viz/plot_plan_vs_executed.py trace.json planned.json executed.csv --save out.png
```

`trace.json` is a search trace from `search_viz_dump` (used for the map), `planned.json` is a list of `{x, y}` points, `executed.csv` has an `x,y` header. `--save` is required.

## Time the planner

`plan_benchmark` times the planner on a scenario and prints one result line per setting, after a few header lines with the configuration in use.

```bash
/ws/build/planner/tools/plan_benchmark --scenario test/integration/scenarios/known_door_preferred.yaml --repeats 3 --shot on
```

Options:

- `--scenario <file.yaml>`: the scenario to run (required). It supplies the map, start, goal and search settings.
- `--robot <robot_params.yaml>`: use this body polygon and `rmin` instead of the scenario's.
- `--n-headings <list>`: heading-bin counts to sweep, comma separated (default `16,32,64`).
- `--vmax <m/s>`, `--atomic-time <s>`: override the primitive speed (default 0.5) and duration (default 0.6).
- `--repeats <n>`: plans per setting (default 5).
- `--shot on|off|both`: the analytic goal shot (default `both`).
- `--max-plan-time <s>`, `--max-expansions <n>`: override the budgets.
- `--cost-policy <file>`, `--planner <file>`: replace the cost policy or the search settings.
- `--start x,y,theta`, `--goal x,y,theta`: replace the scenario's poses (metres, metres, radians, map frame).

Each output line is `key=value` pairs for one scenario, heading count and shot setting: build and plan times in milliseconds (median and minimum over the repeats), whether a path was found, expansion and collision-check counters, path cost, and a hash of the path so two runs can be compared.

Two other modes take `--scenario` and optionally `--robot`:

- `--shot-cost`: times the exact collision check of straight segments from the scenario's start pose, in microseconds.
- `--cover-split`: splits the exact-cover time of every primitive into rasterization and the rest; also takes `--vmax`, `--atomic-time` and `--n-headings`.

## Redraw the swept-cover figure

`swept_cover_dump` writes the data for the swept-cover figure: the cells cached for a straight motion in one heading bin, and the cells actually touched at a fan of start headings across that bin. `plot_swept_cover.py` draws it. Make the output folder first with `mkdir -p tools/viz/output`.

```bash
/ws/build/planner/tools/swept_cover_dump config/robot/robot_params.yaml > tools/viz/output/swept_cover.json
python3 tools/viz/plot_swept_cover.py tools/viz/output/swept_cover.json ../../docs/media/swept_cover.png
```

## Redraw the primitives figure

`primitives_dump` writes the data for the primitives figure: the twelve motions of the shipped robot from one pose, and the states reached after one, two and three motions. `plot_primitives.py` draws it. The output folder is the same as above.

```bash
/ws/build/planner/tools/primitives_dump config/robot/robot_params.yaml > tools/viz/output/primitives.json
python3 tools/viz/plot_primitives.py tools/viz/output/primitives.json ../../docs/media/primitives.png
```

## Redraw the cost and heuristic figure

`cost_field_dump` writes the data for the cost and heuristic figure: the cost of a metre of travel over the scenario map, the obstacle-aware cost to the goal, and the planned path. `plot_cost_field.py` draws it. It also prints three measurements on stderr that the design page quotes.

```bash
/ws/build/planner/tools/cost_field_dump test/integration/scenarios/known_door_preferred.yaml > tools/viz/output/cost_field.json
python3 tools/viz/plot_cost_field.py tools/viz/output/cost_field.json ../../docs/media/cost_field.png
```

## Redraw the navigation figure

`escape_move_dump` writes the data for the bottom panel of the navigation figure: the escape move found for a robot standing in a wall, with the start check at each step. `plot_navigation.py` draws it. It also prints one line of measurements on stderr that the design page quotes. The mode diagram in the top panel is laid out by hand in the script and is not generated.

```bash
/ws/build/planner/tools/escape_move_dump config/robot/robot_params.yaml config/navigation/navigator.yaml config/controller/controller.yaml > tools/viz/output/escape_move.json
python3 tools/viz/plot_navigation.py tools/viz/output/escape_move.json ../../docs/media/navigation.png
```

## Redraw the stop zone figure

`stop_zone_dump` writes the region of the robot frame where a single scan return would block the command, for a straight run, an arc and a turn on the spot. `plot_stop_zone.py` draws it. `stop_zone_dump` also prints measurements on stderr that the design page quotes. The optional third argument is the reaction time in seconds (default 0.2).

```bash
/ws/build/planner/tools/stop_zone_dump config/robot/robot_params.yaml config/safety/scan_safety_guard.yaml > tools/viz/output/stop_zone.json
python3 tools/viz/plot_stop_zone.py tools/viz/output/stop_zone.json ../../docs/media/stop_zone.png
```

Apache-2.0, see [LICENSE](../../../../LICENSE).
