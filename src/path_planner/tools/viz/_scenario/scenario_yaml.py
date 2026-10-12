#!/usr/bin/env python3
"""Write / patch a complete scenario YAML matching the loader schema (internal).

``write_scenario`` emits a full scenario file accepted by ``loadScenario``
(``src/core/scenario/scenario_loader.cpp``): an image-backed grid plus robot,
diff-drive, heuristic, cost-policy, and start/goal query, mirroring the field
layout of ``scenarios/straight_corridor.yaml`` (but image-backed instead of
inline rects). ``patch_poses`` rewrites only ``input.start`` / ``input.goal`` in
an existing file, round-trip preserving the rest.
"""

import os
import sys


# Robot footprint: 0.2 m square, body-frame, CCW (matches straight_corridor.yaml).
DEFAULT_ROBOT_VERTICES = [[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]

# nav2 trinary defaults applied when a scenario omits the image thresholds.
DEFAULT_OCCUPIED_THRESH = 0.65
DEFAULT_FREE_THRESH = 0.196


def read_scenario_meta(scenario_path):
    """Read the map + pose-conversion params a scenario YAML already carries.

    This is the scenario-side counterpart of ``mapmeta.read_meta`` (which reads a
    nav2 sibling .yaml given a MAP path). Here the input is the SCENARIO YAML, and
    every field comes from inside it: ``input.image.path`` (resolved to an absolute
    path the same way the C++ loader does -- a relative path is resolved against the
    scenario YAML's parent dir, an absolute path passes through), the image
    thresholds (defaulting to nav2's 0.65 / 0.196 when absent), and the grid
    ``resolution`` / ``origin`` ([x, y]).

    Returns a dict: ``image`` (resolved absolute path), ``resolution``,
    ``origin`` ((x, y)), ``occupied_thresh``, ``free_thresh``. Raises
    ``FileNotFoundError`` if the resolved image does not exist and ``KeyError`` /
    ``ValueError`` for a malformed scenario.
    """
    import yaml
    with open(scenario_path, "r") as f:
        data = yaml.safe_load(f) or {}

    inp = data.get("input")
    if not isinstance(inp, dict):
        raise KeyError("input")

    image_node = inp.get("image")
    if not isinstance(image_node, dict) or not image_node.get("path"):
        raise KeyError("input.image.path")
    image_rel = str(image_node["path"])

    # Resolve a relative image path against the scenario YAML's parent dir (matches
    # occupancyFromImage in src/core/scenario/scenario_loader.cpp); absolute passes through.
    if os.path.isabs(image_rel):
        image_abs = image_rel
    else:
        scenario_dir = os.path.dirname(os.path.abspath(scenario_path))
        image_abs = os.path.normpath(os.path.join(scenario_dir, image_rel))
    if not os.path.isfile(image_abs):
        raise FileNotFoundError(
            "scenario image not found: %s (from input.image.path=%s)" % (image_abs, image_rel))

    occupied_thresh = float(image_node.get("occupied_thresh", DEFAULT_OCCUPIED_THRESH))
    free_thresh = float(image_node.get("free_thresh", DEFAULT_FREE_THRESH))

    grid = inp.get("grid")
    if not isinstance(grid, dict):
        raise KeyError("input.grid")
    resolution = float(grid["resolution"])
    origin_raw = grid.get("origin", [0.0, 0.0])
    origin = (float(origin_raw[0]), float(origin_raw[1]))

    return {
        "image": image_abs,
        "resolution": resolution,
        "origin": origin,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
    }


def build_scenario_dict(name, meta, image_rel, start, goal, big=False):
    """Build the ordered scenario mapping (the dict ``write_scenario`` serializes).

    meta is the dict from ``mapmeta.read_meta``. ``image_rel`` is the image path as
    it should appear in the file (relative to the scenarios/ dir, e.g.
    ``maps/depot.png``); the loader resolves it against the YAML's parent dir.
    """
    start = [round(float(v), 6) for v in start]
    goal = [round(float(v), 6) for v in goal]

    inp = {
        "grid": {
            "cols": int(meta["cols"]),
            "rows": int(meta["rows"]),
            "resolution": float(meta["resolution"]),
            "origin": [float(meta["origin"][0]), float(meta["origin"][1])],
        },
        "image": {
            "path": image_rel,
            "occupied_thresh": float(meta["occupied_thresh"]),
            "free_thresh": float(meta["free_thresh"]),
        },
        "robot": {
            "vertices": [list(v) for v in DEFAULT_ROBOT_VERTICES],
        },
        "diff_drive": {
            "rmin": 0.5,
            "vmax": 1.0,
            "atomic_time": 0.3,
            "n_headings": 16,
        },
    }

    if big:
        # Larger maps need a bigger search budget / time ceiling than the struct defaults.
        inp["a_star"] = {
            "max_plan_time_sec": 30.0,
            "max_expansions": 800000,
            "node_pool_reserve": 800000,
            "closed_set_reserve": 800000,
        }

    inp["heuristic"] = {
        "weight": 1.0,
        "use_obstacle_heuristic": True,
    }
    inp["cost_policy"] = {
        "clearance_radius": 0.5,
        "clearance_decay": 15.0,
        "max_clearance_cost": 30.0,
        "unknown_cost_factor": 1.0,
    }
    inp["start"] = start
    inp["goal"] = goal

    return {
        "name": name,
        "input": inp,
        "expectations": {"expect_found": True},
    }


def write_scenario(out_path, name, meta, image_rel, start, goal, big=False):
    """Write a complete scenario YAML to ``out_path``. Returns the backend used.

    Prefers ruamel.yaml (block style, key order preserved, inline flow for the
    short coordinate sequences); falls back to PyYAML.
    """
    data = build_scenario_dict(name, meta, image_rel, start, goal, big=big)

    try:
        from ruamel.yaml import YAML
        from ruamel.yaml.comments import CommentedSeq

        def flow_seq(values):
            seq = CommentedSeq(values)
            seq.fa.set_flow_style()
            return seq

        # Inline the coordinate / vertex sequences for readability.
        data["input"]["grid"]["origin"] = flow_seq(data["input"]["grid"]["origin"])
        data["input"]["robot"]["vertices"] = flow_seq(
            [flow_seq(v) for v in data["input"]["robot"]["vertices"]])
        data["input"]["start"] = flow_seq(data["input"]["start"])
        data["input"]["goal"] = flow_seq(data["input"]["goal"])

        ryaml = YAML()
        ryaml.default_flow_style = False
        with open(out_path, "w") as f:
            ryaml.dump(data, f)
        return "ruamel"
    except ImportError:
        import yaml
        with open(out_path, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        return "pyyaml"


def patch_poses(yaml_path, start, goal):
    """Patch input.start / input.goal in-place, preserving the rest of the file.

    Prefers ruamel.yaml (comment/order preserving); falls back to PyYAML. Returns
    the backend used.
    """
    start = [round(float(v), 6) for v in start]
    goal = [round(float(v), 6) for v in goal]
    try:
        from ruamel.yaml import YAML
        ryaml = YAML()
        ryaml.preserve_quotes = True
        with open(yaml_path, "r") as f:
            data = ryaml.load(f)
        if "input" not in data or data["input"] is None:
            raise KeyError("input")
        data["input"]["start"] = start
        data["input"]["goal"] = goal
        with open(yaml_path, "w") as f:
            ryaml.dump(data, f)
        return "ruamel"
    except ImportError:
        import yaml
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        if data is None:
            print("ERROR: empty YAML: %s" % yaml_path, file=sys.stderr)
            raise
        data.setdefault("input", {})
        data["input"]["start"] = start
        data["input"]["goal"] = goal
        with open(yaml_path, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        return "pyyaml"
