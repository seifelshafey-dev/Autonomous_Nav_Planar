#!/usr/bin/env python3
"""scenario_studio.py — the front door for building & running planner scenarios.

One obvious workflow: take a map image, preview/decode it, click start and goal,
write a complete scenario YAML, run the planner, and show the resulting path.
Everything else (trinary decode, coordinate math, the interactive picker, the
nav2-meta reader, the YAML writer) is an internal helper under ``_scenario/``;
the only other front-facing script is ``plot_search.py`` (the result renderer),
which this tool calls for you.

Scenarios are organized as **self-contained bundles**: a scenario YAML and its map
live together in ``scenarios/<name>/`` (``<name>.yaml`` + ``<name>.pgm``), and every
generated artifact — preview PNG, trace JSON, path PNG, animation — lands in that
bundle's ``output/`` subdir. ``new`` creates a fresh bundle from a source map (copying
the map in); ``run``/``pick --run``/``preview`` route their output into the bundle of
the scenario or map they operate on.

One rule keeps the CLI consistent: map-commands take a MAP, scenario-commands take
a SCENARIO YAML, one positional each. A scenario YAML already names its map (via
``input.image.path``) and carries the grid resolution/origin, so ``pick`` and
``run`` need only the scenario; ``new`` and ``preview`` start from a raw map.

Subcommands:
  new      <map.pgm> [--name] [--bin]   make bundle scenarios/<name>/, copy map in,
                                        pick start/goal -> write scenario -> run + show
  pick     <scenario.yaml> [--out]      re-pick start/goal into a scenario; the map is
                          [--run]       read from the scenario's input.image.path
  run      <scenario.yaml> [--bin]      run search_viz_dump -> render the trace into
                                        the scenario's bundle output/
  preview  <map.pgm> [--out]            headless decode: histogram/counts/connectivity
                                        + save a color preview PNG into the bundle output/

Examples:
  python3 scenario_studio.py new scenarios/maps-or-anywhere/foo.pgm
  python3 scenario_studio.py pick scenarios/depot/depot.yaml
  python3 scenario_studio.py run scenarios/depot/depot.yaml
"""

import argparse
import os
import subprocess
import sys

import numpy as np
import cv2

# Make the internal helper package importable whether run as a script or module.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from _scenario import decode as _decode          # noqa: E402
from _scenario import mapmeta as _mapmeta        # noqa: E402
from _scenario import scenario_yaml as _scenario_yaml  # noqa: E402

# Package root is two levels up from tools/viz/.
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
# `new` creates a self-contained bundle scenarios/<name>/ holding both the (copied) map and the
# scenario YAML. The committed CI corpus lives in test/integration/scenarios/ and is not written by
# this tool.
_SCENARIOS_DIR = os.path.join(_PKG_ROOT, "scenarios")
_WAREHOUSE_CELL_THRESHOLD = 1_000_000  # cells above this -> big search budget


# --------------------------------------------------------------------------- #
# Helpers shared by subcommands
# --------------------------------------------------------------------------- #
def _read_image(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("ERROR: could not read image: %s" % image_path, file=sys.stderr)
        return None
    return img


def _find_viz_binary(explicit=None):
    """Locate the search_viz_dump binary. Honors --bin, else searches build dirs."""
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        print("ERROR: --bin path does not exist: %s" % explicit, file=sys.stderr)
        return None
    candidates = []
    for build in ("build-gcc12", "build"):
        base = os.path.join(_PKG_ROOT, build)
        if not os.path.isdir(base):
            continue
        for root, _dirs, files in os.walk(base):
            if "search_viz_dump" in files:
                candidates.append(os.path.join(root, "search_viz_dump"))
    if candidates:
        return candidates[0]
    print("ERROR: search_viz_dump binary not found under build-gcc12/ or build/.\n"
          "       Build it (BUILD_TOOLS=ON) or pass --bin <path>.", file=sys.stderr)
    return None


def _is_big_map(meta):
    return int(meta["cols"]) * int(meta["rows"]) > _WAREHOUSE_CELL_THRESHOLD


def bundle_output_dir(path):
    """Return (and create) the ``output/`` dir of the bundle holding ``path``.

    The "bundle" is the directory containing the scenario YAML or the map. All
    generated artifacts (preview PNG, trace JSON, path PNG, animation) for that
    scenario/map land here, keeping each map's outputs self-contained.
    """
    out = os.path.join(os.path.dirname(os.path.abspath(path)), "output")
    os.makedirs(out, exist_ok=True)
    return out


# --------------------------------------------------------------------------- #
# preview
# --------------------------------------------------------------------------- #
def cmd_preview(args):
    img = _read_image(args.image)
    if img is None:
        return 2
    meta = _mapmeta.read_meta(args.image)
    occ_t = args.occupied_thresh if args.occupied_thresh is not None else meta["occupied_thresh"]
    free_t = args.free_thresh if args.free_thresh is not None else meta["free_thresh"]
    if free_t >= occ_t:
        print("ERROR: free-thresh must be < occupied-thresh", file=sys.stderr)
        return 2

    rows, cols = img.shape
    print("image: %s" % args.image)
    print("meta source: %s" % meta["meta_source"])
    print("dimensions: %d cols x %d rows (1 px = 1 cell)" % (cols, rows))
    print("resolution: %.6f m/cell   origin: (%.4f, %.4f)"
          % (meta["resolution"], meta["origin"][0], meta["origin"][1]))
    print("thresholds: occupied=%.4f free=%.4f" % (occ_t, free_t))

    hist = _decode.histogram(img)
    print("grayscale histogram (level: count, non-zero buckets):")
    for level in range(256):
        if hist[level]:
            print("  %3d: %d" % (level, hist[level]))

    occ_mask, free_mask, unk_mask = _decode.decode(img, occ_t, free_t)
    n_occ = int(np.count_nonzero(occ_mask))
    n_free = int(np.count_nonzero(free_mask))
    n_unk = int(np.count_nonzero(unk_mask))
    print("decoded: OCCUPIED=%d FREE=%d UNKNOWN=%d (total %d)"
          % (n_occ, n_free, n_unk, n_occ + n_free + n_unk))

    conn = _decode.connectivity(occ_mask, free_mask)
    if conn is None:
        print("connectivity: SKIPPED (fewer than 2 FREE cells)")
    else:
        print("connectivity (point-robot, 4-connected, OCCUPIED blocks, "
              "UNKNOWN traversable): %s" % ("CONNECTED" if conn else "BLOCKED"))
        print("  note: point-robot check; does not account for robot footprint radius.")

    out = args.out
    if out is None:
        # Default preview lands in the map's bundle output/ (not beside the image).
        stem = os.path.splitext(os.path.basename(args.image))[0]
        out = os.path.join(bundle_output_dir(args.image), stem + "_decoded.png")
    bgr = _decode.color_preview(img, occ_t, free_t)
    if not cv2.imwrite(out, bgr):
        print("ERROR: could not write preview: %s" % out, file=sys.stderr)
        return 2
    print("preview saved: %s" % out)
    return 0


# --------------------------------------------------------------------------- #
# pick
# --------------------------------------------------------------------------- #
def cmd_pick(args):
    if not os.path.exists(args.scenario):
        print("ERROR: scenario YAML does not exist: %s" % args.scenario, file=sys.stderr)
        return 2
    # The scenario YAML already names its map (input.image.path) and carries the grid
    # resolution/origin + image thresholds, so the map and pose-conversion params all
    # come from it -- no separate image argument.
    try:
        meta = _scenario_yaml.read_scenario_meta(args.scenario)
    except (KeyError, ValueError, FileNotFoundError) as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 2

    from _scenario import pick as _pick
    start, goal = _pick.pick_poses(meta["image"], meta["resolution"], meta["origin"],
                                   meta["occupied_thresh"], meta["free_thresh"])
    if start is None:
        print("nothing patched (fewer than 2 poses picked).", file=sys.stderr)
        return 1

    # --out: copy the source scenario to a new file, then patch poses into the copy, leaving the
    # original untouched. Without it, patch in place as before.
    target = args.scenario
    if args.out:
        import shutil
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        shutil.copyfile(args.scenario, args.out)
        target = args.out

    backend = _scenario_yaml.patch_poses(target, start, goal)
    print("patched input.start/input.goal in %s (via %s)" % (target, backend))

    # --run: chain straight into the run pipeline on the resulting YAML.
    if args.run:
        print("running patched scenario: %s" % target)
        return cmd_run(argparse.Namespace(scenario=target, bin=args.bin))
    return 0


# --------------------------------------------------------------------------- #
# run
# --------------------------------------------------------------------------- #
def cmd_run(args):
    scenario = os.path.abspath(args.scenario)
    if not os.path.exists(scenario):
        print("ERROR: scenario YAML does not exist: %s" % scenario, file=sys.stderr)
        return 2
    binary = _find_viz_binary(args.bin)
    if binary is None:
        return 2

    # Route the trace into the scenario's bundle output/ by passing it as search_viz_dump's 2nd
    # arg (argv[2]), overriding the tool's default tools/viz/output/<stem>.json. The binary is
    # still run from the package root so its relative-path scenario resolution matches the loader.
    stem = os.path.splitext(os.path.basename(scenario))[0]
    trace_json = os.path.join(bundle_output_dir(scenario), stem + ".json")

    print("running: %s %s %s  (cwd=%s)" % (binary, scenario, trace_json, _PKG_ROOT))
    proc = subprocess.run([binary, scenario, trace_json], cwd=_PKG_ROOT)
    if proc.returncode != 0:
        print("ERROR: search_viz_dump exited with code %d" % proc.returncode, file=sys.stderr)
        return proc.returncode
    if not os.path.exists(trace_json):
        print("ERROR: expected trace not found: %s" % trace_json, file=sys.stderr)
        return 2
    print("trace: %s" % trace_json)

    plot = os.path.join(_HERE, "plot_search.py")
    # plot_search.py derives the PNG from the trace path (foo.json -> foo.png), so the render lands
    # beside the trace in the same bundle output/.
    png = os.path.splitext(trace_json)[0] + ".png"
    print("rendering: %s %s" % (plot, trace_json))
    rproc = subprocess.run([sys.executable, plot, trace_json])
    if rproc.returncode != 0:
        print("ERROR: plot_search.py exited with code %d" % rproc.returncode, file=sys.stderr)
        return rproc.returncode
    print("path render: %s" % png)
    return 0


# --------------------------------------------------------------------------- #
# new (headline flow)
# --------------------------------------------------------------------------- #
def cmd_new(args):
    if _read_image(args.image) is None:
        return 2
    # Read meta from the SOURCE map's sibling nav2 .yaml (if present, else defaults) BEFORE copying,
    # so the source bundle's metadata is what gets baked into the new scenario.
    meta = _mapmeta.read_meta(args.image)
    name = args.name or os.path.splitext(os.path.basename(args.image))[0]

    from _scenario import pick as _pick
    print("pick start then goal (click+drag each), then close the window.")
    start, goal = _pick.pick_poses(args.image, meta["resolution"], meta["origin"],
                                   meta["occupied_thresh"], meta["free_thresh"])
    if start is None:
        print("aborted: fewer than 2 poses picked.", file=sys.stderr)
        return 1

    # Build a self-contained bundle scenarios/<name>/ and COPY the source map into it as <name>.pgm,
    # so the scenario references its map same-dir-relative (image: <name>.pgm) and travels with it.
    bundle_dir = os.path.join(_SCENARIOS_DIR, name)
    os.makedirs(bundle_dir, exist_ok=True)
    src_ext = os.path.splitext(args.image)[1] or ".pgm"
    map_dst = os.path.join(bundle_dir, name + src_ext)
    if os.path.abspath(args.image) != os.path.abspath(map_dst):
        import shutil
        shutil.copyfile(args.image, map_dst)
    out_yaml = os.path.join(bundle_dir, name + ".yaml")
    image_rel = name + src_ext  # same-dir relative; loader resolves against the YAML's parent
    big = _is_big_map(meta)
    backend = _scenario_yaml.write_scenario(out_yaml, name, meta, image_rel,
                                            start, goal, big=big)
    print("created bundle: %s" % bundle_dir)
    print("  map:      %s" % map_dst)
    print("  scenario: %s (via %s, big=%s)" % (out_yaml, backend, big))

    # Chain straight into run so `new` is one obvious end-to-end gesture; output lands in the bundle.
    run_args = argparse.Namespace(scenario=out_yaml, bin=args.bin)
    return cmd_run(run_args)


# --------------------------------------------------------------------------- #
# selftest (pure-function checks, headless)
# --------------------------------------------------------------------------- #
def cmd_selftest(_args):
    """Unit-check the pure pose math (pixel_to_world / heading_from_drag), headless."""
    import math
    from _scenario.pick import pixel_to_world, heading_from_drag

    ok = True

    def check(name, got, want, tol=1e-6):
        nonlocal ok
        if isinstance(got, tuple):
            passed = all(abs(g - w) <= tol for g, w in zip(got, want))
        else:
            passed = abs(got - want) <= tol
        ok = ok and passed
        print("  [%s] %s: got=%s want=%s"
              % ("PASS" if passed else "FAIL", name, got, want))

    check("pixel_to_world(192,192,384,0.05,(-10,-10))",
          pixel_to_world(192, 192, 384, 0.05, (-10, -10)), (-0.375, -0.425))
    check("pixel_to_world(0,383,384,0.05,(-10,-10))",
          pixel_to_world(0, 383, 384, 0.05, (-10, -10)), (-9.975, -9.975))
    check("heading rightward", heading_from_drag(10, 10, 20, 10, 0.05), 0.0)
    check("heading upward", heading_from_drag(10, 20, 10, 10, 0.05), math.pi / 2)
    print("SELFTEST: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
def build_parser():
    ap = argparse.ArgumentParser(
        description="Build and run planner scenarios. Map-commands (new, preview) take a "
                    "map image; scenario-commands (pick, run) take a scenario YAML and read "
                    "the map from it. One positional each.")
    sub = ap.add_subparsers(dest="command", required=True)

    p_new = sub.add_parser("new", help="map -> pick poses -> write scenario -> run + show")
    p_new.add_argument("image", metavar="map", help="grayscale map image (1 px = 1 cell)")
    p_new.add_argument("--name", default=None, help="scenario name (default: map stem)")
    p_new.add_argument("--bin", default=None, help="path to search_viz_dump binary")
    p_new.set_defaults(func=cmd_new)

    p_run = sub.add_parser("run", help="run search_viz_dump on a scenario YAML and render the trace")
    p_run.add_argument("scenario", help="scenario YAML")
    p_run.add_argument("--bin", default=None, help="path to search_viz_dump binary")
    p_run.set_defaults(func=cmd_run)

    p_pick = sub.add_parser(
        "pick",
        help="re-pick start/goal into a scenario YAML (map read from the scenario)")
    p_pick.add_argument("scenario",
                        help="scenario YAML; its input.image.path names the map and "
                             "input.grid supplies resolution/origin")
    p_pick.add_argument("--out", default=None,
                        help="write the picked poses to this NEW file (copy of <scenario>), "
                             "leaving the original untouched")
    p_pick.add_argument("--run", action="store_true",
                        help="after patching, run search_viz_dump on the result and render it")
    p_pick.add_argument("--bin", default=None, help="path to search_viz_dump binary (with --run)")
    p_pick.set_defaults(func=cmd_pick)

    p_prev = sub.add_parser("preview", help="headless decode + counts + connectivity + color PNG")
    p_prev.add_argument("image", metavar="map", help="grayscale map image")
    p_prev.add_argument("--out", default=None,
                        help="output PNG path (default <stem>_decoded.png beside input)")
    p_prev.add_argument("--occupied-thresh", type=float, default=None,
                        help="override darkness OCCUPIED cutoff (default: sibling yaml / 0.65)")
    p_prev.add_argument("--free-thresh", type=float, default=None,
                        help="override darkness FREE cutoff (default: sibling yaml / 0.196)")
    p_prev.set_defaults(func=cmd_preview)

    p_self = sub.add_parser("selftest", help="headless unit-check of the pure pose math")
    p_self.set_defaults(func=cmd_selftest)

    return ap


def main(argv=None):
    ap = build_parser()
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
