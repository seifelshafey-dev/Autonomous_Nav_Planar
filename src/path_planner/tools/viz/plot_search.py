#!/usr/bin/env python3
"""Render a hybrid A* SearchTrace (from search_viz_dump) with matplotlib.

A trace is a small JSON metadata file (`<stem>.json`: meta, start, goal, footprint, found, path) plus a
binary side-car (`<stem>.bin`, little-endian) holding the bulk arrays — occupancy (int8), costmap
(float32) and the per-node columns x/y/theta/g/h (float32) + parent (int32). The JSON's `binary` block
maps each array name to its {dtype, shape, byte offset}. `load()` memmaps the grid arrays (no RSS) and
reads the node columns; pass the `.json` path on the command line (the `.bin` is found beside it).

Usage:
    python3 plot_search.py [trace.json] [--save out.png]
    python3 plot_search.py [trace.json] --animate [--save-anim out.mp4] [--frames 150]
    options: --max-anim-nodes 6000 --no-tree --no-costmap

--save-anim picks a writer from the extension: an `.mp4` path uses FFMpeg (H.264) when ffmpeg is on
PATH and falls back to a GIF (same name, `.gif`) otherwise; any other extension uses the Pillow GIF
writer. With no --save-anim the animation pops up interactively (or, headless, returns without saving).
The animated node set is capped via --max-anim-nodes (uniform-stride subset in expansion order) so the
replay stays interactive on traces with hundreds of thousands of expansions; the static render is never
capped.

By default (no flags) this renders the static figure and saves a PNG next to the trace JSON
(trace.json -> trace.png), then shows it interactively only if a display is available. With no
display (CI / SSH / `$DISPLAY` unset) it forces the non-interactive Agg backend and never blocks on
plt.show(); the PNG is still written. Pass --save to override the static PNG path.

Draws the occupancy grid (free/occupied/unknown), an optional costmap heatmap, the explored-node
cloud coloured by expansion order, the parent-pointer search tree, the final path (waypoints + dense
samples), the robot footprint stamped along the path, and start/goal with heading arrows.

--animate replays the search in true expansion (pop) order: the trace nodes are emitted from the
planner's SearchObserver stream, one per node as it is popped and expanded, so the replay shows the
frontier in the exact order the planner committed to it. This is the matplotlib sink for the
renderer-agnostic SearchTrace; a ROS layer would consume the same data to publish MarkerArray /
OccupancyGrid / nav_msgs::Path.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

import matplotlib

# Choose the backend BEFORE importing pyplot. With a display present, explicitly select the interactive
# TkAgg backend so a GUI window reliably pops (and blocks until closed -- intended for interactive use);
# if TkAgg is unavailable we fall back to matplotlib's default rather than crash. With no usable display,
# force the non-interactive Agg backend (save-only) so a zero-flag run still saves a PNG and never hangs
# on a blocking plt.show() (CI / headless).
HAVE_DISPLAY = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
if HAVE_DISPLAY:
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass
else:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon


def default_png_path(trace_path):
    """Derive the default static-PNG path from the trace JSON path (foo.json -> foo.png)."""
    root, _ = os.path.splitext(trace_path)
    return root + ".png"


def maybe_show():
    """Show interactively only when a display is available; a no-op (and no block) when headless."""
    if HAVE_DISPLAY:
        plt.show()
    else:
        plt.close("all")


_BIN_DTYPES = {"int8": np.int8, "float32": np.float32, "int32": np.int32}


def load(path):
    """Load a trace: small JSON metadata + a binary side-car for the bulk arrays.

    The JSON carries meta/start/goal/footprint/found/path and a `binary` descriptor naming the side-car
    (`<stem>.bin`, little-endian) with each array's {name, dtype, shape, offset}. The grid arrays
    (occupancy int8, costmap float32) are mapped read-only with np.memmap so they never inflate RSS; the
    per-node columns (x, y, theta, g, h as float32, parent as int32) are read into numpy arrays. The
    returned dict adds `occupancy`/`costmap` (memmaps) and `nodes` (a dict of column arrays + `count`).
    """
    with open(path) as f:
        t = json.load(f)

    desc = t["binary"]
    if desc.get("endianness", "little") != "little":
        raise ValueError(f"unsupported endianness {desc.get('endianness')!r} in {path}")
    bin_path = os.path.join(os.path.dirname(path), desc["file"])

    cols = {}
    for arr in desc["arrays"]:
        dt = _BIN_DTYPES[arr["dtype"]]
        count = int(arr["shape"][0])
        offset = int(arr["offset"])
        name = arr["name"]
        if name in ("occupancy", "costmap"):
            # Big grid arrays: memmap read-only so they are never copied into process memory.
            cols[name] = np.memmap(bin_path, dtype=dt, mode="r", offset=offset, shape=(count,))
        else:
            cols[name] = np.fromfile(bin_path, dtype=dt, count=count,
                                     offset=offset) if count else np.empty(0, dtype=dt)

    t["occupancy"] = cols.pop("occupancy")
    t["costmap"] = cols.pop("costmap")
    t["nodes"] = {**cols, "count": int(desc.get("n_nodes", len(cols.get("x", []))))}
    return t


def occupancy_image(meta, occ):
    """unknown -> gray, free -> white, occupied -> black."""
    cols, rows = meta["cols"], meta["rows"]
    a = np.asarray(occ).reshape(rows, cols)
    img = np.ones((rows, cols), dtype=float)
    img[a < 0] = 0.5
    img[a >= 50] = 0.0
    return img


def transform_footprint(verts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(x + c * vx - s * vy, y + s * vx + c * vy) for vx, vy in verts]


def draw_background(ax, t, extent, no_costmap):
    ax.imshow(occupancy_image(t["meta"], t["occupancy"]), cmap="gray", origin="lower",
              extent=extent, vmin=0, vmax=1, zorder=0)
    if not no_costmap and len(t["costmap"]):
        cm = np.asarray(t["costmap"], dtype=float).reshape(t["meta"]["rows"], t["meta"]["cols"])
        ax.imshow(np.ma.masked_less_equal(cm, 0.0), cmap="hot_r", origin="lower",
                  extent=extent, alpha=0.45, zorder=1)
    for pose, color, name in ((t["start"], "lime", "start"), (t["goal"], "magenta", "goal")):
        ax.plot(pose[0], pose[1], "o", color=color, markersize=10, zorder=7, label=name)
        ax.arrow(pose[0], pose[1], 0.4 * math.cos(pose[2]), 0.4 * math.sin(pose[2]),
                 head_width=0.12, color=color, zorder=7)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")


def tree_segments_for(nodes, indices):
    """Parent-pointer segments for an expansion-ordered subset of node indices, as an (m, 2, 2) array.

    `nodes` is the column store {x, y, parent, ...}; `indices` selects the subset to draw (the full
    range for the static figure, a decimated slice for the animation). Each kept non-root node `i`
    yields the segment [(x[i], y[i]), (x[parent[i]], y[parent[i]])] — the parent endpoint is read from
    the FULL columns, so a kept node whose parent was decimated out still draws its true edge. Fully
    vectorized; built once and sliced per frame for the animation, never rebuilt.
    """
    idx = np.asarray(indices)
    parent = nodes["parent"][idx]
    keep = parent >= 0
    child = idx[keep]
    par = parent[keep]
    nx, ny = nodes["x"], nodes["y"]
    segs = np.empty((child.size, 2, 2), dtype=float)
    segs[:, 0, 0] = nx[child]
    segs[:, 0, 1] = ny[child]
    segs[:, 1, 0] = nx[par]
    segs[:, 1, 1] = ny[par]
    return segs


def tree_segments(nodes):
    """All parent-pointer segments (static figure), as an (m, 2, 2) array."""
    return tree_segments_for(nodes, np.arange(nodes["count"]))


# Shared final-path styling — the SINGLE source of truth for how the followed path, its node-point
# waypoints, and the faint hint are drawn/labeled. Both the static render (draw_path) and the animation
# (render_animation) consume these so their legends can never diverge again.
_PATH_SAMPLES_KW = dict(color="red", linewidth=2.0, zorder=5)            # dense followed path
_PATH_HINT_KW = dict(color="darkred", linewidth=0.6, alpha=0.35, zorder=4)  # faint node-point chord hint
_PATH_WAYPOINT_KW = dict(color="darkred", markersize=5, markerfacecolor="none",
                         markeredgewidth=1.2, zorder=6)                   # hollow segment-endpoint markers
_WAYPOINT_LABEL = "segment endpoints"


def path_samples_label(total_cost):
    """Legend label for the prominent dense-sample path line (shared by static + animation)."""
    return f"path (cost {total_cost:.2f})"


def draw_path(ax, t):
    """Draw the final path: the dense per-arc `samples` are the followed path (prominent, labeled with
    cost); the sparse `node_points` are the per-segment endpoints, shown only as waypoint markers so a
    connecting chord is never mistaken for the true (arc-following) path.

    Styling/labels come from the shared `_PATH_*` constants + `path_samples_label`, which the animation
    (render_animation) reuses so the two legends stay identical."""
    if not t["found"]:
        return
    path = t["path"]
    samples = path.get("samples") or []
    total_cost = path["total_cost"]
    if samples:
        # The dense sampled poses trace the real arcs the robot follows -> this is THE path.
        ax.plot([p[0] for p in samples], [p[1] for p in samples], "-",
                label=path_samples_label(total_cost), **_PATH_SAMPLES_KW)
    pts = path["node_points"]
    if pts:
        # Sparse per-segment endpoints: markers only (no solid connecting line -- a straight chord
        # across the arcs would misread as the path). A faint dashed hint keeps them associated.
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "--", **_PATH_HINT_KW)
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "o",
                label=_WAYPOINT_LABEL, **_PATH_WAYPOINT_KW)
    if t["footprint"] and pts:
        stride = max(1, len(pts) // 12)
        for p in pts[::stride]:
            ax.add_patch(Polygon(transform_footprint(t["footprint"], p), closed=True,
                                 fill=False, edgecolor="darkred", linewidth=0.8, zorder=6))


def samples_polyline(t):
    """The dense followed path as a single (k, 2) array of sampled-pose (x, y), or None if no samples.

    These are the per-arc samples the robot actually follows -> the prominent path line. Used to
    populate the animation's path Line2D lazily on the last frame (absent during the expansion replay,
    revealed only once every node is shown), matching the static `draw_path` samples artist.
    """
    if not t["found"]:
        return None
    samples = t["path"].get("samples") or []
    if not samples:
        return None
    return np.array([(p[0], p[1]) for p in samples], dtype=float)


def path_polyline(t):
    """The final path's sparse node-points as a single (k, 2) array of (x, y), or None if none.

    These are the per-segment endpoints -> the hollow waypoint markers + faint dashed chord hint (NOT
    the followed path; that is `samples_polyline`). Used to populate the animation's waypoint/hint
    artists lazily on the last frame, matching the static `draw_path` node-point artists.
    """
    if not t["found"]:
        return None
    pts = t["path"]["node_points"]
    if not pts:
        return None
    return np.array([(p[0], p[1]) for p in pts], dtype=float)


def footprint_segments(t):
    """Stamped-footprint outlines along the final path as an (m, 2, 2) segment array, or None.

    Each sampled pose contributes the closed polygon outline of the transformed footprint, expanded
    into its edge segments so a single LineCollection can render every stamp. Feeds the footprint
    artist that is revealed alongside the path on the animation's final frame.
    """
    if not t["found"] or not t["footprint"]:
        return None
    pts = t["path"]["node_points"]
    if not pts:
        return None
    stride = max(1, len(pts) // 12)
    segs = []
    for p in pts[::stride]:
        ring = transform_footprint(t["footprint"], p)
        ring = ring + ring[:1]  # close the polygon
        for a, b in zip(ring[:-1], ring[1:]):
            segs.append((a, b))
    if not segs:
        return None
    return np.array(segs, dtype=float)


def render_static(t, extent, args):
    fig, ax = plt.subplots(figsize=(9, 9))
    draw_background(ax, t, extent, args.no_costmap)
    nodes = t["nodes"]
    n = nodes["count"]
    if n:
        if not args.no_tree:
            ax.add_collection(LineCollection(tree_segments(nodes),
                              colors="#3070b0", linewidths=0.3, alpha=0.4, zorder=2))
        ax.scatter(nodes["x"], nodes["y"], c=np.arange(n), cmap="viridis", s=6, zorder=3,
                   label=f"explored nodes ({n})")
    draw_path(ax, t)
    ax.set_title("hybrid A* search (final)")
    ax.legend(loc="upper left", fontsize=8)
    # Always write a PNG (path from --save, else derived from the trace path); show only if a display
    # is present so a zero-flag headless run produces a file and returns immediately.
    png_path = args.save or default_png_path(args.trace)
    fig.savefig(png_path, dpi=130, bbox_inches="tight")
    print(f"saved {png_path}")
    maybe_show()


def anim_stride(n, max_nodes):
    """Deterministic decimation stride for the ANIMATED node set: `max(1, ceil(n / max_nodes))`.

    Threshold-gated and a strict no-op below the cap: when `n <= max_nodes` (or `max_nodes <= 0`, i.e.
    "no cap") the stride is 1, so `nodes[::stride]` is the full, unchanged set — normal/small traces
    animate byte-identically to the un-capped behavior. Above the cap the stride is the smallest integer
    that keeps `ceil(n/stride) <= max_nodes`, applied as a plain expansion-ordered slice `[::stride]`
    (so index 0 / the root is always kept). The slice only controls what is DRAWN; parent coordinates
    are still resolved against the full node list, so the search tree stays faithful.
    """
    if max_nodes <= 0 or n <= max_nodes:
        return 1
    return -(-n // max_nodes)  # ceil(n / max_nodes)


def make_anim_writer(save_path, fps):
    """Pick an animation writer for `save_path`: prefer FFMpeg (H.264 MP4), fall back to Pillow (GIF).

    Returns `(writer, out_path)`. An `.mp4` request uses FFMpegWriter when ffmpeg is on PATH; if it is
    not, we degrade to a GIF (rewriting the extension to `.gif`) rather than fail. A `.gif` request (or
    any non-`.mp4` extension) always uses the Pillow GIF writer.
    """
    from matplotlib.animation import FFMpegWriter, PillowWriter
    root, ext = os.path.splitext(save_path)
    want_mp4 = ext.lower() == ".mp4"
    if want_mp4 and FFMpegWriter.isAvailable():
        return FFMpegWriter(fps=fps, codec="libx264"), save_path
    if want_mp4:
        gif_path = root + ".gif"
        print(f"ffmpeg unavailable; falling back to GIF: {gif_path}")
        return PillowWriter(fps=fps), gif_path
    return PillowWriter(fps=fps), save_path


def render_animation(t, extent, args):
    from matplotlib.animation import FuncAnimation
    nodes = t["nodes"]
    n = nodes["count"]
    if not n:
        print("no nodes to animate")
        return

    # Cap the ANIMATED set: drawing all ~170k expansions is O(n^2) over frames and visually saturated.
    # A strict no-op below the cap (stride==1 => the full set). `sel` holds the TRUE expansion indices of
    # the kept nodes; full coordinates/parents still live in `nodes`.
    stride = anim_stride(n, args.max_anim_nodes)
    sel = np.arange(n)[::stride]
    nx = nodes["x"][sel]
    ny = nodes["y"][sel]
    order = sel  # colour by TRUE expansion index (vmax=n below) so viridis matches the static figure

    # Size the figure to the map so the saved frames carry no white bands: axes box of at most 7.5 in
    # on the long side, plus fixed margins (inches) for the tick labels and the legend below the axes.
    W, H = extent[1] - extent[0], extent[3] - extent[2]
    aw = max(3.0, 7.5 * W / max(W, H))
    ah = max(3.0, 7.5 * H / max(W, H))
    fig_w, fig_h = aw + 0.85, ah + 1.35
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    fig.subplots_adjust(left=0.65 / fig_w, right=1 - 0.20 / fig_w,
                        bottom=0.95 / fig_h, top=1 - 0.40 / fig_h)
    draw_background(ax, t, extent, args.no_costmap)
    ax.set_title("hybrid A* search (expansion-order replay)")
    if stride > 1:
        print(f"animating {len(sel)} of {n} nodes (stride {stride}, --max-anim-nodes {args.max_anim_nodes})")

    tree = LineCollection([], colors="#3070b0", linewidths=0.3, alpha=0.4, zorder=2)
    ax.add_collection(tree)
    scat = ax.scatter([], [], c=[], cmap="viridis", s=6, zorder=3, vmin=0, vmax=n)

    # Path + footprint artists are created EMPTY/invisible here and added to the axes so the blit
    # manager tracks them as part of the fixed per-frame artist tuple. They stay empty during the
    # expansion replay and are populated only on the final frame(s) — so the path appears ONLY after
    # the last node is revealed, never alongside the still-growing frontier. Their styling/labels come
    # from the SAME shared constants as the static draw_path, so the two legends stay identical: the
    # prominent line is the dense `samples` (the followed path); the sparse `node_points` are hollow
    # waypoint markers plus a faint dashed chord hint, never a bold straight chord.
    samples_xy = samples_polyline(t)         # dense followed path -> prominent line
    waypoints_xy = path_polyline(t)          # sparse node-points -> markers + hint
    foot_segs = footprint_segments(t)
    total_cost = t["path"]["total_cost"] if t["found"] else None
    path_line, = ax.plot([], [], "-", **_PATH_SAMPLES_KW,
                         label=(path_samples_label(total_cost) if total_cost is not None else "path"))
    waypoint_hint, = ax.plot([], [], "--", **_PATH_HINT_KW)
    waypoints, = ax.plot([], [], "o", label=_WAYPOINT_LABEL, **_PATH_WAYPOINT_KW)
    footprint = LineCollection([], colors="darkred", linewidths=0.8, zorder=6)
    ax.add_collection(footprint)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=2, fontsize=8, frameon=False)

    # Build the decimated tree segments ONCE (an (m, 2, 2) array); per frame we only slice it — no
    # per-frame rebuild. Parent endpoints are read from the full columns, so a kept node whose parent
    # was decimated out still draws its true edge.
    all_segs = tree_segments_for(nodes, sel) if not args.no_tree else None
    # seg_upto[m] = how many segments are revealed once m kept nodes are shown (segments skip roots).
    seg_upto = np.zeros(len(sel) + 1, dtype=int)
    if not args.no_tree:
        seg_upto[1:] = np.cumsum(nodes["parent"][sel] >= 0)

    n_frames = max(1, args.frames)
    step = max(1, len(sel) // n_frames)
    frames = len(sel) // step + 2 + args.hold_frames  # extra frames repeat the final picture

    artists = (scat, tree, path_line, waypoint_hint, waypoints, footprint)

    def init():
        scat.set_offsets(np.empty((0, 2)))
        scat.set_array(np.empty(0))
        tree.set_segments([])
        path_line.set_data([], [])
        waypoint_hint.set_data([], [])
        waypoints.set_data([], [])
        footprint.set_segments([])
        return artists

    def update(frame):
        m = min(len(sel), (frame + 1) * step)
        scat.set_offsets(np.column_stack([nx[:m], ny[:m]]))
        scat.set_array(order[:m])
        if not args.no_tree:
            tree.set_segments(all_segs[:seg_upto[m]])
        # Reveal the final path + waypoints + footprint ONLY once the replay has shown every animated
        # node; keep them empty while the frontier is still expanding. Return the SAME artist tuple
        # every frame so blitting stays satisfied.
        if m >= len(sel):
            if samples_xy is not None:
                path_line.set_data(samples_xy[:, 0], samples_xy[:, 1])
            if waypoints_xy is not None:
                waypoint_hint.set_data(waypoints_xy[:, 0], waypoints_xy[:, 1])
                waypoints.set_data(waypoints_xy[:, 0], waypoints_xy[:, 1])
            if foot_segs is not None:
                footprint.set_segments(foot_segs)
        else:
            path_line.set_data([], [])
            waypoint_hint.set_data([], [])
            waypoints.set_data([], [])
            footprint.set_segments([])
        return artists

    anim = FuncAnimation(fig, update, init_func=init, frames=frames,
                         interval=40, blit=True, repeat=False)

    if args.save_anim:
        writer, out_path = make_anim_writer(args.save_anim, fps=25)
        anim.save(out_path, writer=writer)
        print(f"saved {out_path}")
    else:
        # Interactive pop-up (default). The path/footprint are revealed by the final frame(s) of the
        # replay itself; headless falls through maybe_show() without blocking.
        maybe_show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", help="search-trace JSON dumped by search_viz_dump (per-scenario <stem>.json)")
    ap.add_argument("--save", default=None,
                    help="static-PNG output path (default: derived from trace path, foo.json -> foo.png)")
    ap.add_argument("--animate", action="store_true", help="replay search in expansion (pop) order")
    ap.add_argument("--save-anim", default=None,
                    help="save animation instead of showing; .mp4 uses FFMpeg/H.264 (falls back to GIF "
                         "if ffmpeg is missing), any other extension uses the Pillow GIF writer")
    ap.add_argument("--frames", type=int, default=150)
    ap.add_argument("--max-anim-nodes", type=int, default=6000,
                    help="cap the animated node set (uniform-stride subset, expansion order kept); "
                         "0 = no cap. Drawing all ~170k nodes is O(n^2) over frames and visually "
                         "saturated. Static render is unaffected.")
    ap.add_argument("--hold-frames", type=int, default=50,
                    help="extra frames that repeat the final picture, path included, at the end of the "
                         "animation; 0 disables")
    ap.add_argument("--no-tree", action="store_true")
    ap.add_argument("--no-costmap", action="store_true")
    args = ap.parse_args()
    if args.hold_frames < 0:
        ap.error("--hold-frames must be >= 0")

    t = load(args.trace)
    meta = t["meta"]
    res, (ox, oy) = meta["resolution"], meta["origin"]
    extent = [ox, ox + meta["cols"] * res, oy, oy + meta["rows"] * res]

    if args.animate or args.save_anim:
        render_animation(t, extent, args)
    else:
        render_static(t, extent, args)


if __name__ == "__main__":
    sys.exit(main())
