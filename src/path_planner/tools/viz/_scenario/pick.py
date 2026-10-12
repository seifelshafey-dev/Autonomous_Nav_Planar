#!/usr/bin/env python3
"""Pixel<->world pose math + interactive start/goal picker (internal helper).

The two pure functions (``pixel_to_world`` / ``heading_from_drag``) are the
coordinate contract shared with ``scenario_loader.cpp`` and are unit-tested via
``scenario_studio selftest``. ``pick_poses`` adds the interactive picker: it shows
the COLOR-DECODED map (occ/free/unknown) so the user sees what they are clicking
on, then captures a click+drag for start and a click+drag for goal.

Coordinate conventions (must match scenario_loader.cpp):
  - Image displayed with origin='upper' (img_row 0 at top).
  - grid_row = rows - 1 - img_row     (loader flips vertically; bottom-origin grid)
  - world_x  = origin_x + (col + 0.5) * resolution      (cell-center)
  - world_y  = origin_y + (grid_row + 0.5) * resolution
  - heading: dx in image-col direction, dy in WORLD-up direction
             (a drag toward smaller img_row points to +y, i.e. theta -> +pi/2).

The matplotlib TkAgg import is deferred into ``pick_poses`` so importing this
module headless (for the pure-function tests) never requires a display backend.
"""

import math
import sys


def pixel_to_world(col, img_row, rows, resolution, origin_xy):
    """Map an (col, img_row) pixel to a world (x, y) cell-center coordinate.

    img_row is the row index as displayed (0 = top). rows is the image height.
    origin_xy is the world position of the bottom-left grid cell corner.
    """
    grid_row = rows - 1 - img_row
    world_x = origin_xy[0] + (col + 0.5) * resolution
    world_y = origin_xy[1] + (grid_row + 0.5) * resolution
    return world_x, world_y


def heading_from_drag(col0, row0, col1, row1, resolution):
    """Heading (rad) from a drag in image pixels.

    dx points along +col (world +x). dy points along world +y, which is the
    direction of DECREASING image row (because the grid is bottom-origin), hence
    the negation. resolution scales both equally so it does not change the angle,
    but is kept for unit consistency. A rightward drag -> 0; an upward drag
    (img_row decreasing) -> +pi/2.
    """
    dx = (col1 - col0) * resolution
    dy = -(row1 - row0) * resolution
    return math.atan2(dy, dx)


def pick_poses(image_path, resolution, origin, occupied_thresh, free_thresh):
    """Interactively pick start then goal poses over the color-decoded map.

    Opens a matplotlib (TkAgg) window showing the trinary-decoded map
    (OCC=black, FREE=white, UNK=orange). Click+drag once to set ``start`` (click =
    position, drag = heading), again to set ``goal``, then close the window.
    Returns ``(start_xyt, goal_xyt)`` as ``(x, y, theta)`` tuples, or
    ``(None, None)`` if fewer than two poses were picked.
    """
    import cv2
    import numpy as np  # noqa: F401  (kept for parity / future overlays)
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    from . import decode as _decode

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("ERROR: could not read image: %s" % image_path, file=sys.stderr)
        return None, None
    rows, cols = img.shape
    origin_xy = (origin[0], origin[1])

    rgb = _decode.color_preview_rgb(img, occupied_thresh, free_thresh)

    fig, ax = plt.subplots()
    ax.imshow(rgb, origin="upper", interpolation="nearest")
    ax.set_title("click+drag = pose (1st=start, 2nd=goal). Close window when done.")
    ax.set_xlabel("col")
    ax.set_ylabel("img_row (0=top)")
    ax.legend(handles=[
        Patch(facecolor="white", edgecolor="black", label="FREE"),
        Patch(facecolor="black", label="OCCUPIED"),
        Patch(facecolor="orange", label="UNKNOWN"),
    ], loc="upper right", fontsize=8)

    state = {"press": None, "poses": [], "labels": ["start", "goal"]}

    def on_press(event):
        if event.inaxes != ax or event.xdata is None:
            return
        state["press"] = (event.xdata, event.ydata)

    def on_release(event):
        if state["press"] is None or event.inaxes != ax or event.xdata is None:
            state["press"] = None
            return
        c0, r0 = state["press"]
        c1, r1 = event.xdata, event.ydata
        state["press"] = None
        if len(state["poses"]) >= 2:
            print("both start and goal already set; close the window to finish.")
            return
        x, y = pixel_to_world(c0, r0, rows, resolution, origin_xy)
        theta = heading_from_drag(c0, r0, c1, r1, resolution)
        label = state["labels"][len(state["poses"])]
        state["poses"].append((x, y, theta))
        print("%s: [%.6f, %.6f, %.6f]" % (label, x, y, theta))
        ax.plot([c0], [r0], "o", color="lime" if label == "start" else "red")
        ax.annotate("", xy=(c1, r1), xytext=(c0, r0),
                    arrowprops=dict(arrowstyle="->",
                                    color="lime" if label == "start" else "red"))
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    plt.show()

    if len(state["poses"]) < 2:
        print("note: fewer than 2 poses picked.", file=sys.stderr)
        return None, None
    return state["poses"][0], state["poses"][1]
