#!/usr/bin/env python3
"""Read a nav2 map .yaml sibling into planner grid metadata (internal helper).

A nav2 map image (e.g. ``depot.pgm``) is typically paired with a sibling YAML of
the same stem (``depot.yaml``) carrying ``resolution``, ``origin: [x, y, yaw]``,
and the trinary ``occupied_thresh`` / ``free_thresh``. ``read_meta`` finds that
sibling, parses it, and returns the fields the scenario writer needs. The grid's
``[x, y]`` origin is taken from the nav2 ``[x, y, yaw]`` (yaw is dropped — the
planner grid is axis-aligned). cols/rows come from the actual image dimensions,
not the YAML.
"""

import os

import cv2

# nav2 defaults used when no sibling YAML is found.
DEFAULT_RESOLUTION = 0.05
DEFAULT_ORIGIN = (0.0, 0.0)
DEFAULT_OCCUPIED_THRESH = 0.65
DEFAULT_FREE_THRESH = 0.196


def _image_dims(image_path):
    """Return (cols, rows) of a grayscale image; raises if it cannot be read."""
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError("could not read image: %s" % image_path)
    rows, cols = img.shape
    return cols, rows


def read_meta(image_path):
    """Resolve grid metadata for ``image_path`` from its sibling nav2 YAML.

    Returns a dict with keys: ``resolution`` (m/cell), ``origin`` ((x, y) world
    position of the bottom-left grid corner), ``occupied_thresh``, ``free_thresh``,
    ``cols``, ``rows``, and ``meta_source`` (the sibling YAML path or the string
    "defaults" when none was found). cols/rows always come from the image itself;
    a sibling YAML supplies only resolution/origin/thresholds.
    """
    cols, rows = _image_dims(image_path)

    stem = os.path.splitext(image_path)[0]
    sibling = stem + ".yaml"
    if not os.path.exists(sibling):
        sibling_yml = stem + ".yml"
        sibling = sibling_yml if os.path.exists(sibling_yml) else sibling

    if not os.path.exists(sibling):
        return {
            "resolution": DEFAULT_RESOLUTION,
            "origin": DEFAULT_ORIGIN,
            "occupied_thresh": DEFAULT_OCCUPIED_THRESH,
            "free_thresh": DEFAULT_FREE_THRESH,
            "cols": cols,
            "rows": rows,
            "meta_source": "defaults",
        }

    import yaml
    with open(sibling, "r") as f:
        data = yaml.safe_load(f) or {}

    resolution = float(data.get("resolution", DEFAULT_RESOLUTION))
    origin_raw = data.get("origin", list(DEFAULT_ORIGIN))
    # nav2 origin is [x, y, yaw]; the planner grid origin is [x, y].
    origin = (float(origin_raw[0]), float(origin_raw[1]))
    occupied_thresh = float(data.get("occupied_thresh", DEFAULT_OCCUPIED_THRESH))
    free_thresh = float(data.get("free_thresh", DEFAULT_FREE_THRESH))

    return {
        "resolution": resolution,
        "origin": origin,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
        "cols": cols,
        "rows": rows,
        "meta_source": sibling,
    }
