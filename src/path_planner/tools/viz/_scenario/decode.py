#!/usr/bin/env python3
"""Trinary occupancy decode + color preview + connectivity (internal helper).

Decodes a grayscale map image with nav2's trinary scheme, mirroring
``occupancyFromImage`` in ``src/core/scenario/scenario_loader.cpp``. This is the
verified decode reused by ``scenario_studio preview``; it is not a front-facing
script (the underscore package marks it internal).

Decode rule (matches the C++ loader exactly):
    p = (255 - px) / 255            # darkness fraction
    p > occupied_thresh  -> OCCUPIED
    p < free_thresh      -> FREE
    otherwise            -> UNKNOWN

1 pixel = 1 grid cell. Image row 0 is the TOP; the grid uses a bottom-origin
convention (grid_row = rows - 1 - img_row), but counts/connectivity are
flip-invariant so the decode here operates directly on image rows.
"""

from collections import deque

import numpy as np

OCCUPIED = 100
FREE = 0
UNKNOWN = -1


def decode(img, occupied_thresh, free_thresh):
    """Decode a grayscale image into three boolean masks (occ, free, unk).

    img is a uint8 grayscale array in IMAGE-row order (row 0 = top). Returns
    ``(occ_mask, free_mask, unk_mask)`` boolean arrays of the same shape; the
    three masks are mutually exclusive and exhaustive.
    """
    p = (255.0 - img.astype(np.float64)) / 255.0
    occ = p > occupied_thresh
    free = p < free_thresh
    unk = ~(occ | free)
    return occ, free, unk


def decode_grid(img, occupied_thresh, free_thresh):
    """Decode into an int16 grid of OCCUPIED/FREE/UNKNOWN level codes (image order)."""
    occ, free, unk = decode(img, occupied_thresh, free_thresh)
    grid = np.full(img.shape, UNKNOWN, dtype=np.int16)
    grid[free] = FREE
    grid[occ] = OCCUPIED
    return grid


def histogram(img):
    """Grayscale histogram as a 256-length int array (level -> pixel count)."""
    return np.bincount(img.ravel(), minlength=256)


def color_preview(img, occupied_thresh, free_thresh):
    """Build a BGR preview: OCCUPIED=black, FREE=white, UNKNOWN=orange.

    Returned in BGR (OpenCV) order so it can be written with cv2.imwrite; the
    orange is (B=0, G=165, R=255).
    """
    occ, free, unk = decode(img, occupied_thresh, free_thresh)
    rows, cols = img.shape
    bgr = np.zeros((rows, cols, 3), dtype=np.uint8)
    bgr[free] = (255, 255, 255)
    bgr[occ] = (0, 0, 0)
    bgr[unk] = (0, 165, 255)  # BGR orange
    return bgr


def color_preview_rgb(img, occupied_thresh, free_thresh):
    """RGB preview for matplotlib (OCC=black, FREE=white, UNK=orange)."""
    occ, free, unk = decode(img, occupied_thresh, free_thresh)
    rows, cols = img.shape
    rgb = np.zeros((rows, cols, 3), dtype=np.uint8)
    rgb[free] = (255, 255, 255)
    rgb[occ] = (0, 0, 0)
    rgb[unk] = (255, 165, 0)  # RGB orange
    return rgb


def _two_free_cells(free_mask):
    """Pick two FREE cells far apart (first and last in row-major scan) for the
    connectivity probe. Returns (rc_a, rc_b) or (None, None) if < 2 FREE cells."""
    free_rc = np.argwhere(free_mask)
    if free_rc.shape[0] < 2:
        return None, None
    a = tuple(int(x) for x in free_rc[0])
    b = tuple(int(x) for x in free_rc[-1])
    return a, b


def connectivity(occ_mask, free_mask=None):
    """4-connected reachability between two FREE cells (point-robot model).

    OCCUPIED cells block; FREE and UNKNOWN are traversable. With only ``occ_mask``
    the two probe endpoints are the first/last non-OCCUPIED cells in row-major
    scan; pass ``free_mask`` to probe between actual FREE cells. Returns True if
    reachable, False if blocked, and None if there are fewer than two probe cells.
    """
    rows, cols = occ_mask.shape
    if free_mask is None:
        probe = ~occ_mask
    else:
        probe = free_mask
    a, b = _two_free_cells(probe)
    if a is None:
        return None
    if occ_mask[a] or occ_mask[b]:
        return False
    visited = np.zeros(occ_mask.shape, dtype=bool)
    q = deque([a])
    visited[a] = True
    while q:
        r, c = q.popleft()
        if (r, c) == b:
            return True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr, nc] \
                    and not occ_mask[nr, nc]:
                visited[nr, nc] = True
                q.append((nr, nc))
    return False
