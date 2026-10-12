#!/usr/bin/env python3
"""Overlay a planned dense path against the robot's executed trajectory on the scenario map.

Reuses plot_search's trace loader / occupancy background to draw the same e2e_demo map, then overlays:
  * the planned dense `/plan` (JSON list of {x, y}) the controller tracked, and
  * the executed `map -> base_link` trajectory (CSV `x,y`) the robot actually drove.
The visual gap between the two is the tracking (cross-track) error — e.g. a pure-pursuit corner-cut shows
as the executed line running slightly inside the planned detour.

Usage:
    python3 plot_plan_vs_executed.py TRACE.json PLANNED.json EXECUTED.csv --save OUT.png
"""
import argparse
import csv
import json
import sys

import numpy as np

import plot_search as ps  # backend selection + load()/occupancy_image() live here

import matplotlib.pyplot as plt


def load_planned(path):
    """Planned dense path: JSON list of {x, y} -> (k, 2) array."""
    with open(path) as f:
        pts = json.load(f)
    return np.array([(p["x"], p["y"]) for p in pts], dtype=float)


def load_executed(path):
    """Executed trajectory: CSV with x,y header -> (m, 2) array."""
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return np.column_stack([xs, ys]) if xs else np.empty((0, 2))


def max_cross_track(executed, planned):
    """Max distance from any executed sample to the planned polyline (segment-wise), in metres."""
    if len(executed) == 0 or len(planned) < 2:
        return 0.0
    worst = 0.0
    for pt in executed:
        best = float("inf")
        for a, b in zip(planned[:-1], planned[1:]):
            ab = b - a
            denom = float(ab @ ab) + 1e-12
            t = np.clip(float((pt - a) @ ab) / denom, 0.0, 1.0)
            best = min(best, float(np.linalg.norm(pt - (a + t * ab))))
        worst = max(worst, best)
    return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("trace", help="search-trace JSON (for the map background: meta + occupancy)")
    ap.add_argument("planned", help="planned dense path JSON ([{x, y}, ...])")
    ap.add_argument("executed", help="executed trajectory CSV (x,y header)")
    ap.add_argument("--save", required=True, help="output PNG path")
    args = ap.parse_args()

    t = ps.load(args.trace)
    meta = t["meta"]
    res, (ox, oy) = meta["resolution"], meta["origin"]
    extent = [ox, ox + meta["cols"] * res, oy, oy + meta["rows"] * res]

    planned = load_planned(args.planned)
    executed = load_executed(args.executed)

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(ps.occupancy_image(meta, t["occupancy"]), cmap="gray", origin="lower",
              extent=extent, vmin=0, vmax=1, zorder=0)

    ax.plot(planned[:, 0], planned[:, 1], "-", color="red", linewidth=2.2, zorder=4,
            label=f"planned (dense samples, {len(planned)})")
    ax.plot(executed[:, 0], executed[:, 1], "-", color="royalblue", linewidth=1.8, alpha=0.9,
            zorder=5, label=f"executed (odom, {len(executed)})")

    start, goal = t["start"], t["goal"]
    ax.plot(start[0], start[1], "o", color="lime", markersize=11, zorder=7, label="start")
    ax.plot(goal[0], goal[1], "o", color="magenta", markersize=11, zorder=7, label="goal")

    gap = max_cross_track(executed, planned)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_title(f"E-E plan vs execution  (max cross-track {gap:.2f} m)")
    ax.legend(loc="upper left", fontsize=9)

    fig.savefig(args.save, dpi=130, bbox_inches="tight")
    print(f"saved {args.save}  (max cross-track gap {gap:.3f} m)")
    ps.maybe_show()


if __name__ == "__main__":
    sys.exit(main())
