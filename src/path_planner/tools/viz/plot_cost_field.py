#!/usr/bin/env python3
"""Draw the cost and heuristic figure from the JSON written by cost_field_dump.

Top: what a metre of travel costs at each place on the map. Bottom: the obstacle-aware estimate of the
cost from each place to the goal, with the planned path over both.

Usage:
    python3 plot_cost_field.py <cost_field.json> <out.png>
"""
import argparse
import json
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, ListedColormap

ap = argparse.ArgumentParser(description="Draw the cost and heuristic figure.")
ap.add_argument("cost_field", help="JSON written by cost_field_dump")
ap.add_argument("out", help="PNG file to write")
args = ap.parse_args()
d = json.load(open(args.cost_field)); h = d["h"]; C, R = d["cols"], d["rows"]
ch = np.array(d["charge"]).reshape(R, C); fl = np.array(d["field"]).reshape(R, C)
path = np.array(d["path"]).reshape(-1, 2)
INK, MUTED, GRID, WALL, ACC = "#1f2933", "#5c6773", "#d9dee4", "#2b3440", "#d9541e"
ext = [0, C*h, 0, R*h]
fig, (ax, az) = plt.subplots(2, 1, figsize=(9.2, 11.6), dpi=150)
wall = np.where(ch < 0, 1.0, np.nan)
def base(a, title):
    a.imshow(wall, origin="lower", extent=ext, cmap=ListedColormap([WALL]), zorder=3, interpolation="nearest")
    if len(path): a.plot(path[:, 0], path[:, 1], color=ACC, lw=2.0, zorder=5)
    a.plot(*d["start"], "o", color=INK, markeredgecolor="white", markersize=8, zorder=6)
    a.plot(*d["goal"], "*", color=INK, markeredgecolor="white", markersize=15, zorder=6)
    a.text(d["start"][0], d["start"][1] - 0.7, "start", ha="center", fontsize=9, color=INK, zorder=7)
    a.text(d["goal"][0], d["goal"][1] - 0.7, "goal", ha="center", fontsize=9, color=INK, zorder=7)
    a.set_title(title, fontsize=11, color=INK, loc="left")
    a.set_xlabel("x (m)", color=MUTED); a.set_ylabel("y (m)", color=MUTED); a.set_aspect("equal")
    a.tick_params(colors=MUTED, labelsize=8)
    for sp in a.spines.values(): sp.set_color(GRID)

# ---- top: cost of one metre of travel
im = ax.imshow(np.where(ch < 0, np.nan, ch), origin="lower", extent=ext, cmap="Blues", norm=LogNorm(1, 45), zorder=1, interpolation="nearest")
base(ax, "What a metre of travel costs at each place (1 = open floor)")
cb = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02, ticks=[1, 2, 5, 10, 20, 40]); cb.ax.set_yticklabels(["1", "2", "5", "10", "20", "40"]); cb.ax.tick_params(labelsize=8, colors=MUTED)
ax.annotate("unknown space\n(%.1f per metre and up)" % (1 + d["u"]), xy=(10.7, 2.6), xytext=(11.4, 1.3), fontsize=9, color=INK,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8), zorder=8, bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=1.5))
ax.annotate("dearer near walls", xy=(10.45, 5.6), xytext=(11.0, 5.2), fontsize=9, color=INK,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8), zorder=8, bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=1.5))

# ---- bottom: obstacle-aware cost to the goal
f = np.where(fl < 0, np.nan, fl); top = np.nanmax(f)
im2 = az.imshow(f, origin="lower", extent=ext, cmap="Blues", vmin=0, vmax=top, zorder=1, interpolation="nearest")
xs = (np.arange(C) + 0.5) * h; ys = (np.arange(R) + 0.5) * h
cs = az.contour(xs, ys, f, levels=np.arange(5, top, 5), colors="white", linewidths=0.8, zorder=2)
base(az, "Estimated cost from each place to the goal (the obstacle-aware estimate; lines every 5)")
cb2 = fig.colorbar(im2, ax=az, fraction=0.03, pad=0.02); cb2.ax.tick_params(labelsize=8, colors=MUTED)
fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight", pad_inches=0.15); print("saved")
