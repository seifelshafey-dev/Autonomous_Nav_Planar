#!/usr/bin/env python3
"""Draw the stop zone figure: where a single scan return blocks the command, for three commands.

Each panel shows the robot footprint (drawn as a square of half-side r_in, right for the shipped
robot) and the region of the robot frame in which one scan return blocks the command, for a straight
run, an arc and a turn on the spot. Drawn from the
JSON written by stop_zone_dump.

Usage:
    python3 plot_stop_zone.py <stop_zone.json> <out.png>
"""
import argparse
import json
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch

ap = argparse.ArgumentParser(description="Draw the stop zone figure.")
ap.add_argument("stop_zone", help="JSON written by stop_zone_dump")
ap.add_argument("out", help="PNG file to write")
args = ap.parse_args()
d = json.load(open(args.stop_zone)); n, lo, st, r = d["n"], d["lo"], d["step"], d["r_in"]
INK, MUTED, GRID, BLUE, LIGHT = "#1f2933", "#5c6773", "#d9dee4", "#3b7dbf", "#9fc3e6"
names = ["straight, %.1f m/s", "arc, %.2f m/s and %.2f rad/s", "turn on the spot, %.1f rad/s"]
fig, axes = plt.subplots(1, 3, figsize=(12.6, 4.9), dpi=150, sharey=True)
axis = np.linspace(lo, lo + (n-1)*st, n)
for ax, c, nm in zip(axes, d["cases"], names):
    m = np.frombuffer(c["mask"].encode(), dtype=np.uint8).reshape(n, n) - 48
    ax.contourf(axis, axis, m, levels=[0.5, 1.5], colors=[LIGHT], alpha=0.75, zorder=2)
    ax.contour(axis, axis, m, levels=[0.5], colors=[BLUE], linewidths=1.4, zorder=3)
    ax.add_patch(Rectangle((-r, -r), 2*r, 2*r, facecolor="white", edgecolor=INK, lw=1.8, zorder=4))
    ax.plot(0, 0, "o", color=INK, markersize=5, zorder=5)
    ax.annotate("", xy=(0.2, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.4), zorder=5)
    t = nm % ((c["v"],) if c["w"] == 0 else ((c["w"],) if c["v"] == 0 else (c["v"], c["w"])))
    ax.set_title(t, fontsize=10.5, color=INK, loc="left")
    ax.set_xlim(-0.5, 0.75); ax.set_ylim(-0.55, 0.6); ax.set_aspect("equal")
    ax.set_xlabel("x (m), robot frame", color=MUTED); ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(color=GRID, lw=0.6, zorder=1); ax.set_axisbelow(True)
    for sp in ax.spines.values(): sp.set_color(GRID)
axes[0].set_ylabel("y (m)", color=MUTED)
fig.legend(handles=[Patch(facecolor="white", edgecolor=INK, lw=1.8, label="robot footprint now"),
                    Patch(facecolor=LIGHT, edgecolor=BLUE, lw=1.4, alpha=0.75, label="stop zone: a scan return here blocks the command")],
           loc="lower center", ncol=2, fontsize=9, frameon=False, bbox_to_anchor=(0.5, -0.02))
fig.suptitle("Where the robot will be by the time it has stopped (reaction time %.1f s)" % d["tr"], fontsize=11.5, color=INK, x=0.012, ha="left")
fig.tight_layout(rect=(0, 0.05, 1, 0.97)); fig.savefig(args.out, bbox_inches="tight", pad_inches=0.15); print("saved")
