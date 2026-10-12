#!/usr/bin/env python3
"""Draw the control-primitives figure from the JSON written by primitives_dump.

Top: the twelve motions of the robot from one pose. Bottom: the states reached after two and three
motions, over the 0.15 m duplicate cells of the shipped planner settings (fixed in this script). The states are continuous and do not snap to
the grid.

Usage:
    python3 plot_primitives.py <primitives.json> <out.png>
"""
import argparse
import json, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Arc
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser(description="Draw the control-primitives figure.")
ap.add_argument("primitives", help="JSON written by primitives_dump")
ap.add_argument("out", help="PNG file to write")
args = ap.parse_args()
d = json.load(open(args.primitives)); h = d["h"]
GRID, INK, MUTED, BLUE, ACC, LIGHT = "#d9dee4", "#1f2933", "#5c6773", "#3b7dbf", "#d9541e", "#9fc3e6"
sx, sy, _ = d["start"]
fig, (ax, az) = plt.subplots(2, 1, figsize=(8.6, 10.6), dpi=150, gridspec_kw=dict(height_ratios=[1, 2.1]))

# ---- top: the twelve motions from one pose
for p in d["prims"]:
    xs = [q[0] for q in p["path"]]; ys = [q[1] for q in p["path"]]
    if p["v"] == 0:                                   # pivot: draw the turn as an arc about the centre
        deg = math.degrees(p["end"][2]); r = 0.045
        ax.add_patch(Arc((sx, sy), 2*r, 2*r, theta1=min(0, deg), theta2=max(0, deg), color=ACC, lw=2.0, zorder=5))
        a = math.radians(deg)
        ax.add_patch(FancyArrowPatch((sx + r*math.cos(a*0.9), sy + r*math.sin(a*0.9)), (sx + r*math.cos(a), sy + r*math.sin(a)),
                                     arrowstyle="-|>", mutation_scale=11, color=ACC, lw=0, zorder=6))
        continue
    col = MUTED if p["v"] < 0 else BLUE
    ax.plot(xs, ys, color=col, lw=2.0, zorder=4, solid_capstyle="round")
    ex, ey, eth = p["path"][-1]
    ax.add_patch(FancyArrowPatch((ex, ey), (ex + 0.035*math.cos(eth), ey + 0.035*math.sin(eth)), arrowstyle="-|>", mutation_scale=11, color=col, lw=1.4, zorder=5))
ax.plot(sx, sy, "o", color=INK, markersize=7, markeredgecolor="white", zorder=7)
ends = {abs(p["v"]/p["w"]): p["path"][-1] for p in d["prims"] if p["w"] > 0 and p["v"] > 0}
for R, (ex, ey, eth) in ends.items():
    ax.text(ex + 0.052*math.cos(eth), ey + 0.052*math.sin(eth) + 0.004, f"{R:.2g}", fontsize=9, color=INK, ha="center", va="center", zorder=8)
ax.text(0.46, -0.095, "numbers at the arcs: turning radius (m)", fontsize=9, color=MUTED, ha="right", va="center", zorder=8)
ax.text(sx + 0.345, sy, "straight", fontsize=9, color=INK, ha="left", va="center", zorder=8)
ax.text(sx - 0.30, sy + 0.014, "reverse", fontsize=9, color=MUTED, ha="left", va="bottom", zorder=8)
ax.text(sx - 0.022, sy + 0.050, "pivot", fontsize=9, color=ACC, ha="right", va="bottom", zorder=8)
ax.set_xlim(-0.33, 0.47); ax.set_ylim(-0.115, 0.125); ax.set_aspect("equal")
ax.set_xlabel("x (m)", color=MUTED); ax.set_ylabel("y (m)", color=MUTED)
ax.set_title(f"The twelve motions from one pose, {d['prims'][0]['dt']:g} s each", fontsize=11, color=INK, loc="left")

# ---- bottom: where two and three motions lead, over the duplicate cells
s = 3 * h                                           # shipped closed_cell_size, 0.15 m on the 0.05 m grid
lv2, lv3 = d["levels"][1], d["levels"][2]
X0, X1, Y0, Y1 = -0.90, 1.05, -0.60, 0.60
i0, i1 = math.floor(X0 / s), math.ceil(X1 / s); j0, j1 = math.floor(Y0 / s), math.ceil(Y1 / s)
for i in range(i0, i1):
    for j in range(j0, j1):
        az.add_patch(Rectangle((i*s, j*s), s, s, facecolor="white", edgecolor=GRID, linewidth=1.0, zorder=1))
az.scatter([q[0] for q in lv3], [q[1] for q in lv3], s=5, color=LIGHT, zorder=3, linewidths=0)
az.scatter([q[0] for q in lv2], [q[1] for q in lv2], s=14, color=BLUE, zorder=4, linewidths=0)
az.plot(sx, sy, "o", color=INK, markersize=7, markeredgecolor="white", zorder=6)
az.set_xlim(i0*s, i1*s); az.set_ylim(j0*s, j1*s); az.set_aspect("equal")
az.set_xlabel("x (m)", color=MUTED); az.set_ylabel("y (m)", color=MUTED)
az.set_title("Where two and three motions lead: nothing snaps to the grid\n(the grid is the 0.15 m cell used only to drop duplicates)", fontsize=11, color=INK, loc="left")
az.legend(handles=[
    Line2D([0], [0], marker="o", color="none", markerfacecolor=INK, markeredgecolor="white", markersize=7, label="start"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=BLUE, markersize=6, label=f"after two motions ({len(lv2)})"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=LIGHT, markersize=4, label=f"after three motions ({len(lv3)})"),
], loc="upper center", bbox_to_anchor=(0.5, -0.10), ncol=3, fontsize=9, frameon=False)
for a in (ax, az):
    a.tick_params(colors=MUTED, labelsize=8)
    for sp in a.spines.values(): sp.set_color(GRID)
fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight", pad_inches=0.15); print("saved")
