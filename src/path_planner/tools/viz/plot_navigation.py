#!/usr/bin/env python3
"""Draw the navigation figure: the five navigator modes and one escape move.

Top: the five modes and the transitions between them. This panel is laid out by hand in this script
from the transition code in src/core/navigation/navigation_mode.cpp; it is not generated from it.
Bottom: one escape move from a start inside a wall, drawn from the JSON written by escape_move_dump.

Usage:
    python3 plot_navigation.py <escape_move.json> <out.png>
"""
import argparse
import json
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser(description="Draw the navigation figure.")
ap.add_argument("escape_move", help="JSON written by escape_move_dump")
ap.add_argument("out", help="PNG file to write")
args = ap.parse_args()
d = json.load(open(args.escape_move))
INK, MUTED, GRID, WALL, BLUE, ACC, LIGHT, RED = "#1f2933", "#5c6773", "#d9dee4", "#2b3440", "#3b7dbf", "#d9541e", "#9fc3e6", "#c0392b"
fig, (ax, az) = plt.subplots(2, 1, figsize=(9.2, 10.4), dpi=150, gridspec_kw=dict(height_ratios=[1.25, 1]))

# ---- top: the modes (hand-laid-out)
pos = {"Idle": (1.0, 5.0), "Planning": (4.3, 5.0), "Following": (8.6, 5.0), "Clearing": (4.3, 1.4), "Blocked": (8.6, 1.4)}
W, H = 1.9, 0.8
for name, (x, y) in pos.items():
    ax.add_patch(FancyBboxPatch((x - W/2, y - H/2), W, H, boxstyle="round,pad=0.02,rounding_size=0.12", facecolor="white",
                                edgecolor=ACC if name == "Blocked" else BLUE, linewidth=1.8, zorder=3))
    ax.text(x, y, name, ha="center", va="center", fontsize=11, color=INK, zorder=4)
def arrow(a, b, label, rad=0.0, da=(0, 0), db=(0, 0), lab=(0, 0), ha="center", at=None):
    (x0, y0), (x1, y1) = pos[a], pos[b]
    p0, p1 = (x0 + da[0], y0 + da[1]), (x1 + db[0], y1 + db[1])
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=13, color=MUTED, lw=1.2,
                                 connectionstyle=f"arc3,rad={rad}", zorder=2))
    lx, ly = at if at else ((p0[0] + p1[0]) / 2 + lab[0], (p0[1] + p1[1]) / 2 + lab[1])
    ax.text(lx, ly, label, fontsize=8.5, color=INK, ha=ha, va="center", zorder=5,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.92, pad=1.0))
arrow("Idle", "Planning", "new goal", da=(W/2, 0), db=(-W/2, 0), lab=(0, 0.22))
arrow("Planning", "Following", "path found", da=(W/2, 0.15), db=(-W/2, 0.15), lab=(0, 0.25))
arrow("Following", "Planning", "path now blocked,\nor no progress", da=(-W/2, -0.15), db=(W/2, -0.15), at=(7.05, 4.5))
arrow("Following", "Idle", "arrived", rad=0.3, da=(-0.3, H/2), db=(0.3, H/2), lab=(0, 1.3))
arrow("Planning", "Clearing", "start blocked", da=(-0.35, -H/2), db=(-0.35, H/2), lab=(-0.12, 0), ha="right")
arrow("Clearing", "Planning", "escape move\ndriven", da=(0.35, H/2), db=(0.35, -H/2), at=(4.77, 2.75), ha="left")
arrow("Planning", "Blocked", "no path, or no\nescape moves left", da=(W/2, -0.3), db=(-W/2 + 0.25, H/2), at=(7.55, 3.75))
# The "2 s" is the shipped blocked_retry_period, written by hand; it does not follow the config.
arrow("Blocked", "Planning", "retry every 2 s", da=(-W/2, 0.1), db=(W/2 - 0.3, -H/2), at=(6.25, 2.05))
arrow("Clearing", "Blocked", "no escape found, or\nescape move stalled", da=(W/2, -0.2), db=(-W/2, -0.2), lab=(0, -0.38))
arrow("Following", "Blocked", "no progress,\nreplans used up", da=(0.3, -H/2), db=(0.3, H/2), lab=(0.12, 0), ha="left")
ax.text(8.6, 0.42, "reasons: no path to goal, no escape, stalled\n(stalled is not retried)", fontsize=8.5, color=MUTED, ha="center", va="top")
ax.set_xlim(-0.3, 11.2); ax.set_ylim(-0.4, 7.2); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("The five modes (a new goal goes to Planning from any mode)", fontsize=11, color=INK, loc="left")

# ---- bottom: one escape move from the real search
h, wx, r = d["h"], d["wall_x"], d["r_in"]; sx, sy = d["start"]; ex = sx + (-1 if d["v"] < 0 else 1) * d["travel"]
for i in range(int(4.3 / h), int(5.6 / h) + 1):
    az.plot([i*h, i*h], [4.55, 5.5], color=GRID, lw=0.6, zorder=1)
for j in range(int(4.55 / h), int(5.5 / h) + 1):
    az.plot([4.3, 5.6], [j*h, j*h], color=GRID, lw=0.6, zorder=1)
az.add_patch(Rectangle((wx, 4.55), 5.6 - wx, 0.95, facecolor=WALL, edgecolor="none", zorder=2))
az.text(wx + 0.14, 5.02, "wall", color="white", fontsize=10, ha="center", va="center", rotation=90, zorder=3)
az.add_patch(Rectangle((sx - r, sy - r), 2*r, 2*r, facecolor="none", edgecolor=RED, lw=1.8, ls=(0, (4, 2)), zorder=4))
az.add_patch(Rectangle((ex - r, sy - r), 2*r, 2*r, facecolor=LIGHT, alpha=0.35, edgecolor=BLUE, lw=1.8, zorder=3))
K = round(d["travel"] / d["delta"])
for k, (x, c0, c1) in enumerate(d["steps"][:K + 1]):
    az.plot(x, sy, "o", color=(BLUE if c1 else (LIGHT if c0 else RED)), markeredgecolor="white", markersize=7, zorder=6)
az.annotate("", xy=(ex, sy + 0.07), xytext=(ex + d["stop_short"], sy + 0.07), arrowprops=dict(arrowstyle="<->", color=INK, lw=1.0), zorder=6)
az.text(ex + d["stop_short"] / 2, sy + 0.095, "the follower may stop\nanywhere in the last %.2f m" % d["stop_short"], fontsize=8.5, color=INK, ha="center", va="bottom", bbox=dict(facecolor="white", edgecolor="none", alpha=0.85, pad=1.0), zorder=7)
az.text(wx - 0.02, sy - r - 0.04, "start: %.2f m into the wall" % (sx + r - wx), fontsize=9, color=RED, ha="right", va="top")
az.text(ex - r, sy + r + 0.04, "end of the move", fontsize=9, color=BLUE, ha="left", va="bottom")
az.set_xlim(4.3, 5.6); az.set_ylim(4.55, 5.5); az.set_aspect("equal")
az.set_xlabel("x (m)", color=MUTED); az.set_ylabel("y (m)", color=MUTED); az.tick_params(colors=MUTED, labelsize=8)
for sp in az.spines.values(): sp.set_color(GRID)
az.set_title("One escape move: straight back %.2f m in %.2f m steps, each step checked" % (d["travel"], d["delta"]), fontsize=11, color=INK, loc="left")
az.legend(handles=[
    Line2D([0], [0], marker="o", color="none", markerfacecolor=RED, markeredgecolor="white", markersize=7, label="robot centre: start still blocked"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=LIGHT, markeredgecolor="white", markersize=7, label="start clear"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=BLUE, markeredgecolor="white", markersize=7, label="clear with one cell to spare"),
], loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3, fontsize=8.5, frameon=False)
fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight", pad_inches=0.15); print("saved")
