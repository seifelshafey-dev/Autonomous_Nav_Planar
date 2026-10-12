#!/usr/bin/env python3
"""Draw the swept-cell cover figure from the JSON written by swept_cover_dump.

Left: the cells cached for one straight motion and one heading bin. Dark cells are the ones the exact
check looks at for some heading in the bin, light cells are the rest of the cached list, and the outlines are the areas swept at five
headings across the bin. Right: a zoom on the start cell and the heading bin, which is all the cache
knows about the start pose.

Usage:
    python3 plot_swept_cover.py <cover.json> <out.png>
"""
import argparse
import json, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, FancyArrowPatch, Wedge
from matplotlib.lines import Line2D

ap = argparse.ArgumentParser(description="Draw the swept-cell cover figure.")
ap.add_argument("cover", help="JSON written by swept_cover_dump")
ap.add_argument("out", help="PNG file to write")
args = ap.parse_args()
d = json.load(open(args.cover)); h = d["h"]
cover = set(map(tuple, d["cover"]))
fan = d["fan"]; union = set()
for f in fan: union |= set(map(tuple, f["cells"]))
NEED, EXTRA, GRID, INK, MUTED, ACC = "#3b7dbf", "#bcd6ee", "#d9dee4", "#1f2933", "#5c6773", "#d9541e"

def body(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return [(x + c*vx - s*vy, y + s*vx + c*vy) for vx, vy in d["verts"]]

def hull(pts):                                   # convex hull, monotone chain
    pts = sorted(set(pts))
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and (out[-1][0]-out[-2][0])*(p[1]-out[-2][1]) - (out[-1][1]-out[-2][1])*(p[0]-out[-2][0]) <= 0:
                out.pop()
            out.append(p)
        return out
    lo, up = half(pts), half(pts[::-1])
    return lo[:-1] + up[:-1]

fig, (ax, az) = plt.subplots(1, 2, figsize=(12.6, 7.4), dpi=150, gridspec_kw=dict(width_ratios=[1.7, 1]))
PAD = 3
I0 = min(i for i, _ in cover) - PAD; I1 = max(i for i, _ in cover) + PAD
J0 = min(j for _, j in cover) - PAD; J1 = max(j for _, j in cover) + PAD
for i in range(I0, I1 + 1):
    for j in range(J0, J1 + 1):
        fc = NEED if (i, j) in union else EXTRA if (i, j) in cover else "white"
        ax.add_patch(Rectangle((i*h, j*h), h, h, facecolor=fc, edgecolor=GRID, linewidth=0.5, zorder=1))

n = len(fan) - 1
picks = [0, n // 4, n // 2, 3 * n // 4, n]       # five headings across the bin, both edges included
for idx in picks:
    f = fan[idx]; edge = idx in (0, n)
    swept = hull(body(*f["start"]) + body(*f["end"]))
    ax.add_patch(Polygon(swept, closed=True, fill=False, edgecolor=INK,
                         linewidth=2.2 if edge else 0.8, linestyle="-" if edge else (0, (4, 3)), zorder=6 if edge else 5))
    ax.add_patch(FancyArrowPatch(f["start"][:2], f["end"][:2], arrowstyle="-|>", mutation_scale=13,
                                 color=ACC, linewidth=1.8, zorder=8))
sx, sy, _ = fan[0]["start"]
ax.plot(sx, sy, "o", color=ACC, markersize=7, markeredgecolor="white", zorder=9)
bb = dict(facecolor="white", edgecolor="none", alpha=0.9, pad=1.5)
tips = {0: ((0.625, -0.275), (0.80, -0.46), "heading 0°"), n: (None, (0.70, 0.66), "heading 22.5°")}
for idx, (corner, pos, name) in tips.items():
    if corner is None:
        hp = hull(body(*fan[idx]["start"]) + body(*fan[idx]["end"])); corner = max(hp, key=lambda q: q[1])
    ax.annotate(name, corner, xytext=pos, fontsize=9.5, color=INK, ha="center", va="center", bbox=bb, zorder=10,
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.9))
ax.set_xlim(I0*h, (I1+1)*h); ax.set_ylim(J0*h, (J1+1)*h); ax.set_aspect("equal")
ax.set_xlabel("x (m)", color=MUTED); ax.set_ylabel("y (m)", color=MUTED)
ax.set_title("One cached list covers every heading in the bin\n"
             f"straight ahead {d['v']*d['dt']:.1f} m, heading bin 0° to {math.degrees(d['delta']):.1f}°", fontsize=11, color=INK, loc="left")
ax.legend(handles=[
    Rectangle((0, 0), 1, 1, facecolor=NEED, edgecolor=GRID, label=f"checked for some heading in the bin ({len(union)})"),
    Rectangle((0, 0), 1, 1, facecolor=EXTRA, edgecolor=GRID, label=f"rest of the cached list ({len(cover - union)})"),
    Rectangle((0, 0), 1, 1, facecolor="white", edgecolor=GRID, label="not cached"),
    Line2D([0], [0], color=INK, lw=2.2, label="area swept at the two edge headings"),
    Line2D([0], [0], color=INK, lw=0.8, linestyle=(0, (4, 3)), label="area swept at headings in between"),
    Line2D([0], [0], color=ACC, lw=1.8, marker="o", markevery=[0], markeredgecolor="white", label="robot centre, start to end"),
], loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, fontsize=9, frameon=False)

# ---- right: the cell and the bin, zoomed
Z = 1
for i in range(-Z, Z + 2):
    for j in range(-Z, Z + 1):
        az.add_patch(Rectangle((i*h, j*h), h, h, facecolor="white", edgecolor=GRID, linewidth=1.2, zorder=1))
az.add_patch(Rectangle((0, 0), h, h, facecolor=ACC, alpha=0.12, edgecolor=ACC, linewidth=2.6, zorder=3))
L = 1.3 * h; th0 = d["theta_k"]; dl = d["delta"]
az.add_patch(Wedge((sx, sy), L, math.degrees(th0), math.degrees(th0 + dl), facecolor=ACC, alpha=0.18, edgecolor=ACC, linewidth=1.2, zorder=5))
for idx in picks:
    th = fan[idx]["start"][2]
    az.add_patch(FancyArrowPatch((sx, sy), (sx + 0.93*L*math.cos(th), sy + 0.93*L*math.sin(th)), arrowstyle="-|>", mutation_scale=12,
                                 color=ACC, linewidth=1.6, zorder=7))
az.plot(sx, sy, "o", color=ACC, markersize=9, markeredgecolor="white", zorder=8)
az.text(h/2, h + 0.004, "the robot's cell", fontsize=9.5, color=ACC, ha="center", va="bottom", zorder=9)
az.text(sx - 0.004, sy - 0.006, "robot centre", fontsize=9.5, color=INK, ha="right", va="top", zorder=9)
az.text(sx + L + 0.004, sy + 0.5*L*math.sin(dl), "heading bin\n0° to 22.5°", fontsize=9.5, color=ACC, ha="left", va="center", zorder=9)
az.set_xlim(-Z*h, (Z+2)*h); az.set_ylim(-Z*h, (Z+1)*h); az.set_aspect("equal")
az.set_xticks([]); az.set_yticks([])
az.set_title("Zoom on the start\nthe cache knows only the cell and the bin", fontsize=11, color=INK, loc="left")
for sp in az.spines.values(): sp.set_color(GRID)
ax.tick_params(colors=MUTED, labelsize=8)
for sp in ax.spines.values(): sp.set_color(GRID)
fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight", pad_inches=0.15); print("saved", len(cover), len(union))
