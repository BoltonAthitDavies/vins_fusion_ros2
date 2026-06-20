#!/usr/bin/env python3
"""Generate figures/live_stereo.gif — the LIVE closed loop for the STEREO-ONLY
(visual-odometry) variant, as a companion to live_mode.gif (stereo+IMU).

Key differences from the stereo+IMU loop:
  * NO IMU  -> no 200 Hz inner rate; VINS updates once per 20 Hz stereo frame.
  * metric scale comes from the 0.5 m stereo baseline (depth = f*B / disparity),
    not from IMU — so stereo-only is fully metric AND deterministic.

Run:  python figures/make_livemode_stereo_gif.py
"""
import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Polygon, Circle
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_GIF = os.path.join(HERE, "live_stereo.gif")

FPS = 20
FRAMES_PER_LOOP = 40
N_LOOP = 3
TOTAL = FRAMES_PER_LOOP * N_LOOP

CX, CY, R = 5.0, 5.0, 2.1       # ring center + token-orbit radius
NODE_R = 2.8                    # node boxes sit just outside the orbit
CYCLE_DT = 0.05                 # sim seconds per stereo/control cycle (20 Hz)

COL_BELT = "#c9ced6"
COL_BOX = "#eef1f6"
COL_ACCENT = "#2d6cdf"
COL_CAR = "#2ca02c"
COL_EDGE = "#5b6472"

# Ring nodes, clockwise from top — stereo-only pipeline (no IMU, no GPS).
NODES = [
    ("CARLA (sync)",  "200 Hz world tick"),
    ("stereo only",   "20 Hz · no IMU"),
    ("VINS-Fusion",   "stereo VO → {x,y,ψ,v}"),
    ("sampling MPC",  "bicycle rollout"),
    ("apply + tick",  "→ next frame"),
]
N = len(NODES)
NODE_DEG = [90 - k * (360 / N) for k in range(N)]
NODE_XY = [(CX + NODE_R * math.cos(math.radians(d)), CY + NODE_R * math.sin(math.radians(d)))
           for d in NODE_DEG]


def ang_norm(a):
    return (a + 180) % 360 - 180


fig, ax = plt.subplots(figsize=(7.4, 6.5), dpi=95)


def draw_node(idx, hot):
    x, y = NODE_XY[idx]
    title, sub = NODES[idx]
    fc = "#dce8ff" if hot else COL_BOX
    ec = COL_ACCENT if hot else COL_EDGE
    lw = 2.4 if hot else 1.3
    if hot:
        ax.add_patch(Circle((x, y), 0.95, facecolor=COL_ACCENT, alpha=0.12, zorder=3))
    ax.add_patch(FancyBboxPatch((x - 0.92, y - 0.5), 1.84, 1.0,
                 boxstyle="round,pad=0.02,rounding_size=0.13",
                 facecolor=fc, edgecolor=ec, lw=lw, zorder=4))
    ax.text(x, y + 0.16, title, ha="center", va="center", fontsize=9.5,
            fontweight="bold", zorder=5)
    ax.text(x, y - 0.22, sub, ha="center", va="center", fontsize=7.3,
            color="#555", zorder=5)


def animate(f):
    ax.cla()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.axis("off")

    g = f / FRAMES_PER_LOOP
    loop_i = int(g)
    t = g - loop_i
    tok_deg = 90 - 360 * t

    # header
    ax.text(5.0, 9.68, "LIVE — stereo-only (visual odometry)", ha="center",
            fontsize=13, fontweight="bold")
    ax.text(5.0, 9.24, "no IMU · metric scale from the 0.5 m baseline · deterministic",
            ha="center", fontsize=9, style="italic", color="#555")

    # ring track
    th = np.linspace(0, 2 * np.pi, 240)
    ax.plot(CX + R * np.cos(th), CY + R * np.sin(th), color=COL_BELT, lw=7,
            zorder=1, solid_capstyle="round")

    # clockwise arrowheads
    for k in range(N):
        md = (NODE_DEG[k] + NODE_DEG[(k + 1) % N]) / 2
        if k == N - 1:
            md = (NODE_DEG[k] + (NODE_DEG[0] + 360)) / 2
        mx, my = CX + R * math.cos(math.radians(md)), CY + R * math.sin(math.radians(md))
        tangent = math.radians(md - 90)
        dx, dy = math.cos(tangent), math.sin(tangent)
        col = COL_ACCENT if k == N - 1 else "#9aa3b2"
        ax.add_patch(Polygon([[mx + 0.16 * dx, my + 0.16 * dy],
                              [mx - 0.12 * dx + 0.09 * dy, my - 0.12 * dy - 0.09 * dx],
                              [mx - 0.12 * dx - 0.09 * dy, my - 0.12 * dy + 0.09 * dx]],
                             closed=True, facecolor=col, edgecolor="none", zorder=2))

    # comet trail
    for j in range(1, 14):
        a = tok_deg + j * 3.2
        tx, ty = CX + R * math.cos(math.radians(a)), CY + R * math.sin(math.radians(a))
        ax.scatter([tx], [ty], s=70, color=COL_ACCENT, alpha=max(0, 0.5 - j * 0.035),
                   zorder=5)

    diffs = [abs(ang_norm(tok_deg - d)) for d in NODE_DEG]
    active = int(np.argmin(diffs))
    for k in range(N):
        draw_node(k, hot=(k == active and diffs[k] < 34))

    tx, ty = CX + R * math.cos(math.radians(tok_deg)), CY + R * math.sin(math.radians(tok_deg))
    ax.scatter([tx], [ty], s=240, color=COL_ACCENT, edgecolor="white",
               linewidth=1.5, zorder=7)

    ax.text(2.95, 7.5, "closed loop", ha="center", fontsize=8,
            color=COL_ACCENT, fontweight="bold")

    # ---- center: stereo-only runs at the 20 Hz frame rate (one update / cycle).
    # A loading bar fills once per loop -> one stereo frame -> one VINS update.
    # (Contrast the stereo+IMU GIF, whose meter ticks 10× per cycle at 200 Hz.)
    ax.text(CX, CY + 0.5, "one 20 Hz stereo cycle", ha="center", va="center",
            fontsize=8.0, color="#555", fontweight="bold")
    bar_w, bar_h, bar_y = 2.0, 0.22, CY + 0.1
    ax.add_patch(FancyBboxPatch((CX - bar_w / 2, bar_y - bar_h / 2), bar_w, bar_h,
                 boxstyle="round,pad=0.0,rounding_size=0.11",
                 facecolor="#e6e9ef", edgecolor="#c7ccd6", lw=1.0, zorder=3))
    ax.add_patch(Rectangle((CX - bar_w / 2 + 0.03, bar_y - bar_h / 2 + 0.03),
                 max(0.0, (bar_w - 0.06) * t), bar_h - 0.06,
                 facecolor=COL_ACCENT, edgecolor="none", zorder=4))
    ax.text(CX, bar_y, f"{int(t * 100):3d}%", ha="center", va="center",
            fontsize=7.0, color="white" if t > 0.5 else "#888", zorder=5)
    ax.text(CX, CY - 0.32, "1 stereo frame  →  1 VINS update", ha="center", va="center",
            fontsize=7.6, color="#777")
    ax.text(CX, CY - 0.58, "no IMU · scale ← 0.5 m baseline", ha="center",
            va="center", fontsize=7.2, color="#9aa3b2")

    # ---- status bar ----
    sim_t = g * CYCLE_DT
    ax.text(0.4, 8.82,
            f"sim t = {sim_t:5.3f} s      ·      frame #{loop_i}      ·      VINS: 1 update / frame",
            ha="left", fontsize=8.6, color="#333")
    ax.text(9.6, 8.82, "backlog 0", ha="right", fontsize=8.8, color=COL_CAR)

    # ---- ego car strip ----
    ax.add_patch(Rectangle((0.7, 0.55), 8.6, 0.9, facecolor="#f0f2f5",
                           edgecolor="#cfd5de", zorder=1))
    for xd in np.arange(1.0, 9.3, 0.7):
        ax.plot([xd, xd + 0.32], [1.0, 1.0], color="#cfd5de", lw=2, zorder=2)
    car_x = 1.2 + (g / N_LOOP) * 7.2
    ax.add_patch(FancyBboxPatch((car_x - 0.32, 0.78), 0.64, 0.42,
                 boxstyle="round,pad=0.02,rounding_size=0.08",
                 facecolor=COL_CAR, edgecolor="#1d6b1d", lw=1.2, zorder=4))
    ax.scatter([car_x - 0.16, car_x + 0.16], [0.78, 0.78], s=24, color="#222", zorder=5)
    ax.text(5.0, 0.28, "bit-identical run-to-run — stereo VO is the most repeatable variant",
            ha="center", fontsize=7.8, color="#666")
    return []


anim = FuncAnimation(fig, animate, frames=TOTAL, interval=1000 / FPS, blit=False)
anim.save(OUT_GIF, writer=PillowWriter(fps=FPS))
print("wrote", OUT_GIF)
