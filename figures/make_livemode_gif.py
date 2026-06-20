#!/usr/bin/env python3
"""Generate figures/live_mode.gif — an animation of the LIVE closed loop from
README sec 4.4:

  CARLA tick() -> stereo/IMU/GNSS -> VINS -> state {x,y,yaw,v} -> MPC ->
  VehicleControl{throttle,steer,brake} -> apply -> request next tick().

Synchronous & closed: the estimate feeds control feeds the NEXT sensor frame, so
SLAM error actually steers the car (unlike the open-loop replay paths). A token
circulates the ring; each completed cycle advances the sim clock and the ego car.

Run:  python figures/make_livemode_gif.py
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
OUT_GIF = os.path.join(HERE, "live_mode.gif")

FPS = 20
FRAMES_PER_LOOP = 40
N_LOOP = 3
TOTAL = FRAMES_PER_LOOP * N_LOOP

CX, CY, R = 5.0, 5.0, 2.1       # ring center + token-orbit radius
NODE_R = 2.8                    # node boxes sit just outside the orbit
DT_TICK = 0.005                # s per CARLA world tick (200 Hz)
CYCLE_DT = 0.05                # sim seconds advanced per control cycle (~20 Hz)

COL_BELT = "#c9ced6"
COL_BOX = "#eef1f6"
COL_ACCENT = "#2d6cdf"
COL_HOT = "#d62728"
COL_CAR = "#2ca02c"
COL_EDGE = "#5b6472"

# Ring nodes, clockwise from top — the actual components we implement.
NODES = [
    ("CARLA (sync)",  "200 Hz world tick"),
    ("sensors",       "stereo·IMU·GNSS"),
    ("VINS-Fusion",   "state {x,y,ψ,v}"),
    ("sampling MPC",  "bicycle rollout"),
    ("apply + tick",  "→ next frame"),
]
N = len(NODES)
NODE_DEG = [90 - k * (360 / N) for k in range(N)]   # clockwise placement
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
    if hot:                                   # glow
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
    t = g - loop_i                                   # 0..1 within this loop
    tok_deg = 90 - 360 * t                           # clockwise

    # header (title -> subtitle -> status bar, each on its own clear line)
    ax.text(5.0, 9.68, "LIVE — the closed control loop", ha="center",
            fontsize=13, fontweight="bold")
    ax.text(5.0, 9.24, "synchronous · lossless · the estimate steers the car",
            ha="center", fontsize=9, style="italic", color="#555")

    # faint ring track
    th = np.linspace(0, 2 * np.pi, 240)
    ax.plot(CX + R * np.cos(th), CY + R * np.sin(th), color=COL_BELT, lw=7,
            zorder=1, solid_capstyle="round")

    # clockwise arrowheads at segment midpoints
    for k in range(N):
        md = (NODE_DEG[k] + NODE_DEG[(k + 1) % N]) / 2
        if k == N - 1:                               # wrap midpoint top-left→top
            md = (NODE_DEG[k] + (NODE_DEG[0] + 360)) / 2
        mx, my = CX + R * math.cos(math.radians(md)), CY + R * math.sin(math.radians(md))
        tangent = math.radians(md - 90)              # clockwise tangent
        dx, dy = math.cos(tangent), math.sin(tangent)
        closing = (k == N - 1)
        col = COL_ACCENT if closing else "#9aa3b2"
        ax.add_patch(Polygon([[mx + 0.16 * dx, my + 0.16 * dy],
                              [mx - 0.12 * dx + 0.09 * dy, my - 0.12 * dy - 0.09 * dx],
                              [mx - 0.12 * dx - 0.09 * dy, my - 0.12 * dy + 0.09 * dx]],
                             closed=True, facecolor=col, edgecolor="none", zorder=2))

    # comet trail behind the token
    for j in range(1, 14):
        a = tok_deg + j * 3.2
        tx, ty = CX + R * math.cos(math.radians(a)), CY + R * math.sin(math.radians(a))
        ax.scatter([tx], [ty], s=70, color=COL_ACCENT, alpha=max(0, 0.5 - j * 0.035),
                   zorder=5)

    # nodes (highlight the nearest to the token)
    diffs = [abs(ang_norm(tok_deg - d)) for d in NODE_DEG]
    active = int(np.argmin(diffs))
    for k in range(N):
        draw_node(k, hot=(k == active and diffs[k] < 34))

    # the token
    tx, ty = CX + R * math.cos(math.radians(tok_deg)), CY + R * math.sin(math.radians(tok_deg))
    ax.scatter([tx], [ty], s=240, color=COL_ACCENT, edgecolor="white",
               linewidth=1.5, zorder=7)

    # closed-loop label near the apply→CARLA closing edge (upper-left)
    ax.text(2.95, 7.5, "closed loop", ha="center", fontsize=8,
            color=COL_ACCENT, fontweight="bold")
    # dual-rate explainer: ONE lap of this ring = one 20 Hz control cycle, which
    # spans TEN 200 Hz world ticks (10 IMU samples + 1 stereo frame).
    ax.text(CX, CY + 0.66, "one 20 Hz control cycle", ha="center",
            va="center", fontsize=8.0, color="#555", fontweight="bold")
    ticks_in_frame = min(10, int(t * 10) + 1)
    for i in range(10):
        px = CX - 0.9 + i * (1.8 / 9)
        c = COL_ACCENT if i < ticks_in_frame else "#d6dae3"
        ax.add_patch(Circle((px, CY + 0.2), 0.062, facecolor=c, edgecolor="none", zorder=4))
    ax.text(CX, CY - 0.16, f"world ticks @200 Hz:  {ticks_in_frame}/10", ha="center",
            va="center", fontsize=7.5, color="#777")
    ax.text(CX, CY - 0.46, "10 IMU + 1 stereo  =  0.05 s", ha="center",
            va="center", fontsize=7.2, color="#9aa3b2")

    # ---- readout + ego car strip ----
    sim_t = g * CYCLE_DT
    ticks = int(round(sim_t / DT_TICK))
    ax.text(0.4, 8.82,
            f"sim t = {sim_t:5.3f} s      ·      frame #{loop_i}      ·      tick #{ticks:>3}",
            ha="left", fontsize=8.8, color="#333")
    ax.text(9.6, 8.82, "backlog 0", ha="right", fontsize=8.8, color=COL_CAR)

    # road strip
    ax.add_patch(Rectangle((0.7, 0.55), 8.6, 0.9, facecolor="#f0f2f5",
                           edgecolor="#cfd5de", zorder=1))
    for xd in np.arange(1.0, 9.3, 0.7):
        ax.plot([xd, xd + 0.32], [1.0, 1.0], color="#cfd5de", lw=2, zorder=2)
    # ego advances each completed cycle (closed-loop effect)
    car_x = 1.2 + (g / N_LOOP) * 7.2
    ax.add_patch(FancyBboxPatch((car_x - 0.32, 0.78), 0.64, 0.42,
                 boxstyle="round,pad=0.02,rounding_size=0.08",
                 facecolor=COL_CAR, edgecolor="#1d6b1d", lw=1.2, zorder=4))
    ax.scatter([car_x - 0.16, car_x + 0.16], [0.78, 0.78], s=24, color="#222", zorder=5)
    ax.text(5.0, 0.28, "ego advances every cycle — SLAM error here actually moves the car",
            ha="center", fontsize=7.8, color="#666")
    return []


anim = FuncAnimation(fig, animate, frames=TOTAL, interval=1000 / FPS, blit=False)
anim.save(OUT_GIF, writer=PillowWriter(fps=FPS))
print("wrote", OUT_GIF)
