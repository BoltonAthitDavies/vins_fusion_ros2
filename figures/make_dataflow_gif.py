#!/usr/bin/env python3
"""Generate figures/online_vs_offline.gif — a side-by-side animation of the two
replay paths described in README sec 4.4:

  ONLINE  : ros2 bag play --clock -> DDS pub/sub -> VINS.  Lossy: when VINS lags,
            frames are DROPPED at the DDS buffer (non-deterministic).
  OFFLINE : vins_bag_reader -> SequentialReader hands EVERY message straight into
            VINS in header.stamp order.  Lossless + deterministic.

Deterministic by construction (fixed drop pattern) so the GIF is reproducible.
Run:  python figures/make_dataflow_gif.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_GIF = os.path.join(HERE, "online_vs_offline.gif")

# ---- timeline -------------------------------------------------------------
FPS = 20
TOTAL = 150                  # frames -> 7.5 s loop
SRC_X, BUF_X, DST_X = 1.4, 6.6, 8.8   # source / DDS-buffer / VINS x positions
BELT_A, BELT_B = SRC_X + 0.85, DST_X - 0.85   # message travel endpoints (box edges)
TRAVEL = 26                 # frames to cross the belt
SPACING = 6                 # frames between emitted messages
N_MSG = 20
ONLINE_Y, OFFLINE_Y = 7.0, 2.6

# Frames the online path drops at the DDS buffer (congestion bursts).
DROP = {3, 4, 8, 12, 13, 17}

COL_MSG = "#2d6cdf"
COL_DROP = "#d62728"
COL_OK = "#2ca02c"
COL_BELT = "#c9ced6"
COL_BOX = "#eef1f6"


def emit_frame(i):
    return 6 + i * SPACING


def progress(f, i):
    return (f - emit_frame(i)) / TRAVEL


def setup_lane(ax, y, title, src_label, mid_label, dst_label, dst_note):
    ax.add_patch(Rectangle((SRC_X - 0.9, y - 1.7), DST_X - SRC_X + 1.8, 3.4,
                           facecolor="white", edgecolor="none", zorder=0))
    # belt
    ax.plot([SRC_X, DST_X], [y, y], color=COL_BELT, lw=9, solid_capstyle="round", zorder=1)
    # source + consumer boxes
    for x, label in [(SRC_X, src_label), (DST_X, dst_label)]:
        ax.add_patch(FancyBboxPatch((x - 0.75, y - 0.55), 1.5, 1.1,
                     boxstyle="round,pad=0.02,rounding_size=0.15",
                     facecolor=COL_BOX, edgecolor="#5b6472", lw=1.4, zorder=4))
        ax.text(x, y, label, ha="center", va="center", fontsize=9,
                fontweight="bold", zorder=5)
    ax.text((SRC_X + DST_X) / 2, y + 1.15, mid_label, ha="center", va="center",
            fontsize=8.5, style="italic", color="#444")
    ax.text(DST_X, y - 1.0, dst_note, ha="center", va="center", fontsize=7.5,
            color="#666")
    ax.text(SRC_X - 0.95, y + 1.45, title, ha="left", va="center", fontsize=11,
            fontweight="bold")


fig, ax = plt.subplots(figsize=(8.6, 5.0), dpi=110)
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.axis("off")

# Divider + header
ax.plot([0.3, 9.7], [4.85, 4.85], color="#dde1e8", lw=1)
ax.text(5.0, 9.55, "Same VINS code, two replay paths", ha="center",
        fontsize=12.5, fontweight="bold")

setup_lane(ax, ONLINE_Y, "ONLINE", "rosbag", "DDS pub/sub  (wall-clock × rate)",
           "VINS", "lossy · non-deterministic")
setup_lane(ax, OFFLINE_Y, "OFFLINE", "rosbag", "SequentialReader  (header.stamp order)",
           "VINS", "lossless · deterministic · ~10×")

# DDS buffer marker on the online lane
ax.text(BUF_X, ONLINE_Y + 0.7, "buffer", ha="center", fontsize=7, color="#888")

# dynamic artists
on_dots = ax.scatter([], [], s=130, zorder=6)
off_dots = ax.scatter([], [], s=130, zorder=6)
on_counter = ax.text(SRC_X - 0.95, ONLINE_Y - 1.5, "", fontsize=8.5, color="#333")
off_counter = ax.text(SRC_X - 0.95, OFFLINE_Y - 1.5, "", fontsize=8.5, color="#333")
busy = ax.text(DST_X, ONLINE_Y + 0.75, "", ha="center", fontsize=7.5,
               color=COL_DROP, fontweight="bold")
clock = ax.text(9.55, 9.05, "", ha="right", fontsize=8, color="#777")


def animate(f):
    on_pos, on_col = [], []
    off_pos, off_col = [], []
    delivered_on = dropped_on = delivered_off = 0

    for i in range(N_MSG):
        p = progress(f, i)
        if p < 0:
            continue
        # ---- OFFLINE: never dropped, all reach VINS ----
        if p <= 1.0:
            x = BELT_A + p * (BELT_B - BELT_A)
            off_pos.append((x, OFFLINE_Y))
            off_col.append(COL_MSG)
        if p >= 1.0:
            delivered_off += 1

        # ---- ONLINE: dropped messages fall off at the buffer ----
        if i in DROP:
            # travels to buffer, then drops down & fades
            bp = (BUF_X - BELT_A) / (BELT_B - BELT_A)   # progress fraction at buffer
            if p <= bp:
                x = BELT_A + p * (BELT_B - BELT_A)
                on_pos.append((x, ONLINE_Y))
                on_col.append(COL_MSG)
            elif p <= bp + 0.55:
                fall = (p - bp) / 0.55
                on_pos.append((BUF_X, ONLINE_Y - fall * 2.0))
                on_col.append(COL_DROP)
            if p > bp:
                dropped_on += 1
        else:
            if p <= 1.0:
                x = BELT_A + p * (BELT_B - BELT_A)
                on_pos.append((x, ONLINE_Y))
                on_col.append(COL_MSG)
            if p >= 1.0:
                delivered_on += 1

    on_dots.set_offsets(on_pos if on_pos else [(-5, -5)])
    on_dots.set_color(on_col if on_col else [COL_MSG])
    off_dots.set_offsets(off_pos if off_pos else [(-5, -5)])
    off_dots.set_color(off_col if off_col else [COL_MSG])

    on_counter.set_text(f"delivered {delivered_on}   dropped {dropped_on}")
    off_counter.set_text(f"delivered {delivered_off}   dropped 0")
    busy.set_text("VINS busy → drop" if dropped_on and (f // 6) % 2 == 0 else "")
    clock.set_text(f"t = {f / FPS:4.1f} s  (wall clock)")
    return on_dots, off_dots, on_counter, off_counter, busy, clock


anim = FuncAnimation(fig, animate, frames=TOTAL, interval=1000 / FPS, blit=False)
anim.save(OUT_GIF, writer=PillowWriter(fps=FPS))
print("wrote", OUT_GIF)
