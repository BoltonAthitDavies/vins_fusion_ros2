#!/usr/bin/env python3
"""Draw an OpenDRIVE map overview with drivable lanes, ego path, and signals."""

from __future__ import annotations

import argparse
import bisect
import math
import sqlite3
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parents[1]
PROJECT_DIR = SCRIPT_DIR.parents[2]


@dataclass(frozen=True)
class LanePolygon:
    road_id: str
    lane_id: str
    lane_type: str
    points: np.ndarray


@dataclass(frozen=True)
class TrafficSignal:
    signal_id: str
    x: float
    y: float
    z: float
    heading: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Draw a full Town10HD map with drivable lanes, odom path, and traffic lights."
    )
    parser.add_argument("--map", type=Path, default=SRC_DIR / "Town10HD.xodr")
    parser.add_argument(
        "--bag",
        type=Path,
        default=SRC_DIR / "rosbag2_2026_05_28-21_41_58_0-001.db3",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_DIR / "outputs" / "map_overview" / "town10hd_path_signals_full.png",
    )
    parser.add_argument("--lane-step", type=float, default=1.2)
    parser.add_argument("--dpi", type=int, default=220)
    parser.add_argument(
        "--path-stride",
        type=int,
        default=8,
        help="Draw every Nth odom point marker. The path line still uses all samples.",
    )
    return parser.parse_args()


def rotation_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s], [s, c]], dtype=float)


def poly_eval(element: ET.Element, ds: float) -> float:
    return (
        float(element.get("a", "0"))
        + float(element.get("b", "0")) * ds
        + float(element.get("c", "0")) * ds * ds
        + float(element.get("d", "0")) * ds * ds * ds
    )


def pick_record(records: Sequence[ET.Element], s: float, attr: str) -> Optional[ET.Element]:
    if not records:
        return None
    chosen = records[0]
    for record in sorted(records, key=lambda item: float(item.get(attr, "0"))):
        if float(record.get(attr, "0")) <= s + 1e-9:
            chosen = record
        else:
            break
    return chosen


def road_elevation(road: ET.Element, s_abs: float) -> float:
    profile = road.find("elevationProfile")
    if profile is None:
        return 0.0
    record = pick_record(profile.findall("elevation"), s_abs, "s")
    if record is None:
        return 0.0
    return poly_eval(record, s_abs - float(record.get("s", "0")))


def road_xy_heading(road: ET.Element, s_abs: float) -> tuple[float, float, float]:
    geometries = []
    plan_view = road.find("planView")
    if plan_view is None:
        raise ValueError(f"road {road.get('id')} has no planView")
    for geom in plan_view.findall("geometry"):
        children = list(geom)
        geometries.append(
            (
                float(geom.get("s", "0")),
                float(geom.get("x", "0")),
                float(geom.get("y", "0")),
                float(geom.get("hdg", "0")),
                float(geom.get("length", "0")),
                children[0] if children else None,
            )
        )
    geometries.sort(key=lambda item: item[0])
    chosen = geometries[-1]
    for item in geometries:
        s0, _, _, _, length, _ = item
        if s0 <= s_abs <= s0 + length + 1e-6:
            chosen = item
            break
    s0, x0, y0, heading, length, child = chosen
    ds = max(0.0, min(s_abs - s0, length))
    if child is not None and child.tag == "arc":
        curvature = float(child.get("curvature", "0"))
        if abs(curvature) > 1e-12:
            return (
                x0 + (math.sin(heading + curvature * ds) - math.sin(heading)) / curvature,
                y0 - (math.cos(heading + curvature * ds) - math.cos(heading)) / curvature,
                heading + curvature * ds,
            )
    return x0 + ds * math.cos(heading), y0 + ds * math.sin(heading), heading


def road_st_to_xy(road: ET.Element, s_abs: float, t_abs: float) -> tuple[float, float, float]:
    x_ref, y_ref, heading = road_xy_heading(road, s_abs)
    return x_ref - t_abs * math.sin(heading), y_ref + t_abs * math.cos(heading), heading


def lane_width_at(lane: ET.Element, s_rel: float) -> float:
    width_record = pick_record(lane.findall("width"), s_rel, "sOffset")
    if width_record is None:
        return 0.0
    return max(
        0.0,
        poly_eval(width_record, s_rel - float(width_record.get("sOffset", "0"))),
    )


def lane_offset_at(lane_offsets: Sequence[ET.Element], s_abs: float) -> float:
    offset = pick_record(lane_offsets, s_abs, "s")
    if offset is None:
        return 0.0
    return poly_eval(offset, s_abs - float(offset.get("s", "0")))


def side_lanes(section: ET.Element, side_name: str) -> list[ET.Element]:
    side = section.find(side_name)
    if side is None:
        return []
    return sorted(side.findall("lane"), key=lambda item: abs(int(item.get("id", "0"))))


def lane_t_bounds(
    section: ET.Element,
    side_name: str,
    target_lane: ET.Element,
    s_rel: float,
    t_offset: float,
) -> tuple[float, float]:
    sign = 1.0 if side_name == "left" else -1.0
    accumulated = 0.0
    for lane in side_lanes(section, side_name):
        width = lane_width_at(lane, s_rel)
        inner = t_offset + sign * accumulated
        outer = t_offset + sign * (accumulated + width)
        if lane is target_lane:
            return inner, outer
        accumulated += width
    return t_offset, t_offset


def build_lane_polygons(map_path: Path, step: float) -> tuple[list[LanePolygon], list[TrafficSignal]]:
    root = ET.parse(map_path).getroot()
    lane_polygons: list[LanePolygon] = []
    signals: list[TrafficSignal] = []

    for road in root.findall("road"):
        road_id = road.get("id", "")
        road_length = float(road.get("length", "0"))
        lanes = road.find("lanes")
        if lanes is None:
            continue
        lane_offsets = lanes.findall("laneOffset")
        sections = sorted(lanes.findall("laneSection"), key=lambda item: float(item.get("s", "0")))
        section_starts = [float(section.get("s", "0")) for section in sections]

        for section_index, section in enumerate(sections):
            section_start = float(section.get("s", "0"))
            section_end = (
                section_starts[section_index + 1]
                if section_index + 1 < len(section_starts)
                else road_length
            )
            sample_count = max(2, int(math.ceil((section_end - section_start) / step)) + 1)
            s_values = np.linspace(section_start, section_end, sample_count)
            for side_name in ("left", "right"):
                for lane in side_lanes(section, side_name):
                    lane_type = lane.get("type", "")
                    if lane_type == "none":
                        continue
                    inner_points = []
                    outer_points = []
                    for s_abs in s_values:
                        s_rel = s_abs - section_start
                        t_offset = lane_offset_at(lane_offsets, s_abs)
                        t_inner, t_outer = lane_t_bounds(
                            section, side_name, lane, s_rel, t_offset
                        )
                        inner_points.append(road_st_to_xy(road, s_abs, t_inner)[:2])
                        outer_points.append(road_st_to_xy(road, s_abs, t_outer)[:2])
                    polygon = np.array(inner_points + outer_points[::-1], dtype=float)
                    if polygon.shape[0] >= 4:
                        lane_polygons.append(
                            LanePolygon(
                                road_id=road_id,
                                lane_id=lane.get("id", ""),
                                lane_type=lane_type,
                                points=polygon,
                            )
                        )

        signal_parent = road.find("signals")
        if signal_parent is None:
            continue
        for signal in signal_parent.findall("signal"):
            if (
                signal.get("type") != "1000001"
                or signal.get("dynamic") != "yes"
                or not signal.get("name", "").startswith("Signal_3Light")
            ):
                continue
            s_abs = float(signal.get("s", "0"))
            t_abs = float(signal.get("t", "0"))
            x, y, heading = road_st_to_xy(road, s_abs, t_abs)
            signals.append(
                TrafficSignal(
                    signal_id=signal.get("id", ""),
                    x=x,
                    y=y,
                    z=road_elevation(road, s_abs) + float(signal.get("zOffset", "0")),
                    heading=heading + float(signal.get("hOffset", "0")),
                )
            )

    return lane_polygons, signals


def bag_topic_id(connection: sqlite3.Connection, topic_name: str) -> int:
    row = connection.execute("select id from topics where name=?", (topic_name,)).fetchone()
    if row is None:
        raise RuntimeError(f"missing topic: {topic_name}")
    return int(row[0])


def load_odom_path(bag_path: Path) -> np.ndarray:
    connection = sqlite3.connect(str(bag_path))
    topic_id = bag_topic_id(connection, "/carla/ego_vehicle/odometry")
    points = []
    for (data,) in connection.execute(
        "select data from messages where topic_id=? order by timestamp", (topic_id,)
    ):
        message = deserialize_message(data, Odometry)
        position = message.pose.pose.position
        points.append((float(position.x), float(position.y)))
    connection.close()
    if not points:
        raise RuntimeError(f"no odometry samples in {bag_path}")
    return np.array(points, dtype=float)


def draw_signal(ax: plt.Axes, signal: TrafficSignal) -> None:
    ax.scatter(
        [signal.x],
        [signal.y],
        s=46,
        marker="o",
        facecolor="#ffcf33",
        edgecolor="#6b4700",
        linewidth=0.7,
        zorder=8,
    )
    arrow_len = 4.2
    dx = math.cos(signal.heading) * arrow_len
    dy = math.sin(signal.heading) * arrow_len
    arrow = FancyArrowPatch(
        (signal.x, signal.y),
        (signal.x + dx, signal.y + dy),
        arrowstyle="-|>",
        mutation_scale=8,
        color="#a100ff",
        linewidth=0.9,
        zorder=9,
    )
    ax.add_patch(arrow)
    ax.text(
        signal.x + 1.0,
        signal.y + 1.0,
        signal.signal_id,
        color="#3b005a",
        fontsize=5.8,
        weight="bold",
        ha="left",
        va="bottom",
        zorder=10,
        bbox={
            "facecolor": "white",
            "edgecolor": "#7b4ca5",
            "linewidth": 0.25,
            "alpha": 0.82,
            "boxstyle": "round,pad=0.12",
        },
    )


def add_margin(points: np.ndarray, margin: float) -> tuple[float, float, float, float]:
    min_x = float(np.min(points[:, 0])) - margin
    max_x = float(np.max(points[:, 0])) + margin
    min_y = float(np.min(points[:, 1])) - margin
    max_y = float(np.max(points[:, 1])) + margin
    return min_x, max_x, min_y, max_y


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    lane_polygons, signals = build_lane_polygons(args.map, args.lane_step)
    path = load_odom_path(args.bag)

    drivable_polys = [lane.points for lane in lane_polygons if lane.lane_type == "driving"]
    non_drivable_polys = [
        lane.points for lane in lane_polygons if lane.lane_type != "driving"
    ]
    all_polygon_points = np.vstack([lane.points for lane in lane_polygons])
    all_points = np.vstack(
        [
            all_polygon_points,
            path,
            np.array([(signal.x, signal.y) for signal in signals], dtype=float),
        ]
    )
    min_x, max_x, min_y, max_y = add_margin(all_points, margin=12.0)

    fig, ax = plt.subplots(figsize=(16, 16), dpi=args.dpi)
    ax.set_facecolor("#eee9df")
    fig.patch.set_facecolor("#eee9df")

    if non_drivable_polys:
        ax.add_collection(
            PolyCollection(
                non_drivable_polys,
                facecolors="#cfc8bd",
                edgecolors="#b2aaa0",
                linewidths=0.18,
                alpha=0.95,
                zorder=1,
            )
        )
    ax.add_collection(
        PolyCollection(
            drivable_polys,
            facecolors="#5c6368",
            edgecolors="#f4f1e8",
            linewidths=0.28,
            alpha=0.98,
            zorder=2,
        )
    )

    ax.plot(path[:, 0], path[:, 1], color="#00d9ff", linewidth=2.4, zorder=6)
    if args.path_stride > 0:
        marker_points = path[:: args.path_stride]
        ax.scatter(
            marker_points[:, 0],
            marker_points[:, 1],
            s=5,
            color="#006d8f",
            alpha=0.35,
            zorder=5,
        )
    ax.scatter(
        [path[0, 0]],
        [path[0, 1]],
        s=72,
        marker="o",
        facecolor="#26d07c",
        edgecolor="black",
        linewidth=0.6,
        zorder=11,
    )
    ax.scatter(
        [path[-1, 0]],
        [path[-1, 1]],
        s=72,
        marker="X",
        facecolor="#ff4d4d",
        edgecolor="black",
        linewidth=0.6,
        zorder=11,
    )

    for signal in sorted(signals, key=lambda item: int(item.signal_id)):
        draw_signal(ax, signal)

    legend_items = [
        Line2D([0], [0], color="#5c6368", lw=8, label="drivable lane"),
        Line2D([0], [0], color="#cfc8bd", lw=8, label="non-drivable lane / sidewalk"),
        Line2D([0], [0], color="#eee9df", lw=8, label="outside mapped lane area"),
        Line2D([0], [0], color="#00d9ff", lw=2.4, label="ego odom path"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="#ffcf33",
            markeredgecolor="#6b4700",
            markersize=7,
            label="traffic light signal",
        ),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="#26d07c",
            markeredgecolor="black",
            markersize=7,
            label="path start",
        ),
        Line2D(
            [0],
            [0],
            marker="X",
            color="w",
            markerfacecolor="#ff4d4d",
            markeredgecolor="black",
            markersize=7,
            label="path end",
        ),
    ]
    ax.legend(
        handles=legend_items,
        loc="upper right",
        frameon=True,
        framealpha=0.92,
        facecolor="white",
        edgecolor="#b8b8b8",
        fontsize=8,
    )

    ax.set_title(
        "Town10HD: drivable lanes, non-drivable areas, ego odom path, and traffic lights",
        fontsize=12,
        pad=12,
    )
    ax.set_xlabel("OpenDRIVE / odom X (m)")
    ax.set_ylabel("OpenDRIVE / odom Y (m)")
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="white", linewidth=0.35, alpha=0.65)

    fig.tight_layout()
    fig.savefig(args.output)
    plt.close(fig)

    print(f"lane polygons: {len(lane_polygons)}")
    print(f"drivable lanes: {len(drivable_polys)}")
    print(f"non-drivable lanes: {len(non_drivable_polys)}")
    print(f"traffic lights: {len(signals)}")
    print(f"odom points: {len(path)}")
    print(f"output: {args.output}")


if __name__ == "__main__":
    main()
