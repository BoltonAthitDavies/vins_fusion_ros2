#!/usr/bin/env python3
"""Validate OpenDRIVE traffic-light projection on recorded CARLA camera frames.

This script reads the rosbag SQLite file directly, projects traffic-light
landmarks from Town10HD.xodr into the left/right camera images, and exports an
annotated playback video. It deliberately does not use CARLA traffic-light
state topics; the goal is only to validate transforms and axis conventions.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sqlite3
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parents[1]
PROJECT_DIR = SCRIPT_DIR.parents[2]


@dataclass(frozen=True)
class CameraConfig:
    sensor_id: str
    topic_prefix: str
    position_ego: np.ndarray
    rotation_ego_camera: np.ndarray
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class TrafficLightBox:
    signal_id: str
    box_index: int
    x: float
    y: float
    z: float
    extent_x: float
    extent_y: float
    extent_z: float


@dataclass(frozen=True)
class TrafficSignal:
    signal_id: str
    road_id: str
    x: float
    y: float
    z: float
    heading: float
    width: float
    height: float
    boxes: Tuple[TrafficLightBox, ...] = ()


@dataclass(frozen=True)
class SignalReference:
    signal_id: str
    road_id: str
    s: float
    x: float
    y: float
    heading: float
    orientation: str
    turn_relation: str
    validity: Tuple[Tuple[str, str], ...]


@dataclass(frozen=True)
class OdomSample:
    timestamp_ns: int
    position_map: np.ndarray
    rotation_map_ego: np.ndarray


@dataclass(frozen=True)
class ImageRow:
    row_id: int
    timestamp_ns: int


@dataclass(frozen=True)
class Candidate:
    ref: SignalReference
    signal: TrafficSignal
    distance_to_ref: float
    signal_distance: float
    path_support_m: float = 0.0
    matched_lane_id: str = ""


@dataclass(frozen=True)
class LaneSample:
    x: float
    y: float
    road_id: str
    lane_id: str
    s: float
    road_heading: float
    drive_heading: float
    travel_dir: str


@dataclass(frozen=True)
class PathMatch:
    distance_along: float
    x: float
    y: float
    yaw: float
    sample: LaneSample
    match_distance: float
    heading_error: float


@dataclass(frozen=True)
class LaneSampleIndex:
    samples: Sequence[LaneSample]
    grid: Dict[Tuple[int, int], List[LaneSample]]
    cell_size: float


@dataclass(frozen=True)
class YoloConfirmation:
    status: str
    confidence: Optional[float] = None
    box_xyxy: Optional[Tuple[float, float, float, float]] = None
    roi_xyxy: Optional[Tuple[int, int, int, int]] = None
    light_state: str = "unknown"
    light_state_confidence: Optional[float] = None
    color_scores: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ProjectionResult:
    u: float
    v: float
    box_w: float
    box_h: float
    depth: float
    roi_label: str
    roi_distance: float
    status: str
    yolo_status: str = "disabled"
    yolo_confidence: Optional[float] = None
    yolo_box_xyxy: Optional[Tuple[float, float, float, float]] = None
    yolo_roi_xyxy: Optional[Tuple[int, int, int, int]] = None
    light_state: str = "unknown"
    light_state_confidence: Optional[float] = None
    color_scores: Tuple[float, float, float] = (0.0, 0.0, 0.0)


TURN_RELATION_TO_LIGHT_BOX_INDEX = {
    "right": 0,
    "left": 1,
    "straight": 2,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export a camera playback video with projected traffic-light ROIs."
    )
    parser.add_argument("--bag", type=Path, default=SRC_DIR / "rosbag2_00_0.db3")
    parser.add_argument("--map", type=Path, default=SRC_DIR / "Town10HD.xodr")
    parser.add_argument("--objects", type=Path, default=SCRIPT_DIR / "objects.json")
    parser.add_argument(
        "--traffic-light-boxes",
        type=Path,
        default=SRC_DIR / "carla_light_boxes.csv",
        help=(
            "Optional CARLA light-box geometry CSV. If present, projected ROIs use "
            "actual lamp-head centers from this file instead of the OpenDRIVE signal anchor."
        ),
    )
    parser.add_argument(
        "--ignore-traffic-light-boxes",
        action="store_true",
        help="Ignore --traffic-light-boxes and project OpenDRIVE signal anchors only.",
    )
    parser.add_argument(
        "--min-light-box-center-z",
        type=float,
        default=3.0,
        help="Minimum world Z for CARLA light boxes used as lamp-head ROI targets.",
    )
    parser.add_argument(
        "--min-light-box-extent-z",
        type=float,
        default=0.25,
        help="Minimum vertical half-extent for CARLA light boxes used as lamp-head ROI targets.",
    )
    parser.add_argument(
        "--traffic-light-box-y-sign",
        choices=("auto", "as-is", "flip"),
        default="auto",
        help=(
            "Y sign for light-box centers loaded from --traffic-light-boxes. auto "
            "matches actor positions to OpenDRIVE signals; as-is uses CSV Y; flip "
            "uses -CSV Y."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_DIR / "outputs" / "projection_validation",
    )
    parser.add_argument("--output-name", default="projection_overlay_path.mp4")
    parser.add_argument("--trigger-distance", type=float, default=5.0)
    parser.add_argument(
        "--candidate-mode",
        choices=("manual", "reference", "facing", "path"),
        default="path",
        help=(
            "manual projects --manual-signal-id directly; "
            "reference uses the signal id attached to the nearest signalReference; "
            "facing triggers on nearby references but draws front-facing physical lights; "
            "path uses future odometry to choose the signalReference road/lane."
        ),
    )
    parser.add_argument(
        "--manual-signal-id",
        default="958",
        help="Traffic light signal id to project when --candidate-mode manual is used.",
    )
    parser.add_argument(
        "--camera-id",
        choices=("cam_front_left", "cam_front_right"),
        default="cam_front_right",
        help="Camera stream used for projection validation.",
    )
    parser.add_argument(
        "--max-signal-distance",
        type=float,
        default=60.0,
        help="Maximum XY distance to a physical signal in --candidate-mode facing.",
    )
    parser.add_argument(
        "--facing-angle-deg",
        type=float,
        default=75.0,
        help="Maximum angle between signal forward direction and vector toward ego.",
    )
    parser.add_argument("--axis-mode", choices=("carla", "ros"), default="carla")
    parser.add_argument(
        "--image-horizontal-sign",
        choices=("normal", "flip"),
        default="normal",
        help=(
            "Flip the projected image horizontal axis. This is equivalent to using "
            "--axis-mode ros for carla-sensor cameras, but keeps the intent explicit."
        ),
    )
    parser.add_argument(
        "--camera-frame-mode",
        choices=("carla-sensor", "ros-optical"),
        default="carla-sensor",
        help=(
            "Frame convention used by the selected camera transform. carla-sensor "
            "expects X-forward/Y-right/Z-up and applies --axis-mode. ros-optical "
            "expects X-right/Y-down/Z-forward and projects directly."
        ),
    )
    parser.add_argument(
        "--use-tf-camera-transform",
        action="store_true",
        help=(
            "Use the measured tf2_echo transform for cam_front_right: "
            "translation=(1.5,-0.25,1.5), RPY=(-90,0,-90), ros-optical frame."
        ),
    )
    parser.add_argument(
        "--camera-roll-offset-deg",
        type=float,
        default=0.0,
        help="Temporary roll correction applied to the selected camera extrinsic.",
    )
    parser.add_argument(
        "--camera-pitch-offset-deg",
        type=float,
        default=0.0,
        help="Temporary pitch correction applied to the selected camera extrinsic.",
    )
    parser.add_argument(
        "--camera-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Temporary yaw correction applied to the selected camera extrinsic.",
    )
    parser.add_argument(
        "--camera-x-offset",
        type=float,
        default=0.0,
        help="Temporary x correction in ego frame applied to the selected camera position.",
    )
    parser.add_argument(
        "--camera-y-offset",
        type=float,
        default=0.0,
        help="Temporary y correction in ego frame applied to the selected camera position.",
    )
    parser.add_argument(
        "--camera-z-offset",
        type=float,
        default=0.0,
        help="Temporary z correction in ego frame applied to the selected camera position.",
    )
    parser.add_argument(
        "--camera-y-sign",
        choices=("as-is", "flip"),
        default="as-is",
        help=(
            "Interpret the camera lateral offset from objects.json as-is, or flip its "
            "sign to test ROS/CARLA left-right convention mismatch."
        ),
    )
    parser.add_argument(
        "--light-center-height",
        type=float,
        default=5.0,
        help="Approximate lamp-center height above road in meters.",
    )
    parser.add_argument(
        "--height-line-min",
        type=float,
        default=3.0,
        help="Minimum height above road for the vertical diagnostic line.",
    )
    parser.add_argument(
        "--height-line-max",
        type=float,
        default=6.0,
        help="Maximum height above road for the vertical diagnostic line.",
    )
    parser.add_argument("--max-candidates", type=int, default=1)
    parser.add_argument(
        "--roi-signal-mode",
        choices=("reference",),
        default="reference",
        help=(
            "Draw the physical signal id named by the path/lane signalReference."
        ),
    )
    parser.add_argument(
        "--light-box-selection",
        choices=("turn-index", "all"),
        default="turn-index",
        help=(
            "turn-index draws one CARLA light box by turnRelation using "
            "Right=0, Left=1, Straight=2. all draws every usable box."
        ),
    )
    parser.add_argument(
        "--min-detectable-signal-height",
        type=float,
        default=10.0,
        help="A projected small ROI is considered ML-ready when its raw signal height reaches this many pixels.",
    )
    parser.add_argument(
        "--yolo-model",
        type=Path,
        default=None,
        help=(
            "Optional YOLO model used to confirm traffic-light detections inside "
            "each expanded projected ROI."
        ),
    )
    parser.add_argument(
        "--yolo-conf",
        type=float,
        default=0.05,
        help="YOLO confidence threshold for ROI traffic-light confirmation.",
    )
    parser.add_argument(
        "--yolo-imgsz",
        type=int,
        default=640,
        help="YOLO inference image size for each ROI crop.",
    )
    parser.add_argument(
        "--yolo-class-id",
        type=int,
        default=9,
        help="YOLO class id used for traffic lights. COCO pretrained models use 9.",
    )
    parser.add_argument(
        "--yolo-device",
        default="",
        help="Optional YOLO device string, e.g. cpu, 0, cuda:0. Empty lets Ultralytics choose.",
    )
    parser.add_argument(
        "--yolo-roi-scale",
        type=float,
        default=5.0,
        help="Scale factor applied to the projected ROI before running YOLO.",
    )
    parser.add_argument(
        "--yolo-min-roi-width",
        type=float,
        default=160.0,
        help="Minimum expanded ROI width in pixels before running YOLO.",
    )
    parser.add_argument(
        "--yolo-min-roi-height",
        type=float,
        default=180.0,
        help="Minimum expanded ROI height in pixels before running YOLO.",
    )
    parser.add_argument(
        "--lookahead-distance",
        type=float,
        default=45.0,
        help="Future odometry distance in meters used by --candidate-mode path.",
    )
    parser.add_argument(
        "--lookahead-seconds",
        type=float,
        default=30.0,
        help="Future odometry time horizon used by --candidate-mode path.",
    )
    parser.add_argument(
        "--path-sample-spacing",
        type=float,
        default=1.0,
        help="Approximate spacing between future odometry samples for lane matching.",
    )
    parser.add_argument(
        "--lane-match-max-distance",
        type=float,
        default=2.5,
        help="Maximum distance from odometry point to matched lane center.",
    )
    parser.add_argument(
        "--lane-heading-weight",
        type=float,
        default=3.0,
        help="Meters of score penalty per radian of heading mismatch.",
    )
    parser.add_argument(
        "--min-path-lane-support",
        type=float,
        default=4.0,
        help="Minimum future-path meters on a road/lane before using its signalReference.",
    )
    parser.add_argument(
        "--signal-reference-path-radius",
        type=float,
        default=6.0,
        help="Maximum lateral distance from future path to a signalReference in path mode.",
    )
    parser.add_argument(
        "--reference-heading-tolerance-deg",
        type=float,
        default=75.0,
        help="Maximum heading mismatch between future path and signalReference orientation.",
    )
    parser.add_argument(
        "--path-signal-facing-tolerance-deg",
        type=float,
        default=75.0,
        help=(
            "Maximum mismatch from opposite-of-path heading for a physical traffic "
            "light chosen by path mode."
        ),
    )
    parser.add_argument(
        "--path-facing-signal-radius",
        type=float,
        default=35.0,
        help=(
            "Maximum XY distance from a path signalReference to a replacement "
            "physical signal that faces the ego path."
        ),
    )
    parser.add_argument(
        "--path-physical-signal-mode",
        choices=("reference", "same-heading", "facing-replacement"),
        default="same-heading",
        help=(
            "How path mode maps a signalReference to a physical signal. reference "
            "uses the referenced id plus --signal-id-override corrections. "
            "same-heading uses the referenced id only when its heading follows "
            "the selected path direction. "
            "facing-replacement replaces rear-facing signals with nearby lights "
            "that face opposite the path."
        ),
    )
    parser.add_argument(
        "--signal-id-override",
        action="append",
        default=[],
        help=(
            "Physical signal correction in FROM:TO form. Can be repeated."
        ),
    )
    parser.add_argument("--fps", type=float, default=6.15)
    parser.add_argument(
        "--playback-rate",
        type=float,
        default=1.0,
        help="Live --show playback speed multiplier. 0.5 is half speed, 2.0 is double speed.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Limit processed frames for quick tests. 0 means the whole bag.",
    )
    parser.add_argument(
        "--save-keyframes",
        type=int,
        default=24,
        help="Maximum trigger frames to also save as JPG snapshots.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open a live OpenCV window while exporting, if DISPLAY is available.",
    )
    return parser.parse_args()


def rotation_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rotation_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rotation_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_from_rpy_degrees(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return (
        rotation_z(math.radians(yaw))
        @ rotation_y(math.radians(pitch))
        @ rotation_x(math.radians(roll))
    )


def rotation_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


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
    records = profile.findall("elevation")
    record = pick_record(records, s_abs, "s")
    if record is None:
        return 0.0
    return poly_eval(record, s_abs - float(record.get("s", "0")))


def road_xy_heading(road: ET.Element, s_abs: float) -> Tuple[float, float, float]:
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
    if not geometries:
        raise ValueError(f"road {road.get('id')} has no geometry")
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


def road_st_to_xyz(
    road: ET.Element, s_abs: float, t_abs: float, z_abs: float
) -> Tuple[float, float, float, float]:
    x_ref, y_ref, heading = road_xy_heading(road, s_abs)
    x = x_ref - t_abs * math.sin(heading)
    y = y_ref + t_abs * math.cos(heading)
    z = road_elevation(road, s_abs) + z_abs
    return x, y, z, heading


def vector_signal_turn_relation(signal_reference: ET.Element) -> str:
    for vector_signal in signal_reference.findall("./userData/vectorSignal"):
        relation = vector_signal.get("turnRelation")
        if relation:
            return relation
    return ""


def signal_validity(signal_reference: ET.Element) -> Tuple[Tuple[str, str], ...]:
    values = []
    for validity in signal_reference.findall("validity"):
        values.append((validity.get("fromLane", ""), validity.get("toLane", "")))
    return tuple(values)


def lane_travel_direction(lane: ET.Element, s_rel: float) -> str:
    vector_lane = pick_record(lane.findall("./userData/vectorLane"), s_rel, "sOffset")
    if vector_lane is None:
        return ""
    return vector_lane.get("travelDir", "")


def lane_id_in_validity(lane_id: str, validity: Tuple[Tuple[str, str], ...]) -> bool:
    if not validity:
        return True
    try:
        lane_value = int(lane_id)
    except ValueError:
        return False
    for from_lane, to_lane in validity:
        try:
            a = int(from_lane)
            b = int(to_lane)
        except ValueError:
            continue
        low = min(a, b)
        high = max(a, b)
        if low <= lane_value <= high:
            return True
    return False


def reference_matches_lane_direction(ref: SignalReference, sample: LaneSample) -> bool:
    if ref.orientation in ("", "none"):
        return True
    # OpenDRIVE signal orientation '+' applies to traffic in increasing road s;
    # '-' applies to traffic in decreasing road s. CARLA's vectorLane travelDir
    # tells us which way vehicles use this lane relative to the reference line.
    if sample.travel_dir == "backward":
        return ref.orientation == "-"
    if sample.travel_dir == "forward":
        return ref.orientation == "+"
    # Fallback when vectorLane metadata is absent: infer from heading alignment.
    is_forward = angle_error(sample.drive_heading, sample.road_heading) < math.pi / 2.0
    return ref.orientation == ("+" if is_forward else "-")


def build_lane_samples(map_path: Path, step: float = 0.8) -> List[LaneSample]:
    root = ET.parse(map_path).getroot()
    samples: List[LaneSample] = []
    for road in root.findall("road"):
        road_id = road.get("id", "")
        road_length = float(road.get("length", "0"))
        lanes = road.find("lanes")
        if lanes is None:
            continue
        lane_offsets = lanes.findall("laneOffset")
        sections = sorted(lanes.findall("laneSection"), key=lambda item: float(item.get("s", "0")))
        if not sections:
            continue
        section_starts = [float(section.get("s", "0")) for section in sections]
        count = max(2, int(math.ceil(road_length / step)) + 1)
        for index in range(count):
            s_abs = min(road_length, index * step)
            section_index = max(0, bisect.bisect_right(section_starts, s_abs) - 1)
            section = sections[section_index]
            section_s = float(section.get("s", "0"))
            offset = pick_record(lane_offsets, s_abs, "s")
            t_offset = (
                poly_eval(offset, s_abs - float(offset.get("s", "0")))
                if offset is not None
                else 0.0
            )
            x_ref, y_ref, road_heading = road_xy_heading(road, s_abs)
            for side_name, sign in (("left", 1.0), ("right", -1.0)):
                side = section.find(side_name)
                if side is None:
                    continue
                accumulated_width = 0.0
                lanes_on_side = sorted(
                    side.findall("lane"), key=lambda item: abs(int(item.get("id", "0")))
                )
                for lane in lanes_on_side:
                    width_record = pick_record(
                        lane.findall("width"), s_abs - section_s, "sOffset"
                    )
                    lane_width = (
                        poly_eval(
                            width_record,
                            s_abs - section_s - float(width_record.get("sOffset", "0")),
                        )
                        if width_record is not None
                        else 0.0
                    )
                    if lane.get("type") == "driving" and lane_width > 0.1:
                        t = t_offset + sign * (accumulated_width + lane_width / 2.0)
                        x = x_ref - t * math.sin(road_heading)
                        y = y_ref + t * math.cos(road_heading)
                        travel_dir = lane_travel_direction(lane, s_abs - section_s)
                        drive_heading = (
                            road_heading + math.pi
                            if travel_dir == "backward"
                            else road_heading
                        )
                        samples.append(
                            LaneSample(
                                x=x,
                                y=y,
                                road_id=road_id,
                                lane_id=lane.get("id", ""),
                                s=s_abs,
                                road_heading=road_heading,
                                drive_heading=drive_heading,
                                travel_dir=travel_dir,
                            )
                        )
                    accumulated_width += max(lane_width, 0.0)
    if not samples:
        raise RuntimeError(f"no lane samples built from {map_path}")
    return samples


def build_lane_sample_index(samples: Sequence[LaneSample], cell_size: float = 5.0) -> LaneSampleIndex:
    grid: Dict[Tuple[int, int], List[LaneSample]] = {}
    for sample in samples:
        key = (math.floor(sample.x / cell_size), math.floor(sample.y / cell_size))
        grid.setdefault(key, []).append(sample)
    return LaneSampleIndex(samples=samples, grid=grid, cell_size=cell_size)


def nearby_lane_samples(
    lane_index: LaneSampleIndex, x: float, y: float, radius: float
) -> Sequence[LaneSample]:
    cell_radius = max(1, int(math.ceil(radius / lane_index.cell_size)))
    center_x = math.floor(x / lane_index.cell_size)
    center_y = math.floor(y / lane_index.cell_size)
    nearby: List[LaneSample] = []
    for gx in range(center_x - cell_radius, center_x + cell_radius + 1):
        for gy in range(center_y - cell_radius, center_y + cell_radius + 1):
            nearby.extend(lane_index.grid.get((gx, gy), ()))
    return nearby if nearby else lane_index.samples


def parse_opendrive(
    map_path: Path, light_center_height: float
) -> Tuple[Dict[str, TrafficSignal], List[SignalReference]]:
    root = ET.parse(map_path).getroot()
    signals: Dict[str, TrafficSignal] = {}
    references: List[SignalReference] = []

    for road in root.findall("road"):
        signal_parent = road.find("signals")
        if signal_parent is None:
            continue
        for signal in signal_parent.findall("signal"):
            is_traffic_light = (
                signal.get("type") == "1000001"
                and signal.get("dynamic") == "yes"
                and signal.get("name", "").startswith("Signal_3Light")
            )
            if not is_traffic_light:
                continue
            s_abs = float(signal.get("s", "0"))
            t_abs = float(signal.get("t", "0"))
            x, y, _, heading = road_st_to_xyz(road, s_abs, t_abs, 0.0)
            z = road_elevation(road, s_abs) + light_center_height
            signals[signal.get("id", "")] = TrafficSignal(
                signal_id=signal.get("id", ""),
                road_id=road.get("id", ""),
                x=x,
                y=y,
                z=z,
                heading=heading + float(signal.get("hOffset", "0")),
                width=float(signal.get("width", "0.7") or "0.7"),
                height=float(signal.get("height", "1.2") or "1.2"),
            )

    for road in root.findall("road"):
        signal_parent = road.find("signals")
        if signal_parent is None:
            continue
        for signal_reference in signal_parent.findall("signalReference"):
            signal_id = signal_reference.get("id", "")
            if signal_id not in signals:
                continue
            s_abs = float(signal_reference.get("s", "0"))
            t_abs = float(signal_reference.get("t", "0"))
            x, y, _, heading = road_st_to_xyz(road, s_abs, t_abs, 0.0)
            references.append(
                SignalReference(
                    signal_id=signal_id,
                    road_id=road.get("id", ""),
                    s=s_abs,
                    x=x,
                    y=y,
                    heading=heading,
                    orientation=signal_reference.get("orientation", ""),
                    turn_relation=vector_signal_turn_relation(signal_reference),
                    validity=signal_validity(signal_reference),
                )
            )

    return signals, references


def csv_float(row: Dict[str, str], *names: str) -> Optional[float]:
    for name in names:
        value = row.get(name)
        if value not in (None, ""):
            return float(value)
    return None


def infer_light_box_y_sign(
    rows: Sequence[Dict[str, str]],
    signals: Dict[str, TrafficSignal],
) -> float:
    errors: Dict[float, List[float]] = {1.0: [], -1.0: []}
    for row in rows:
        signal = signals.get(row.get("opendrive_id", "").strip())
        if signal is None:
            continue
        actor_x = csv_float(row, "actor_x")
        actor_y = csv_float(row, "actor_y")
        if actor_x is None or actor_y is None:
            continue
        for y_sign in errors:
            errors[y_sign].append(math.hypot(actor_x - signal.x, y_sign * actor_y - signal.y))
    mean_errors = {
        y_sign: sum(values) / len(values)
        for y_sign, values in errors.items()
        if values
    }
    if not mean_errors:
        return 1.0
    return min(mean_errors, key=mean_errors.get)


def load_traffic_light_boxes(
    path: Path,
    signals: Dict[str, TrafficSignal],
    min_center_z: float,
    min_extent_z: float,
    y_sign_mode: str,
) -> Tuple[Dict[str, TrafficSignal], int, float]:
    if not path.exists():
        return signals, 0, 1.0

    with path.open("r", encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if not rows:
        return signals, 0, 1.0

    if y_sign_mode == "auto":
        y_sign = infer_light_box_y_sign(rows, signals)
    elif y_sign_mode == "flip":
        y_sign = -1.0
    else:
        y_sign = 1.0
    boxes_by_signal: Dict[str, List[TrafficLightBox]] = {}
    for row in rows:
        signal_id = row.get("opendrive_id", "").strip()
        if signal_id not in signals:
            continue

        # The first CSV collected for this project used local_offset_* for the
        # raw light-box center from CARLA. Newer dumps write box_center_*.
        x = csv_float(row, "box_center_x", "local_offset_x", "world_center_x")
        y = csv_float(row, "box_center_y", "local_offset_y", "world_center_y")
        z = csv_float(row, "box_center_z", "local_offset_z", "world_center_z")
        extent_x = csv_float(row, "extent_x")
        extent_y = csv_float(row, "extent_y")
        extent_z = csv_float(row, "extent_z")
        if None in (x, y, z, extent_x, extent_y, extent_z):
            continue
        if z < min_center_z or extent_z < min_extent_z:
            continue

        boxes_by_signal.setdefault(signal_id, []).append(
            TrafficLightBox(
                signal_id=signal_id,
                box_index=int(row.get("box_index", "-1") or "-1"),
                x=x,
                y=y_sign * y,
                z=z,
                extent_x=extent_x,
                extent_y=extent_y,
                extent_z=extent_z,
            )
        )

    patched_signals = dict(signals)
    box_count = 0
    for signal_id, boxes in boxes_by_signal.items():
        boxes.sort(key=lambda item: item.box_index)
        signal = patched_signals[signal_id]
        box_tuple = tuple(boxes)
        box_count += len(box_tuple)
        patched_signals[signal_id] = TrafficSignal(
            signal_id=signal.signal_id,
            road_id=signal.road_id,
            x=signal.x,
            y=signal.y,
            z=signal.z,
            heading=signal.heading,
            width=signal.width,
            height=signal.height,
            boxes=box_tuple,
        )

    return patched_signals, box_count, y_sign


def bag_topics(connection: sqlite3.Connection) -> Dict[str, int]:
    return {
        name: topic_id
        for topic_id, name in connection.execute("select id, name from topics order by id")
    }


def get_first_camera_info(
    connection: sqlite3.Connection, topic_id: int
) -> Tuple[int, int, float, float, float, float]:
    row = connection.execute(
        "select data from messages where topic_id=? order by timestamp limit 1", (topic_id,)
    ).fetchone()
    if row is None:
        raise ValueError(f"camera_info topic id {topic_id} has no messages")
    message = deserialize_message(row[0], CameraInfo)
    return (
        int(message.width),
        int(message.height),
        float(message.k[0]),
        float(message.k[4]),
        float(message.k[2]),
        float(message.k[5]),
    )


def load_cameras(
    connection: sqlite3.Connection,
    objects_path: Path,
    topics: Dict[str, int],
    camera_ids: Sequence[str] = ("cam_front_left", "cam_front_right"),
) -> Dict[str, CameraConfig]:
    with objects_path.open("r", encoding="utf-8") as file:
        objects = json.load(file)
    sensors = objects["objects"][0]["sensors"]
    wanted_camera_ids = set(camera_ids)
    cameras = {}
    for sensor in sensors:
        if sensor.get("type") != "sensor.camera.rgb":
            continue
        sensor_id = sensor.get("id", "")
        if sensor_id not in wanted_camera_ids:
            continue
        topic_prefix = f"/carla/ego_vehicle/{sensor_id}"
        info_topic = f"{topic_prefix}/camera_info"
        if info_topic not in topics:
            raise ValueError(f"missing camera_info topic: {info_topic}")
        width, height, fx, fy, cx, cy = get_first_camera_info(connection, topics[info_topic])
        spawn = sensor["spawn_point"]
        cameras[sensor_id] = CameraConfig(
            sensor_id=sensor_id,
            topic_prefix=topic_prefix,
            position_ego=np.array(
                [float(spawn["x"]), float(spawn["y"]), float(spawn["z"])], dtype=float
            ),
            rotation_ego_camera=rotation_from_rpy_degrees(
                float(spawn.get("roll", 0.0)),
                float(spawn.get("pitch", 0.0)),
                float(spawn.get("yaw", 0.0)),
            ),
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
    if not cameras:
        raise ValueError(f"no front cameras found in {objects_path}")
    missing_camera_ids = wanted_camera_ids - cameras.keys()
    if missing_camera_ids:
        raise ValueError(f"missing camera definitions: {', '.join(sorted(missing_camera_ids))}")
    return cameras


def apply_camera_calibration_offsets(
    camera: CameraConfig,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    x_offset: float,
    y_offset: float,
    z_offset: float,
    camera_y_sign: str,
) -> CameraConfig:
    rotation_offset = rotation_from_rpy_degrees(roll_deg, pitch_deg, yaw_deg)
    position_ego = camera.position_ego.copy()
    if camera_y_sign == "flip":
        position_ego[1] *= -1.0
    return CameraConfig(
        sensor_id=camera.sensor_id,
        topic_prefix=camera.topic_prefix,
        position_ego=position_ego + np.array([x_offset, y_offset, z_offset], dtype=float),
        rotation_ego_camera=camera.rotation_ego_camera @ rotation_offset,
        width=camera.width,
        height=camera.height,
        fx=camera.fx,
        fy=camera.fy,
        cx=camera.cx,
        cy=camera.cy,
    )


def camera_with_transform(
    camera: CameraConfig,
    position_ego: np.ndarray,
    rotation_ego_camera: np.ndarray,
) -> CameraConfig:
    return CameraConfig(
        sensor_id=camera.sensor_id,
        topic_prefix=camera.topic_prefix,
        position_ego=position_ego,
        rotation_ego_camera=rotation_ego_camera,
        width=camera.width,
        height=camera.height,
        fx=camera.fx,
        fy=camera.fy,
        cx=camera.cx,
        cy=camera.cy,
    )


def load_odometry(connection: sqlite3.Connection, topic_id: int) -> List[OdomSample]:
    samples = []
    for timestamp_ns, data in connection.execute(
        "select timestamp, data from messages where topic_id=? order by timestamp", (topic_id,)
    ):
        message = deserialize_message(data, Odometry)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        samples.append(
            OdomSample(
                timestamp_ns=int(timestamp_ns),
                position_map=np.array([position.x, position.y, position.z], dtype=float),
                rotation_map_ego=rotation_from_quaternion(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
            )
        )
    if not samples:
        raise ValueError("odometry topic has no messages")
    return samples


def load_image_rows(connection: sqlite3.Connection, topic_id: int) -> List[ImageRow]:
    rows = []
    for row_id, timestamp_ns in connection.execute(
        "select id, timestamp from messages where topic_id=? order by timestamp", (topic_id,)
    ):
        rows.append(ImageRow(int(row_id), int(timestamp_ns)))
    if not rows:
        raise ValueError(f"image topic id {topic_id} has no messages")
    return rows


def nearest_index(timestamps: Sequence[int], timestamp_ns: int) -> int:
    index = bisect.bisect_left(timestamps, timestamp_ns)
    if index <= 0:
        return 0
    if index >= len(timestamps):
        return len(timestamps) - 1
    before = timestamps[index - 1]
    after = timestamps[index]
    return index - 1 if abs(timestamp_ns - before) <= abs(after - timestamp_ns) else index


def nearest_odom(odometry: Sequence[OdomSample], timestamps: Sequence[int], timestamp_ns: int) -> OdomSample:
    return odometry[nearest_index(timestamps, timestamp_ns)]


def odom_yaw(odom: OdomSample) -> float:
    forward = odom.rotation_map_ego @ np.array([1.0, 0.0, 0.0], dtype=float)
    return math.atan2(float(forward[1]), float(forward[0]))


def angle_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def match_lane_sample(
    odom: OdomSample,
    lane_index: LaneSampleIndex,
    heading_weight: float,
    search_radius: float,
) -> Tuple[LaneSample, float, float]:
    yaw = odom_yaw(odom)
    ego_x = float(odom.position_map[0])
    ego_y = float(odom.position_map[1])
    lane_samples = nearby_lane_samples(lane_index, ego_x, ego_y, search_radius)

    def score(sample: LaneSample) -> float:
        distance_sq = (sample.x - ego_x) ** 2 + (sample.y - ego_y) ** 2
        heading_penalty = heading_weight * angle_error(sample.drive_heading, yaw)
        return distance_sq + heading_penalty * heading_penalty

    sample = min(lane_samples, key=score)
    match_distance = math.hypot(sample.x - ego_x, sample.y - ego_y)
    heading = angle_error(sample.drive_heading, yaw)
    return sample, match_distance, heading


def future_path_matches(
    odometry: Sequence[OdomSample],
    odom_timestamps: Sequence[int],
    timestamp_ns: int,
    lane_index: LaneSampleIndex,
    lookahead_distance: float,
    lookahead_seconds: float,
    sample_spacing: float,
    lane_match_max_distance: float,
    heading_weight: float,
) -> List[PathMatch]:
    start_index = nearest_index(odom_timestamps, timestamp_ns)
    start_time = odom_timestamps[start_index]
    matches: List[PathMatch] = []
    distance_along = 0.0
    last_position = odometry[start_index].position_map[:2]
    last_kept_position: Optional[np.ndarray] = None

    for odom in odometry[start_index:]:
        if (odom.timestamp_ns - start_time) / 1e9 > lookahead_seconds:
            break
        current_position = odom.position_map[:2]
        if odom.timestamp_ns != start_time:
            distance_along += float(np.linalg.norm(current_position - last_position))
        last_position = current_position
        if distance_along > lookahead_distance:
            break
        if (
            last_kept_position is not None
            and float(np.linalg.norm(current_position - last_kept_position)) < sample_spacing
        ):
            continue
        sample, match_distance, heading = match_lane_sample(
            odom,
            lane_index,
            heading_weight,
            search_radius=max(8.0, lane_match_max_distance * 3.0),
        )
        if match_distance <= lane_match_max_distance:
            matches.append(
                PathMatch(
                    distance_along=distance_along,
                    x=float(current_position[0]),
                    y=float(current_position[1]),
                    yaw=odom_yaw(odom),
                    sample=sample,
                    match_distance=match_distance,
                    heading_error=heading,
                )
            )
            last_kept_position = current_position.copy()
    return matches


def path_lane_support(matches: Sequence[PathMatch]) -> Dict[Tuple[str, str], float]:
    support: Dict[Tuple[str, str], float] = {}
    if not matches:
        return support
    previous = matches[0]
    support[(previous.sample.road_id, previous.sample.lane_id)] = 0.0
    for current in matches[1:]:
        segment = max(0.0, current.distance_along - previous.distance_along)
        key = (current.sample.road_id, current.sample.lane_id)
        support[key] = support.get(key, 0.0) + segment
        previous = current
    return support


def path_reference_support(
    matches: Sequence[PathMatch],
    ref: SignalReference,
    require_orientation: bool = False,
) -> Tuple[float, str, float]:
    best_by_lane: Dict[str, float] = {}
    first_distance_by_lane: Dict[str, float] = {}
    if not matches:
        return 0.0, "", 0.0
    previous = matches[0]
    for current in matches[1:]:
        segment = max(0.0, current.distance_along - previous.distance_along)
        sample = current.sample
        if sample.road_id == ref.road_id:
            lane_is_valid = lane_id_in_validity(sample.lane_id, ref.validity)
            direction_is_valid = (
                reference_matches_lane_direction(ref, sample)
                if require_orientation
                else True
            )
            if lane_is_valid and direction_is_valid:
                best_by_lane[sample.lane_id] = best_by_lane.get(sample.lane_id, 0.0) + segment
                first_distance_by_lane.setdefault(sample.lane_id, current.distance_along)
        previous = current
    if not best_by_lane:
        return 0.0, "", 0.0
    lane_id, support = max(best_by_lane.items(), key=lambda item: item[1])
    return support, lane_id, first_distance_by_lane[lane_id]


def reference_drive_heading(ref: SignalReference) -> float:
    if ref.orientation == "-":
        return ref.heading + math.pi
    return ref.heading


def infer_path_turn(matches: Sequence[PathMatch]) -> str:
    if len(matches) < 2:
        return ""
    start_yaw = matches[0].yaw
    end_yaw = matches[-1].yaw
    delta = math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw))
    if abs(math.degrees(delta)) < 30.0:
        return "Straight"
    return "Left" if delta > 0.0 else "Right"


def signal_opposes_path_heading(
    signal: TrafficSignal,
    path_yaw: float,
    tolerance_deg: float,
) -> Tuple[bool, float]:
    error = angle_error(signal.heading, path_yaw + math.pi)
    return math.degrees(error) <= tolerance_deg, error


def signal_matches_path_heading(
    signal: TrafficSignal,
    path_yaw: float,
    tolerance_deg: float,
) -> Tuple[bool, float]:
    error = angle_error(signal.heading, path_yaw)
    return math.degrees(error) <= tolerance_deg, error


def path_facing_signal(
    ref: SignalReference,
    referenced_signal: TrafficSignal,
    signals: Dict[str, TrafficSignal],
    ego_xy: np.ndarray,
    path_match: PathMatch,
    facing_tolerance_deg: float,
    replacement_radius: float,
) -> Optional[TrafficSignal]:
    referenced_faces_path, _ = signal_opposes_path_heading(
        referenced_signal,
        path_match.yaw,
        facing_tolerance_deg,
    )
    if referenced_faces_path:
        return referenced_signal

    ref_xy = np.array([ref.x, ref.y], dtype=float)
    candidates: List[Tuple[float, TrafficSignal]] = []
    for signal in signals.values():
        faces_path, heading_error = signal_opposes_path_heading(
            signal,
            path_match.yaw,
            facing_tolerance_deg,
        )
        if not faces_path:
            continue
        signal_xy = np.array([signal.x, signal.y], dtype=float)
        distance_to_ref = float(np.linalg.norm(signal_xy - ref_xy))
        if distance_to_ref > replacement_radius:
            continue
        distance_to_ego = float(np.linalg.norm(signal_xy - ego_xy))
        # Prefer a light that faces the path, is close to the signalReference,
        # and is not needlessly farther from ego. This fixes CARLA Town10
        # references that name the rear-facing physical signal.
        score = math.degrees(heading_error) + 0.08 * distance_to_ref + 0.02 * distance_to_ego
        candidates.append((score, signal))

    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])[1]


def parse_signal_id_overrides(values: Sequence[str]) -> Dict[str, str]:
    overrides: Dict[str, str] = {}
    for value in values:
        if not value:
            continue
        if ":" not in value:
            raise ValueError(f"invalid --signal-id-override {value!r}; expected FROM:TO")
        source, target = value.split(":", 1)
        source = source.strip()
        target = target.strip()
        if not source or not target:
            raise ValueError(f"invalid --signal-id-override {value!r}; expected FROM:TO")
        overrides[source] = target
    return overrides


def path_reference_candidates(
    odom: OdomSample,
    references: Sequence[SignalReference],
    signals: Dict[str, TrafficSignal],
    path_matches: Sequence[PathMatch],
    max_candidates: int,
    min_path_lane_support: float,
    path_radius: float,
    heading_tolerance_deg: float,
    signal_facing_tolerance_deg: float,
    facing_signal_radius: float,
    physical_signal_mode: str,
    signal_id_overrides: Dict[str, str],
) -> List[Candidate]:
    if not path_matches:
        return []

    ego_xy = odom.position_map[:2]
    inferred_turn = infer_path_turn(path_matches)
    candidates_by_signal: Dict[str, Tuple[float, Candidate]] = {}

    for ref in references:
        support, matched_lane_id, first_distance_along = path_reference_support(
            path_matches,
            ref,
            require_orientation=False,
        )
        if support < min_path_lane_support:
            continue

        matching_path_points = [
            match
            for match in path_matches
            if match.sample.road_id == ref.road_id
            and match.sample.lane_id == matched_lane_id
            and lane_id_in_validity(match.sample.lane_id, ref.validity)
        ]
        best_match = min(
            matching_path_points,
            key=lambda match: math.hypot(match.x - ref.x, match.y - ref.y),
        )
        lateral_distance = math.hypot(best_match.x - ref.x, best_match.y - ref.y)
        if lateral_distance > path_radius:
            continue

        physical_signal_id = signal_id_overrides.get(ref.signal_id, ref.signal_id)
        if physical_signal_id not in signals:
            continue
        signal = signals[physical_signal_id]
        if physical_signal_mode == "same-heading":
            signal_follows_path, _ = signal_matches_path_heading(
                signal,
                best_match.yaw,
                signal_facing_tolerance_deg,
            )
            if not signal_follows_path:
                continue
        if physical_signal_mode == "facing-replacement":
            replacement = path_facing_signal(
                ref,
                signal,
                signals,
                ego_xy,
                best_match,
                signal_facing_tolerance_deg,
                facing_signal_radius,
            )
            if replacement is None:
                continue
            signal = replacement
        distance_to_ref = float(np.linalg.norm(np.array([ref.x, ref.y]) - ego_xy))
        signal_distance = float(
            np.linalg.norm(np.array([signal.x, signal.y]) - ego_xy)
        )
        turn_penalty = (
            0.0
            if not inferred_turn or not ref.turn_relation or ref.turn_relation == inferred_turn
            else 1000.0
        )
        heading_error = math.degrees(
            angle_error(best_match.yaw, reference_drive_heading(ref))
        )
        heading_penalty = 0.0 if heading_error <= heading_tolerance_deg else 100.0
        # The hard gate is road/lane validity from the future path. Heading is
        # only a penalty because CARLA Town10 signalReference orientation is
        # inconsistent for this junction.
        score = (
            turn_penalty
            + heading_penalty
            + lateral_distance
            + 0.02 * first_distance_along
            - 0.05 * support
        )
        candidate = Candidate(
            ref,
            signal,
            distance_to_ref,
            signal_distance,
            path_support_m=support,
            matched_lane_id=matched_lane_id,
        )
        previous = candidates_by_signal.get(ref.signal_id)
        if previous is None or score < previous[0]:
            candidates_by_signal[ref.signal_id] = (score, candidate)

    ranked = sorted(candidates_by_signal.values(), key=lambda item: item[0])
    return [candidate for _, candidate in ranked[:max_candidates]]


def nearest_image_row(rows: Sequence[ImageRow], timestamps: Sequence[int], timestamp_ns: int) -> ImageRow:
    return rows[nearest_index(timestamps, timestamp_ns)]


def load_image_bgr(connection: sqlite3.Connection, row_id: int) -> Image:
    row = connection.execute("select data from messages where id=?", (row_id,)).fetchone()
    if row is None:
        raise ValueError(f"message row {row_id} not found")
    return deserialize_message(row[0], Image)


def image_msg_to_bgr(message: Image) -> np.ndarray:
    if message.encoding.lower() != "bgra8":
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    image = np.frombuffer(message.data, dtype=np.uint8).reshape(
        (message.height, message.width, 4)
    )
    return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)


def project_map_point(
    point_map: np.ndarray,
    odom: OdomSample,
    camera: CameraConfig,
    axis_mode: str,
    camera_frame_mode: str,
    image_horizontal_sign: str,
) -> Optional[Tuple[float, float, float, np.ndarray]]:
    point_ego = odom.rotation_map_ego.T @ (point_map - odom.position_map)
    point_camera = camera.rotation_ego_camera.T @ (point_ego - camera.position_ego)

    if camera_frame_mode == "ros-optical":
        x_opt = point_camera[0]
        y_opt = point_camera[1]
        z_opt = point_camera[2]
    elif axis_mode == "carla":
        x_opt = point_camera[1]
        y_opt = -point_camera[2]
        z_opt = point_camera[0]
    else:
        x_opt = -point_camera[1]
        y_opt = -point_camera[2]
        z_opt = point_camera[0]

    if image_horizontal_sign == "flip":
        x_opt = -x_opt

    if z_opt <= 0.05:
        return None

    u = camera.fx * x_opt / z_opt + camera.cx
    v = camera.fy * y_opt / z_opt + camera.cy
    return u, v, z_opt, point_camera


def manual_signal_candidate(
    odom: OdomSample,
    signals: Dict[str, TrafficSignal],
    signal_id: str,
) -> Candidate:
    signal = signals.get(signal_id)
    if signal is None:
        available = ", ".join(sorted(signals))
        raise ValueError(f"manual signal id {signal_id} not found; available ids: {available}")

    signal_xy = np.array([signal.x, signal.y], dtype=float)
    signal_distance = float(np.linalg.norm(signal_xy - odom.position_map[:2]))
    ref = SignalReference(
        signal_id=signal.signal_id,
        road_id=signal.road_id,
        s=0.0,
        x=signal.x,
        y=signal.y,
        heading=signal.heading,
        orientation="",
        turn_relation="Manual",
        validity=(),
    )
    return Candidate(
        ref=ref,
        signal=signal,
        distance_to_ref=signal_distance,
        signal_distance=signal_distance,
    )


def candidates_for_pose(
    odom: OdomSample,
    references: Sequence[SignalReference],
    signals: Dict[str, TrafficSignal],
    trigger_distance: float,
    max_candidates: int,
    candidate_mode: str,
    max_signal_distance: float,
    facing_angle_deg: float,
    path_matches: Sequence[PathMatch],
    min_path_lane_support: float,
    signal_reference_path_radius: float,
    reference_heading_tolerance_deg: float,
    path_signal_facing_tolerance_deg: float,
    path_facing_signal_radius: float,
    path_physical_signal_mode: str,
    signal_id_overrides: Dict[str, str],
) -> List[Candidate]:
    ego_xy = odom.position_map[:2]

    if candidate_mode == "path":
        return path_reference_candidates(
            odom,
            references,
            signals,
            path_matches,
            max_candidates,
            min_path_lane_support,
            signal_reference_path_radius,
            reference_heading_tolerance_deg,
            path_signal_facing_tolerance_deg,
            path_facing_signal_radius,
            path_physical_signal_mode,
            signal_id_overrides,
        )

    trigger_references = []
    for ref in references:
        ref_xy = np.array([ref.x, ref.y])
        distance_to_ref = float(np.linalg.norm(ref_xy - ego_xy))
        if distance_to_ref > trigger_distance:
            continue
        trigger_references.append((distance_to_ref, ref))

    if not trigger_references:
        return []

    trigger_references.sort(key=lambda item: item[0])

    if candidate_mode == "facing":
        nearest_trigger_distance, nearest_trigger_ref = trigger_references[0]
        candidates = []
        min_facing_score = math.cos(math.radians(facing_angle_deg))
        for signal in signals.values():
            signal_xy = np.array([signal.x, signal.y])
            signal_distance = float(np.linalg.norm(signal_xy - ego_xy))
            if signal_distance > max_signal_distance:
                continue
            vector_to_ego = ego_xy - signal_xy
            norm = float(np.linalg.norm(vector_to_ego))
            if norm < 1e-6:
                continue
            signal_forward = np.array(
                [math.cos(signal.heading), math.sin(signal.heading)], dtype=float
            )
            facing_score = float(signal_forward @ (vector_to_ego / norm))
            if facing_score < min_facing_score:
                continue
            candidates.append(
                Candidate(
                    nearest_trigger_ref,
                    signal,
                    nearest_trigger_distance,
                    signal_distance,
                )
            )
        candidates.sort(key=lambda item: item.signal_distance)
        return candidates[:max_candidates]

    nearest_by_signal: Dict[str, Candidate] = {}
    for distance_to_ref, ref in trigger_references:
        signal = signals[ref.signal_id]
        signal_distance = float(
            np.linalg.norm(np.array([signal.x, signal.y]) - ego_xy)
        )
        candidate = Candidate(ref, signal, distance_to_ref, signal_distance)
        previous = nearest_by_signal.get(ref.signal_id)
        if previous is None or candidate.distance_to_ref < previous.distance_to_ref:
            nearest_by_signal[ref.signal_id] = candidate
    candidates = list(nearest_by_signal.values())
    candidates.sort(key=lambda item: item.distance_to_ref)
    return candidates[:max_candidates]


def color_for_signal(signal_id: str) -> Tuple[int, int, int]:
    palette = [
        (40, 220, 40),
        (40, 160, 255),
        (255, 80, 80),
        (220, 80, 255),
        (0, 220, 255),
        (255, 180, 40),
    ]
    return palette[sum(ord(char) for char in signal_id) % len(palette)]


def signal_facing_score(signal: TrafficSignal, odom: OdomSample) -> float:
    signal_xy = np.array([signal.x, signal.y], dtype=float)
    vector_to_ego = odom.position_map[:2] - signal_xy
    norm = float(np.linalg.norm(vector_to_ego))
    if norm < 1e-6:
        return 0.0
    signal_forward = np.array(
        [math.cos(signal.heading), math.sin(signal.heading)], dtype=float
    )
    return float(signal_forward @ (vector_to_ego / norm))


def draw_label(
    image: np.ndarray,
    text: str,
    origin: Tuple[int, int],
    color: Tuple[int, int, int],
    scale: float = 0.30,
) -> None:
    x, y = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    (width, height), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(
        image,
        (x - 2, y - height - baseline - 3),
        (x + width + 2, y + baseline + 1),
        (0, 0, 0),
        -1,
    )
    cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_header(image: np.ndarray, lines: Sequence[str]) -> None:
    x = 10
    y = 22
    for line in lines:
        draw_label(image, line, (x, y), (255, 255, 255))
        y += 22


def load_yolo_model(model_path: Optional[Path]) -> Optional[object]:
    if model_path is None:
        return None
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "Could not import ultralytics. Activate the YOLO environment or install ultralytics."
        ) from exc
    return YOLO(str(model_path))


def expanded_roi_xyxy(
    u: float,
    v: float,
    box_w: float,
    box_h: float,
    image_width: int,
    image_height: int,
    scale: float,
    min_width: float,
    min_height: float,
) -> Optional[Tuple[int, int, int, int]]:
    roi_w = max(box_w * scale, min_width)
    roi_h = max(box_h * scale, min_height)
    left = int(math.floor(u - roi_w / 2.0))
    top = int(math.floor(v - roi_h / 2.0))
    right = int(math.ceil(u + roi_w / 2.0))
    bottom = int(math.ceil(v + roi_h / 2.0))
    left = max(0, min(image_width, left))
    top = max(0, min(image_height, top))
    right = max(0, min(image_width, right))
    bottom = max(0, min(image_height, bottom))
    if right <= left or bottom <= top:
        return None
    return left, top, right, bottom


def clipped_box_crop(
    image: np.ndarray,
    box_xyxy: Tuple[float, float, float, float],
    padding: int = 2,
) -> Optional[np.ndarray]:
    x1, y1, x2, y2 = box_xyxy
    left = max(0, int(math.floor(x1)) - padding)
    top = max(0, int(math.floor(y1)) - padding)
    right = min(image.shape[1], int(math.ceil(x2)) + padding)
    bottom = min(image.shape[0], int(math.ceil(y2)) + padding)
    if right <= left or bottom <= top:
        return None
    crop = image[top:bottom, left:right]
    return crop if crop.size else None


def classify_traffic_light_state(
    image: np.ndarray,
    box_xyxy: Tuple[float, float, float, float],
) -> Tuple[str, Optional[float], Tuple[float, float, float]]:
    crop = clipped_box_crop(image, box_xyxy, padding=1)
    if crop is None:
        return "unknown", None, (0.0, 0.0, 0.0)

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    value_threshold = max(80.0, float(np.percentile(value, 95)))
    active = (value >= value_threshold) & (saturation >= 30)
    if int(active.sum()) < 2:
        value_threshold = max(80.0, float(np.percentile(value, 97)))
        active = value >= value_threshold

    weight = (value.astype(float) / 255.0) * (1.0 + saturation.astype(float) / 255.0)
    active_weight = weight * active.astype(float)
    total_score = float(active_weight.sum())
    if total_score < 1e-6:
        return "unknown", None, (0.0, 0.0, 0.0)

    height = crop.shape[0]
    y_positions = np.arange(height, dtype=float)[:, None]
    y_norm = float((active_weight * y_positions).sum() / total_score)
    if height > 1:
        y_norm /= float(height - 1)

    top_mask = np.zeros_like(active, dtype=bool)
    middle_mask = np.zeros_like(active, dtype=bool)
    bottom_mask = np.zeros_like(active, dtype=bool)
    top_end = max(1, int(round(height / 3.0)))
    bottom_start = min(height - 1, int(round(2.0 * height / 3.0)))
    top_mask[:top_end, :] = True
    middle_mask[top_end:bottom_start, :] = True
    bottom_mask[bottom_start:, :] = True
    scores = {
        "red": float(active_weight[top_mask].sum()),
        "yellow": float(active_weight[middle_mask].sum()),
        "green": float(active_weight[bottom_mask].sum()),
    }

    if y_norm < 0.38:
        state = "red"
    elif y_norm > 0.62:
        state = "green"
    else:
        state = "yellow"
    confidence = scores[state] / max(sum(scores.values()), 1e-9)
    if confidence < 0.35:
        return "unknown", confidence, (scores["red"], scores["yellow"], scores["green"])
    return state, confidence, (scores["red"], scores["yellow"], scores["green"])


def color_for_light_state(state: str) -> Tuple[int, int, int]:
    if state == "red":
        return (0, 0, 255)
    if state == "yellow":
        return (0, 255, 255)
    if state == "green":
        return (0, 255, 0)
    return (255, 255, 255)


def confirm_traffic_light_in_roi(
    yolo_model: object,
    source_image: np.ndarray,
    u: float,
    v: float,
    box_w: float,
    box_h: float,
    roi_scale: float,
    min_roi_width: float,
    min_roi_height: float,
    yolo_conf: float,
    yolo_imgsz: int,
    yolo_class_id: int,
    yolo_device: str,
) -> YoloConfirmation:
    roi = expanded_roi_xyxy(
        u,
        v,
        box_w,
        box_h,
        source_image.shape[1],
        source_image.shape[0],
        roi_scale,
        min_roi_width,
        min_roi_height,
    )
    if roi is None:
        return YoloConfirmation(status="roi_outside")

    left, top, right, bottom = roi
    crop = source_image[top:bottom, left:right]
    if crop.size == 0:
        return YoloConfirmation(status="roi_empty", roi_xyxy=roi)

    predict_kwargs = {
        "imgsz": yolo_imgsz,
        "conf": yolo_conf,
        "classes": [yolo_class_id],
        "verbose": False,
    }
    if yolo_device:
        predict_kwargs["device"] = yolo_device

    results = yolo_model(crop, **predict_kwargs)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return YoloConfirmation(status="not_found", roi_xyxy=roi)

    target_x = u - left
    target_y = v - top
    roi_diag = max(1.0, math.hypot(right - left, bottom - top))
    best_score = float("inf")
    best_box: Optional[Tuple[float, float, float, float]] = None
    best_confidence: Optional[float] = None
    for box in boxes:
        xyxy_values = box.xyxy[0].detach().cpu().numpy()
        x1, y1, x2, y2 = (float(value) for value in xyxy_values)
        confidence = float(box.conf[0].detach().cpu())
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        normalized_distance = math.hypot(center_x - target_x, center_y - target_y) / roi_diag
        score = normalized_distance - 0.20 * confidence
        if score < best_score:
            best_score = score
            best_confidence = confidence
            best_box = (x1 + left, y1 + top, x2 + left, y2 + top)

    if best_box is None:
        return YoloConfirmation(status="not_found", roi_xyxy=roi)
    light_state, state_confidence, color_scores = classify_traffic_light_state(
        source_image,
        best_box,
    )
    return YoloConfirmation(
        status="confirmed",
        confidence=best_confidence,
        box_xyxy=best_box,
        roi_xyxy=roi,
        light_state=light_state,
        light_state_confidence=state_confidence,
        color_scores=color_scores,
    )


def light_box_index_for_turn(turn_relation: str) -> Optional[int]:
    return TURN_RELATION_TO_LIGHT_BOX_INDEX.get(turn_relation.strip().lower())


def roi_targets_for_candidate(
    candidate: Candidate,
    fallback_target: TrafficLightBox,
    light_box_selection: str,
) -> Tuple[TrafficLightBox, ...]:
    boxes = candidate.signal.boxes or (fallback_target,)
    if light_box_selection != "turn-index" or not candidate.signal.boxes:
        return boxes

    target_index = light_box_index_for_turn(candidate.ref.turn_relation)
    if target_index is None:
        return boxes

    selected = tuple(box for box in candidate.signal.boxes if box.box_index == target_index)
    return selected or boxes


def draw_projection(
    image: np.ndarray,
    camera: CameraConfig,
    odom: OdomSample,
    candidate: Candidate,
    axis_mode: str,
    camera_frame_mode: str,
    image_horizontal_sign: str,
    candidate_mode: str,
    roi_signal_mode: str,
    height_line_min: float,
    height_line_max: float,
    min_detectable_signal_height: float,
    light_box_selection: str,
    yolo_model: Optional[object] = None,
    source_image: Optional[np.ndarray] = None,
    yolo_conf: float = 0.05,
    yolo_imgsz: int = 640,
    yolo_class_id: int = 9,
    yolo_device: str = "",
    yolo_roi_scale: float = 5.0,
    yolo_min_roi_width: float = 160.0,
    yolo_min_roi_height: float = 180.0,
) -> List[ProjectionResult]:
    projections: List[ProjectionResult] = []
    fallback_target = TrafficLightBox(
        signal_id=candidate.signal.signal_id,
        box_index=-1,
        x=candidate.signal.x,
        y=candidate.signal.y,
        z=candidate.signal.z,
        extent_x=max(candidate.signal.width / 2.0, 0.35),
        extent_y=0.20,
        extent_z=max(candidate.signal.height / 2.0, 0.60),
    )
    roi_targets = roi_targets_for_candidate(candidate, fallback_target, light_box_selection)

    for target in roi_targets:
        signal = candidate.signal
        color = color_for_signal(signal.signal_id)
        center = project_map_point(
            np.array([target.x, target.y, target.z], dtype=float),
            odom,
            camera,
            axis_mode,
            camera_frame_mode,
            image_horizontal_sign,
        )
        if center is None:
            continue

        u, v, depth, _ = center
        physical_w = max(2.0 * target.extent_x, 2.0 * target.extent_y, 0.35)
        physical_h = max(2.0 * target.extent_z, 0.70)
        raw_box_w = camera.fx * physical_w / depth * 1.35
        raw_box_h = camera.fy * physical_h / depth * 1.35
        box_w = max(16.0, min(100.0, raw_box_w))
        box_h = max(24.0, min(140.0, raw_box_h))
        left = int(round(u - box_w / 2.0))
        top = int(round(v - box_h / 2.0))
        right = int(round(u + box_w / 2.0))
        bottom = int(round(v + box_h / 2.0))
        visible = right >= 0 and left < camera.width and bottom >= 0 and top < camera.height
        if not visible:
            status = "outside_image"
        elif raw_box_h < min_detectable_signal_height:
            status = "too_small"
        else:
            status = "ready"
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)

        yolo_confirmation = YoloConfirmation(status="disabled")
        if yolo_model is not None and source_image is not None:
            yolo_confirmation = confirm_traffic_light_in_roi(
                yolo_model,
                source_image,
                u,
                v,
                box_w,
                box_h,
                yolo_roi_scale,
                yolo_min_roi_width,
                yolo_min_roi_height,
                yolo_conf,
                yolo_imgsz,
                yolo_class_id,
                yolo_device,
            )
            if yolo_confirmation.roi_xyxy is not None:
                roi_left, roi_top, roi_right, roi_bottom = yolo_confirmation.roi_xyxy
                cv2.rectangle(
                    image,
                    (roi_left, roi_top),
                    (roi_right, roi_bottom),
                    (255, 255, 0),
                    1,
                )
            if yolo_confirmation.box_xyxy is not None:
                yx1, yy1, yx2, yy2 = yolo_confirmation.box_xyxy
                state_color = color_for_light_state(yolo_confirmation.light_state)
                cv2.rectangle(
                    image,
                    (int(round(yx1)), int(round(yy1))),
                    (int(round(yx2)), int(round(yy2))),
                    state_color,
                    2,
                )
                state_label = yolo_confirmation.light_state.upper()
                if yolo_confirmation.light_state_confidence is not None:
                    yolo_label = f"{state_label} {yolo_confirmation.light_state_confidence:.2f}"
                elif yolo_confirmation.confidence is not None:
                    yolo_label = f"YOLO {yolo_confirmation.confidence:.2f}"
                else:
                    yolo_label = "YOLO"
                draw_label(
                    image,
                    yolo_label,
                    (int(round(yx1)), max(18, int(round(yy1 - 6)))),
                    state_color,
                )

        roi_label = (
            signal.signal_id
            if target.box_index < 0
            else f"{signal.signal_id}:{target.box_index}"
        )
        roi_distance = float(
            np.linalg.norm(np.array([target.x, target.y]) - odom.position_map[:2])
        )
        projections.append(
            ProjectionResult(
                u=u,
                v=v,
                box_w=box_w,
                box_h=box_h,
                depth=depth,
                roi_label=roi_label,
                roi_distance=roi_distance,
                status=status,
                yolo_status=yolo_confirmation.status,
                yolo_confidence=yolo_confirmation.confidence,
                yolo_box_xyxy=yolo_confirmation.box_xyxy,
                yolo_roi_xyxy=yolo_confirmation.roi_xyxy,
                light_state=yolo_confirmation.light_state,
                light_state_confidence=yolo_confirmation.light_state_confidence,
                color_scores=yolo_confirmation.color_scores,
            )
        )

    return projections


def write_event_rows(
    writer: csv.writer,
    frame_index: int,
    camera_id: str,
    timestamp_ns: int,
    odom: OdomSample,
    candidate: Candidate,
    projection: Optional[ProjectionResult],
) -> None:
    if projection is None:
        writer.writerow(
            [
                frame_index,
                camera_id,
                timestamp_ns,
                candidate.signal.signal_id,
                "",
                candidate.ref.road_id,
                f"{candidate.distance_to_ref:.3f}",
                f"{candidate.signal_distance:.3f}",
                f"{odom.position_map[0]:.3f}",
                f"{odom.position_map[1]:.3f}",
                "",
                "",
                "",
                "",
                "",
                "behind_camera",
                candidate.matched_lane_id,
                f"{candidate.path_support_m:.3f}",
                candidate.ref.turn_relation,
                ";".join(f"{a}:{b}" for a, b in candidate.ref.validity),
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
            ]
        )
        return
    yolo_box = projection.yolo_box_xyxy
    yolo_roi = projection.yolo_roi_xyxy
    red_score, yellow_score, green_score = projection.color_scores
    writer.writerow(
        [
            frame_index,
            camera_id,
            timestamp_ns,
            candidate.signal.signal_id,
            projection.roi_label,
            candidate.ref.road_id,
            f"{candidate.distance_to_ref:.3f}",
            f"{projection.roi_distance:.3f}",
            f"{odom.position_map[0]:.3f}",
            f"{odom.position_map[1]:.3f}",
            f"{projection.u:.2f}",
            f"{projection.v:.2f}",
            f"{projection.box_w:.2f}",
            f"{projection.box_h:.2f}",
            f"{projection.depth:.2f}",
            projection.status,
            candidate.matched_lane_id,
            f"{candidate.path_support_m:.3f}",
            candidate.ref.turn_relation,
            ";".join(f"{a}:{b}" for a, b in candidate.ref.validity),
            projection.yolo_status,
            "" if projection.yolo_confidence is None else f"{projection.yolo_confidence:.3f}",
            "" if yolo_box is None else f"{yolo_box[0]:.2f}",
            "" if yolo_box is None else f"{yolo_box[1]:.2f}",
            "" if yolo_box is None else f"{yolo_box[2]:.2f}",
            "" if yolo_box is None else f"{yolo_box[3]:.2f}",
            "" if yolo_roi is None else yolo_roi[0],
            "" if yolo_roi is None else yolo_roi[1],
            "" if yolo_roi is None else yolo_roi[2],
            "" if yolo_roi is None else yolo_roi[3],
            projection.light_state,
            (
                ""
                if projection.light_state_confidence is None
                else f"{projection.light_state_confidence:.3f}"
            ),
            f"{red_score:.3f}",
            f"{yellow_score:.3f}",
            f"{green_score:.3f}",
        ]
    )


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    keyframe_dir = args.output_dir / "keyframes"
    keyframe_dir.mkdir(parents=True, exist_ok=True)
    for old_keyframe in keyframe_dir.glob("frame_*.jpg"):
        old_keyframe.unlink()

    signals, references = parse_opendrive(args.map, args.light_center_height)
    if not signals:
        raise RuntimeError(f"no traffic light signals found in {args.map}")
    if not references:
        raise RuntimeError(f"no traffic light signalReferences found in {args.map}")
    light_box_count = 0
    light_box_y_sign = 1.0
    if not args.ignore_traffic_light_boxes:
        signals, light_box_count, light_box_y_sign = load_traffic_light_boxes(
            args.traffic_light_boxes,
            signals,
            args.min_light_box_center_z,
            args.min_light_box_extent_z,
            args.traffic_light_box_y_sign,
        )
    lane_index: Optional[LaneSampleIndex] = None
    if args.candidate_mode == "path":
        lane_index = build_lane_sample_index(build_lane_samples(args.map))
    signal_id_overrides = parse_signal_id_overrides(args.signal_id_override)
    yolo_model = load_yolo_model(args.yolo_model)

    connection = sqlite3.connect(str(args.bag))
    topics = bag_topics(connection)
    cameras = load_cameras(connection, args.objects, topics, camera_ids=(args.camera_id,))

    image_topic = f"/carla/ego_vehicle/{args.camera_id}/image"
    required_topics = [
        "/carla/ego_vehicle/odometry",
        image_topic,
    ]
    for topic in required_topics:
        if topic not in topics:
            raise RuntimeError(f"missing required topic: {topic}")

    odometry = load_odometry(connection, topics["/carla/ego_vehicle/odometry"])
    odom_timestamps = [sample.timestamp_ns for sample in odometry]
    image_rows = load_image_rows(connection, topics[image_topic])

    base_camera = cameras[args.camera_id]
    camera_frame_mode = args.camera_frame_mode
    if args.use_tf_camera_transform:
        if args.camera_id != "cam_front_right":
            raise ValueError("--use-tf-camera-transform is currently measured for cam_front_right only")
        base_camera = camera_with_transform(
            base_camera,
            np.array([1.5, -0.25, 1.5], dtype=float),
            rotation_from_rpy_degrees(-90.0, 0.0, -90.0),
        )
        camera_frame_mode = "ros-optical"

    camera = apply_camera_calibration_offsets(
        base_camera,
        args.camera_roll_offset_deg,
        args.camera_pitch_offset_deg,
        args.camera_yaw_offset_deg,
        args.camera_x_offset,
        args.camera_y_offset,
        args.camera_z_offset,
        args.camera_y_sign,
    )
    output_video = args.output_dir / args.output_name
    writer = cv2.VideoWriter(
        str(output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (camera.width, camera.height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer: {output_video}")

    event_csv = args.output_dir / "projection_events.csv"
    event_count = 0
    ready_event_count = 0
    yolo_confirmed_count = 0
    light_state_counts: Dict[str, int] = {}
    keyframes_saved = 0
    first_trigger_frame: Optional[int] = None
    first_ready_frame: Optional[int] = None

    with event_csv.open("w", newline="", encoding="utf-8") as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "frame_index",
                "camera_id",
                "timestamp_ns",
                "signal_id",
                "roi_signal_id",
                "reference_road_id",
                "distance_to_reference_m",
                "distance_to_signal_xy_m",
                "ego_x",
                "ego_y",
                "u",
                "v",
                "roi_width_px",
                "roi_height_px",
                "depth_m",
                "status",
                "matched_lane_id",
                "path_support_m",
                "turn_relation",
                "lane_validity",
                "yolo_status",
                "yolo_confidence",
                "yolo_x1",
                "yolo_y1",
                "yolo_x2",
                "yolo_y2",
                "yolo_roi_left",
                "yolo_roi_top",
                "yolo_roi_right",
                "yolo_roi_bottom",
                "light_state",
                "light_state_confidence",
                "top_red_score",
                "middle_yellow_score",
                "bottom_green_score",
            ]
        )

        frame_limit = len(image_rows) if args.max_frames <= 0 else min(args.max_frames, len(image_rows))
        for frame_index, image_row in enumerate(image_rows[:frame_limit]):
            image_msg = load_image_bgr(connection, image_row.row_id)
            image = image_msg_to_bgr(image_msg)
            raw_image = image.copy()
            odom = nearest_odom(odometry, odom_timestamps, image_row.timestamp_ns)

            if args.candidate_mode == "manual":
                candidates = [manual_signal_candidate(odom, signals, args.manual_signal_id)]
            else:
                path_matches: Sequence[PathMatch] = ()
                if args.candidate_mode == "path":
                    if lane_index is None:
                        raise RuntimeError("lane index was not built for --candidate-mode path")
                    path_matches = future_path_matches(
                        odometry,
                        odom_timestamps,
                        image_row.timestamp_ns,
                        lane_index,
                        args.lookahead_distance,
                        args.lookahead_seconds,
                        args.path_sample_spacing,
                        args.lane_match_max_distance,
                        args.lane_heading_weight,
                    )
                candidates = candidates_for_pose(
                    odom,
                    references,
                    signals,
                    args.trigger_distance,
                    args.max_candidates,
                    args.candidate_mode,
                    args.max_signal_distance,
                    args.facing_angle_deg,
                    path_matches,
                    args.min_path_lane_support,
                    args.signal_reference_path_radius,
                    args.reference_heading_tolerance_deg,
                    args.path_signal_facing_tolerance_deg,
                    args.path_facing_signal_radius,
                    args.path_physical_signal_mode,
                    signal_id_overrides,
                )

            if candidates:
                event_count += 1
                if first_trigger_frame is None:
                    first_trigger_frame = frame_index

            header = [
                f"{camera.sensor_id} frame {frame_index}",
                (
                    f"axis={args.axis_mode} camera_frame={camera_frame_mode} "
                    f"h_sign={args.image_horizontal_sign} "
                    f"mode={args.candidate_mode}"
                    + (
                        f" manual_signal={args.manual_signal_id}"
                        if args.candidate_mode == "manual"
                        else ""
                    )
                ),
                (
                    f"cam_offset rpy=({args.camera_roll_offset_deg:.1f},"
                    f"{args.camera_pitch_offset_deg:.1f},{args.camera_yaw_offset_deg:.1f})"
                    f" xyz=({args.camera_x_offset:.2f},{args.camera_y_offset:.2f},"
                    f"{args.camera_z_offset:.2f}) y_sign={args.camera_y_sign}"
                ),
                f"boxes={light_box_count} y_sign={light_box_y_sign:+.0f}",
                f"small_roi_ready_h={args.min_detectable_signal_height:.0f}px",
                f"box_select={args.light_box_selection} R0 L1 S2",
                f"ego=({odom.position_map[0]:.1f},{odom.position_map[1]:.1f})",
            ]
            if yolo_model is not None:
                header.append(
                    f"yolo={args.yolo_model} conf={args.yolo_conf:.2f} roi_scale={args.yolo_roi_scale:.1f}"
                )
            draw_header(image, header)

            for candidate in candidates:
                projections = draw_projection(
                    image,
                    camera,
                    odom,
                    candidate,
                    args.axis_mode,
                    camera_frame_mode,
                    args.image_horizontal_sign,
                    args.candidate_mode,
                    args.roi_signal_mode,
                    args.height_line_min,
                    args.height_line_max,
                    args.min_detectable_signal_height,
                    args.light_box_selection,
                    yolo_model,
                    raw_image,
                    args.yolo_conf,
                    args.yolo_imgsz,
                    args.yolo_class_id,
                    args.yolo_device,
                    args.yolo_roi_scale,
                    args.yolo_min_roi_width,
                    args.yolo_min_roi_height,
                )
                if not projections:
                    write_event_rows(
                        csv_writer,
                        frame_index,
                        camera.sensor_id,
                        image_row.timestamp_ns,
                        odom,
                        candidate,
                        None,
                    )
                for projection in projections:
                    write_event_rows(
                        csv_writer,
                        frame_index,
                        camera.sensor_id,
                        image_row.timestamp_ns,
                        odom,
                        candidate,
                        projection,
                    )
                    if projection.status == "ready":
                        ready_event_count += 1
                        if first_ready_frame is None:
                            first_ready_frame = frame_index
                    if projection.yolo_status == "confirmed":
                        yolo_confirmed_count += 1
                        light_state_counts[projection.light_state] = (
                            light_state_counts.get(projection.light_state, 0) + 1
                        )

            writer.write(image)

            if candidates and keyframes_saved < args.save_keyframes:
                cv2.imwrite(
                    str(keyframe_dir / f"frame_{frame_index:04d}.jpg"),
                    image,
                )
                keyframes_saved += 1

            if args.show:
                cv2.imshow("projection validation", image)
                delay_ms = 1
                if args.playback_rate > 0 and args.fps > 0:
                    delay_ms = max(1, int(round(1000.0 / (args.fps * args.playback_rate))))
                if cv2.waitKey(delay_ms) & 0xFF == ord("q"):
                    break

    writer.release()
    connection.close()
    if args.show:
        cv2.destroyAllWindows()

    print(f"signals: {len(signals)}")
    print(f"signal references: {len(references)}")
    print(f"traffic light boxes loaded: {light_box_count}")
    print(f"traffic light box y sign: {light_box_y_sign:+.0f}")
    print(f"frames written: {frame_limit}")
    print(f"trigger frames: {event_count}")
    print(f"first trigger frame: {first_trigger_frame}")
    print(f"ready projection rows: {ready_event_count}")
    print(f"first ready frame: {first_ready_frame}")
    print(f"yolo-confirmed projection rows: {yolo_confirmed_count}")
    if light_state_counts:
        state_summary = ", ".join(
            f"{state}={count}" for state, count in sorted(light_state_counts.items())
        )
        print(f"light state rows: {state_summary}")
    print(f"video: {output_video}")
    print(f"events: {event_csv}")
    print(f"keyframes: {keyframe_dir} ({keyframes_saved} saved)")


if __name__ == "__main__":
    main()
