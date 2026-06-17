#!/usr/bin/env python3
"""Estimate traffic-light world positions from image pixels.

This is a calibration/debug helper for cases where OpenDRIVE signal positions
do not line up with the rendered CARLA traffic-light heads. It back-projects a
clicked/measured pixel through the camera ray and intersects it with an assumed
lamp-center height.
"""

from __future__ import annotations

import argparse
import csv
import sqlite3
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

import validate_projection as vp


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parents[1]
PROJECT_DIR = SCRIPT_DIR.parents[2]


def parse_point(value: str) -> Tuple[str, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("point must be label,u,v")
    label, u_text, v_text = parts
    if not label:
        raise argparse.ArgumentTypeError("point label must not be empty")
    return label, float(u_text), float(v_text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Back-project measured traffic-light pixels to map/world coordinates."
    )
    parser.add_argument("--bag", type=Path, default=SRC_DIR / "rosbag2_00_0.db3")
    parser.add_argument("--objects", type=Path, default=SCRIPT_DIR / "objects.json")
    parser.add_argument("--frame", type=int, default=93)
    parser.add_argument(
        "--camera-id",
        choices=("cam_front_left", "cam_front_right"),
        default="cam_front_right",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=5.0,
        help="Assumed lamp-center world Z in meters.",
    )
    parser.add_argument(
        "--point",
        action="append",
        type=parse_point,
        default=[],
        help="Traffic-light pixel as label,u,v. Can be repeated.",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=PROJECT_DIR / "outputs" / "projection_validation" / "measured_light_positions.csv",
    )
    parser.add_argument(
        "--output-image",
        type=Path,
        default=PROJECT_DIR / "outputs" / "projection_validation" / "measured_light_positions.jpg",
    )
    return parser.parse_args()


def pixel_to_world_at_z(
    u: float,
    v: float,
    z_world: float,
    odom: vp.OdomSample,
    camera: vp.CameraConfig,
) -> Tuple[np.ndarray, float]:
    x_opt = (u - camera.cx) / camera.fx
    y_opt = (v - camera.cy) / camera.fy

    # CARLA camera convention used by validate_projection:
    # camera X forward, Y right, Z up. Optical x=right, y=down, z=forward.
    ray_camera = np.array([1.0, x_opt, -y_opt], dtype=float)
    origin_map = odom.position_map + odom.rotation_map_ego @ camera.position_ego
    ray_map = odom.rotation_map_ego @ (camera.rotation_ego_camera @ ray_camera)
    if abs(ray_map[2]) < 1e-9:
        raise ValueError("camera ray is nearly parallel to the requested height plane")
    scale = (z_world - origin_map[2]) / ray_map[2]
    return origin_map + scale * ray_map, float(scale)


def main() -> None:
    args = parse_args()
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    args.output_image.parent.mkdir(parents=True, exist_ok=True)

    connection = sqlite3.connect(str(args.bag))
    topics = vp.bag_topics(connection)
    camera = vp.load_cameras(connection, args.objects, topics, camera_ids=(args.camera_id,))[
        args.camera_id
    ]
    image_topic = f"/carla/ego_vehicle/{args.camera_id}/image"
    image_rows = vp.load_image_rows(connection, topics[image_topic])
    if args.frame < 0 or args.frame >= len(image_rows):
        raise ValueError(f"frame {args.frame} is outside 0..{len(image_rows) - 1}")

    odometry = vp.load_odometry(connection, topics["/carla/ego_vehicle/odometry"])
    odom_timestamps = [sample.timestamp_ns for sample in odometry]
    image_row = image_rows[args.frame]
    odom = vp.nearest_odom(odometry, odom_timestamps, image_row.timestamp_ns)
    image = vp.image_msg_to_bgr(vp.load_image_bgr(connection, image_row.row_id))

    rows: List[dict] = []
    for label, u, pixel_v in args.point:
        world, ray_scale = pixel_to_world_at_z(u, pixel_v, args.height, odom, camera)
        rows.append(
            {
                "label": label,
                "frame": args.frame,
                "camera_id": args.camera_id,
                "pixel_u": f"{u:.2f}",
                "pixel_v": f"{pixel_v:.2f}",
                "world_x": f"{world[0]:.3f}",
                "world_y": f"{world[1]:.3f}",
                "world_z": f"{world[2]:.3f}",
                "ego_x": f"{odom.position_map[0]:.3f}",
                "ego_y": f"{odom.position_map[1]:.3f}",
                "ray_scale_m": f"{ray_scale:.3f}",
            }
        )
        cv2.drawMarker(
            image,
            (int(round(u)), int(round(pixel_v))),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        cv2.putText(
            image,
            f"{label} ({world[0]:.1f},{world[1]:.1f},{world[2]:.1f})",
            (int(round(u)) + 8, int(round(pixel_v)) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    with args.output_csv.open("w", newline="", encoding="utf-8") as csv_file:
        fieldnames = [
            "label",
            "frame",
            "camera_id",
            "pixel_u",
            "pixel_v",
            "world_x",
            "world_y",
            "world_z",
            "ego_x",
            "ego_y",
            "ray_scale_m",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    cv2.imwrite(str(args.output_image), image)
    connection.close()

    print(f"camera: {args.camera_id}")
    print(f"frame: {args.frame}")
    print(f"height: {args.height:.2f} m")
    print(f"rows: {len(rows)}")
    print(f"csv: {args.output_csv}")
    print(f"image: {args.output_image}")
    for row in rows:
        print(
            f"{row['label']}: pixel=({row['pixel_u']},{row['pixel_v']}) "
            f"world=({row['world_x']},{row['world_y']},{row['world_z']})"
        )


if __name__ == "__main__":
    main()
