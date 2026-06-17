#!/usr/bin/env python3
"""Dump CARLA traffic-light actor transforms and light-box centers.

This helper uses CARLA's Python API, not traffic-light status. It records the
geometry needed to project the actual lamp heads: actor transform plus each
light-box center/extent. CARLA's get_light_boxes() returns boxes whose
locations are already in world coordinates in the current CARLA release used by
this project, so the center is not transformed through the actor again.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable


PROJECT_DIR = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dump traffic-light light-box offsets from a running CARLA world."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_DIR / "outputs" / "projection_validation" / "carla_light_boxes.csv",
    )
    return parser.parse_args()


def average_location(vertices: Iterable[object]) -> tuple[float, float, float]:
    points = list(vertices)
    if not points:
        return 0.0, 0.0, 0.0
    return (
        sum(float(point.x) for point in points) / len(points),
        sum(float(point.y) for point in points) / len(points),
        sum(float(point.z) for point in points) / len(points),
    )


def maybe_call(actor: object, method_name: str, default: object = "") -> object:
    method = getattr(actor, method_name, None)
    if method is None:
        return default
    try:
        return method()
    except Exception:
        return default


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    try:
        import carla
    except ImportError as exc:
        raise SystemExit(
            "Could not import the CARLA Python API. Run this inside the CARLA/ROS bridge "
            "environment where `import carla` works."
        ) from exc

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    traffic_lights = [
        actor
        for actor in world.get_actors()
        if "traffic_light" in getattr(actor, "type_id", "")
    ]

    rows = []
    for actor in sorted(traffic_lights, key=lambda item: item.id):
        transform = actor.get_transform()
        actor_location = transform.location
        actor_rotation = transform.rotation
        opendrive_id = maybe_call(actor, "get_opendrive_id")
        pole_index = maybe_call(actor, "get_pole_index")
        light_boxes = maybe_call(actor, "get_light_boxes", default=[])

        if not light_boxes:
            light_boxes = [actor.bounding_box]

        for box_index, light_box in enumerate(light_boxes):
            box_center = light_box.location
            extent = light_box.extent
            vertices = light_box.get_world_vertices(carla.Transform())
            world_center = average_location(vertices)
            rows.append(
                {
                    "actor_id": actor.id,
                    "type_id": actor.type_id,
                    "opendrive_id": opendrive_id,
                    "pole_index": pole_index,
                    "actor_x": f"{actor_location.x:.6f}",
                    "actor_y": f"{actor_location.y:.6f}",
                    "actor_z": f"{actor_location.z:.6f}",
                    "actor_pitch_deg": f"{actor_rotation.pitch:.6f}",
                    "actor_yaw_deg": f"{actor_rotation.yaw:.6f}",
                    "actor_roll_deg": f"{actor_rotation.roll:.6f}",
                    "box_index": box_index,
                    "box_center_x": f"{box_center.x:.6f}",
                    "box_center_y": f"{box_center.y:.6f}",
                    "box_center_z": f"{box_center.z:.6f}",
                    "actor_to_box_x": f"{box_center.x - actor_location.x:.6f}",
                    "actor_to_box_y": f"{box_center.y - actor_location.y:.6f}",
                    "actor_to_box_z": f"{box_center.z - actor_location.z:.6f}",
                    "extent_x": f"{extent.x:.6f}",
                    "extent_y": f"{extent.y:.6f}",
                    "extent_z": f"{extent.z:.6f}",
                    "world_center_x": f"{world_center[0]:.6f}",
                    "world_center_y": f"{world_center[1]:.6f}",
                    "world_center_z": f"{world_center[2]:.6f}",
                    "world_vertices": json.dumps(
                        [
                            {
                                "x": round(float(vertex.x), 6),
                                "y": round(float(vertex.y), 6),
                                "z": round(float(vertex.z), 6),
                            }
                            for vertex in vertices
                        ],
                        separators=(",", ":"),
                    ),
                }
            )

    fieldnames = [
        "actor_id",
        "type_id",
        "opendrive_id",
        "pole_index",
        "actor_x",
        "actor_y",
        "actor_z",
        "actor_pitch_deg",
        "actor_yaw_deg",
        "actor_roll_deg",
        "box_index",
        "box_center_x",
        "box_center_y",
        "box_center_z",
        "actor_to_box_x",
        "actor_to_box_y",
        "actor_to_box_z",
        "extent_x",
        "extent_y",
        "extent_z",
        "world_center_x",
        "world_center_y",
        "world_center_z",
        "world_vertices",
    ]
    with args.output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"traffic light actors: {len(traffic_lights)}")
    print(f"light boxes: {len(rows)}")
    print(f"csv: {args.output}")


if __name__ == "__main__":
    main()
