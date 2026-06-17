#!/usr/bin/env python3
"""Live ROS2 traffic-light state node for CARLA camera images.

The offline validator in validate_projection.py is still useful for rosbag
debugging. This node reuses its projection, ROI, YOLO confirmation, and light
state logic in a live ROS2 graph.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import validate_projection as vp  # noqa: E402


TURN_ALIASES = {
    "": "",
    "auto": "",
    "none": "",
    "right": "Right",
    "r": "Right",
    "เลี้ยวขวา": "Right",
    "left": "Left",
    "l": "Left",
    "เลี้ยวซ้าย": "Left",
    "straight": "Straight",
    "s": "Straight",
    "ตรง": "Straight",
}

ACTION_BY_STATE = {
    "green": "go",
    "yellow": "slow",
    "red": "stop",
}

VALID_ACTIONS = {"go", "slow", "stop"}


def normalize_turn(value: str) -> str:
    return TURN_ALIASES.get(value.strip().lower(), value.strip())


def stamp_to_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ros_image_to_bgr(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding in ("bgra8", "rgba8"):
        channels = 4
    elif encoding in ("bgr8", "rgb8"):
        channels = 3
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(f"unsupported image encoding: {message.encoding}")

    data = np.frombuffer(message.data, dtype=np.uint8)
    if channels == 1:
        row = data.reshape((message.height, message.step))[:, : message.width]
        return cv2.cvtColor(row, cv2.COLOR_GRAY2BGR)

    row = data.reshape((message.height, message.step))[:, : message.width * channels]
    image = row.reshape((message.height, message.width, channels))
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def bgr_to_ros_image(image: np.ndarray, source: Image) -> Image:
    message = Image()
    message.header = source.header
    message.height = int(image.shape[0])
    message.width = int(image.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(image.shape[1] * 3)
    message.data = image.tobytes()
    return message


class TrafficLightStateNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_state_node")

        default_model = vp.SRC_DIR / "yolo11s.pt"
        default_yolo_model = str(default_model) if default_model.exists() else ""

        self.camera_id = self.declare_parameter("camera_id", "cam_front_right").value
        self.image_topic = self.declare_parameter(
            "image_topic", f"/carla/ego_vehicle/{self.camera_id}/image"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", f"/carla/ego_vehicle/{self.camera_id}/camera_info"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/carla/ego_vehicle/odometry"
        ).value
        self.turn_topic = self.declare_parameter(
            "turn_topic", "/traffic_light/route_turn"
        ).value
        self.state_topic = self.declare_parameter(
            "state_topic", "/traffic_light/state"
        ).value
        self.action_topic = self.declare_parameter(
            "action_topic", "/traffic_light/action"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/traffic_light/status"
        ).value
        self.debug_image_topic = self.declare_parameter(
            "debug_image_topic", "/traffic_light/debug_image"
        ).value

        self.map_path = Path(
            self.declare_parameter("map_path", str(vp.SRC_DIR / "Town10HD.xodr")).value
        )
        self.objects_path = Path(
            self.declare_parameter("objects_path", str(vp.SCRIPT_DIR / "objects.json")).value
        )
        self.traffic_light_boxes_path = Path(
            self.declare_parameter(
                "traffic_light_boxes_path", str(vp.SRC_DIR / "carla_light_boxes.csv")
            ).value
        )
        yolo_model_value = str(
            self.declare_parameter("yolo_model", default_yolo_model).value
        ).strip()
        self.yolo_model_path: Optional[Path] = (
            None
            if yolo_model_value.lower() in ("", "none", "disabled", "off")
            else Path(yolo_model_value)
        )

        self.candidate_mode = self.declare_parameter(
            "candidate_mode", "route_turn"
        ).value
        self.route_turn = normalize_turn(
            self.declare_parameter("route_turn", "auto").value
        )
        self.manual_signal_id = self.declare_parameter("manual_signal_id", "958").value
        self.light_box_selection = self.declare_parameter(
            "light_box_selection", "turn-index"
        ).value
        self.path_physical_signal_mode = self.declare_parameter(
            "path_physical_signal_mode", "same-heading"
        ).value
        self.axis_mode = self.declare_parameter("axis_mode", "carla").value
        self.camera_frame_mode = self.declare_parameter(
            "camera_frame_mode", "carla-sensor"
        ).value
        self.image_horizontal_sign = self.declare_parameter(
            "image_horizontal_sign", "flip"
        ).value
        self.use_tf_camera_transform = bool(
            self.declare_parameter("use_tf_camera_transform", False).value
        )

        self.trigger_distance = float(self.declare_parameter("trigger_distance", 60.0).value)
        self.max_reference_angle_deg = float(
            self.declare_parameter("max_reference_angle_deg", 110.0).value
        )
        self.max_candidates = int(self.declare_parameter("max_candidates", 1).value)
        self.min_detectable_signal_height = float(
            self.declare_parameter("min_detectable_signal_height", 10.0).value
        )
        self.path_signal_facing_tolerance_deg = float(
            self.declare_parameter("path_signal_facing_tolerance_deg", 75.0).value
        )
        self.light_center_height = float(
            self.declare_parameter("light_center_height", 5.0).value
        )
        self.min_light_box_center_z = float(
            self.declare_parameter("min_light_box_center_z", 3.0).value
        )
        self.min_light_box_extent_z = float(
            self.declare_parameter("min_light_box_extent_z", 0.25).value
        )
        self.traffic_light_box_y_sign = self.declare_parameter(
            "traffic_light_box_y_sign", "auto"
        ).value

        self.yolo_conf = float(self.declare_parameter("yolo_conf", 0.05).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_class_id = int(self.declare_parameter("yolo_class_id", 9).value)
        self.yolo_device = self.declare_parameter("yolo_device", "").value
        self.yolo_roi_scale = float(self.declare_parameter("yolo_roi_scale", 5.0).value)
        self.yolo_min_roi_width = float(
            self.declare_parameter("yolo_min_roi_width", 160.0).value
        )
        self.yolo_min_roi_height = float(
            self.declare_parameter("yolo_min_roi_height", 180.0).value
        )

        self.publish_debug_image = bool(
            self.declare_parameter("publish_debug_image", True).value
        )
        self.process_every_n_frames = max(
            1, int(self.declare_parameter("process_every_n_frames", 1).value)
        )
        self.max_odom_age_sec = float(
            self.declare_parameter("max_odom_age_sec", 0.5).value
        )
        self.unknown_action = self._validated_action(
            self.declare_parameter("unknown_action", "slow").value,
            fallback="slow",
        )
        self.no_light_action = self._validated_action(
            self.declare_parameter("no_light_action", "go").value,
            fallback="go",
        )

        self.camera_roll_offset_deg = float(
            self.declare_parameter("camera_roll_offset_deg", 0.0).value
        )
        self.camera_pitch_offset_deg = float(
            self.declare_parameter("camera_pitch_offset_deg", 0.0).value
        )
        self.camera_yaw_offset_deg = float(
            self.declare_parameter("camera_yaw_offset_deg", 0.0).value
        )
        self.camera_x_offset = float(self.declare_parameter("camera_x_offset", 0.0).value)
        self.camera_y_offset = float(self.declare_parameter("camera_y_offset", 0.0).value)
        self.camera_z_offset = float(self.declare_parameter("camera_z_offset", 0.0).value)
        self.camera_y_sign = self.declare_parameter("camera_y_sign", "as-is").value

        self.signals, self.references = vp.parse_opendrive(
            self.map_path,
            self.light_center_height,
        )
        self.signals, self.light_box_count, self.light_box_y_sign = vp.load_traffic_light_boxes(
            self.traffic_light_boxes_path,
            self.signals,
            self.min_light_box_center_z,
            self.min_light_box_extent_z,
            self.traffic_light_box_y_sign,
        )
        self.signal_id_overrides: Dict[str, str] = {}
        self.base_camera = self._load_camera_extrinsic()
        self.camera: Optional[vp.CameraConfig] = None
        self.last_odom: Optional[vp.OdomSample] = None
        self.last_odom_stamp_ns: Optional[int] = None
        self.frame_count = 0

        self.yolo_model = None
        if self.yolo_model_path is not None:
            self.yolo_model = vp.load_yolo_model(self.yolo_model_path)

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.action_pub = self.create_publisher(String, self.action_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.debug_image_pub = (
            self.create_publisher(Image, self.debug_image_topic, 1)
            if self.publish_debug_image
            else None
        )

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, self.turn_topic, self.turn_callback, 10)

        self.get_logger().info(
            "traffic light node ready: "
            f"image={self.image_topic}, odom={self.odom_topic}, "
            f"turn={self.route_turn or 'auto'}, "
            f"yolo={self.yolo_model_path if self.yolo_model_path is not None else 'disabled'}"
        )

    def _validated_action(self, value: str, fallback: str) -> str:
        action = str(value).strip().lower()
        if action not in VALID_ACTIONS:
            self.get_logger().warn(
                f"invalid action {value!r}; using {fallback!r}"
            )
            return fallback
        return action

    def _load_camera_extrinsic(self) -> vp.CameraConfig:
        with self.objects_path.open("r", encoding="utf-8") as file:
            objects = json.load(file)

        for sensor in objects["objects"][0]["sensors"]:
            if sensor.get("type") != "sensor.camera.rgb":
                continue
            if sensor.get("id", "") != self.camera_id:
                continue
            spawn = sensor["spawn_point"]
            camera = vp.CameraConfig(
                sensor_id=self.camera_id,
                topic_prefix=f"/carla/ego_vehicle/{self.camera_id}",
                position_ego=np.array(
                    [
                        float(spawn["x"]),
                        float(spawn["y"]),
                        float(spawn["z"]),
                    ],
                    dtype=float,
                ),
                rotation_ego_camera=vp.rotation_from_rpy_degrees(
                    float(spawn.get("roll", 0.0)),
                    float(spawn.get("pitch", 0.0)),
                    float(spawn.get("yaw", 0.0)),
                ),
                width=0,
                height=0,
                fx=0.0,
                fy=0.0,
                cx=0.0,
                cy=0.0,
            )
            if self.use_tf_camera_transform:
                if self.camera_id != "cam_front_right":
                    raise ValueError(
                        "use_tf_camera_transform is measured for cam_front_right only"
                    )
                self.camera_frame_mode = "ros-optical"
                camera = vp.camera_with_transform(
                    camera,
                    np.array([1.5, -0.25, 1.5], dtype=float),
                    vp.rotation_from_rpy_degrees(-90.0, 0.0, -90.0),
                )
            return camera

        raise RuntimeError(f"camera {self.camera_id!r} not found in {self.objects_path}")

    def camera_info_callback(self, message: CameraInfo) -> None:
        fx = float(message.k[0] or message.p[0])
        fy = float(message.k[4] or message.p[5])
        cx = float(message.k[2] or message.p[2])
        cy = float(message.k[5] or message.p[6])
        camera = vp.CameraConfig(
            sensor_id=self.base_camera.sensor_id,
            topic_prefix=self.base_camera.topic_prefix,
            position_ego=self.base_camera.position_ego,
            rotation_ego_camera=self.base_camera.rotation_ego_camera,
            width=int(message.width),
            height=int(message.height),
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
        self.camera = vp.apply_camera_calibration_offsets(
            camera,
            self.camera_roll_offset_deg,
            self.camera_pitch_offset_deg,
            self.camera_yaw_offset_deg,
            self.camera_x_offset,
            self.camera_y_offset,
            self.camera_z_offset,
            self.camera_y_sign,
        )

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.last_odom = vp.OdomSample(
            timestamp_ns=stamp_to_ns(message.header.stamp),
            position_map=np.array([position.x, position.y, position.z], dtype=float),
            rotation_map_ego=vp.rotation_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        self.last_odom_stamp_ns = self.last_odom.timestamp_ns

    def turn_callback(self, message: String) -> None:
        self.route_turn = normalize_turn(message.data)

    def image_callback(self, message: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        if self.camera is None:
            self.get_logger().warn(
                "waiting for camera_info before processing images",
                throttle_duration_sec=5.0,
            )
            return
        if self.last_odom is None:
            self.get_logger().warn(
                "waiting for odometry before processing images",
                throttle_duration_sec=5.0,
            )
            return

        image_stamp_ns = stamp_to_ns(message.header.stamp)
        if image_stamp_ns > 0 and self.last_odom_stamp_ns is not None:
            age_sec = abs(image_stamp_ns - self.last_odom_stamp_ns) / 1e9
            if age_sec > self.max_odom_age_sec:
                self.get_logger().warn(
                    f"latest odometry is {age_sec:.2f}s from image stamp",
                    throttle_duration_sec=5.0,
                )

        try:
            image = ros_image_to_bgr(message)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        debug_image = image.copy()
        candidates = self._select_candidates(self.last_odom)
        state, action, status = self._evaluate_candidates(
            image,
            debug_image,
            self.last_odom,
            candidates,
        )
        status.update(
            {
                "state": state,
                "action": action,
                "route_turn": self.route_turn or "auto",
                "candidate_mode": self.candidate_mode,
                "image_stamp_ns": image_stamp_ns,
                "odom_stamp_ns": self.last_odom_stamp_ns,
            }
        )

        self.state_pub.publish(String(data=state))
        self.action_pub.publish(String(data=action))
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        if self.debug_image_pub is not None:
            self.debug_image_pub.publish(bgr_to_ros_image(debug_image, message))

    def _select_candidates(self, odom: vp.OdomSample) -> List[vp.Candidate]:
        if self.candidate_mode == "manual":
            return [vp.manual_signal_candidate(odom, self.signals, self.manual_signal_id)]
        if self.candidate_mode not in ("reference", "route_turn"):
            self.get_logger().warn(
                f"unsupported candidate_mode={self.candidate_mode!r}; using route_turn",
                throttle_duration_sec=5.0,
            )

        target_turn = self.route_turn if self.candidate_mode == "route_turn" else ""
        return self._reference_candidates(odom, target_turn)

    def _reference_candidates(
        self,
        odom: vp.OdomSample,
        target_turn: str,
    ) -> List[vp.Candidate]:
        ego_xy = odom.position_map[:2]
        yaw = vp.odom_yaw(odom)
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        ranked: List[Tuple[float, vp.Candidate]] = []

        for ref in self.references:
            ref_xy = np.array([ref.x, ref.y], dtype=float)
            vector_to_ref = ref_xy - ego_xy
            distance_to_ref = float(np.linalg.norm(vector_to_ref))
            if distance_to_ref > self.trigger_distance:
                continue

            if distance_to_ref > 1e-6:
                angle = math.degrees(
                    math.acos(clamp(float((vector_to_ref / distance_to_ref) @ forward), -1.0, 1.0))
                )
                if angle > self.max_reference_angle_deg:
                    continue
            else:
                angle = 0.0

            if target_turn and normalize_turn(ref.turn_relation) != target_turn:
                continue

            signal_id = self.signal_id_overrides.get(ref.signal_id, ref.signal_id)
            signal = self.signals.get(signal_id)
            if signal is None:
                continue

            if self.path_physical_signal_mode == "same-heading":
                follows_path, heading_error = vp.signal_matches_path_heading(
                    signal,
                    yaw,
                    self.path_signal_facing_tolerance_deg,
                )
                if not follows_path:
                    continue
                heading_penalty = math.degrees(heading_error) * 0.02
            else:
                heading_penalty = 0.0

            signal_xy = np.array([signal.x, signal.y], dtype=float)
            signal_distance = float(np.linalg.norm(signal_xy - ego_xy))
            score = distance_to_ref + 0.10 * signal_distance + heading_penalty + 0.02 * angle
            ranked.append(
                (
                    score,
                    vp.Candidate(
                        ref=ref,
                        signal=signal,
                        distance_to_ref=distance_to_ref,
                        signal_distance=signal_distance,
                    ),
                )
            )

        ranked.sort(key=lambda item: item[0])
        return [candidate for _, candidate in ranked[: self.max_candidates]]

    def _evaluate_candidates(
        self,
        source_image: np.ndarray,
        debug_image: np.ndarray,
        odom: vp.OdomSample,
        candidates: Sequence[vp.Candidate],
    ) -> Tuple[str, str, Dict[str, object]]:
        if not candidates:
            return (
                "none",
                self.no_light_action,
                {
                    "reason": "no_candidate",
                    "candidate_count": 0,
                    "yolo_status": "not_run",
                },
            )

        projections: List[Tuple[vp.Candidate, vp.ProjectionResult]] = []
        for candidate in candidates:
            candidate_projections = vp.draw_projection(
                debug_image,
                self.camera,
                odom,
                candidate,
                self.axis_mode,
                self.camera_frame_mode,
                self.image_horizontal_sign,
                self.candidate_mode,
                "reference",
                3.0,
                6.0,
                self.min_detectable_signal_height,
                self.light_box_selection,
                self.yolo_model,
                source_image,
                self.yolo_conf,
                self.yolo_imgsz,
                self.yolo_class_id,
                self.yolo_device,
                self.yolo_roi_scale,
                self.yolo_min_roi_width,
                self.yolo_min_roi_height,
            )
            for projection in candidate_projections:
                projections.append((candidate, projection))

        confirmed = [
            (candidate, projection)
            for candidate, projection in projections
            if projection.yolo_status == "confirmed" and projection.light_state in ACTION_BY_STATE
        ]
        if not confirmed:
            yolo_status = "disabled" if self.yolo_model is None else "not_confirmed"
            if projections:
                yolo_status = projections[0][1].yolo_status
            return (
                "unknown",
                self.unknown_action,
                {
                    "reason": "no_confirmed_state",
                    "candidate_count": len(candidates),
                    "projection_count": len(projections),
                    "yolo_status": yolo_status,
                },
            )

        def projection_score(item: Tuple[vp.Candidate, vp.ProjectionResult]) -> float:
            candidate, projection = item
            state_conf = projection.light_state_confidence or 0.0
            yolo_conf = projection.yolo_confidence or 0.0
            return state_conf + 0.25 * yolo_conf - 0.002 * candidate.signal_distance

        best_candidate, best_projection = max(confirmed, key=projection_score)
        state = best_projection.light_state
        action = ACTION_BY_STATE[state]
        red_score, yellow_score, green_score = best_projection.color_scores
        return (
            state,
            action,
            {
                "reason": "confirmed",
                "candidate_count": len(candidates),
                "projection_count": len(projections),
                "signal_id": best_candidate.signal.signal_id,
                "roi_signal_id": best_projection.roi_label,
                "turn_relation": best_candidate.ref.turn_relation,
                "distance_to_reference_m": best_candidate.distance_to_ref,
                "distance_to_signal_m": best_candidate.signal_distance,
                "yolo_status": best_projection.yolo_status,
                "yolo_confidence": best_projection.yolo_confidence,
                "light_state_confidence": best_projection.light_state_confidence,
                "top_red_score": red_score,
                "middle_yellow_score": yellow_score,
                "bottom_green_score": green_score,
            },
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrafficLightStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
