from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _first_existing(*candidates: Path) -> Path:
    for candidate in candidates:
        if candidate and candidate.exists():
            return candidate
    # Fall back to the first candidate so the failure message points somewhere sane.
    return candidates[0]


def generate_launch_description() -> LaunchDescription:
    # This launch file lives at <src>/tf_detect/launch/. Resolve paths relative to
    # it so the package is portable across machines instead of hard-coding an
    # absolute home directory. PROJECT_MOBILE_SRC still overrides if set.
    launch_dir = Path(__file__).resolve().parent          # <src>/tf_detect/launch
    pkg_src_dir = launch_dir.parents[1]                   # <src>  (contains tf_detect/)
    ws_root = launch_dir.parents[2]                       # workspace root (one above <src>)

    env_src = os.environ.get("PROJECT_MOBILE_SRC")
    src_dir = Path(env_src) if env_src else pkg_src_dir

    # Town10HD.xodr may live under <src> (khunanon layout) or under <ws>/map
    # (this workspace). carla_light_boxes.csv / yolo11s.pt live under <src>.
    map_default = _first_existing(
        src_dir / "Town10HD.xodr",
        ws_root / "map" / "Town10HD.xodr",
        Path(os.environ.get("CARLA_MAP_XODR", "")) if os.environ.get("CARLA_MAP_XODR") else None,
    )

    launch_arguments = [
        DeclareLaunchArgument("camera_id", default_value="cam_front_right"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/carla/ego_vehicle/cam_front_right/image",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/carla/ego_vehicle/cam_front_right/camera_info",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/vins_stereo_vel/odometry",
        ),
        DeclareLaunchArgument("route_turn", default_value="auto"),
        DeclareLaunchArgument("map_path", default_value=str(map_default)),
        DeclareLaunchArgument(
            "objects_path",
            default_value=str(src_dir / "tf_detect/scripts/objects.json"),
        ),
        DeclareLaunchArgument(
            "traffic_light_boxes_path",
            default_value=str(src_dir / "carla_light_boxes.csv"),
        ),
        DeclareLaunchArgument("yolo_model", default_value=str(src_dir / "yolo11s.pt")),
        DeclareLaunchArgument("yolo_device", default_value=""),
        # Size of the crop sent to YOLO (smaller = tighter ROI, less background):
        #   roi_w = max(box_w * yolo_roi_scale, yolo_min_roi_width)
        #   roi_h = max(box_h * yolo_roi_scale, yolo_min_roi_height)
        DeclareLaunchArgument("yolo_roi_scale", default_value="5.0"),
        DeclareLaunchArgument("yolo_min_roi_width", default_value="160.0"),
        DeclareLaunchArgument("yolo_min_roi_height", default_value="180.0"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        DeclareLaunchArgument("expected_odom_frame_id", default_value="world"),
        DeclareLaunchArgument("expected_odom_child_frame_id", default_value="body"),
        # VINS world -> CARLA map registration, measured by register_vins_to_map.py
        # for the Town10HD spawn 100,-10,yaw90 + stereo VINS. Re-run the calibrator
        # and override these if the spawn point or the VINS init changes.
        DeclareLaunchArgument("odom_map_x_offset", default_value="99.5"),
        DeclareLaunchArgument("odom_map_y_offset", default_value="12.076"),
        DeclareLaunchArgument("odom_map_z_offset", default_value="-0.35"),
        DeclareLaunchArgument("odom_map_yaw_offset_deg", default_value="-88.992"),
        DeclareLaunchArgument("odom_child_to_ego_x", default_value="0.0"),
        DeclareLaunchArgument("odom_child_to_ego_y", default_value="0.0"),
        DeclareLaunchArgument("odom_child_to_ego_z", default_value="0.0"),
        DeclareLaunchArgument("odom_child_to_ego_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("odom_child_to_ego_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("odom_child_to_ego_yaw_deg", default_value="0.0"),
        # Camera extrinsic fine-tune (pixel-level nudge of the projected box):
        #   +yaw  -> box moves RIGHT in image; +pitch -> box moves UP.
        DeclareLaunchArgument("camera_roll_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch_offset_deg", default_value="-1.0"),
        DeclareLaunchArgument("camera_yaw_offset_deg", default_value="1.0"),
        DeclareLaunchArgument("camera_x_offset", default_value="0.0"),
        DeclareLaunchArgument("camera_y_offset", default_value="0.0"),
        DeclareLaunchArgument("camera_z_offset", default_value="0.0"),
        # Brightness-position state classifier gates + temporal smoothing.
        DeclareLaunchArgument("min_state_bbox_height_px", default_value="13.5"),
        DeclareLaunchArgument("max_state_distance_m", default_value="40.0"),
        DeclareLaunchArgument("state_score_ratio", default_value="1.15"),
        DeclareLaunchArgument("state_x_band_lo", default_value="0.3"),
        DeclareLaunchArgument("state_x_band_hi", default_value="0.7"),
        # Min saturation for a pixel to count as the LIT lamp (separates the lit
        # coloured lamp from grey unlit lamps + pale sky). Lower it if a clearly-on
        # light reads as too_dark/no_lamp (diag.lit_px ~ 0 in /traffic_light/status).
        DeclareLaunchArgument("state_sat_min", default_value="20.0"),
        DeclareLaunchArgument("state_history_size", default_value="7"),
        DeclareLaunchArgument("state_confirm_frames", default_value="3"),
        DeclareLaunchArgument("unknown_hold_seconds", default_value="0.9"),
        # Merge red+yellow -> one "caution" class that confirms to STOP (robust to
        # red<->yellow flicker; only green frees the car). false -> red=stop/yellow=slow.
        DeclareLaunchArgument("caution_stop", default_value="true"),
        # Stop obeying a light once the ego heading diverges from the signal heading
        # by more than this (deg) -> we have turned off the lane it controls. Stateless
        # per frame (cannot get stuck). 0 disables.
        DeclareLaunchArgument("ignore_turn_heading_deg", default_value="40.0"),
        # Stop obeying a light once its head is abeam/overhead/behind: angle from the
        # ego forward to the light head exceeds this (deg). Handles the STRAIGHT case
        # (drove just past the light). Stateless per frame. 0 disables.
        DeclareLaunchArgument("ignore_abeam_angle_deg", default_value="85.0"),
        # Stop obeying a light once within this distance of its head (overhead/too
        # late). 0 disables. Keep below the stop-line distance or it releases red early.
        DeclareLaunchArgument("ignore_near_distance_m", default_value="5.0"),
    ]

    node = Node(
        package="tf_detect",
        executable="traffic_light_state_node.py",
        name="traffic_light_state_node",
        output="screen",
        parameters=[
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "route_turn": LaunchConfiguration("route_turn"),
                "map_path": LaunchConfiguration("map_path"),
                "objects_path": LaunchConfiguration("objects_path"),
                "traffic_light_boxes_path": LaunchConfiguration("traffic_light_boxes_path"),
                "yolo_model": LaunchConfiguration("yolo_model"),
                "yolo_device": LaunchConfiguration("yolo_device"),
                "yolo_roi_scale": LaunchConfiguration("yolo_roi_scale"),
                "yolo_min_roi_width": LaunchConfiguration("yolo_min_roi_width"),
                "yolo_min_roi_height": LaunchConfiguration("yolo_min_roi_height"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "expected_odom_frame_id": LaunchConfiguration("expected_odom_frame_id"),
                "expected_odom_child_frame_id": LaunchConfiguration(
                    "expected_odom_child_frame_id"
                ),
                "odom_map_x_offset": LaunchConfiguration("odom_map_x_offset"),
                "odom_map_y_offset": LaunchConfiguration("odom_map_y_offset"),
                "odom_map_z_offset": LaunchConfiguration("odom_map_z_offset"),
                "odom_map_yaw_offset_deg": LaunchConfiguration("odom_map_yaw_offset_deg"),
                "odom_child_to_ego_x": LaunchConfiguration("odom_child_to_ego_x"),
                "odom_child_to_ego_y": LaunchConfiguration("odom_child_to_ego_y"),
                "odom_child_to_ego_z": LaunchConfiguration("odom_child_to_ego_z"),
                "odom_child_to_ego_roll_deg": LaunchConfiguration(
                    "odom_child_to_ego_roll_deg"
                ),
                "odom_child_to_ego_pitch_deg": LaunchConfiguration(
                    "odom_child_to_ego_pitch_deg"
                ),
                "odom_child_to_ego_yaw_deg": LaunchConfiguration(
                    "odom_child_to_ego_yaw_deg"
                ),
                "camera_roll_offset_deg": LaunchConfiguration("camera_roll_offset_deg"),
                "camera_pitch_offset_deg": LaunchConfiguration("camera_pitch_offset_deg"),
                "camera_yaw_offset_deg": LaunchConfiguration("camera_yaw_offset_deg"),
                "camera_x_offset": LaunchConfiguration("camera_x_offset"),
                "camera_y_offset": LaunchConfiguration("camera_y_offset"),
                "camera_z_offset": LaunchConfiguration("camera_z_offset"),
                "min_state_bbox_height_px": LaunchConfiguration("min_state_bbox_height_px"),
                "max_state_distance_m": LaunchConfiguration("max_state_distance_m"),
                "state_score_ratio": LaunchConfiguration("state_score_ratio"),
                "state_x_band_lo": LaunchConfiguration("state_x_band_lo"),
                "state_x_band_hi": LaunchConfiguration("state_x_band_hi"),
                "state_sat_min": LaunchConfiguration("state_sat_min"),
                "state_history_size": LaunchConfiguration("state_history_size"),
                "state_confirm_frames": LaunchConfiguration("state_confirm_frames"),
                "unknown_hold_seconds": LaunchConfiguration("unknown_hold_seconds"),
                "caution_stop": LaunchConfiguration("caution_stop"),
                "ignore_turn_heading_deg": LaunchConfiguration("ignore_turn_heading_deg"),
                "ignore_abeam_angle_deg": LaunchConfiguration("ignore_abeam_angle_deg"),
                "ignore_near_distance_m": LaunchConfiguration("ignore_near_distance_m"),
                "candidate_mode": "route_turn",
                "path_physical_signal_mode": "same-heading",
                "image_horizontal_sign": "flip",
                "light_box_selection": "turn-index",
                "unknown_action": "slow",
                "no_light_action": "go",
            }
        ],
    )

    return LaunchDescription([*launch_arguments, node])
