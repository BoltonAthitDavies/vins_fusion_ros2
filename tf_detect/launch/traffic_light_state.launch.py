from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    src_dir = Path(os.environ.get("PROJECT_MOBILE_SRC", "/home/khunanon/Project_Mobile/src"))

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
            default_value="/carla/ego_vehicle/odometry",
        ),
        DeclareLaunchArgument("route_turn", default_value="auto"),
        DeclareLaunchArgument("map_path", default_value=str(src_dir / "Town10HD.xodr")),
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
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
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
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
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
