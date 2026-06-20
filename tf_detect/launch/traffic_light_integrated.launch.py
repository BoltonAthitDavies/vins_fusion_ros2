from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Traffic-light state node + route_turn bridge in one launch.

    Bring up CARLA, the VINS controller (carla_vins_multi_node ... --trajectory
    --mpc-state /vins_stereo_vel/odometry) and play_gt_path.py SEPARATELY, then
    run this launch. The route_turn bridge derives Right/Left/Straight from the
    MPC target so you do NOT pass route_turn:= by hand.

    Set auto_route_turn:=false to drive route_turn manually (then the
    traffic_light node's route_turn argument is used as-is).
    """
    launch_dir = Path(__file__).resolve().parent

    odom_topic = LaunchConfiguration("odom_topic")
    target_topic = LaunchConfiguration("target_topic")

    # These are forwarded verbatim into traffic_light_state.launch.py so the
    # frame-registration offsets from register_vins_to_map.py actually take
    # effect (a plain LaunchConfiguration here would otherwise be dropped).
    # Defaults mirror traffic_light_state.launch.py (the calibrated Town10 values).
    # Anything not listed here keeps that launch's own default.
    PASSTHROUGH = [
        ("yolo_model", str(launch_dir.parents[1] / "yolo11s.pt")),
        ("odom_map_x_offset", "99.5"),
        ("odom_map_y_offset", "12.076"),
        ("odom_map_z_offset", "-0.45"),
        ("odom_map_yaw_offset_deg", "-88.992"),
        ("camera_pitch_offset_deg", "-1.0"),
        ("camera_yaw_offset_deg", "1.0"),
    ]

    args = [
        DeclareLaunchArgument("auto_route_turn", default_value="true"),
        DeclareLaunchArgument("route_turn", default_value="auto"),
        DeclareLaunchArgument("odom_topic", default_value="/vins_stereo_vel/odometry"),
        DeclareLaunchArgument(
            "target_topic", default_value="/carla/ego_vehicle/trajectory_cmd"
        ),
        *[DeclareLaunchArgument(name, default_value=default) for name, default in PASSTHROUGH],
    ]

    traffic_light = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "traffic_light_state.launch.py")),
        launch_arguments={
            "route_turn": LaunchConfiguration("route_turn"),
            "odom_topic": odom_topic,
            **{name: LaunchConfiguration(name) for name, _ in PASSTHROUGH},
        }.items(),
    )

    route_turn_bridge = Node(
        package="tf_detect",
        executable="route_turn_from_path.py",
        name="route_turn_from_path",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_route_turn")),
        parameters=[
            {
                "odom_topic": odom_topic,
                "target_topic": target_topic,
            }
        ],
    )

    return LaunchDescription([*args, traffic_light, route_turn_bridge])
