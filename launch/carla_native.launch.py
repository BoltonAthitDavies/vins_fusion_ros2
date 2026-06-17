# Native LibCarla VINS driver. Requires a running CARLA server (default localhost:2000).
# Connects directly to the sim (no Python carla_ros_bridge, no rosbag), spawns ego +
# stereo + IMU + GNSS, feeds VINS live, and publishes /vins_estimator/odometry + /path.
# Run:  (start CARLA first)  ros2 launch vins_fusion_ros2 carla_native.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# carla_vins_node links the CUDA OpenCV 4.10 in ~/local (via vins_lib).
LOCAL_OPENCV_LIB = '/home/fibo3/local/lib'


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('vins_fusion_ros2'),
        'config', 'carla', 'carla_native.yaml')
    ld_path = LOCAL_OPENCV_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    town = LaunchConfiguration('town')
    spawn = LaunchConfiguration('spawn')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town10HD'),
        DeclareLaunchArgument('spawn', default_value='100.0,10.0,1.0,0.0,0.0,-90.0'),
        DeclareLaunchArgument('config', default_value=default_cfg),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        Node(
            package='vins_fusion_ros2',
            executable='carla_vins_node',
            name='vins_estimator',
            output='screen',
            emulate_tty=True,
            # positional: <config> <host> <port>; flags: --town --spawn --autopilot.
            # Autopilot is forced on: under `ros2 launch` stdin is not a TTY so manual
            # keyboard control is unavailable.
            arguments=[config, host, port, '--town', town, '--spawn', spawn, '--autopilot'],
        ),
    ])
