from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vins_fusion_ros2'),
        'config',
        'euroc',
        'euroc_mono_imu_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_graph_save_path',
            default_value='',
            description='Override the config yaml\'s pose_graph_save_path. '
                        'Empty keeps the value from the config file.',
        ),
        Node(
            package='vins_fusion_ros2',
            executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'config_file': config_file},
                        {'pose_graph_save_path':
                            LaunchConfiguration('pose_graph_save_path')}],
        )
    ])

