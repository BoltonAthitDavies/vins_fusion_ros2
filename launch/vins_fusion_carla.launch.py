from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Tuned + GPU config (skip=1, show_track off, inflated IMU, use_gpu=1). The GPU paths need
# the CUDA-enabled OpenCV 4.10 installed in ~/local, so prepend it to the loader path.
LOCAL_OPENCV_LIB = '/home/fibo3/local/lib'


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vins_fusion_ros2'),
        'config',
        'carla',
        'carla_stereo_imu_config.yaml'
    )

    ld_path = LOCAL_OPENCV_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        Node(
            package='vins_fusion_ros2',
            executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'config_file': config_file}],
        )
    ])
