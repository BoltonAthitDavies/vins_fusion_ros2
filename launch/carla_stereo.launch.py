# CARLA STEREO-ONLY (works on CARLA: bounded VO).
# Run: ros2 launch vins_fusion_ros2 carla_stereo.launch.py ; ros2 bag play <bag> --clock
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# vins links the CUDA OpenCV 4.10 in ~/local; prepend it to the loader path.
LOCAL_OPENCV_LIB = '/home/fibo3/local/lib'


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vins_fusion_ros2'),
                       'config', 'carla', 'carla_stereo.yaml')
    ld_path = LOCAL_OPENCV_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    nodes = [
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        Node(package='vins_fusion_ros2', executable='vins_fusion_ros2_node',
             name='vins_fusion_ros2_node', output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True}, {'config_file': cfg}]),
    ]

    return LaunchDescription(nodes)
