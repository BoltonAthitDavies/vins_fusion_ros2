# CARLA STEREO + IMU + GPS (full VINS-Fusion+GPS). The project-goal architecture; on CARLA the VIO part still diverges (loose GPS coupling cannot fix an exploding VIO).
# Run: ros2 launch vins_fusion_ros2 carla_stereo_imu_gps.launch.py ; ros2 bag play <bag> --clock
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# vins links the CUDA OpenCV 4.10 in ~/local; prepend it to the loader path.
LOCAL_OPENCV_LIB = '/home/fibo3/local/lib'


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vins_fusion_ros2'),
                       'config', 'carla', 'carla_stereo_imu.yaml')
    ld_path = LOCAL_OPENCV_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    nodes = [
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        Node(package='vins_fusion_ros2', executable='vins_fusion_ros2_node',
             name='vins_fusion_ros2_node', output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True}, {'config_file': cfg}]),
    ]
    nodes.append(
        Node(package='global_fusion', executable='global_fusion_node',
             name='global_fusion_node', output='screen',
             parameters=[{'use_sim_time': True}],
             remappings=[('/vins_estimator/odometry', '/odometry'),
                         ('/gps', '/carla/ego_vehicle/gnss')]))
    return LaunchDescription(nodes)
