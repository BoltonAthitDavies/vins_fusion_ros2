"""ROS2 launch: VINS + RTAB-Map mapping from a CARLA rosbag2 recording.

Ported to the main vins_fusion_ros2 project:
  - package 'vins'            -> 'vins_fusion_ros2'
  - executable carla_raw_test -> carla_bag_test   (refactored-API bag reader)
  - carla_rtab_direct keeps its name (refactored-API VINS -> RTAB-Map)
  - carla_plot runs as a repo-root script via python3 (not an installed entry point)
  - default config -> config/carla/carla_native_stereo.yaml
  - default paths  -> /home/fibo3 (~)

NOTE: carla_bag_test and carla_rtab_direct are NOT yet registered in CMakeLists.
Add their add_executable() entries (and RTAB-Map deps for carla_rtab_direct)
before launching.

Two modes selected by  direct:=false|true
------------------------------------------
direct=false
  carla_bag_test  →  ROS topics  →  rtabmap_ros node
direct=true (default)
  carla_rtab_direct  (single C++ binary, no ROS between VINS and RTAB)

Common launch arguments
-----------------------
bag_path    : rosbag2 directory  (REQUIRED)
config_path : VINS config YAML   (default: carla_stereo_imu_config.yaml)
rtab_db     : RTAB-Map database  (default: ~/rtab_carla.db)
use_plotter : live trajectory plotter  (default: true)
use_rviz    : launch RViz  (default: false, only meaningful when direct=false)
output      : plotter .npz path  (default: ~/ros2_ws/vins_carla)
direct      : use carla_rtab_direct binary  (default: true)

Examples
--------
ros2 launch vins_fusion_ros2 carla_bag_rtab.launch.py bag_path:=~/ros2_ws/rosbag2_2026_06_07-11_29_12_town01_drivenormal
ros2 launch vins_fusion_ros2 carla_bag_rtab.launch.py bag_path:=~/ros2_ws/<bag> direct:=false
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

PACKAGE = 'vins_fusion_ros2'
CARLA_PLOT = os.path.expanduser('~/ros2_ws/carla_plot.py')


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    vins_share = get_package_share_directory(PACKAGE)

    bag_path        = os.path.expanduser(LaunchConfiguration('bag_path').perform(context))
    config_override = LaunchConfiguration('config_path').perform(context)
    use_rviz        = LaunchConfiguration('use_rviz').perform(context).strip().lower() in ('true', '1', 'yes')
    use_plotter     = LaunchConfiguration('use_plotter').perform(context).strip().lower() in ('true', '1', 'yes')
    output          = LaunchConfiguration('output').perform(context)
    rtab_db         = os.path.expanduser(LaunchConfiguration('rtab_db').perform(context))
    direct          = LaunchConfiguration('direct').perform(context).strip().lower() in ('true', '1', 'yes')

    if not bag_path:
        print('[carla_bag_rtab] ERROR: bag_path is required')
        return []
    if not os.path.isdir(bag_path):
        print(f'[carla_bag_rtab] ERROR: bag not found at {bag_path}')
        return []

    config_path = os.path.expanduser(config_override) if config_override else \
        os.path.join(vins_share, 'config', 'carla', 'carla_stereo_0.5m_config.yaml')

    if not output:
        output = os.path.expanduser('~/ros2_ws/vins_carla')

    mode = 'direct (no ROS between VINS→RTAB)' if direct else 'ROS bridge'
    print(f'[carla_bag_rtab] bag     = {bag_path}')
    print(f'[carla_bag_rtab] config  = {config_path}')
    print(f'[carla_bag_rtab] rtab_db = {rtab_db}')
    print(f'[carla_bag_rtab] output  = {output}.npz')
    print(f'[carla_bag_rtab] mode    = {mode}')

    vins_remappings = [
        ('imu_propagate',      '/vins_estimator/imu_propagate'),
        ('path',               '/vins_estimator/path'),
        ('odometry',           '/vins_estimator/odometry'),
        ('point_cloud',        '/vins_estimator/point_cloud'),
        ('margin_cloud',       '/vins_estimator/margin_cloud'),
        ('key_poses',          '/vins_estimator/key_poses'),
        ('camera_pose',        '/vins_estimator/camera_pose'),
        ('camera_pose_visual', '/vins_estimator/camera_pose_visual'),
        ('keyframe_pose',      '/vins_estimator/keyframe_pose'),
        ('keyframe_point',     '/vins_estimator/keyframe_point'),
        ('extrinsic',          '/vins_estimator/extrinsic'),
        ('image_track',        '/vins_estimator/image_track'),
    ]

    plotter = ExecuteProcess(
        cmd=['python3', CARLA_PLOT,
             '--gt-bag',   bag_path,
             '--gt-topic', '/carla/ego_vehicle/odometry',
             '--output',   output,
             '--title',    'CARLA-direct' if direct else 'CARLA+RTAB'],
        output='screen',
    )

    # —— direct mode: standalone binary, no ROS between VINS and RTAB ————————
    # carla_rtab_direct is built standalone (rtab_tool/) against the CPU-twin
    # OpenCV + GTSAM, not as a package executable, so run it via its wrapper
    # (which sets LD_LIBRARY_PATH for the ~/local + ~/local_ocv OpenCV split).
    if direct:
        rtab_bin = os.path.expanduser('~/ros2_ws/rtab_tool/run_carla_rtab_direct.sh')
        nodes = [
            ExecuteProcess(
                cmd=[rtab_bin, config_path, bag_path, rtab_db],
                output='screen',
            ),
        ]
        if use_plotter:
            nodes.append(plotter)
        return nodes

    # —— ROS bridge mode: carla_bag_test + rtabmap_ros ————————————————————
    rtab_params = [
        '--delete_db_on_start',
        '--Rtabmap/DetectionRate',             '0',
        '--Vis/MinInliers',                    '20',
        '--Mem/STMSize',                       '10',
        '--RGBD/LoopClosureReextractFeatures', 'true',
        '--Rtabmap/LoopRatio',                 '0.0',
        '--Rtabmap/LoopGPS',                   'true',
        '--RGBD/LocalRadius',                  '10',
        '--Optimizer/Strategy',                '2',
        '--RGBD/OptimizeMaxError',             '3.0',
        '--Optimizer/Robust',                  'true',
        '--Optimizer/GravitySigma',            '0',
        '--Optimizer/PriorsIgnored',           'false',
        '--Reg/Force3DoF',                     'false',
        '--Kp/MaxFeatures',                    '250',
        '--Kp/DetectorStrategy',               '8',
        '--Vis/FeatureType',                   '8',
        '--Vis/EpipolarGeometry',              '2',
        '--Vis/MaxDepth',                      '20.0',
        '--Kp/MaxDepth',                       '20.0',
        '--Stereo/MinDisparity',               '2.0',
        '--Grid/CellSize',                     '0.2',
        '--Grid/3D',                           'true',
        '--Grid/RayTracing',                   'true',
    ]

    try:
        rtabmap_dir = FindPackageShare('rtabmap_launch').find('rtabmap_launch')
        rtabmap_launch_file = os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
    except Exception:
        print('[carla_bag_rtab] ERROR: rtabmap_launch not found — install it first')
        print('  sudo apt install ros-humble-rtabmap-ros')
        return []

    nodes = [
        Node(
            package=PACKAGE,
            executable='carla_bag_test',
            name='vins_estimator',
            output='screen',
            arguments=[config_path, bag_path],
            remappings=vins_remappings,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_to_right_camera_tf',
            arguments=['0.5', '0', '0', '0', '0', '0', 'camera', 'right_camera'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'rtabmap_args':             ' '.join(rtab_params),
                'database_path':            rtab_db,
                'frame_id':                 'body',
                'odom_frame_id':            'world',
                'visual_odometry':          'false',
                'stereo':                   'true',
                'left_image_topic':         '/leftImage',
                'right_image_topic':        '/rightImage',
                'left_camera_info_topic':   '/leftCameraInfo',
                'right_camera_info_topic':  '/rightCameraInfo',
                'odom_topic':               '/vins_estimator/odometry',
                'gps_topic':                '/carla/ego_vehicle/gnss',
                'approx_sync':              'true',
                'approx_sync_max_interval': '0.1',
                'wait_for_transform':       '0.2',
                'qos':                      '1',
                'subscribe_scan':           'false',
                'subscribe_scan_cloud':     'false',
                'use_sim_time':             'false',
            }.items(),
        ),
    ]

    if use_plotter:
        nodes.append(plotter)

    if use_rviz:
        default_rviz = os.path.join(vins_share, 'config', 'vins_rviz_config.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='vins_rviz',
            output='log',
            arguments=['-d', default_rviz],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path', default_value='',
            description='Path to the rosbag2 directory (REQUIRED)'),
        DeclareLaunchArgument(
            'config_path', default_value='',
            description='Override VINS config YAML (empty = carla_stereo_0.5m_config.yaml)'),
        DeclareLaunchArgument(
            'direct', default_value='true',
            description='Use carla_rtab_direct (no ROS between VINS and RTAB)'),
        DeclareLaunchArgument(
            'use_plotter', default_value='true',
            description='Launch real-time trajectory plotter'),
        DeclareLaunchArgument(
            'output', default_value='',
            description='Plotter .npz output path (default: ~/ros2_ws/vins_carla)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz (ROS bridge mode only)'),
        DeclareLaunchArgument(
            'rtab_db', default_value='~/rtab_carla.db',
            description='RTAB-Map database path'),
        OpaqueFunction(function=launch_setup),
    ])
