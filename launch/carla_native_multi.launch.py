# Multi-variant live VINS on CARLA: ONE node runs mono+imu / stereo / stereo+imu
# + 2 in-process global_fusion, all off one live drive. Publishes:
#   /vins_mono/odometry  /vins_stereo/odometry  /vins_stereo_imu/odometry
#   /vins_stereo_gps/odometry  /vins_stereo_imu_gps/odometry
# Pair with: python3 src/plot_result_rtab_cpp.py  (adds GT from CARLA on_tick).
# Requires a running CARLA server. Autopilot forced (launch has no TTY).
#
# use_rtab:=true also runs RTAB-Map live (apt rtabmap_ros), fed by the live stereo
# + camera_info + a VINS odom (rtab_odom, default /vins_stereo/odometry = stereo VINS)
# + GNSS. A republisher applies RTAB's loop-closure correction to that VINS odom and
# publishes /rtabmap/corrected_odom (control-rate Odometry) -- use it as the MPC state:
#   ros2 launch ... carla_native_multi.launch.py drive:=trajectory \
#       mpc_state:=/rtabmap/corrected_odom bootstrap_secs:=2.0
# (and point play_gt_path.py --state at /rtabmap/corrected_odom --anchor-to-state).
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            GroupAction, IncludeLaunchDescription, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

LOCAL_OPENCV_LIB = '/home/fibo3/local/lib'
# Clean library path for the apt rtabmap_ros node: its own ROS libs + system
# OpenCV 4.5.4 only — NO ~/local (our rtabmap 0.23 + CUDA OpenCV would ABI-clash).
RTAB_CLEAN_LD = '/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'


def generate_launch_description():
    cfg_dir = os.path.join(
        get_package_share_directory('vins_fusion_ros2'), 'config', 'carla')
    ld_path = LOCAL_OPENCV_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    town = LaunchConfiguration('town')
    spawn = LaunchConfiguration('spawn')
    drive = LaunchConfiguration('drive')
    mpc_state = LaunchConfiguration('mpc_state')
    wheel_noise = LaunchConfiguration('wheel_noise')
    noise_odom = LaunchConfiguration('noise_odom')
    bootstrap_secs = LaunchConfiguration('bootstrap_secs')
    target_speed = LaunchConfiguration('target_speed')
    horizon = LaunchConfiguration('horizon')
    variants = LaunchConfiguration('variants')
    use_rtab = LaunchConfiguration('use_rtab')
    rtab_odom = LaunchConfiguration('rtab_odom')
    rtab_db = LaunchConfiguration('rtab_db')
    rtab_viz = LaunchConfiguration('rtab_viz')

    # RTAB-Map live (apt rtabmap_ros) fed by the live VINS odom + stereo + GNSS.
    try:
        rtabmap_launch_file = os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
    except Exception:
        rtabmap_launch_file = ''

    rtab_params = [
        '--delete_db_on_start',
        '--Rtabmap/DetectionRate', '1',
        '--Rtabmap/LoopGPS', 'true',
        '--RGBD/LoopClosureReextractFeatures', 'true',
        '--Optimizer/Robust', 'true',
        '--Reg/Force3DoF', 'false',
        '--Vis/MinInliers', '20',
        '--Kp/MaxFeatures', '250',
        '--Vis/MaxDepth', '20.0',
        '--Stereo/MinDisparity', '2.0',
    ]

    rtab_group = GroupAction(
        condition=IfCondition(use_rtab),
        scoped=True,
        actions=[
            # Isolate the apt rtabmap libs from ~/local (ABI clash).
            SetEnvironmentVariable('LD_LIBRARY_PATH', RTAB_CLEAN_LD),
            # Connect the VINS odom child frame (body) to the camera frames
            # (ego_vehicle/*): world(odom) -> body -> ego_vehicle -> cam_*.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='body_to_ego', output='log',
                 arguments=['0', '0', '0', '0', '0', '0', 'body', 'ego_vehicle']),
            # Bridge the VINS odom (rtab_odom, world->body) onto /tf. Without this
            # rtabmap can't connect 'world' to 'body' (the pose is only a topic,
            # not TF) and aborts every frame -> empty map. Completes the tree:
            # map(rtab) -> world -> body -> ego_vehicle -> cam_*.
            ExecuteProcess(
                cmd=['python3', os.path.expanduser('~/ros2_ws/odom_to_tf.py'),
                     '--odom', rtab_odom],
                output='log'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                launch_arguments={
                    'rtabmap_args':            ' '.join(rtab_params),
                    'database_path':           rtab_db,
                    'frame_id':                'body',
                    'odom_frame_id':           'world',
                    'visual_odometry':         'false',
                    'stereo':                  'true',
                    'left_image_topic':        '/carla/ego_vehicle/cam_front_left/image',
                    'right_image_topic':       '/carla/ego_vehicle/cam_front_right/image',
                    'left_camera_info_topic':  '/carla/ego_vehicle/cam_front_left/camera_info',
                    'right_camera_info_topic': '/carla/ego_vehicle/cam_front_right/camera_info',
                    'odom_topic':              rtab_odom,
                    'gps_topic':               '/carla/ego_vehicle/gnss',
                    'approx_sync':             'true',
                    'approx_sync_max_interval':'0.1',
                    # Bigger queues + more TF slack: the world->body TF (from the
                    # VINS odom) lags the image stream, so RTAB needs to hold
                    # images longer to match a TF at their stamp (rtabmap's own
                    # suggestion for the "did not receive / extrapolation" warning).
                    'topic_queue_size':        '30',
                    'sync_queue_size':         '30',
                    'wait_for_transform':      '0.5',
                    'qos':                     '1',
                    'subscribe_scan':          'false',
                    'subscribe_scan_cloud':    'false',
                    'use_sim_time':            'true',
                    'rtabmap_viz':             rtab_viz,
                }.items(),
            ),
            # Apply RTAB's loop-closure correction (map<-world TF) to the stereo-VINS
            # odom -> /rtabmap/corrected_odom (control-rate). Use it as mpc_state.
            ExecuteProcess(
                cmd=['python3', os.path.expanduser('~/ros2_ws/rtab_corrected_odom.py'),
                     '--in', rtab_odom, '--out', '/rtabmap/corrected_odom',
                     '--map-frame', 'map'],
                output='screen'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town10HD'),
        DeclareLaunchArgument('spawn', default_value='100.0,10.0,1.0,0.0,0.0,-90.0'),
        # coverage = C-mode right-loop route (default); autopilot = plain Traffic Manager;
        # trajectory = T-mode MPC following /carla/ego_vehicle/trajectory_cmd
        DeclareLaunchArgument('drive', default_value='coverage'),
        # T-mode MPC state source; empty = CARLA ground truth. Set to the topic
        # play_gt_path.py uses for --state (e.g. /vins_stereo_imu_gps/odometry).
        DeclareLaunchArgument('mpc_state', default_value=''),
        # Synthesised wheel odometry on /carla/ego_vehicle/wheel_odometry:
        # -1 = off (default); 0 = clean; >0 = realistic drift. Use as an MPC
        # bootstrap state (mpc_state:=/carla/ego_vehicle/wheel_odometry).
        DeclareLaunchArgument('wheel_noise', default_value='-1'),
        # Noise odometry = GT pose + bounded noise on /carla/ego_vehicle/noise_odometry.
        # -1 = off; 0 = exact GT; >0 = noisier. Auto-enabled (level 0.5) when
        # bootstrap_secs > 0. Used as the cheap t=0 bootstrap state for T-mode.
        DeclareLaunchArgument('noise_odom', default_value='-1'),
        # T-mode bootstrap: drive on noise_odom for >= this many seconds, then
        # hand over to mpc_state once it is live. 0 = no bootstrap (off).
        DeclareLaunchArgument('bootstrap_secs', default_value='0'),
        # MPC cruise speed (m/s). -1 = keep the built-in default (3.5). Raise to
        # match a faster reference trajectory (e.g. ~5.5 for autopilot-recorded GT).
        DeclareLaunchArgument('target_speed', default_value='-1'),
        # MPC prediction horizon in steps (× 0.10 s). -1 = built-in default (100 =
        # 10 s). Keep it ~ lookahead/target_speed (e.g. ~12-15 for a 6 m lookahead)
        # so the rollout doesn't overshoot a close target and crawl.
        DeclareLaunchArgument('horizon', default_value='-1'),
        # Which VINS estimators to run live: 'all' (default), 'none' (GT-only
        # control), or a CSV of variant names (e.g. 'stereo_gps'). Fewer = less
        # CPU so the real-time path keeps up. Empty + an mpc_state /vins_* topic
        # auto-selects just that estimator.
        DeclareLaunchArgument('variants', default_value='auto'),
        # Run RTAB-Map live (apt rtabmap_ros) -> /rtabmap/odom + /rtabmap/mapPath.
        DeclareLaunchArgument('use_rtab', default_value='true'),
        # Which VINS odom feeds RTAB (its base odom, then loop-closure-corrected).
        # Default = stereo VINS; the corrected result is on /rtabmap/corrected_odom.
        DeclareLaunchArgument('rtab_odom', default_value='/vins_stereo/odometry'),
        DeclareLaunchArgument('rtab_db', default_value=os.path.expanduser('~/rtab_carla_live.db')),
        # Show the live RTAB-Map GUI (3D map, camera, loop-closure graph) while running.
        DeclareLaunchArgument('rtab_viz', default_value='false'),
        # Gate the T-mode MPC on /traffic_light/action (stop->brake, slow->cap
        # speed, go/stale->cruise). 'false' (default) = perception never touches
        # control. tl_slow_speed = m/s used when the light says "slow".
        DeclareLaunchArgument('traffic_light', default_value='false'),
        DeclareLaunchArgument('tl_slow_speed', default_value='2.0'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', ld_path),
        Node(
            package='vins_fusion_ros2',
            executable='carla_vins_multi_node',
            name='vins_estimator',
            output='screen',
            emulate_tty=True,
            arguments=[cfg_dir, host, port, '--town', town, '--spawn', spawn,
                       ['--', drive], '--mpc-state', mpc_state,
                       '--wheel-noise', wheel_noise,
                       '--noise-odom', noise_odom,
                       '--bootstrap-secs', bootstrap_secs,
                       '--target-speed', target_speed,
                       '--horizon', horizon,
                       '--variants', variants,
                       '--traffic-light', LaunchConfiguration('traffic_light'),
                       '--tl-slow-speed', LaunchConfiguration('tl_slow_speed')],
        ),
        rtab_group,
    ])
