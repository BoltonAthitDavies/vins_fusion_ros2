#!/usr/bin/env python3
"""VINS-Fusion + YOLO dynamic filtering, in one command.

    ros2 launch vins_fusion_ros2 wil_sim_vins_filter.launch.py \
        output_path:=/home/ambushee/wil_project/output/output_vins_yolo/simulation/<bag>/<run-id> \
        experiment_id:=<run-id> \
        dataset_path:=/media/ambushee/32E4AAB1E4AA772F/dataset/<bag>

    # then, in one other terminal:
    ros2 bag play <bag> --rate 1.0

WHY THIS FILE EXISTS

    A VINS filter run needed FOUR terminals started in the right order, and
    forgetting the detector is silent -- the node subscribes, nothing publishes,
    and every frame records matched=0. That cost four Experiment 3 runs on
    2026-09-16 and several more on 2026-10-15/16.

CROSS-PACKAGE REFERENCE, DELIBERATE

    This package is a git submodule, and the detector node it starts
    (`dynamic_detector_node.py`) lives in the workspace sibling
    `orbslam3_ros2`. That is why the package shipped no WiL launch file for so
    long, and why this one lived in `script/` and was launched by absolute path
    until 2026-10-16.

    It is referenced by PACKAGE NAME, resolved by the ament index at launch
    time, and is NOT declared in package.xml -- so `colcon build` of this
    submodule alone still succeeds, and only `filter:=true` needs
    `orbslam3_ros2` present. With `filter:=false` this file has no cross-package
    reference at all. If the submodule is ever used outside this workspace, this
    is the one file to delete.

WHAT IT STARTS

    1. `dynamic_detector_node.py` from orbslam3_ros2, when filter:=true;
    2. the VINS estimator.

    Both read /camN/image_raw/compressed directly, so NO republish helpers are
    started by default -- VINS decodes the JPEG in-process since 2026-10-16.
    `republish:=true` brings the old pair back for a raw-only source.

    Starting them together also fixes the ordering: everything is up before you
    play the bag, instead of the detector joining halfway through.

DEPLOYMENT, NOT DETERMINISM

    The detector runs concurrently with the estimator and competes for the
    machine, which is the point -- Experiment 3 measures the pipeline as
    deployed. Mask availability therefore varies with load (measured 1-100 %
    across repeats on this laptop). ALWAYS read `detection_match_rate` and
    `filter_status` out of filter_summary.csv next to any result: a low match
    rate means the filter did not RUN, not that filtering did not help.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORKSPACE = os.path.expanduser('~/wil_project')
DEFAULT_CONFIG = os.path.join(
    WORKSPACE, 'vins_fusion_ros2', 'config', 'wil_sim', 'stereo_imu.yaml')
DEFAULT_WEIGHTS = os.path.join(WORKSPACE, 'weight', 'best.pt')

# Base topics, without any /compressed suffix. The republishers below turn the
# bag's compressed topics into these.
CAM0 = '/cam0/image_raw'
CAM1 = '/cam1/image_raw'


def _str(name):
    """Free text as a STRING parameter.

    launch_ros YAML-parses every string parameter, so a run_notes containing a
    colon-space becomes a dict and aborts the launch before anything starts.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'output_path',
            description='Run directory. Give every run its own -- the node '
                        'overwrites in place, and a reused directory ends up '
                        'holding a mixture of two runs.'),
        DeclareLaunchArgument('experiment_id', default_value=''),
        DeclareLaunchArgument('dataset_path', default_value=''),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('run_notes', default_value=''),
        DeclareLaunchArgument('run_command', default_value=''),
        DeclareLaunchArgument(
            'replay_rate', default_value='1.0',
            description='METADATA ONLY -- it records what you intend to pass to '
                        '`ros2 bag play --rate`, nothing enforces it. Verify '
                        'afterwards from the IMU arrival rate in '
                        'performance.csv; a mislogged value survived twelve '
                        'September manifests undetected.'),
        DeclareLaunchArgument('config_file', default_value=DEFAULT_CONFIG),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument(
            'filter', default_value='false',
            description='YOLO dynamic-object filtering. When true the detector '
                        'node is started here too. false = stock VINS.'),
        DeclareLaunchArgument(
            'filter_mode', default_value='class',
            description="'class' masks every detected box (RY-SLAM). 'motion' "
                        'masks only boxes seen to move -- a documented NEGATIVE '
                        'result for VINS, see its record section 11.'),
        DeclareLaunchArgument('weights_path', default_value=DEFAULT_WEIGHTS),
        DeclareLaunchArgument(
            'dynamic_classes', default_value="['auto']",
            description="'auto' masks every class the chosen weight knows."),
        DeclareLaunchArgument('publish_debug_image', default_value='false'),
        DeclareLaunchArgument(
            'image_transport', default_value='compressed',
            description="'compressed' (default) -- VINS decodes the JPEG "
                        'itself, so no republish helpers are needed at all. '
                        "'raw' for a source that already publishes Image."),
        DeclareLaunchArgument(
            'republish', default_value='false',
            description='Start two compressed->raw republishers. Only needed '
                        'with image_transport:=raw against a compressed-only '
                        'bag; the republish path loses and duplicates frames, '
                        'so prefer image_transport:=compressed.'),
    ]

    republishers = [
        Node(
            package='image_transport', executable='republish',
            name=f'republish_cam{i}', output='log',
            condition=IfCondition(LaunchConfiguration('republish')),
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', f'{cam}/compressed'), ('out', cam)],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
        for i, cam in enumerate((CAM0, CAM1))
    ]

    # Only when filtering: loading ultralytics costs seconds and a GPU context.
    detector = Node(
        package='orbslam3_ros2', executable='dynamic_detector_node.py',
        name='dynamic_detector', output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('filter')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'weights_path': LaunchConfiguration('weights_path'),
            'dynamic_classes': LaunchConfiguration('dynamic_classes'),
            # Left camera only: VINS applies the mask to image0, and the right
            # points are tracked FROM the already-masked left set, so masking
            # image0 is sufficient and halves the inference cost.
            'image_topic': CAM0,
            'image_transport': LaunchConfiguration('image_transport'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    estimator = Node(
        package='vins_fusion_ros2', executable='vins_fusion_ros2_node',
        name='vins_estimator', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': LaunchConfiguration('config_file'),
            'output_path': _str('output_path'),
            'experiment_id': _str('experiment_id'),
            'dataset_path': _str('dataset_path'),
            'world_path': _str('world_path'),
            'run_notes': _str('run_notes'),
            'run_command': _str('run_command'),
            'replay_rate': LaunchConfiguration('replay_rate'),
            'filter': LaunchConfiguration('filter'),
            'filter_mode': LaunchConfiguration('filter_mode'),
            'image_transport': LaunchConfiguration('image_transport'),
        }],
    )

    return LaunchDescription(args + republishers + [detector, estimator])
