#!/usr/bin/env python3
# HH_260528: Unified IMU launch — selects driver based on imu_model parameter.
#
# Models:
#   cv7  — MicroStrain CV7-AHRS (IMU only, direct node launch with respawn)
#   gq7  — MicroStrain GQ7 (GNSS/INS, upstream microstrain_launch.py + optional NTRIP)
#
# Model selection: set imu_model in camrod_bringup/config/bringup/launch_defaults.yaml
#   sensing:
#     imu_model: cv7       # or gq7
#     imu_param_file: __module_default__   # auto-resolves to config/imu/microstrain_<model>.yaml
#
# Standalone usage:
#   ros2 launch camrod_sensing imu.launch.py imu_model:=cv7
#   ros2 launch camrod_sensing imu.launch.py imu_model:=gq7 use_ntrip:=true

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap

_sensing_share = get_package_share_directory('camrod_sensing')
_microstrain_share = get_package_share_directory('microstrain_inertial_driver')
_microstrain_default_params_file = os.path.join(
    _microstrain_share, 'microstrain_inertial_driver_common', 'config', 'params.yml')


_PARAM_FILE_SENTINELS = {'__model_default__', '__module_default__', 'module_default', 'default', ''}


def _resolve_imu_param_file(context, *args, **kwargs):
    """Resolve imu_param_file from imu_model when value is a sentinel."""
    model = context.perform_substitution(LaunchConfiguration('imu_model'))
    raw   = context.perform_substitution(LaunchConfiguration('imu_param_file')).strip()
    if raw in _PARAM_FILE_SENTINELS:
        resolved = os.path.join(_sensing_share, 'config', 'imu', f'microstrain_{model}.yaml')
    else:
        resolved = raw
    return [SetLaunchConfiguration('_imu_param_file_resolved', resolved)]


def generate_launch_description():
    velocity_converter_launch = os.path.join(
        _sensing_share, 'launch', 'platform_velocity_converter.launch.py')

    default_ntrip_params = os.path.join(_sensing_share, 'config', 'gnss', 'ntrip_client.yaml')
    default_converter_params = os.path.join(
        _sensing_share, 'config', 'imu', 'platform_velocity_converter.yaml')

    # Load MicroStrain upstream defaults at launch-description time (static file).
    microstrain_default_params = yaml.safe_load(
        open(_microstrain_default_params_file, 'r', encoding='utf-8'))

    imu_model        = LaunchConfiguration('imu_model')
    enable_imu       = LaunchConfiguration('enable_imu')
    use_ntrip        = LaunchConfiguration('use_ntrip')
    module_namespace = LaunchConfiguration('module_namespace')

    return LaunchDescription([
        DeclareLaunchArgument('enable_imu',      default_value='true'),
        DeclareLaunchArgument('imu_model',        default_value='cv7',
                              description='IMU model: cv7 | gq7'),
        DeclareLaunchArgument('imu_param_file',   default_value='__model_default__',
                              description='Param YAML path, or __model_default__ to auto-resolve from imu_model'),
        DeclareLaunchArgument('use_ntrip',        default_value='true',
                              description='(gq7 only) Launch NTRIP client for RTK corrections'),
        DeclareLaunchArgument('ntrip_param_file', default_value=default_ntrip_params),
        DeclareLaunchArgument('module_namespace', default_value='imu'),

        DeclareLaunchArgument('velocity_converter_param_file', default_value=default_converter_params),
        DeclareLaunchArgument('velocity_topic',   default_value='/platform/status/velocity'),
        DeclareLaunchArgument('imu_topic',        default_value='data_ros'),
        DeclareLaunchArgument('imu_data_output_topic', default_value='data'),
        DeclareLaunchArgument('output_topic',     default_value='twist_with_covariance'),
        DeclareLaunchArgument('imu_status_topic', default_value='status'),

        OpaqueFunction(function=_resolve_imu_param_file),

        GroupAction([
            PushRosNamespace(module_namespace),
            # HH_260720 - Both CV7 and GQ7 drivers publish standard ROS IMU on data_ros.
            SetRemap(src='imu/data', dst='data_ros'),

            # ── CV7 model (direct node launch, respawn for serial lock recovery) ───
            GroupAction([
                SetRemap(src='/ekf/status', dst='ekf/status'),
                Node(
                    package='microstrain_inertial_driver',
                    executable='microstrain_inertial_driver_node',
                    name='microstrain_inertial_driver',
                    output='screen',
                    parameters=[
                        microstrain_default_params,
                        LaunchConfiguration('_imu_param_file_resolved'),
                    ],
                    respawn=True,
                    respawn_delay=2.0,
                ),
            ], condition=IfCondition(PythonExpression(["'", imu_model, "' == 'cv7'"]))),

            # ── GQ7 model (upstream microstrain_launch.py + optional NTRIP) ────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(_microstrain_share, 'launch', 'microstrain_launch.py')),
                launch_arguments={
                    'params_file': LaunchConfiguration('_imu_param_file_resolved'),
                }.items(),
                condition=IfCondition(PythonExpression(["'", imu_model, "' == 'gq7'"])),
            ),
            Node(
                package='ntrip_client',
                executable='ntrip_ros.py',
                name='ntrip_client',
                output='screen',
                parameters=[
                    LaunchConfiguration('ntrip_param_file'),
                    {'rtcm_message_package': 'rtcm_msgs'},
                    {'rtcm_topic': '/rtcm'},
                ],
                remappings=[('fix', '/gnss_1/llh_position')],
                condition=IfCondition(PythonExpression([
                    "'", imu_model, "' == 'gq7' and '", use_ntrip, "' == 'true'",
                ])),
            ),

            # ── Velocity converter (both models) ────────────────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(velocity_converter_launch),
                launch_arguments={
                    'params_file':        LaunchConfiguration('velocity_converter_param_file'),
                    'module_namespace':   '',
                    'velocity_topic':     LaunchConfiguration('velocity_topic'),
                    'imu_topic':          LaunchConfiguration('imu_topic'),
                    'imu_data_output_topic': LaunchConfiguration('imu_data_output_topic'),
                    'output_topic':       LaunchConfiguration('output_topic'),
                    'imu_status_topic':   LaunchConfiguration('imu_status_topic'),
                }.items(),
            ),
        ], condition=IfCondition(enable_imu)),
    ])
