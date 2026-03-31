#!/usr/bin/env python3
# HH_260326: integrated diagnostics launcher inside camrod_system package.

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


COMPONENTS_BASE = ['hw', 'sensing', 'localization', 'map', 'perception', 'planning']


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('camrod_system')
    pkg_prefix = get_package_prefix('camrod_system')
    config_profile = LaunchConfiguration('config_profile').perform(context)
    enable_checkers = _as_bool(LaunchConfiguration('enable_checkers').perform(context))
    enable_platform = _as_bool(LaunchConfiguration('enable_platform').perform(context))
    module_namespace = LaunchConfiguration('module_namespace').perform(context)

    config_root = os.path.join(pkg_share, 'config', 'diagnostics')
    profile_config = os.path.join(config_root, config_profile)
    default_config = os.path.join(config_root, 'default')
    config_dir = profile_config if os.path.isdir(profile_config) else default_config
    components_dir = os.path.join(pkg_share, 'launch', 'diagnostics', 'components')

    if not enable_checkers:
        return [LogInfo(msg='[camrod_system] HH_260330: diagnostics checkers disabled (enable_checkers:=false)')]

    system_nodes = [
        Node(
            package='camrod_system',
            executable='aggregator_node',
            name='diagnostics_agg',
            parameters=[{
                'config_file': os.path.join(config_dir, 'aggregator', 'diagnostics_config.yaml'),
            }],
            output='screen',
        ),
    ]

    components = list(COMPONENTS_BASE)
    ranger_checker_exec = os.path.join(
        pkg_prefix, 'lib', 'camrod_system', 'ranger_platform_checker_node')
    if enable_platform and os.path.exists(ranger_checker_exec):
        components.append('platform')
    elif enable_platform:
        system_nodes.append(
            LogInfo(
                msg=(
                    '[camrod_system] HH_260326: enable_platform:=true but '
                    'ranger_platform_checker_node is not installed. Skip platform checker.'
                )
            )
        )
    else:
        system_nodes.append(
            LogInfo(msg='[camrod_system] HH_260326: platform checker disabled (enable_platform:=false)')
        )

    for cat in components:
        system_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(components_dir, f'{cat}.launch.py')),
                launch_arguments={'config_dir': config_dir}.items(),
            )
        )
    # HH_260331: Expose diagnostics under /system/* for runtime observability.
    return [
        GroupAction(actions=[
            PushRosNamespace(module_namespace),
            SetRemap(src='/diagnostics', dst='diagnostics'),
            SetRemap(src='/diagnostics_agg', dst='diagnostics_agg'),
            *system_nodes,
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_profile',
            default_value='default',
            description='config/diagnostics/<profile> 선택 (없으면 default 사용)',
        ),
        DeclareLaunchArgument(
            'enable_checkers',
            default_value='true',
            description='진단 checker/aggregator 실행 여부',
        ),
        DeclareLaunchArgument(
            'enable_platform',
            default_value='false',
            description='ranger_platform_checker 실행 여부 (ranger_msgs 필요)',
        ),
        DeclareLaunchArgument(
            'module_namespace',
            default_value='system',
            description='Namespace for camrod_system diagnostics nodes/topics',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
