import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='robot_base_link'),
        DeclareLaunchArgument('sensor_kit_base_frame_id', default_value='sensor_kit_base_link'),
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share('camrod_sensor_kit', os.path.join('config', 'robot_params.yaml')),
        ),
        DeclareLaunchArgument(
            'robot_visualization_param_file',
            default_value=pkg_share('camrod_platform', os.path.join('config', 'robot_visualization.yaml')),
        ),
        DeclareLaunchArgument('module_namespace', default_value='platform'),
        DeclareLaunchArgument('sensor_kit_namespace', default_value='sensor_kit'),

        # HH_260407: Backward compatibility args (no-op).
        DeclareLaunchArgument('enable_module_validator', default_value='false'),
        DeclareLaunchArgument('system_namespace', default_value='system'),

        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_in_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_out_topic', default_value='/platform/cmd_vel'),
        DeclareLaunchArgument('drive_enable_topic', default_value='/platform/drive_enable'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('use_planning_engage_topic', default_value='true'),
        DeclareLaunchArgument('drive_state_topic', default_value='/platform/drive_enabled'),
        DeclareLaunchArgument('use_estop_topic', default_value='true'),
        # HH_260410: Use Ranger CAN derived /platform/status/estop as the default gate source.
        DeclareLaunchArgument('estop_topic', default_value='/platform/status/estop'),
        DeclareLaunchArgument('drive_allow_on_start', default_value='false'),
        # HH_260410: Keep top-level launch lean.
        # Ranger detailed parameters live in camrod_platform/launch/ranger.launch.py + params YAML.
        DeclareLaunchArgument('ranger_driver_enable', default_value='true'),
        DeclareLaunchArgument(
            'ranger_params_file',
            # HH_260410: Keep Ranger defaults in this package so bringup overrides
            # can consistently reference camrod_platform/config first.
            default_value=pkg_share('camrod_platform', os.path.join('config', 'ranger_params.yaml')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'robot_visualization.launch.py'))
            ),
            launch_arguments={
                'module_namespace': LaunchConfiguration('module_namespace'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'params_file': LaunchConfiguration('params_file'),
                'robot_visualization_param_file': LaunchConfiguration('robot_visualization_param_file'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'cmd_vel_gate.launch.py'))
            ),
            launch_arguments={
                'module_namespace': LaunchConfiguration('module_namespace'),
                'cmd_vel_gate_enable': LaunchConfiguration('cmd_vel_gate_enable'),
                'cmd_vel_in_topic': LaunchConfiguration('cmd_vel_in_topic'),
                'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_out_topic'),
                'drive_enable_topic': LaunchConfiguration('drive_enable_topic'),
                'planning_engage_topic': LaunchConfiguration('planning_engage_topic'),
                'use_planning_engage_topic': LaunchConfiguration('use_planning_engage_topic'),
                'drive_state_topic': LaunchConfiguration('drive_state_topic'),
                'use_estop_topic': LaunchConfiguration('use_estop_topic'),
                'estop_topic': LaunchConfiguration('estop_topic'),
                'drive_allow_on_start': LaunchConfiguration('drive_allow_on_start'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'ranger.launch.py'))
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('ranger_params_file'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('ranger_driver_enable')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'sensor_kit_bridge.launch.py'))
            ),
            launch_arguments={
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'sensor_kit_base_frame_id': LaunchConfiguration('sensor_kit_base_frame_id'),
                'params_file': LaunchConfiguration('params_file'),
                'sensor_kit_namespace': LaunchConfiguration('sensor_kit_namespace'),
            }.items(),
        ),
    ])
