import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        DeclareLaunchArgument('estop_topic', default_value='/planning/state_machine/estop'),
        DeclareLaunchArgument('drive_allow_on_start', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'robot_visualization.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'cmd_vel_gate.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_platform', os.path.join('launch', 'sensor_kit_bridge.launch.py'))
            ),
        ),
    ])
