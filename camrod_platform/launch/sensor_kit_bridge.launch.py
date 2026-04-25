import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Resolves a package-relative file path via package share directory.
def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


# Includes camrod_sensor_kit launch with platform-selected frame/namespace wiring.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='robot_base_link'),
        DeclareLaunchArgument('sensor_kit_base_frame_id', default_value='sensor_kit_base_link'),
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share('camrod_sensor_kit', os.path.join('config', 'robot_params.yaml')),
        ),
        DeclareLaunchArgument('sensor_kit_namespace', default_value='sensor_kit'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_sensor_kit', os.path.join('launch', 'sensor_kit.launch.py'))
            ),
            launch_arguments={
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'sensor_kit_base_frame_id': LaunchConfiguration('sensor_kit_base_frame_id'),
                'params_file': LaunchConfiguration('params_file'),
                'module_namespace': LaunchConfiguration('sensor_kit_namespace'),
                'enable_status': 'false',
            }.items(),
        ),
    ])
