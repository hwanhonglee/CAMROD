import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Map frame id',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='robot_base_link',
        description='Base frame id',
    )
    # HH_260220: Keep sensor kit anchors under robot_base_link.
    sensor_kit_base_frame_arg = DeclareLaunchArgument(
        'sensor_kit_base_frame_id',
        default_value='sensor_kit_base_link',
        description='Sensor kit base frame id',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'sensor_kit', 'robot_params.yaml')),
        description='Sensor kit robot params',
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camping_cart_sensor_kit', 'launch/sensor_kit.launch.py')),
        launch_arguments={
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'sensor_kit_base_frame_id': LaunchConfiguration('sensor_kit_base_frame_id'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        map_frame_arg,
        base_frame_arg,
        sensor_kit_base_frame_arg,
        params_arg,
        sensor_launch,
    ])
