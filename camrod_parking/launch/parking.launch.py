import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_parking')
    docking_launch_path = os.path.join(pkg_dir, 'launch', 'docking.launch.py')

    parking_ns = LaunchConfiguration('parking_ns')
    docking_ns = LaunchConfiguration('docking_ns')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parking_ns',
            default_value='parking',
            description='Top-level namespace for parking features'
        ),
        DeclareLaunchArgument(
            'docking_ns',
            default_value='docking',
            description='Sub-namespace for docking feature'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(docking_launch_path),
            launch_arguments={
                'parking_ns': parking_ns,
                'docking_ns': docking_ns,
            }.items()
        ),
    ])
