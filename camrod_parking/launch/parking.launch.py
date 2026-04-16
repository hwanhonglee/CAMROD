import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_parking')
    planning_pkg_dir = get_package_share_directory('camrod_planning')
    docking_launch_path = os.path.join(pkg_dir, 'launch', 'docking.launch.py')
    docking_param_default = os.path.join(pkg_dir, 'config', 'docking_server.yaml')
    docks_file_default = os.path.join(pkg_dir, 'config', 'docks.yaml')
    navigator_bt_xml_default = os.path.join(
        planning_pkg_dir, 'config', 'bt', 'navigate_to_pose_w_planner_selector_grid.xml'
    )

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
        DeclareLaunchArgument(
            'docking_param_file',
            default_value=docking_param_default,
            description='Optional override for docking server parameter file'
        ),
        DeclareLaunchArgument(
            'docks_file',
            default_value=docks_file_default,
            description='Optional override for docking database yaml'
        ),
        DeclareLaunchArgument(
            'navigator_bt_xml',
            default_value=navigator_bt_xml_default,
            description='Optional override for docking navigator BT XML'
        ),
        DeclareLaunchArgument(
            'controller_costmap_topic',
            default_value='/planning/local_costmap/costmap_raw',
            description='Optional override for docking controller costmap topic'
        ),
        DeclareLaunchArgument(
            'controller_footprint_topic',
            default_value='/planning/local_costmap/published_footprint',
            description='Optional override for docking controller footprint topic'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(docking_launch_path),
            launch_arguments={
                'parking_ns': parking_ns,
                'docking_ns': docking_ns,
                'docking_param_file': LaunchConfiguration('docking_param_file'),
                'docks_file': LaunchConfiguration('docks_file'),
                'navigator_bt_xml': LaunchConfiguration('navigator_bt_xml'),
                'controller_costmap_topic': LaunchConfiguration('controller_costmap_topic'),
                'controller_footprint_topic': LaunchConfiguration('controller_footprint_topic'),
            }.items()
        ),
    ])
