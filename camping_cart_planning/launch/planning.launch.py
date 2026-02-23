import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    # HH_260126 Load defaults from bringup map_info.yaml for map_path/offsets.
    map_info_path = pkg_share('camping_cart_bringup', os.path.join('config', 'map', 'map_info.yaml'))
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    try:
        with open(map_info_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = data.get('/map/lanelet2_map', {}).get('ros__parameters', {})
        map_path_default = str(params.get('map_path', map_path_default))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass

    nav2_param_arg = DeclareLaunchArgument(
        'nav2_param_file',
        # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_lanelet.yaml')),
        description='Nav2 parameter file',
    )
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='rpp',
        description='Controller mode selector: rpp / dwb',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_path_default,
        description='Lanelet2 map path for planning cost grids (optional override)',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value=origin_lat_default)
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value=origin_lon_default)
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value=origin_alt_default)

    nav2_param = LaunchConfiguration('nav2_param_file')
    controller_mode = LaunchConfiguration('controller_mode')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camping_cart_planning', 'launch/nav2_lanelet.launch.py')),
        launch_arguments={
            'nav2_param_file': nav2_param,
            'controller_mode': controller_mode,
        }.items(),
    )

    # HH_260126 Goal snapper: /goal_pose -> /planning/goal_pose (lanelet centerline).
    goal_snapper = Node(
        package='camping_cart_planning',
        executable='goal_snapper_node',
        name='goal_snapper',
        namespace='planning',
        output='screen',
        parameters=[
            pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'goal_snapper.yaml')),
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'input_goal_topic': '/goal_pose',
                'output_goal_topic': '/planning/goal_pose',
            },
        ],
    )

    # HH_260126 Centerline snapper: /localization/pose -> /planning/lanelet_pose.
    centerline_snapper = Node(
        package='camping_cart_planning',
        executable='centerline_snapper_node',
        name='centerline_snapper',
        namespace='planning',
        output='screen',
        parameters=[
            pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'centerline_snapper.yaml')),
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'input_pose_topic': '/localization/pose',
                'output_pose_topic': '/planning/lanelet_pose',
            },
        ],
    )

    # 2026-02-02: Bridge ComputePathToPose results to /planning/global_path for RViz.
    compute_path_bridge = Node(
        package='camping_cart_planning',
        executable='compute_path_bridge_node',
        name='compute_path_bridge',
        namespace='planning',
        output='screen',
        parameters=[
            pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'compute_path_bridge.yaml')),
        ],
    )

    return LaunchDescription([
        nav2_param_arg,
        controller_mode_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        goal_snapper,
        centerline_snapper,
        compute_path_bridge,
        nav2_launch,
    ])
