from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


# HH_260109 Launch fake sensor publisher for simulation without real hardware.
def generate_launch_description():
    pkg_share = get_package_share_directory('camping_cart_bringup')
    default_param = os.path.join(pkg_share, 'config', 'sim', 'fake_sensors.yaml')
    default_lidar_grid_param = os.path.join(pkg_share, 'config', 'sensing', 'lidar_cost_grid.yaml')
    default_radar_grid_param = os.path.join(pkg_share, 'config', 'sensing', 'radar_cost_grid.yaml')

    # 2026-02-25: Keep fake-only cleanup independent from top-level bringup cleanup
    # to avoid pkill/start race when included by bringup.
    clean_before_launch = LaunchConfiguration('fake_clean_before_launch')
    # HH_260114 Use unique arg name to avoid param collisions with other includes.
    param_file_arg = DeclareLaunchArgument(
        'fake_sensors_param_file',
        default_value=default_param,
        description='Fake sensor publisher parameter file',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/hong/cart_test_ws/src/lanelet2_map.osm',
        description='Lanelet2 map path for fake trajectory generation',
    )
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value='36.8435737',
        description='Map origin latitude',
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value='128.0925646',
        description='Map origin longitude',
    )
    origin_alt_arg = DeclareLaunchArgument(
        'origin_alt',
        default_value='299.425',
        description='Map origin altitude',
    )
    lanelet_id_arg = DeclareLaunchArgument(
        'lanelet_id',
        default_value='-1',
        description='Lanelet ID to follow (-1 uses the first valid centerline)',
    )
    fake_lanelet_id_arg = DeclareLaunchArgument(
        'fake_lanelet_id',
        default_value='-1',
        description='[Legacy alias] Lanelet ID to follow (-1 uses lanelet_id value)',
    )
    speed_mps_arg = DeclareLaunchArgument(
        'speed_mps',
        default_value='1.4',
        description='Fake vehicle speed (m/s)',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Fake sensor publish rate (Hz)',
    )
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop the lanelet route when reaching the end',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Global frame id',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='robot_base_link',
        description='Vehicle base frame id',
    )
    obstacle_offset_arg = DeclareLaunchArgument(
        'obstacle_offset',
        default_value='5.0',
        description='Obstacle offset ahead of vehicle (m)',
    )
    obstacle_height_arg = DeclareLaunchArgument(
        'obstacle_height',
        default_value='0.5',
        description='Obstacle height (m)',
    )
    clean_arg = DeclareLaunchArgument(
        'fake_clean_before_launch',
        default_value='false',
        description='Kill existing fake sensor publisher before launching',
    )
    lidar_grid_param_arg = DeclareLaunchArgument(
        'fake_lidar_cost_grid_param_file',
        default_value=default_lidar_grid_param,
        description='Lidar near-cost-grid parameter file for sim mode',
    )
    radar_grid_param_arg = DeclareLaunchArgument(
        'fake_radar_cost_grid_param_file',
        default_value=default_radar_grid_param,
        description='Radar near-cost-grid parameter file for sim mode',
    )
    param_file = LaunchConfiguration('fake_sensors_param_file')
    lidar_grid_param_file = LaunchConfiguration('fake_lidar_cost_grid_param_file')
    radar_grid_param_file = LaunchConfiguration('fake_radar_cost_grid_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    lanelet_id = LaunchConfiguration('lanelet_id')
    fake_lanelet_id = LaunchConfiguration('fake_lanelet_id')
    speed_mps = LaunchConfiguration('speed_mps')
    publish_rate = LaunchConfiguration('publish_rate_hz')
    loop = LaunchConfiguration('loop')
    frame_id = LaunchConfiguration('frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    obstacle_offset = LaunchConfiguration('obstacle_offset')
    obstacle_height = LaunchConfiguration('obstacle_height')

    clean_action = ExecuteProcess(
        cmd=['bash', '-lc', 'pkill -f "[f]ake_sensor_publisher.py" || true'],
        output='screen',
        condition=IfCondition(clean_before_launch),
    )
    # HH_260112 Namespace bringup utilities under /bringup with short names.
    fake_node = Node(
        package='camping_cart_bringup',
        executable='fake_sensor_publisher.py',
        name='fake_sensor_publisher',
        namespace='bringup',
        output='screen',
        parameters=[
            param_file,
            {
                # HH_260109 Default fake sensor settings (overridable via launch args).
                'map_path': map_path,
                'origin_lat': ParameterValue(origin_lat, value_type=float),
                'origin_lon': ParameterValue(origin_lon, value_type=float),
                'origin_alt': ParameterValue(origin_alt, value_type=float),
                'lanelet_id': ParameterValue(lanelet_id, value_type=int),
                'fake_lanelet_id': ParameterValue(fake_lanelet_id, value_type=int),
                'speed_mps': ParameterValue(speed_mps, value_type=float),
                'publish_rate_hz': ParameterValue(publish_rate, value_type=float),
                'loop': ParameterValue(loop, value_type=bool),
                'frame_id': frame_id,
                'base_frame_id': base_frame_id,
                'obstacle_offset': ParameterValue(obstacle_offset, value_type=float),
                'obstacle_height': ParameterValue(obstacle_height, value_type=float),
            },
        ],
    )

    # 2026-03-03: In sim mode, still launch numeric lidar/radar cost-grid nodes
    # so the planning stack sees the same topic graph as hardware mode.
    # Lidar cost grid uses fake /perception/obstacles directly. Radar cost grid
    # will remain empty until /sensing/radar/*/range sources are provided.
    lidar_cost_grid = Node(
        package='camping_cart_sensing',
        executable='lidar_cost_grid_node',
        name='lidar_cost_grid',
        namespace='sensing',
        output='screen',
        parameters=[lidar_grid_param_file],
    )
    radar_cost_grid = Node(
        package='camping_cart_sensing',
        executable='radar_cost_grid_node',
        name='radar_cost_grid',
        namespace='sensing',
        output='screen',
        parameters=[radar_grid_param_file],
    )
    return LaunchDescription([
        clean_arg,
        clean_action,
        param_file_arg,
        lidar_grid_param_arg,
        radar_grid_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        lanelet_id_arg,
        fake_lanelet_id_arg,
        speed_mps_arg,
        publish_rate_arg,
        loop_arg,
        frame_id_arg,
        base_frame_arg,
        obstacle_offset_arg,
        obstacle_height_arg,
        fake_node,
        lidar_cost_grid,
        radar_cost_grid,
    ])
