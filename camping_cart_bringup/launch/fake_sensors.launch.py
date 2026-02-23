from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    clean_before_launch = LaunchConfiguration('clean_before_launch')
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
        default_value='36.7292921',
        description='Map origin latitude',
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value='127.4429577',
        description='Map origin longitude',
    )
    origin_alt_arg = DeclareLaunchArgument(
        'origin_alt',
        default_value='0.0',
        description='Map origin altitude',
    )
    lanelet_id_arg = DeclareLaunchArgument(
        'lanelet_id',
        default_value='-1',
        description='Lanelet ID to follow (-1 uses the first valid centerline)',
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
        'clean_before_launch',
        default_value='true',  # HH_260114 Default on to match prior behavior.
        description='Kill existing fake sensor publisher before launching',
    )
    param_file = LaunchConfiguration('fake_sensors_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    lanelet_id = LaunchConfiguration('lanelet_id')
    speed_mps = LaunchConfiguration('speed_mps')
    publish_rate = LaunchConfiguration('publish_rate_hz')
    loop = LaunchConfiguration('loop')
    frame_id = LaunchConfiguration('frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    obstacle_offset = LaunchConfiguration('obstacle_offset')
    obstacle_height = LaunchConfiguration('obstacle_height')

    clean_action = ExecuteProcess(
        cmd=['bash', '-lc', 'pkill -f fake_sensor_publisher.py || true'],
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
    # HH_260109 Delay launch so cleanup finishes before fake node starts.
    delayed_fake_node = TimerAction(
        period=1.0,
        actions=[fake_node],
    )

    return LaunchDescription([
        clean_arg,
        clean_action,
        param_file_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        lanelet_id_arg,
        speed_mps_arg,
        publish_rate_arg,
        loop_arg,
        frame_id_arg,
        base_frame_arg,
        obstacle_offset_arg,
        obstacle_height_arg,
        delayed_fake_node,
    ])
