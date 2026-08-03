from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    if not isinstance(map_info_cfg, dict):
        return {}
    wildcard = map_info_cfg.get('/**')
    if isinstance(wildcard, dict):
        params = wildcard.get('ros__parameters')
        if isinstance(params, dict):
            return params
    for key in (
        '/map/lanelet2_map',
        'map/lanelet2_map',
        'lanelet2_map',
        '/lanelet2_map',
    ):
        val = map_info_cfg.get(key)
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    for val in map_info_cfg.values():
        if isinstance(val, dict):
            params = val.get('ros__parameters')
            if isinstance(params, dict):
                return params
    return {}


def load_map_defaults() -> dict:
    defaults = {
        'map_path': '',
        'origin_lat': '0.0',
        'origin_lon': '0.0',
        'origin_alt': '0.0',
    }
    map_info_path = os.path.join(
        get_package_share_directory('camrod_map'),
        'config',
        'map_info.yaml',
    )
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        defaults['map_path'] = str(params.get('map_path', '')).strip()
        defaults['origin_lat'] = str(params.get('offset_lat', defaults['origin_lat']))
        defaults['origin_lon'] = str(params.get('offset_lon', defaults['origin_lon']))
        defaults['origin_alt'] = str(params.get('offset_alt', defaults['origin_alt']))
    except Exception:
        pass

    configured = defaults['map_path']
    if configured:
        configured_path = (
            configured
            if os.path.isabs(configured)
            else os.path.abspath(os.path.join(os.path.dirname(map_info_path), configured))
        )
        if os.path.isfile(configured_path):
            defaults['map_path'] = configured_path

    if not defaults['map_path']:
        for candidate in (
            os.path.join(os.path.expanduser('~'), 'camrod_ws', 'src', 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'lanelet2_maps.osm'),
            os.path.join(os.getcwd(), 'src', 'lanelet2_maps.osm'),
        ):
            if os.path.isfile(candidate):
                defaults['map_path'] = os.path.abspath(candidate)
                break
    return defaults


# Launch fake sensor publisher for simulation without real hardware.
def generate_launch_description():
    pkg_share = get_package_share_directory('camrod_bringup')
    sensing_share = get_package_share_directory('camrod_sensing')
    default_param = os.path.join(pkg_share, 'config', 'sim', 'fake_sensors.yaml')
    map_defaults = load_map_defaults()
    default_lidar_grid_param = os.path.join(
        sensing_share, 'config', 'lidar', 'cost_grid.yaml')
    default_radar_grid_param = os.path.join(
        sensing_share, 'config', 'radar', 'cost_grid.yaml')

    # 2026-02-25: Keep fake-only cleanup independent from top-level bringup cleanup
    # to avoid pkill/start race when included by bringup.
    clean_before_launch = LaunchConfiguration('fake_clean_before_launch')
    # Use unique arg name to avoid param collisions with other includes.
    param_file_arg = DeclareLaunchArgument(
        'fake_sensors_param_file',
        default_value=default_param,
        description='Fake sensor publisher parameter file',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_defaults['map_path'],
        description='Lanelet2 map path for fake trajectory generation',
    )
    origin_lat_arg = DeclareLaunchArgument(
        'origin_lat',
        default_value=map_defaults['origin_lat'],
        description='Map origin latitude',
    )
    origin_lon_arg = DeclareLaunchArgument(
        'origin_lon',
        default_value=map_defaults['origin_lon'],
        description='Map origin longitude',
    )
    origin_alt_arg = DeclareLaunchArgument(
        'origin_alt',
        default_value=map_defaults['origin_alt'],
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
        # HH_260618: Match fake sensor launch override with YAML default to reduce sim topic load.
        default_value='10.0',
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
        default_value='robot_center_link',
        description='Vehicle base frame id',
    )
    obstacle_offset_arg = DeclareLaunchArgument(
        'obstacle_offset',
        default_value='12.0',
        description='Obstacle offset distance from vehicle along obstacle_direction (m)',
    )
    obstacle_height_arg = DeclareLaunchArgument(
        'obstacle_height',
        default_value='0.5',
        description='Obstacle height (m)',
    )
    obstacle_direction_arg = DeclareLaunchArgument(
        'obstacle_direction',
        default_value='front',
        description='Synthetic obstacle direction: front|left|right|rear',
    )
    obstacle_lateral_offset_arg = DeclareLaunchArgument(
        'obstacle_lateral_offset',
        default_value='0.0',
        description='Additional obstacle shift along vehicle-left axis (m)',
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
    fake_enable_cost_grids_arg = DeclareLaunchArgument(
        'fake_enable_cost_grids',
        default_value='false',
        description='Enable lidar/radar cost-grid nodes inside fake_sensors.launch (usually false when sensing.launch is active)',
    )
    simulated_platform_status_arg = DeclareLaunchArgument(
        'publish_simulated_platform_status',
        default_value='true',
        description='Publish deterministic raw CAN/BMS feedback in ordinary simulation',
    )
    bringup_namespace_arg = DeclareLaunchArgument(
        'bringup_namespace',
        default_value='bringup',
        description='Namespace for bringup fake sensor node',
    )
    sensing_namespace_arg = DeclareLaunchArgument(
        'sensing_namespace',
        default_value='sensing',
        description='Namespace for sensing helper nodes',
    )
    param_file = LaunchConfiguration('fake_sensors_param_file')
    lidar_grid_param_file = LaunchConfiguration('fake_lidar_cost_grid_param_file')
    radar_grid_param_file = LaunchConfiguration('fake_radar_cost_grid_param_file')
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
    obstacle_direction = LaunchConfiguration('obstacle_direction')
    obstacle_lateral_offset = LaunchConfiguration('obstacle_lateral_offset')
    bringup_namespace = LaunchConfiguration('bringup_namespace')
    sensing_namespace = LaunchConfiguration('sensing_namespace')

    clean_action = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            # HH_260618: Match both ROS node token and script name, then
            # escalate after a short grace period so standalone sim reruns do
            # not leave fake_sensor_publisher consuming CPU.
            '_pids=""; '
            'for _pat in "__node:=fake_sensor_publisher" "fake_sensor_publisher.py"; do '
            'for _pid in $(pgrep -f "$_pat" || true); do '
            '[ "$_pid" = "$$" ] && continue; '
            '[ "$_pid" = "$PPID" ] && continue; '
            '_pids="$_pids $_pid"; '
            'kill "$_pid" 2>/dev/null || true; '
            'done; '
            'done; '
            'sleep 0.5; '
            'for _pid in $_pids; do '
            '[ "$_pid" = "$$" ] && continue; '
            '[ "$_pid" = "$PPID" ] && continue; '
            'kill -0 "$_pid" 2>/dev/null && kill -9 "$_pid" 2>/dev/null || true; '
            'done',
        ],
        output='screen',
        condition=IfCondition(clean_before_launch),
    )
    # Namespace bringup utilities under /bringup with short names.
    fake_node = Node(
        package='camrod_bringup',
        executable='fake_sensor_publisher.py',
        name='fake_sensor_publisher',
        namespace=bringup_namespace,
        output='screen',
        parameters=[
            param_file,
            {
                # Default fake sensor settings (overridable via launch args).
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
                'obstacle_direction': obstacle_direction,
                'obstacle_lateral_offset': ParameterValue(obstacle_lateral_offset, value_type=float),
                # HH_260721 - Dedicated validators can disable ordinary raw CAN/BMS simulation.
                'publish_simulated_platform_status': ParameterValue(
                    LaunchConfiguration('publish_simulated_platform_status'), value_type=bool
                ),
            },
        ],
    )

    # 2026-03-03: In sim mode, still launch numeric lidar/radar cost-grid nodes
    # so the planning stack sees the same topic graph as hardware mode.
    # fake_sensor_publisher also mirrors obstacle cloud to
    # /sensing/lidar/points_filtered for lidar-cost-grid input consistency.
    # Radar cost grid will remain empty until /sensing/radar/*/range sources are provided.
    lidar_cost_grid = Node(
        package='camrod_sensing',
        executable='lidar_cost_grid_node',
        name='lidar_cost_grid',
        namespace=sensing_namespace,
        output='screen',
        parameters=[lidar_grid_param_file],
        condition=IfCondition(LaunchConfiguration('fake_enable_cost_grids')),
    )
    radar_cost_grid = Node(
        package='camrod_sensing',
        executable='radar_cost_grid_node',
        name='radar_cost_grid',
        namespace=sensing_namespace,
        output='screen',
        parameters=[radar_grid_param_file],
        condition=IfCondition(LaunchConfiguration('fake_enable_cost_grids')),
    )
    return LaunchDescription([
        clean_arg,
        clean_action,
        param_file_arg,
        lidar_grid_param_arg,
        radar_grid_param_arg,
        fake_enable_cost_grids_arg,
        simulated_platform_status_arg,
        bringup_namespace_arg,
        sensing_namespace_arg,
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
        obstacle_direction_arg,
        obstacle_lateral_offset_arg,
        fake_node,
        lidar_cost_grid,
        radar_cost_grid,
    ])
