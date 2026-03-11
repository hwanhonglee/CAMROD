"""Top-level orchestrator for camping_cart runtime.

Design rule:
- Bringup owns only cross-package wiring (sim/rviz toggles, shared map origin/path, module includes).
- Each package launch owns node composition and detailed per-node params.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # -------------------------------------------------------------------------
    # [Bringup Core] top-level runtime toggles
    # -------------------------------------------------------------------------
    clean_before_launch = LaunchConfiguration('clean_before_launch')
    sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')
    use_eskf = LaunchConfiguration('use_eskf')
    enable_goal_replanner = LaunchConfiguration('enable_goal_replanner')
    enable_state_machine = LaunchConfiguration('enable_state_machine')
    enable_module_checkers = LaunchConfiguration('enable_module_checkers')

    enable_path_cost_grids = LaunchConfiguration('enable_path_cost_grids')

    def pkg_path(pkg, rel):
        return os.path.join(get_package_share_directory(pkg), rel)

    bringup_cfg = lambda rel: pkg_path('camping_cart_bringup', os.path.join('config', rel))

    # -------------------------------------------------------------------------
    # [Config roots] bringup-level config defaults
    # -------------------------------------------------------------------------
    map_param_file = bringup_cfg('map/map_info.yaml')
    sensing_param_file = bringup_cfg('sensing/sensing_params.yaml')
    perception_param_file = bringup_cfg('perception/perception_params.yaml')
    nav2_base_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_base.yaml'))
    nav2_vehicle_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_vehicle.yaml'))
    nav2_vehicle_dwb_param_file = pkg_path(
        'camping_cart_bringup', os.path.join('config', 'planning', 'nav2_vehicle_dwb.yaml'))
    nav2_lanelet_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_lanelet_overlay.yaml'))
    nav2_behavior_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_behavior.yaml'))
    fake_sensors_param_file = pkg_path('camping_cart_bringup', 'config/sim/fake_sensors.yaml')

    # -------------------------------------------------------------------------
    # [Cross-package shared map origin/path]
    # used by map + localization + fake_sensors
    # -------------------------------------------------------------------------
    map_path_arg = LaunchConfiguration('map_path')
    origin_lat_arg = LaunchConfiguration('origin_lat')
    origin_lon_arg = LaunchConfiguration('origin_lon')
    origin_alt_arg = LaunchConfiguration('origin_alt')
    yaw_offset_deg_arg = LaunchConfiguration('yaw_offset_deg')
    # [sim fake sensors] lanelet selector:
    # - lanelet_id: canonical arg
    # - fake_lanelet_id: legacy alias
    lanelet_id = LaunchConfiguration('lanelet_id')
    fake_lanelet_id = LaunchConfiguration('fake_lanelet_id')

    # -------------------------------------------------------------------------
    # [Per-package parameter-file overrides]
    # Note: bringup passes file paths only; each package launch decides how to apply them.
    # -------------------------------------------------------------------------
    map_param = LaunchConfiguration('map_param_file')
    map_visualization_param = LaunchConfiguration('map_visualization_param_file')
    robot_visualization_param = LaunchConfiguration('robot_visualization_param_file')
    sensing_param = LaunchConfiguration('sensing_param_file')
    radar_param = LaunchConfiguration('radar_param_file')
    radar_cost_grid_param = LaunchConfiguration('radar_cost_grid_param_file')
    lidar_cost_grid_param = LaunchConfiguration('lidar_cost_grid_param_file')
    enable_radar = LaunchConfiguration('enable_radar')
    enable_radar_cost_grid = LaunchConfiguration('enable_radar_cost_grid')
    enable_lidar_cost_grid = LaunchConfiguration('enable_lidar_cost_grid')
    perception_param = LaunchConfiguration('perception_param_file')
    nav2_base_param = LaunchConfiguration('nav2_base_param_file')
    nav2_vehicle_param = LaunchConfiguration('nav2_vehicle_param_file')
    use_dwb_controller = LaunchConfiguration('use_dwb_controller')
    nav2_lanelet_param = LaunchConfiguration('nav2_lanelet_param_file')
    nav2_behavior_param = LaunchConfiguration('nav2_behavior_param_file')
    fake_sensors_param = LaunchConfiguration('fake_sensors_param_file')
    eskf_param = LaunchConfiguration('eskf_param_file')
    supervisor_param = LaunchConfiguration('supervisor_param_file')
    wheel_bridge_enable = LaunchConfiguration('wheel_bridge_enable')
    kimera_bridge_enable = LaunchConfiguration('kimera_bridge_enable')
    kimera_bridge_param = LaunchConfiguration('kimera_bridge_param_file')
    pose_selector_enable = LaunchConfiguration('pose_selector_enable')
    pose_selector_param = LaunchConfiguration('pose_selector_param_file')
    # [localization package] disable wheel bridge in sim to avoid duplicate odometry.
    wheel_bridge_enable_sim_safe = PythonExpression([
        "'", wheel_bridge_enable, "' == 'true' and '", sim, "' == 'false'"
    ])
    drop_zone_param = LaunchConfiguration('drop_zone_param_file')

    # -------------------------------------------------------------------------
    # [map package] read default map_path/origin from map_info.yaml once
    # -------------------------------------------------------------------------
    with open(map_param_file, 'r') as f:
        map_cfg = yaml.safe_load(f)
    map_params = map_cfg.get('/map/lanelet2_map', {}).get('ros__parameters', {})
    map_path_default = map_params.get('map_path', '')
    offset_lat_default = float(map_params.get('offset_lat', 0.0))
    offset_lon_default = float(map_params.get('offset_lon', 0.0))
    offset_alt_default = float(map_params.get('offset_alt', 0.0))
    # HH_260307-00:00 Localization yaw alignment default from localization_origin.yaml.
    localization_origin_default = 0.0
    localization_origin_file = bringup_cfg('localization/localization_origin.yaml')
    try:
        with open(localization_origin_file, 'r') as f:
            loc_cfg = yaml.safe_load(f) or {}
        origin_cfg = loc_cfg.get('origin', {}) or {}
        localization_origin_default = float(origin_cfg.get('yaw_offset_deg', 0.0))
    except Exception:
        localization_origin_default = 0.0

    # -------------------------------------------------------------------------
    # [platform package] sensor kit + robot visualization
    # -------------------------------------------------------------------------
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_platform', 'launch/platform.launch.py')),
        launch_arguments={
            'map_frame_id': 'map',
            'base_frame_id': 'robot_base_link',
            'sensor_kit_base_frame_id': 'sensor_kit_base_link',
            'params_file': bringup_cfg('sensor_kit/robot_params.yaml'),
            'robot_visualization_param_file': robot_visualization_param,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
            # HH_260307-00:00 Force legacy alias for Nav2 recovery behaviors that still query base_link.
            'publish_base_link_alias': 'true',
        }.items(),
    )

    # HH_260309-00:00 Hard fallback alias for legacy Nav2 behaviors still requesting "base_link".
    # Keep in bringup as well (not only platform.launch) so TF alias is guaranteed even if
    # older installed platform launch is used.
    bringup_base_link_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bringup_base_link_alias_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'],
        output='screen',
    )

    # -------------------------------------------------------------------------
    # [map package] lanelet map + base cost grid + map-side visualization
    # -------------------------------------------------------------------------
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_map', 'launch/map.launch.py')),
        launch_arguments={
            'map_param_file': map_param,
            'map_path': map_path_arg,
            'origin_lat': origin_lat_arg,
            'origin_lon': origin_lon_arg,
            'origin_alt': origin_alt_arg,
            'map_visualization_param_file': map_visualization_param,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
        }.items(),
    )

    # -------------------------------------------------------------------------
    # [sensing package] preprocess + optional radar + radar cost grid
    # -------------------------------------------------------------------------
    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_sensing', 'launch/sensing.launch.py')),
        launch_arguments={
            'sensing_param_file': sensing_param,
            'radar_param_file': radar_param,
            'radar_cost_grid_param_file': radar_cost_grid_param,
            'lidar_cost_grid_param_file': lidar_cost_grid_param,
            'enable_radar': enable_radar,
            'enable_radar_cost_grid': enable_radar_cost_grid,
            'enable_lidar_cost_grid': enable_lidar_cost_grid,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
        }.items(),
        condition=UnlessCondition(sim),
    )

    # -------------------------------------------------------------------------
    # [perception package] obstacle fusion
    # -------------------------------------------------------------------------
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_perception', 'launch/perception.launch.py')),
        launch_arguments={
            'perception_param_file': perception_param,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
        }.items(),
        condition=UnlessCondition(sim),
    )

    # -------------------------------------------------------------------------
    # [bringup package] fake sensors (sim only)
    # keep fake-specific tuning inside fake_sensors.launch.py / config/sim/fake_sensors.yaml
    # -------------------------------------------------------------------------
    fake_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_bringup', 'launch/fake_sensors.launch.py')),
        launch_arguments={
            'fake_sensors_param_file': fake_sensors_param,
            'map_path': map_path_arg,
            'origin_lat': origin_lat_arg,
            'origin_lon': origin_lon_arg,
            'origin_alt': origin_alt_arg,
            # 2026-02-25: Pass canonical+legacy values directly.
            # The included launch resolves alias precedence internally.
            'lanelet_id': lanelet_id,
            'fake_lanelet_id': fake_lanelet_id,
        }.items(),
        condition=IfCondition(sim),
    )

    # -------------------------------------------------------------------------
    # [localization package] navsat->pose + ESKF/EKF + supervisor/selector
    # -------------------------------------------------------------------------
    localization_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_localization', 'launch/localization.launch.py')),
        launch_arguments={
            'navsat_topic': '/sensing/gnss/navsatfix',
            'gnss_pose_topic': '/sensing/gnss/pose',
            'gnss_pose_cov_topic': '/sensing/gnss/pose_with_covariance',
            'origin_lat': origin_lat_arg,
            'origin_lon': origin_lon_arg,
            'origin_alt': origin_alt_arg,
            'yaw_offset_deg': yaw_offset_deg_arg,
            'use_eskf': use_eskf,
            'eskf_param_file': eskf_param,
            'supervisor_param_file': supervisor_param,
            'wheel_bridge_enable': wheel_bridge_enable_sim_safe,
            # 2026-02-02: Drop zone matcher params (initial localization OK gate).
            'drop_zone_param_file': drop_zone_param,
            # 2026-02-09: Optional Kimera CSV fallback bridge passthrough.
            'kimera_bridge_enable': kimera_bridge_enable,
            'kimera_bridge_param_file': kimera_bridge_param,
            # 2026-02-09: Optional ESKF/Kimera pose selector passthrough.
            'pose_selector_enable': pose_selector_enable,
            'pose_selector_param_file': pose_selector_param,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
        }.items(),
    )

    # -------------------------------------------------------------------------
    # [planning package] nav2 + goal snapper + replanner
    # -------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_planning', 'launch/planning.launch.py')),
        launch_arguments={
            'nav2_base_param_file': nav2_base_param,
            'nav2_vehicle_param_file': nav2_vehicle_param,
            'nav2_lanelet_param_file': nav2_lanelet_param,
            'nav2_behavior_param_file': nav2_behavior_param,
            'enable_path_cost_grids': enable_path_cost_grids,
            'enable_goal_replanner': enable_goal_replanner,
            'enable_state_machine': enable_state_machine,
            # HH_260311-00:00 Use system/module_checkers.launch as single checker source.
            'enable_module_checker': 'false',
        }.items(),
    )

    # -------------------------------------------------------------------------
    # [system package] per-module health checkers + aggregator
    # -------------------------------------------------------------------------
    module_checkers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_system', 'launch/module_checkers.launch.py')),
        launch_arguments={
            # HH_260311-00:00 Enable unified module checkers for all modules including sensing/perception.
            'enable_checkers': 'true',
            'enable_aggregator': 'true',
        }.items(),
        condition=IfCondition(enable_module_checkers),
    )

    # -------------------------------------------------------------------------
    # [rviz] optional visualization
    # -------------------------------------------------------------------------
    rviz_config = pkg_path('camping_cart_map', 'rviz/camping_cart_dark.rviz')
    qss_path = pkg_path('camping_cart_map', 'rviz/dark_theme.qss')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '-stylesheet', qss_path],
        output='screen',
        additional_env={'QT_STYLE_OVERRIDE': 'Fusion'},
        condition=IfCondition(use_rviz),
    )

    clean_arg = DeclareLaunchArgument(
        'clean_before_launch',
        default_value='true',
        description='Kill existing camping_cart processes before launching',
    )

    # [planning package]
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='true',
        description='Enable planning path-cost-grid helper nodes (/planning/cost_grid/*)',
    )
    enable_goal_replanner_arg = DeclareLaunchArgument(
        'enable_goal_replanner',
        # HH_260309-00:00 Default-on to keep goal->global/local cost-grid refresh immediate
        # in standard bringup runs. Can still be disabled explicitly when needed.
        default_value='true',
        description='Enable /planning/goal_replanner (ComputePathToPose helper; options: true, false)',
    )
    enable_state_machine_arg = DeclareLaunchArgument(
        'enable_state_machine',
        default_value='true',
        description='Enable /planning/planning_state_machine (system-status driven planning policy)',
    )
    enable_module_checkers_arg = DeclareLaunchArgument(
        'enable_module_checkers',
        default_value='true',
        description='Enable per-module checker and system health aggregator',
    )
    use_eskf_arg = DeclareLaunchArgument(
        'use_eskf',
        default_value='true',
        description='Localization filter selector (options: true=ESKF, false=legacy EKF)',
    )
    eskf_param_arg = DeclareLaunchArgument(
        'eskf_param_file',
        default_value=pkg_path('camping_cart_bringup', os.path.join('config', 'localization', 'eskf.yaml')),
        description='ESKF parameter file',
    )
    supervisor_param_arg = DeclareLaunchArgument(
        'supervisor_param_file',
        default_value=pkg_path('camping_cart_bringup', os.path.join('config', 'localization', 'supervisor.yaml')),
        description='Supervisor parameter file',
    )
    drop_zone_param_arg = DeclareLaunchArgument(
        'drop_zone_param_file',
        default_value=pkg_path('camping_cart_bringup', os.path.join('config', 'localization', 'drop_zone_matcher.yaml')),
        description='Drop zone matcher parameter file',
    )
    wheel_bridge_enable_arg = DeclareLaunchArgument(
        'wheel_bridge_enable',
        default_value='true',
        description='Enable wheel odom bridge (/platform/status/wheel -> /platform/wheel/odometry)',
    )
    kimera_bridge_enable_arg = DeclareLaunchArgument(
        'kimera_bridge_enable',
        default_value='false',
        description='Enable Kimera CSV fallback bridge',
    )
    kimera_bridge_param_arg = DeclareLaunchArgument(
        'kimera_bridge_param_file',
        default_value=pkg_path('camping_cart_bringup', os.path.join('config', 'localization', 'kimera_bridge.yaml')),
        description='Kimera CSV bridge parameter file',
    )
    pose_selector_enable_arg = DeclareLaunchArgument(
        'pose_selector_enable',
        # 2026-02-24: Direct ESKF output by default for faster, deterministic startup.
        default_value='false',
        description='Enable localization pose selector (ESKF primary, Kimera fallback)',
    )
    pose_selector_param_arg = DeclareLaunchArgument(
        'pose_selector_param_file',
        default_value=pkg_path('camping_cart_bringup', os.path.join('config', 'localization', 'pose_selector.yaml')),
        description='Pose selector parameter file',
    )

    map_param_arg = DeclareLaunchArgument(
        'map_param_file',
        default_value=map_param_file,
        description='Map info YAML (kept for overrides; default used for offsets)',
    )
    map_visualization_param_arg = DeclareLaunchArgument(
        'map_visualization_param_file',
        default_value=bringup_cfg('map/map_visualization.yaml'),
        description='Map module visualization parameter file',
    )
    robot_visualization_param_arg = DeclareLaunchArgument(
        'robot_visualization_param_file',
        default_value=bringup_cfg('platform/robot_visualization.yaml'),
        description='Platform module robot visualization parameter file',
    )
    # [shared map args] consumed by map/localization/fake_sensors launches.
    map_path_decl = DeclareLaunchArgument(
        'map_path',
        default_value=str(map_path_default),
        description='Lanelet2 map path (exposed so nested includes see it)',
    )
    origin_lat_decl = DeclareLaunchArgument(
        'origin_lat',
        default_value=str(offset_lat_default),
        description='Map origin latitude (propagated to fake sensors and localization)',
    )
    origin_lon_decl = DeclareLaunchArgument(
        'origin_lon',
        default_value=str(offset_lon_default),
        description='Map origin longitude (propagated to fake sensors and localization)',
    )
    origin_alt_decl = DeclareLaunchArgument(
        'origin_alt',
        default_value=str(offset_alt_default),
        description='Map origin altitude (propagated to fake sensors and localization)',
    )
    yaw_offset_deg_decl = DeclareLaunchArgument(
        'yaw_offset_deg',
        default_value=str(localization_origin_default),
        description='Yaw alignment offset in degrees for GNSS->map conversion',
    )
    # NOTE:
    # fake sensor fine-grained args (lanelet_id/speed/rate/loop/obstacle...) are managed
    # inside fake_sensors.launch.py and config/sim/fake_sensors.yaml.
    lanelet_id_arg = DeclareLaunchArgument(
        'lanelet_id',
        default_value='-1',
        description='Fake sensor lanelet id (options: -1=auto, >=0=specific lanelet id)',
    )
    fake_lanelet_id_arg = DeclareLaunchArgument(
        'fake_lanelet_id',
        default_value='-1',
        description='[Legacy alias] fake sensor lanelet id override',
    )
    fake_sensors_param_arg = DeclareLaunchArgument(
        'fake_sensors_param_file',
        default_value=fake_sensors_param_file,
        description='Fake sensor parameter file (declared here to satisfy include)',
    )
    sensing_param_arg = DeclareLaunchArgument(
        'sensing_param_file',
        default_value=sensing_param_file,
        description='Sensing param file override (preprocess nodes)',
    )
    radar_param_arg = DeclareLaunchArgument(
        'radar_param_file',
        default_value=bringup_cfg('sensing/sen0592_radar.yaml'),
        description='SEN0592 radar parameter file',
    )
    radar_cost_grid_param_arg = DeclareLaunchArgument(
        'radar_cost_grid_param_file',
        default_value=bringup_cfg('sensing/radar_cost_grid.yaml'),
        description='Radar near-range cost grid parameter file',
    )
    lidar_cost_grid_param_arg = DeclareLaunchArgument(
        'lidar_cost_grid_param_file',
        default_value=bringup_cfg('sensing/lidar_cost_grid.yaml'),
        description='LiDAR near-range cost grid parameter file',
    )
    enable_radar_arg = DeclareLaunchArgument(
        'enable_radar',
        default_value='false',
        description='Enable SEN0592 radar serial node (options: true, false)',
    )
    enable_radar_cost_grid_arg = DeclareLaunchArgument(
        'enable_radar_cost_grid',
        default_value='true',
        description='Enable radar near-range cost grid node (options: true, false)',
    )
    enable_lidar_cost_grid_arg = DeclareLaunchArgument(
        'enable_lidar_cost_grid',
        default_value='true',
        description='Enable LiDAR near-range cost grid node (options: true, false)',
    )
    perception_param_arg = DeclareLaunchArgument(
        'perception_param_file',
        default_value=perception_param_file,
        description='Perception param file override (obstacle fusion)',
    )
    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=nav2_base_param_file,
        description='Nav2 base profile parameter file',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=nav2_vehicle_param_file,
        description='Nav2 vehicle profile parameter file',
    )
    use_dwb_controller_arg = DeclareLaunchArgument(
        'use_dwb_controller',
        default_value='false',
        description='Enable DWB profile (options: false=FollowPath only, true=FollowPath+DWB)',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=nav2_lanelet_param_file,
        description='Nav2 lanelet profile parameter file',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=nav2_behavior_param_file,
        description='Nav2 behavior profile parameter file',
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Simulation mode switch (options: true, false)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='RViz launch switch (options: true, false)',
    )

    # 2026-02-02: Use bracketed pkill patterns to avoid killing the cleanup shell itself.
    clean_cmd = (
        # 2026-02-27: Prevent duplicate graph by terminating older bringup launch parents first.
        # Exclude current launch parent PID (this bringup invocation).
        'CURRENT_LAUNCH_PID="$PPID"; '
        'for pid in $(ps -eo pid=,cmd= | awk \'/[r]os2 launch camping_cart_bringup bringup.launch.py/ {print $1}\'); do '
        '  if [ "$pid" != "$CURRENT_LAUNCH_PID" ]; then kill -TERM "$pid" 2>/dev/null || true; fi; '
        'done; '
        'sleep 0.2; '
        'for pid in $(ps -eo pid=,cmd= | awk \'/[r]os2 launch camping_cart_bringup bringup.launch.py/ {print $1}\'); do '
        '  if [ "$pid" != "$CURRENT_LAUNCH_PID" ]; then kill -KILL "$pid" 2>/dev/null || true; fi; '
        'done; '
        'pkill -f "[l]ocal_path_extractor_node" || true; '
        'pkill -f "[c]ompute_path_bridge_node" || true; '
        'pkill -f "[c]amera_preprocessor_node" || true; '
        'pkill -f "[l]idar_preprocessor_node" || true; '
        'pkill -f "[s]ensor_calibration_broadcaster_node" || true; '
        'pkill -f "[p]latform_velocity_converter_node" || true; '
        'pkill -f "[s]en0592_radar_node" || true; '
        'pkill -f "[r]adar_cost_grid_node" || true; '
        'pkill -f "[l]idar_cost_grid_node" || true; '
        'pkill -f "[o]bstacle_fusion_node" || true; '
        'pkill -f "[n]avsat_to_pose_node" || true; '
        'pkill -f "[p]ose_cov_bridge_node" || true; '
        'pkill -f "[l]ocalization_supervisor_node" || true; '
        'pkill -f "[d]rop_zone_matcher_node" || true; '
        'pkill -f "[c]enterline_snapper_node" || true; '
        'pkill -f "[o]dometry_to_pose_node" || true; '
        'pkill -f "[l]ocalization_health_monitor_node" || true; '
        'pkill -f "[l]anelet2_map_node" || true; '
        'pkill -f "[l]anelet_cost_grid_node" || true; '
        'pkill -f "[m]arker_array_aggregator_node" || true; '
        'pkill -f "[r]obot_visualization_node" || true; '
        'pkill -f "[c]ost_field_marker_node" || true; '
        'pkill -f "[c]ost_field_node" || true; '
        'pkill -f "[g]oal_snapper_node" || true; '
        'pkill -f "[g]oal_replanner_node" || true; '
        'pkill -f "[e]kf_node" || true; '
        'pkill -f "[n]avsat_transform_node" || true; '
        'pkill -f "[r]obot_state_publisher" || true; '
        'pkill -f "[b]t_navigator" || true; '
        'pkill -f "[c]ontroller_server" || true; '
        'pkill -f "[p]lanner_server" || true; '
        'pkill -f "[n]av2_costmap_2d" || true; '
        'pkill -f "[l]ifecycle_manager" || true; '
        'pkill -f "[r]viz2" || true; '
        'pkill -f "[f]ake_sensor_publisher.py" || true'
    )
    clean_action = ExecuteProcess(
        cmd=['bash', '-lc', clean_cmd],
        output='screen',
        condition=IfCondition(clean_before_launch),
    )

    # 2026-02-26: Optional controller-profile switch without manual file path override.
    # When enabled, replace nav2_vehicle_param_file with nav2_vehicle_dwb.yaml.
    apply_dwb_profile = SetLaunchConfiguration(
        'nav2_vehicle_param_file',
        nav2_vehicle_dwb_param_file,
        condition=IfCondition(use_dwb_controller),
    )

    # HH_260109 Delay launch so cleanup finishes before nodes start.
    launch_stack = GroupAction([
        platform_launch,
        bringup_base_link_alias,
        map_stack,
        fake_sensors_launch,
        sensing_launch,
        perception_launch,
        localization_stack,
        nav2_launch,
        module_checkers_launch,
        rviz_node,
    ])
    delayed_stack = TimerAction(
        period=1.0,
        actions=[
            # 2026-01-27 17:45: Suppress startup LogInfo; keep notes in README instead.
            launch_stack,
        ],
    )

    return LaunchDescription([
        clean_arg,

        enable_path_cost_grids_arg,
        enable_goal_replanner_arg,
        enable_state_machine_arg,
        enable_module_checkers_arg,
        use_eskf_arg,
        eskf_param_arg,
        supervisor_param_arg,
        drop_zone_param_arg,
        wheel_bridge_enable_arg,
        kimera_bridge_enable_arg,
        kimera_bridge_param_arg,
        pose_selector_enable_arg,
        pose_selector_param_arg,

        map_path_decl,
        origin_lat_decl,
        origin_lon_decl,
        origin_alt_decl,
        yaw_offset_deg_decl,
        lanelet_id_arg,
        fake_lanelet_id_arg,
        map_param_arg,
        map_visualization_param_arg,
        robot_visualization_param_arg,
        fake_sensors_param_arg,
        sensing_param_arg,
        radar_param_arg,
        radar_cost_grid_param_arg,
        lidar_cost_grid_param_arg,
        enable_radar_arg,
        enable_radar_cost_grid_arg,
        enable_lidar_cost_grid_arg,
        perception_param_arg,
        nav2_base_param_arg,
        nav2_vehicle_param_arg,
        use_dwb_controller_arg,
        nav2_lanelet_param_arg,
        nav2_behavior_param_arg,
        sim_arg,
        rviz_arg,
        clean_action,
        apply_dwb_profile,
        delayed_stack,
    ])
