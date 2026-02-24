"""Top-level orchestrator for camping_cart runtime.

Design rule:
- Bringup owns only cross-package wiring (sim/rviz toggles, shared map origin/path, module includes).
- Each package launch owns node composition and detailed per-node params.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, TimerAction
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

    # [planning package passthrough] controller plugin selector.
    # options: "rpp" (RegulatedPurePursuit) | "dwb" (DWB local planner)
    controller_mode = LaunchConfiguration('controller_mode')

    def pkg_path(pkg, rel):
        return os.path.join(get_package_share_directory(pkg), rel)

    bringup_cfg = lambda rel: pkg_path('camping_cart_bringup', os.path.join('config', rel))

    # -------------------------------------------------------------------------
    # [Config roots] bringup-level config defaults
    # -------------------------------------------------------------------------
    map_param_file = bringup_cfg('map/map_info.yaml')
    sensing_param_file = bringup_cfg('sensing/sensing_params.yaml')
    perception_param_file = bringup_cfg('perception/perception_params.yaml')
    nav2_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_lanelet.yaml'))
    fake_sensors_param_file = pkg_path('camping_cart_bringup', 'config/sim/fake_sensors.yaml')

    # -------------------------------------------------------------------------
    # [Cross-package shared map origin/path]
    # used by map + localization + fake_sensors
    # -------------------------------------------------------------------------
    map_path_arg = LaunchConfiguration('map_path')
    origin_lat_arg = LaunchConfiguration('origin_lat')
    origin_lon_arg = LaunchConfiguration('origin_lon')
    origin_alt_arg = LaunchConfiguration('origin_alt')

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
    enable_radar = LaunchConfiguration('enable_radar')
    enable_radar_cost_grid = LaunchConfiguration('enable_radar_cost_grid')
    perception_param = LaunchConfiguration('perception_param_file')
    nav2_param = LaunchConfiguration('nav2_param_file')
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
        }.items(),
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
            'enable_radar': enable_radar,
            'enable_radar_cost_grid': enable_radar_cost_grid,
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
        }.items(),
    )

    # -------------------------------------------------------------------------
    # [planning package] nav2 + goal snapper + replanner + path bridge
    # -------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_planning', 'launch/planning.launch.py')),
        launch_arguments={
            'nav2_param_file': nav2_param,
            'controller_mode': controller_mode,
        }.items(),
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
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='rpp',
        description='Controller mode selector (options: rpp, dwb)',
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
    # NOTE:
    # fake sensor fine-grained args (lanelet_id/speed/rate/loop/obstacle...) are managed
    # inside fake_sensors.launch.py and config/sim/fake_sensors.yaml.
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
    perception_param_arg = DeclareLaunchArgument(
        'perception_param_file',
        default_value=perception_param_file,
        description='Perception param file override (obstacle fusion)',
    )
    nav2_param_arg = DeclareLaunchArgument(
        'nav2_param_file',
        default_value=nav2_param_file,
        description='Nav2 param file override (planner/controller/BT)',
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
        'pkill -f "[c]amera_preprocessor_node" || true; '
        'pkill -f "[l]idar_preprocessor_node" || true; '
        'pkill -f "[s]ensor_calibration_broadcaster_node" || true; '
        'pkill -f "[p]latform_velocity_converter_node" || true; '
        'pkill -f "[s]en0592_radar_node" || true; '
        'pkill -f "[r]adar_cost_grid_node" || true; '
        'pkill -f "[o]bstacle_fusion_node" || true; '
        'pkill -f "[n]avsat_to_pose_node" || true; '
        'pkill -f "[p]ose_cov_bridge_node" || true; '
        'pkill -f "[c]enterline_snapper_node" || true; '
        'pkill -f "[o]dometry_to_pose_node" || true; '
        'pkill -f "[l]ocalization_health_monitor_node" || true; '
        'pkill -f "[l]anelet2_map_node" || true; '
        'pkill -f "[l]anelet_cost_grid_node" || true; '
        'pkill -f "[r]obot_visualization_node" || true; '
        'pkill -f "[c]ost_field_marker_node" || true; '
        'pkill -f "[c]ost_field_node" || true; '
        'pkill -f "[g]oal_snapper_node" || true; '
        'pkill -f "[g]oal_replanner_node" || true; '
        'pkill -f "[c]ompute_path_bridge_node" || true; '
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

    # HH_260109 Delay launch so cleanup finishes before nodes start.
    launch_stack = GroupAction([
        platform_launch,
        map_stack,
        fake_sensors_launch,
        sensing_launch,
        perception_launch,
        localization_stack,
        nav2_launch,
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

        # ✅ FIX: controller_mode argument 등록 (상위 bringup에서 받아야 CLI override가 먹음)
        controller_mode_arg,
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
        map_param_arg,
        map_visualization_param_arg,
        robot_visualization_param_arg,
        fake_sensors_param_arg,
        sensing_param_arg,
        radar_param_arg,
        radar_cost_grid_param_arg,
        enable_radar_arg,
        enable_radar_cost_grid_arg,
        perception_param_arg,
        nav2_param_arg,
        sim_arg,
        rviz_arg,
        clean_action,
        delayed_stack,
    ])
