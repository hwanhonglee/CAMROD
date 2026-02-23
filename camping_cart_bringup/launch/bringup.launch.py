from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # HH_260112 Allow optional cleanup of stale nodes before launching.
    clean_before_launch = LaunchConfiguration('clean_before_launch')
    sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')
    use_eskf = LaunchConfiguration('use_eskf')

    # ✅ FIX: expose controller_mode at top-level bringup launch
    # 이유: bringup.launch.py에서 DeclareLaunchArgument로 정의되지 않으면
    #      CLI에서 controller_mode:=dwb 를 줘도 LaunchConfiguration이 존재하지 않아
    #      "name 'controller_mode' is not defined" 혹은 하위 include로 전달 불가 문제가 발생함.
    controller_mode = LaunchConfiguration('controller_mode')

    def pkg_path(pkg, rel):
        return os.path.join(get_package_share_directory(pkg), rel)

    bringup_cfg = lambda rel: pkg_path('camping_cart_bringup', os.path.join('config', rel))

    # HH_260114 Avoid param arg name collisions across included launches.
    map_param_file = bringup_cfg('map/map_info.yaml')
    sensing_param_file = bringup_cfg('sensing/sensing_params.yaml')
    perception_param_file = bringup_cfg('perception/perception_params.yaml')
    nav2_param_file = pkg_path('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_lanelet.yaml'))
    fake_sensors_param_file = pkg_path('camping_cart_bringup', 'config/sim/fake_sensors.yaml')

    # HH_260114 Expose map path args at top-level to satisfy nested includes.
    map_path_arg = LaunchConfiguration('map_path')
    origin_lat_arg = LaunchConfiguration('origin_lat')
    origin_lon_arg = LaunchConfiguration('origin_lon')
    origin_alt_arg = LaunchConfiguration('origin_alt')
    lanelet_id_arg = LaunchConfiguration('lanelet_id')
    speed_mps_arg = LaunchConfiguration('speed_mps')
    publish_rate_arg = LaunchConfiguration('publish_rate_hz')
    loop_arg = LaunchConfiguration('loop')
    frame_id_arg = LaunchConfiguration('frame_id')
    base_frame_arg = LaunchConfiguration('base_frame_id')
    obstacle_offset_arg = LaunchConfiguration('obstacle_offset')
    obstacle_height_arg = LaunchConfiguration('obstacle_height')
    map_param = LaunchConfiguration('map_param_file')
    sensing_param = LaunchConfiguration('sensing_param_file')
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
    # 2026-02-03: Disable wheel bridge in sim to avoid duplicate /platform/wheel/odometry.
    # PythonExpression expects valid Python literals; compare string values explicitly.
    wheel_bridge_enable_sim_safe = PythonExpression([
        "'", wheel_bridge_enable, "' == 'true' and '", sim, "' == 'false'"
    ])
    drop_zone_param = LaunchConfiguration('drop_zone_param_file')
    vio_config = pkg_path(
        'camping_cart_localization',
        os.path.join('config', 'open_vins', 'camping_cart', 'estimator_config.yaml'))

    # HH_260114 Load shared map parameters from map_info.yaml for offsets/map path.
    with open(map_param_file, 'r') as f:
        map_cfg = yaml.safe_load(f)
    map_params = map_cfg.get('/map/lanelet2_map', {}).get('ros__parameters', {})
    map_path_default = map_params.get('map_path', '')
    offset_lat_default = float(map_params.get('offset_lat', 0.0))
    offset_lon_default = float(map_params.get('offset_lon', 0.0))
    offset_alt_default = float(map_params.get('offset_alt', 0.0))

    # HH_260121 Platform stack (sensor kit + TF).
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_platform', 'launch/platform.launch.py')),
        launch_arguments={
            'map_frame_id': 'map',
            'base_frame_id': 'robot_base_link',
            'sensor_kit_base_frame_id': 'sensor_kit_base_link',
            'params_file': bringup_cfg('sensor_kit/robot_params.yaml'),
        }.items(),
    )

    # HH_260121 Map stack (map + cost grid + snappers).
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_map', 'launch/map.launch.py')),
        launch_arguments={
            'map_param_file': map_param,
            'map_path': map_path_arg,
            'origin_lat': origin_lat_arg,
            'origin_lon': origin_lon_arg,
            'origin_alt': origin_alt_arg,
        }.items(),
    )

    # HH_260121 Visualization nodes launched separately.
    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_visualizer', 'launch/visualizer.launch.py')),
        launch_arguments={
            'visualizer_params': bringup_cfg('visualizer/visualizer_params.yaml'),
            'map_info': map_param_file,
            'robot_params': bringup_cfg('sensor_kit/robot_params.yaml'),
        }.items(),
    )

    # HH_260121 Sensing pipeline (LiDAR/Camera preprocessing + velocity conversion).
    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_sensing', 'launch/sensing.launch.py')),
        launch_arguments={
            'sensing_param_file': sensing_param,
        }.items(),
        condition=UnlessCondition(sim),
    )

    # HH_260121 Perception pipeline (obstacle fusion -> /perception/obstacles).
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_perception', 'launch/perception.launch.py')),
        launch_arguments={
            'perception_param_file': perception_param,
        }.items(),
        condition=UnlessCondition(sim),
    )

    # HH_260112 Fake sensors for simulation mode.
    fake_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_bringup', 'launch/fake_sensors.launch.py')),
        launch_arguments={
            'fake_sensors_param_file': fake_sensors_param,
            'map_path': map_path_arg,
            'origin_lat': origin_lat_arg,
            'origin_lon': origin_lon_arg,
            'origin_alt': origin_alt_arg,
            'lanelet_id': lanelet_id_arg,
            'speed_mps': speed_mps_arg,
            'publish_rate_hz': publish_rate_arg,
            'loop': loop_arg,
            'frame_id': frame_id_arg,
            'base_frame_id': base_frame_arg,
            'obstacle_offset': obstacle_offset_arg,
            'obstacle_height': obstacle_height_arg,
        }.items(),
        condition=IfCondition(sim),
    )

    # HH_260121 Localization stack (GNSS → pose + EKF).
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

    # HH_260121 Nav2 stack wrapper with controller_mode passthrough.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_path('camping_cart_planning', 'launch/planning.launch.py')),
        launch_arguments={
            'nav2_param_file': nav2_param,
            'controller_mode': controller_mode,
        }.items(),
    )

    # HH_260114 Optional RViz using camping_cart_map dark theme stylesheet.
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

    # ✅ FIX: controller_mode launch arg 선언 (CLI에서 controller_mode:=dwb 를 주기 위함)
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='rpp',
        description='Controller mode selector: rpp / dwb',
    )
    use_eskf_arg = DeclareLaunchArgument(
        'use_eskf',
        default_value='true',
        description='Use ESKF (true) or legacy EKF (false)',
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
        default_value='true',
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
    lanelet_id_decl = DeclareLaunchArgument(
        'lanelet_id',
        default_value='-1',
        description='Lanelet ID to follow for fake sensors (-1 = first valid centerline)',
    )
    speed_mps_decl = DeclareLaunchArgument(
        'speed_mps',
        default_value='1.4',
        description='Fake vehicle speed (m/s, exposed to avoid missing config)',
    )
    publish_rate_decl = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Fake sensor publish rate (Hz)',
    )
    loop_decl = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop the lanelet route when reaching the end',
    )
    frame_id_decl = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Global frame id for fake sensors',
    )
    base_frame_decl = DeclareLaunchArgument(
        'base_frame_id',
        default_value='robot_base_link',
        description='Vehicle base frame id for fake sensors',
    )
    obstacle_offset_decl = DeclareLaunchArgument(
        'obstacle_offset',
        default_value='5.0',
        description='Obstacle offset ahead of vehicle (m)',
    )
    obstacle_height_decl = DeclareLaunchArgument(
        'obstacle_height',
        default_value='0.5',
        description='Obstacle height (m)',
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
        description='Enable simulation mode (launch fake sensors, skip real sensing/VIO)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 (disable to keep terminal alive for debugging)',
    )

    # 2026-02-02: Use bracketed pkill patterns to avoid killing the cleanup shell itself.
    clean_cmd = (
        'pkill -f "[c]amera_preprocessor_node" || true; '
        'pkill -f "[l]idar_preprocessor_node" || true; '
        'pkill -f "[s]ensor_calibration_broadcaster_node" || true; '
        'pkill -f "[p]latform_velocity_converter_node" || true; '
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
        visualizer_launch,
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
        lanelet_id_decl,
        speed_mps_decl,
        publish_rate_decl,
        loop_decl,
        frame_id_decl,
        base_frame_decl,
        obstacle_offset_decl,
        obstacle_height_decl,
        map_param_arg,
        fake_sensors_param_arg,
        sensing_param_arg,
        perception_param_arg,
        nav2_param_arg,
        sim_arg,
        rviz_arg,
        clean_action,
        delayed_stack,
    ])
