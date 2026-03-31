import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition


# Implements `pkg_share` behavior.
def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


def extract_map_ros_params(map_info_cfg: dict) -> dict:
    # HH_260330: Robust map_info parser for bringup/module key-layout differences.
    if not isinstance(map_info_cfg, dict):
        return {}
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


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # HH_260330: Standalone planning launch uses package-local map config by default.
    map_info_path = pkg_share('camrod_map', os.path.join('config', 'map_info.yaml'))
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    try:
        with open(map_info_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path_default = str(params.get('map_path', map_path_default))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass

    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_base.yaml')),
        description='Nav2 base profile parameter file',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_vehicle.yaml')),
        description='Nav2 vehicle profile parameter file',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_lanelet_overlay.yaml')),
        description='Nav2 lanelet profile parameter file',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_behavior.yaml')),
        description='Nav2 behavior profile parameter file',
    )
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='true',
        description='Enable planning path-cost-grid helper nodes (/planning/cost_grid/*)',
    )
    enable_goal_replanner_arg = DeclareLaunchArgument(
        'enable_goal_replanner',
        # Default OFF:
        # Nav2 planner already computes /planning/global_path from snapped goal input.
        # Keeping goal_replanner ON by default can create duplicate ComputePath action requests
        # and intermittent abort/preemption races.
        default_value='false',
        description='Enable /planning/goal_replanner (ComputePathToPose helper)',
    )
    enable_module_validator_arg = DeclareLaunchArgument(
        'enable_module_validator',
        default_value='true',
        description='Enable planning module validator publisher',
    )
    enable_nav2_lifecycle_retry_arg = DeclareLaunchArgument(
        'enable_nav2_lifecycle_retry',
        # Keep OFF by default to avoid lifecycle transition races.
        # Enable only when you explicitly want retry behavior in unstable environments.
        default_value='false',
        description='Enable Nav2 lifecycle startup retry helper',
    )
    require_localization_ready_arg = DeclareLaunchArgument(
        'require_localization_ready',
        # HH_260327: Default OFF so Nav2 can activate during field debugging
        # even when /localization/initial_match_ok stays false.
        default_value='false',
        description='Wait for localization readiness before Nav2 lifecycle STARTUP',
    )
    enable_state_machine_arg = DeclareLaunchArgument(
        'enable_state_machine',
        # HH_260315-00:00 Default off for manual-goal navigation/debug.
        # Enable explicitly when state-policy automation is needed.
        default_value='false',
        description='Enable /planning/planning_state_machine (system-status driven planning policy)',
    )
    local_path_extractor_param_arg = DeclareLaunchArgument(
        'local_path_extractor_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'local_path_extractor.yaml')),
        description='Parameters for /planning/local_path_extractor',
    )
    planning_state_machine_param_arg = DeclareLaunchArgument(
        'planning_state_machine_param_file',
        default_value=pkg_share('camrod_planning', os.path.join('config', 'planning_state_machine.yaml')),
        description='Parameters for /planning/planning_state_machine',
    )
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_path_default,
        description='Lanelet2 map path for planning cost grids (optional override)',
    )
    origin_lat_arg = DeclareLaunchArgument('origin_lat', default_value=origin_lat_default)
    origin_lon_arg = DeclareLaunchArgument('origin_lon', default_value=origin_lon_default)
    origin_alt_arg = DeclareLaunchArgument('origin_alt', default_value=origin_alt_default)
    nav2_robot_base_frame_arg = DeclareLaunchArgument(
        'nav2_robot_base_frame',
        # Keep Nav2 base frame aligned with the platform TF tree.
        default_value='robot_base_link',
        description='Robot base frame consumed by Nav2 stack',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='planning',
        description='Namespace for planning module nodes',
    )
    centerline_input_pose_topic_arg = DeclareLaunchArgument(
        'centerline_input_pose_topic',
        default_value='/localization/pose',
        description='Input pose topic for centerline snapper',
    )
    local_path_pose_topic_arg = DeclareLaunchArgument(
        'local_path_pose_topic',
        # HH_260331: Use fused localization pose by default to keep controller-local
        # path transform aligned with actual vehicle heading.
        default_value='/localization/pose',
        description='Pose topic consumed by /planning/local_path_extractor',
    )
    local_path_source_arg = DeclareLaunchArgument(
        'local_path_source',
        # HH_260331: Default to controller-local trajectory first so local path
        # reflects real-time local costmap behavior (with global-slice fallback).
        default_value='controller_then_slice',
        description='Local path source policy: controller_then_slice|controller_only|slice_only',
    )
    enable_tracking_error_arg = DeclareLaunchArgument(
        'enable_tracking_error',
        default_value='true',
        description='Enable /planning/path_tracking_error (lateral/heading deviation publisher)',
    )
    tracking_error_topic_arg = DeclareLaunchArgument(
        'tracking_error_topic',
        # HH_260330: Keep requested topic name for compatibility with current tooling.
        default_value='/planning/ltracking_error',
        description='Tracking error output topic',
    )
    cmd_vel_gate_enable_arg = DeclareLaunchArgument(
        'cmd_vel_gate_enable',
        # HH_260331: Require explicit engage trigger before publishing /planning/cmd_vel.
        default_value='true',
        description='Enable planning cmd_vel gate node',
    )
    cmd_vel_raw_topic_arg = DeclareLaunchArgument(
        'cmd_vel_raw_topic',
        default_value='/planning/cmd_vel_raw',
        description='Raw cmd_vel topic from Nav2 controller',
    )
    cmd_vel_output_topic_arg = DeclareLaunchArgument(
        'cmd_vel_output_topic',
        default_value='/planning/cmd_vel',
        description='Final gated cmd_vel topic for platform',
    )
    planning_engage_topic_arg = DeclareLaunchArgument(
        'planning_engage_topic',
        default_value='/planning/engage',
        description='Planning engage trigger topic (std_msgs/Bool)',
    )
    planning_engaged_state_topic_arg = DeclareLaunchArgument(
        'planning_engaged_state_topic',
        default_value='/planning/engaged',
        description='Planning engage state output topic (std_msgs/Bool)',
    )
    cmd_vel_gate_use_estop_topic_arg = DeclareLaunchArgument(
        'cmd_vel_gate_use_estop_topic',
        default_value='true',
        description='Use estop topic in planning cmd_vel gate',
    )
    cmd_vel_gate_estop_topic_arg = DeclareLaunchArgument(
        'cmd_vel_gate_estop_topic',
        default_value='/planning/state_machine/estop',
        description='Planning estop topic (std_msgs/Bool)',
    )
    cmd_vel_gate_allow_on_start_arg = DeclareLaunchArgument(
        'cmd_vel_gate_allow_on_start',
        default_value='false',
        description='If true, planning cmd_vel gate starts enabled without engage trigger',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system validator nodes',
    )

    nav2_base_param = LaunchConfiguration('nav2_base_param_file')
    nav2_vehicle_param = LaunchConfiguration('nav2_vehicle_param_file')
    nav2_lanelet_param = LaunchConfiguration('nav2_lanelet_param_file')
    nav2_behavior_param = LaunchConfiguration('nav2_behavior_param_file')
    enable_path_cost_grids = LaunchConfiguration('enable_path_cost_grids')
    enable_goal_replanner = LaunchConfiguration('enable_goal_replanner')
    local_path_extractor_param = LaunchConfiguration('local_path_extractor_param_file')
    map_path = LaunchConfiguration('map_path')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    nav2_robot_base_frame = LaunchConfiguration('nav2_robot_base_frame')
    enable_module_validator = LaunchConfiguration('enable_module_validator')
    enable_nav2_lifecycle_retry = LaunchConfiguration('enable_nav2_lifecycle_retry')
    require_localization_ready = LaunchConfiguration('require_localization_ready')
    enable_state_machine = LaunchConfiguration('enable_state_machine')
    planning_state_machine_param = LaunchConfiguration('planning_state_machine_param_file')
    module_namespace = LaunchConfiguration('module_namespace')
    centerline_input_pose_topic = LaunchConfiguration('centerline_input_pose_topic')
    local_path_pose_topic = LaunchConfiguration('local_path_pose_topic')
    local_path_source = LaunchConfiguration('local_path_source')
    enable_tracking_error = LaunchConfiguration('enable_tracking_error')
    tracking_error_topic = LaunchConfiguration('tracking_error_topic')
    cmd_vel_gate_enable = LaunchConfiguration('cmd_vel_gate_enable')
    cmd_vel_raw_topic = LaunchConfiguration('cmd_vel_raw_topic')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')
    planning_engage_topic = LaunchConfiguration('planning_engage_topic')
    planning_engaged_state_topic = LaunchConfiguration('planning_engaged_state_topic')
    cmd_vel_gate_use_estop_topic = LaunchConfiguration('cmd_vel_gate_use_estop_topic')
    cmd_vel_gate_estop_topic = LaunchConfiguration('cmd_vel_gate_estop_topic')
    cmd_vel_gate_allow_on_start = LaunchConfiguration('cmd_vel_gate_allow_on_start')
    system_namespace = LaunchConfiguration('system_namespace')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camrod_planning', 'launch/nav2_lanelet.launch.py')),
        launch_arguments={
            'nav2_base_param_file': nav2_base_param,
            'nav2_vehicle_param_file': nav2_vehicle_param,
            'nav2_lanelet_param_file': nav2_lanelet_param,
            'nav2_behavior_param_file': nav2_behavior_param,
            'enable_path_cost_grids': enable_path_cost_grids,
            'map_path': map_path,
            'origin_lat': origin_lat,
            'origin_lon': origin_lon,
            'origin_alt': origin_alt,
            'nav2_robot_base_frame': nav2_robot_base_frame,
            'module_namespace': module_namespace,
            # HH_260327: Disable lifecycle autostart when localization-ready gating is enabled.
            'nav2_autostart': PythonExpression([
                "'false' if ('", require_localization_ready,
                "' in ['true', 'True', '1']) else 'true'"
            ]),
        }.items(),
    )
    # HH_260317-00:00 Start Nav2 directly.
    # TimerAction-based delayed include caused LaunchConfiguration resolution errors
    # in nested bringup include contexts, which prevented Nav2 nodes from launching.
    nav2_launch_delayed = nav2_launch

    # HH_260126 Goal snapper: /goal_pose -> /planning/goal_pose_snapped (lanelet centerline).
    goal_snapper = Node(
        package='camrod_planning',
        executable='goal_snapper_node',
        name='goal_snapper',
        namespace=module_namespace,
        output='screen',
        parameters=[
            pkg_share('camrod_planning', os.path.join('config', 'goal_snapper.yaml')),
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'input_goal_topic': '/goal_pose',
                # HH_260316-00:00 Keep raw and snapped goal streams separated.
                # This prevents direct raw-goal injection into Nav2/path-cost grids.
                'output_goal_topic': '/planning/goal_pose_snapped',
                # HH_260317-00:00 ROS-native snapped goal for Nav2 simple-goal input.
                'output_goal_topic_ros': '/planning/goal_pose_snapped_ros',
                # HH_260331: Keep snapped-goal Z aligned with RViz goal input (z=0)
                # so goal markers remain visible in 2D map view.
                'use_map_z': False,
                'flatten_to_ground': False,
            },
        ],
    )

    # HH_260126 Centerline snapper: /localization/pose -> /planning/lanelet_pose.
    centerline_snapper = Node(
        package='camrod_planning',
        executable='centerline_snapper_node',
        name='centerline_snapper',
        namespace=module_namespace,
        output='screen',
        parameters=[
            pkg_share('camrod_planning', os.path.join('config', 'centerline_snapper.yaml')),
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'input_pose_topic': centerline_input_pose_topic,
                'output_pose_topic': '/planning/lanelet_pose',
                'output_pose_topic_ros': '/planning/lanelet_pose_ros',
            },
        ],
    )

    # 2026-02-27: Stable local path publisher derived from global path + current pose.
    # Avoids local path delay/holes when controller-local trajectory is intermittent.
    local_path_extractor = Node(
        package='camrod_planning',
        executable='local_path_extractor_node',
        name='local_path_extractor',
        namespace=module_namespace,
        output='screen',
        parameters=[
            local_path_extractor_param,
            {
                # HH_260316-00:00 Hard-pin core topics to lanelet-snapped flow.
                # This prevents stale/partial YAML installs from falling back to
                # /localization/pose and producing branch/shortcut local paths.
                # Use Nav2 planner output directly as global route source.
                'global_path_topic': '/planning/global_path',
                # HH_260327: Use localization-driven lanelet pose as local-path anchor.
                # This keeps a single map-frame trajectory source for planning while
                # GNSS remains a reference input inside localization filters.
                'pose_topic': local_path_pose_topic,
                'output_topic': '/planning/local_path',
                # HH_260327: Launch-configurable local path source policy.
                'local_path_source': local_path_source,
                'controller_path_topic': '/planning/local_path_controller',
                'controller_path_timeout_sec': 0.8,
            },
        ],
    )

    # HH_260330: Publish pose-vs-path lateral/heading deviation for debugging and control monitoring.
    path_tracking_error = Node(
        package='camrod_planning',
        executable='path_tracking_error_node',
        name='path_tracking_error',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'pose_topic': local_path_pose_topic,
            'local_path_topic': '/planning/local_path',
            'global_path_topic': '/planning/global_path',
            'output_topic': tracking_error_topic,
            'prefer_local_path': True,
            'pose_timeout_sec': 1.0,
            'publish_rate_hz': 15.0,
            'publish_on_input_update': True,
        }],
        condition=IfCondition(enable_tracking_error),
    )

    # HH_260331: Gate Nav2 raw cmd_vel by explicit /planning/engage trigger.
    planning_cmd_vel_gate = Node(
        package='camrod_planning',
        executable='planning_cmd_vel_gate_node.py',
        name='cmd_vel_gate',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'input_topic': cmd_vel_raw_topic,
            'output_topic': cmd_vel_output_topic,
            'engage_topic': planning_engage_topic,
            'state_topic': planning_engaged_state_topic,
            'use_estop_topic': cmd_vel_gate_use_estop_topic,
            'estop_topic': cmd_vel_gate_estop_topic,
            'allow_on_start': cmd_vel_gate_allow_on_start,
            'publish_zero_when_blocked': True,
        }],
        condition=IfCondition(cmd_vel_gate_enable),
    )

    # 2026-02-23: Keep ComputePathToPose action updated as goal/current pose changes.
    goal_replanner = Node(
        package='camrod_planning',
        executable='goal_replanner_node',
        name='goal_replanner',
        namespace=module_namespace,
        output='screen',
        parameters=[
            pkg_share('camrod_planning', os.path.join('config', 'goal_replanner.yaml')),
            {
                # HH_260306-00:00 Launch-level hard override for stable event-driven behavior.
                # This prevents stale installed YAML/default mismatches from re-enabling timeout/retry storms.
                'replan_rate_hz': 0.0,
                'request_timeout_sec': 0.0,
                'retry_after_failure_sec': 0.8,
                'immediate_replan_on_goal': True,
                'immediate_replan_on_start': False,
                'replan_on_start_change': False,
                'stop_replan_after_goal_reached': True,
                'goal_reached_distance_m': 0.8,
                # HH_260317-00:00 Keep goal-snap ComputePath available even when
                # NavigateToPose action is active, so newly clicked goals can refresh
                # the snapped planning path immediately.
                # HH_260318-00:00 Avoid ComputePath action contention with bt_navigator.
                'pause_when_navigate_active': True,
                'navigate_status_topic': '/planning/navigate_to_pose/_action/status',
                'min_request_interval_sec': 0.25,
                'enable_periodic_replan': False,
                # Keep replanner output on a dedicated topic.
                # Canonical /planning/global_path is owned by Nav2 planner_server.
                'publish_result_path': True,
                'output_path_topic': '/planning/global_path_replanner',
                # HH_260311-00:00 Allow one-step TF fallback when snapped start is temporarily unavailable.
                'start_topic_fallback_to_tf': True,
            },
        ],
        # HH_260317-00:00 Launch gate:
        # use native IfCondition so values like "true"/"True"/"1" are all accepted.
        condition=IfCondition(enable_goal_replanner),
    )

    planning_state_machine = Node(
        package='camrod_planning',
        executable='planning_state_machine_node.py',
        name='planning_state_machine',
        namespace=module_namespace,
        output='screen',
        parameters=[planning_state_machine_param],
        condition=IfCondition(enable_state_machine),
    )

    # HH_260326: Removed planning status/validator runtime nodes as requested.

    nav2_lifecycle_retry = Node(
        package='camrod_planning',
        executable='nav2_lifecycle_startup_retry_node.py',
        name='nav2_lifecycle_startup_retry',
        namespace=module_namespace,
        output='screen',
        condition=IfCondition(PythonExpression([
            "('", enable_nav2_lifecycle_retry, "' in ['true', 'True', '1']) or "
            "('", require_localization_ready, "' in ['true', 'True', '1'])"
        ])),
        parameters=[{
            'manager_service': '/planning/lifecycle_manager_planning/manage_nodes',
            'is_active_service': '/planning/lifecycle_manager_planning/is_active',
            'retry_period_s': 0.5,
            'startup_cooldown_s': 1.0,
            # HH_260327: Localize planning startup to localization readiness signals.
            'require_localization_ready': require_localization_ready,
            'localization_ready_topic': '/localization/initial_match_ok',
            'localization_pose_cov_topic': '/localization/pose_with_covariance',
            'localization_pose_timeout_s': 1.0,
            'max_position_variance': 9.0,
            'max_yaw_variance': 1.0,
        }],
    )

    return LaunchDescription([
        nav2_base_param_arg,
        nav2_vehicle_param_arg,
        nav2_lanelet_param_arg,
        nav2_behavior_param_arg,
        enable_path_cost_grids_arg,
        enable_goal_replanner_arg,
        enable_module_validator_arg,
        enable_nav2_lifecycle_retry_arg,
        require_localization_ready_arg,
        enable_state_machine_arg,
        local_path_extractor_param_arg,
        planning_state_machine_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        nav2_robot_base_frame_arg,
        module_namespace_arg,
        centerline_input_pose_topic_arg,
        local_path_pose_topic_arg,
        local_path_source_arg,
        enable_tracking_error_arg,
        tracking_error_topic_arg,
        cmd_vel_gate_enable_arg,
        cmd_vel_raw_topic_arg,
        cmd_vel_output_topic_arg,
        planning_engage_topic_arg,
        planning_engaged_state_topic_arg,
        cmd_vel_gate_use_estop_topic_arg,
        cmd_vel_gate_estop_topic_arg,
        cmd_vel_gate_allow_on_start_arg,
        system_namespace_arg,
        goal_snapper,
        centerline_snapper,
        local_path_extractor,
        path_tracking_error,
        planning_cmd_vel_gate,
        goal_replanner,
        planning_state_machine,
        nav2_lifecycle_retry,
        nav2_launch_delayed,
    ])
