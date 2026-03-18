import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition


# Implements `pkg_share` behavior.
def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    # HH_260126 Load defaults from bringup map_info.yaml for map_path/offsets.
    map_info_path = pkg_share('camrod_bringup', os.path.join('config', 'map', 'map_info.yaml'))
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

    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'nav2_base.yaml')),
        description='Nav2 base profile parameter file',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'nav2_vehicle.yaml')),
        description='Nav2 vehicle profile parameter file',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'nav2_lanelet_overlay.yaml')),
        description='Nav2 lanelet profile parameter file',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'nav2_behavior.yaml')),
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
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable planning module checker publisher',
    )
    enable_nav2_lifecycle_retry_arg = DeclareLaunchArgument(
        'enable_nav2_lifecycle_retry',
        # Keep OFF by default to avoid lifecycle transition races.
        # Enable only when you explicitly want retry behavior in unstable environments.
        default_value='false',
        description='Enable Nav2 lifecycle startup retry helper',
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
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'local_path_extractor.yaml')),
        description='Parameters for /planning/local_path_extractor',
    )
    planning_state_machine_param_arg = DeclareLaunchArgument(
        'planning_state_machine_param_file',
        default_value=pkg_share('camrod_bringup', os.path.join('config', 'planning', 'planning_state_machine.yaml')),
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
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system checker nodes',
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
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_nav2_lifecycle_retry = LaunchConfiguration('enable_nav2_lifecycle_retry')
    enable_state_machine = LaunchConfiguration('enable_state_machine')
    planning_state_machine_param = LaunchConfiguration('planning_state_machine_param_file')
    module_namespace = LaunchConfiguration('module_namespace')
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
            pkg_share('camrod_bringup', os.path.join('config', 'planning', 'goal_snapper.yaml')),
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
            pkg_share('camrod_bringup', os.path.join('config', 'planning', 'centerline_snapper.yaml')),
            {
                'map_path': map_path,
                'offset_lat': origin_lat,
                'offset_lon': origin_lon,
                'offset_alt': origin_alt,
                'input_pose_topic': '/localization/pose',
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
                'pose_topic': '/planning/lanelet_pose',
                'output_topic': '/planning/local_path',
            },
        ],
    )

    # 2026-02-23: Keep ComputePathToPose action updated as goal/current pose changes.
    goal_replanner = Node(
        package='camrod_planning',
        executable='goal_replanner_node',
        name='goal_replanner',
        namespace=module_namespace,
        output='screen',
        parameters=[
            pkg_share('camrod_bringup', os.path.join('config', 'planning', 'goal_replanner.yaml')),
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

    planning_checker = Node(
        package='camrod_system',
        executable='module_checker_node.py',
        name='planning_checker',
        namespace=system_namespace,
        output='screen',
        condition=IfCondition(enable_module_checker),
        parameters=[{
            'module_name': 'planning',
            'required_nodes': [
                '/planning/planner_server',
                '/planning/controller_server',
                '/planning/behavior_server',
                '/planning/bt_navigator',
                '/planning/local_path_extractor',
            ],
            'required_topics': [
                '/planning/global_path',
                '/planning/local_path',
                '/planning/cost_grid/global_path_markers',
                '/planning/cost_grid/local_path_markers',
            ],
            'required_lifecycle_nodes': [
                '/planning/planner_server',
                '/planning/controller_server',
                '/planning/behavior_server',
                '/planning/bt_navigator',
            ],
            'diagnostic_topic': '/diagnostics',
            'status_name': 'planning/checker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    planning_diagnostic = Node(
        package='camrod_planning',
        executable='planning_diagnostic_node.py',
        name='planning_diagnostic',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            # HH_260318-00:00 Module-local diagnostic topic (namespaced).
            'diagnostic_topic': 'diagnostic',
            'publish_period_s': 0.2,
            'stale_timeout_s': 2.0,
            'topic_goal': '/planning/goal_pose_snapped',
            'topic_lanelet_pose': '/planning/lanelet_pose',
            'topic_global_path': '/planning/global_path',
            'topic_navigate_status': '/planning/navigate_to_pose/_action/status',
            'topic_local_path': '/planning/local_path',
            'topic_local_plan_raw': '/planning/local_plan',
            'topic_local_path_controller': '/planning/local_path_controller',
            'topic_local_path_dwb': '/planning/local_path_dwb',
            'topic_global_costmap': '/planning/global_costmap/costmap',
            'topic_local_costmap': '/planning/local_costmap/costmap',
            'topic_global_cost_markers': '/planning/cost_grid/global_path_markers',
            'topic_local_cost_markers': '/planning/cost_grid/local_path_markers',
        }],
    )

    nav2_lifecycle_retry = Node(
        package='camrod_planning',
        executable='nav2_lifecycle_startup_retry_node.py',
        name='nav2_lifecycle_startup_retry',
        namespace=module_namespace,
        output='screen',
        condition=IfCondition(enable_nav2_lifecycle_retry),
        parameters=[{
            'manager_service': '/planning/lifecycle_manager_planning/manage_nodes',
            'is_active_service': '/planning/lifecycle_manager_planning/is_active',
            'retry_period_s': 0.5,
            'startup_cooldown_s': 1.0,
        }],
    )

    return LaunchDescription([
        nav2_base_param_arg,
        nav2_vehicle_param_arg,
        nav2_lanelet_param_arg,
        nav2_behavior_param_arg,
        enable_path_cost_grids_arg,
        enable_goal_replanner_arg,
        enable_module_checker_arg,
        enable_nav2_lifecycle_retry_arg,
        enable_state_machine_arg,
        local_path_extractor_param_arg,
        planning_state_machine_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        nav2_robot_base_frame_arg,
        module_namespace_arg,
        system_namespace_arg,
        goal_snapper,
        centerline_snapper,
        local_path_extractor,
        goal_replanner,
        planning_state_machine,
        planning_diagnostic,
        nav2_lifecycle_retry,
        planning_checker,
        nav2_launch_delayed,
    ])
