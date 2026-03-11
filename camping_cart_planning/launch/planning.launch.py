import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition


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

    nav2_base_param_arg = DeclareLaunchArgument(
        'nav2_base_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_base.yaml')),
        description='Nav2 base profile parameter file',
    )
    nav2_vehicle_param_arg = DeclareLaunchArgument(
        'nav2_vehicle_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_vehicle.yaml')),
        description='Nav2 vehicle profile parameter file',
    )
    nav2_lanelet_param_arg = DeclareLaunchArgument(
        'nav2_lanelet_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_lanelet_overlay.yaml')),
        description='Nav2 lanelet profile parameter file',
    )
    nav2_behavior_param_arg = DeclareLaunchArgument(
        'nav2_behavior_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'nav2_behavior.yaml')),
        description='Nav2 behavior profile parameter file',
    )
    enable_path_cost_grids_arg = DeclareLaunchArgument(
        'enable_path_cost_grids',
        default_value='true',
        description='Enable planning path-cost-grid helper nodes (/planning/cost_grid/*)',
    )
    enable_goal_replanner_arg = DeclareLaunchArgument(
        'enable_goal_replanner',
        # HH_260309-00:00 Enable by default so RViz 2D Goal Pose (/goal_pose -> /planning/goal_pose)
        # immediately triggers ComputePath and refreshes global/local path-cost grids.
        # Action contention with NavigateToPose is gated in goal_replanner.yaml
        # via pause_when_navigate_active=true.
        default_value='true',
        description='Enable /planning/goal_replanner (ComputePathToPose helper)',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable planning module checker/diagnostics publisher',
    )
    enable_state_machine_arg = DeclareLaunchArgument(
        'enable_state_machine',
        default_value='true',
        description='Enable /planning/planning_state_machine (system-status driven planning policy)',
    )
    enable_diagnostic_arg = DeclareLaunchArgument(
        'planning_enable_diagnostic',
        default_value='true',
        description='Enable /planning/diagnostic publisher',
    )
    diagnostic_publish_period_arg = DeclareLaunchArgument(
        'planning_diagnostic_publish_period_s',
        default_value='0.2',
        description='Publish period for /planning/diagnostic',
    )
    local_path_extractor_param_arg = DeclareLaunchArgument(
        'local_path_extractor_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'local_path_extractor.yaml')),
        description='Parameters for /planning/local_path_extractor',
    )
    planning_state_machine_param_arg = DeclareLaunchArgument(
        'planning_state_machine_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'planning_state_machine.yaml')),
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
    enable_module_checker = LaunchConfiguration('enable_module_checker')
    enable_state_machine = LaunchConfiguration('enable_state_machine')
    enable_diagnostic = LaunchConfiguration('planning_enable_diagnostic')
    diagnostic_publish_period = LaunchConfiguration('planning_diagnostic_publish_period_s')
    planning_state_machine_param = LaunchConfiguration('planning_state_machine_param_file')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camping_cart_planning', 'launch/nav2_lanelet.launch.py')),
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

    # 2026-02-27: Stable local path publisher derived from global path + current pose.
    # Avoids local path delay/holes when controller-local trajectory is intermittent.
    local_path_extractor = Node(
        package='camping_cart_planning',
        executable='local_path_extractor_node',
        name='local_path_extractor',
        namespace='planning',
        output='screen',
        parameters=[local_path_extractor_param],
    )

    # 2026-02-23: Keep ComputePathToPose action updated as goal/current pose changes.
    goal_replanner = Node(
        package='camping_cart_planning',
        executable='goal_replanner_node',
        name='goal_replanner',
        namespace='planning',
        output='screen',
        parameters=[
            pkg_share('camping_cart_bringup', os.path.join('config', 'planning', 'goal_replanner.yaml')),
            {
                # HH_260306-00:00 Launch-level hard override for stable event-driven behavior.
                # This prevents stale installed YAML/default mismatches from re-enabling timeout/retry storms.
                'replan_rate_hz': 0.0,
                'request_timeout_sec': 0.0,
                'retry_after_failure_sec': 0.8,
                'immediate_replan_on_goal': True,
                'immediate_replan_on_start': True,
                'replan_on_start_change': False,
                'stop_replan_after_goal_reached': True,
                'goal_reached_distance_m': 0.8,
                # HH_260309-00:00 Prevent action contention with bt_navigator's own ComputePath.
                'pause_when_navigate_active': True,
                'navigate_status_topic': '/planning/navigate_to_pose/_action/status',
                'min_request_interval_sec': 0.25,
                'enable_periodic_replan': False,
                'publish_result_path': True,
                'output_path_topic': '/planning/global_path',
                # HH_260307-00:00 Use snapped start (/planning/lanelet_pose) only.
                # TF fallback can place start in lethal cell and cause repeated aborts.
                'start_topic_fallback_to_tf': False,
            },
        ],
        # HH_260307-00:00 Strict gate: launch only when explicitly true.
        condition=IfCondition(
            PythonExpression(["'", enable_goal_replanner, "' == 'true'"])
        ),
    )

    planning_state_machine = Node(
        package='camping_cart_planning',
        executable='planning_state_machine_node.py',
        name='planning_state_machine',
        namespace='planning',
        output='screen',
        parameters=[planning_state_machine_param],
        condition=IfCondition(enable_state_machine),
    )

    planning_checker = Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name='planning_checker',
        namespace='system',
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
                '/planning/planning_state_machine',
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
            'health_topic': '/planning/healthchecker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    planning_diagnostic = Node(
        package='camping_cart_planning',
        executable='planning_diagnostic_node.py',
        name='planning_diagnostic',
        namespace='planning',
        output='screen',
        condition=IfCondition(enable_diagnostic),
        parameters=[{
            'msgs_topic': '/planning/messages',
            'diagnostic_topic': '/planning/diagnostic',
            'publish_period_s': diagnostic_publish_period,
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
        enable_state_machine_arg,
        enable_diagnostic_arg,
        diagnostic_publish_period_arg,
        local_path_extractor_param_arg,
        planning_state_machine_param_arg,
        map_path_arg,
        origin_lat_arg,
        origin_lon_arg,
        origin_alt_arg,
        goal_snapper,
        centerline_snapper,
        local_path_extractor,
        goal_replanner,
        planning_state_machine,
        planning_checker,
        planning_diagnostic,
        nav2_launch,
    ])
