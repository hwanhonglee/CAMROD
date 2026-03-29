from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# HH_260327: Generic module checker launcher used by bringup.
def _checker(
    module_name,
    required_nodes,
    required_topics,
    required_lifecycle_nodes=None,
    condition=None,
    namespace=None,
    startup_grace_sec=6.0,
):
    params = {
        'module_name': module_name,
        'required_nodes': required_nodes,
        'required_topics': required_topics,
        'diagnostic_topic': '/diagnostics',
        'status_name': f'{module_name}/checker',
        'check_period_s': 0.5,
        'warn_throttle_sec': 2.0,
        'startup_grace_sec': float(startup_grace_sec),
        'publish_ok': True,
    }
    if required_lifecycle_nodes:
        params['required_lifecycle_nodes'] = required_lifecycle_nodes

    return Node(
        package='camrod_system',
        executable='module_checker_node.py',
        name=f'{module_name}_checker',
        namespace=namespace,
        output='screen',
        condition=condition,
        parameters=[params],
    )


def generate_launch_description():
    enable_checkers_arg = DeclareLaunchArgument(
        'enable_checkers',
        default_value='true',
        description='Launch per-module checker nodes',
    )
    enable_diagnostics_aggregator_arg = DeclareLaunchArgument(
        'enable_diagnostics_aggregator',
        default_value='true',
        description='Launch diagnostics aggregator (/diagnostics -> /diagnostics_agg)',
    )
    enable_system_diagnostic_arg = DeclareLaunchArgument(
        'enable_system_diagnostic',
        default_value='false',
        description='Launch system diagnostic summary node',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system checker/diagnostic nodes',
    )

    enable_checkers = LaunchConfiguration('enable_checkers')
    enable_diagnostics_aggregator = LaunchConfiguration('enable_diagnostics_aggregator')
    enable_system_diagnostic = LaunchConfiguration('enable_system_diagnostic')
    system_namespace = LaunchConfiguration('system_namespace')

    map_checker = _checker(
        module_name='map',
        required_nodes=[
            '/map/lanelet2_map',
            '/map/cost_grid_map',
            '/map/cost_grid_planning_base',
        ],
        required_topics=[
            '/map/cost_grid/lanelet',
            '/map/cost_grid/planning_base',
        ],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    sensing_checker = _checker(
        module_name='sensing',
        required_nodes=[
            '/sensing/lidar/lidar_preprocessor',
            '/sensing/camera/camera_preprocessor',
            '/sensing/imu/platform_velocity_converter',
        ],
        required_topics=[
            '/sensing/lidar/near_cost_grid',
            '/sensing/radar/near_cost_grid',
        ],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    localization_checker = _checker(
        module_name='localization',
        required_nodes=[
            '/localization/navsat_to_pose',
            '/localization/drop_zone_matcher',
        ],
        required_topics=[
            '/localization/pose',
            '/localization/pose_with_covariance',
        ],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    planning_checker = _checker(
        module_name='planning',
        required_nodes=[
            '/planning/planner_server',
            '/planning/controller_server',
            '/planning/behavior_server',
            '/planning/bt_navigator',
            '/planning/local_path_extractor',
            '/planning/goal_snapper',
            '/planning/centerline_snapper',
        ],
        required_topics=[
            '/planning/global_path',
            '/planning/local_path',
            '/planning/cost_grid/global_path_markers',
            '/planning/cost_grid/local_path_markers',
        ],
        required_lifecycle_nodes=[
            '/planning/planner_server',
            '/planning/controller_server',
            '/planning/behavior_server',
            '/planning/bt_navigator',
        ],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    platform_checker = _checker(
        module_name='platform',
        required_nodes=[
            '/platform/robot_visualization',
            '/platform/cmd_vel_gate',
            '/sensor_kit/robot_state_publisher',
        ],
        required_topics=[
            '/platform/drive_enabled',
            '/tf',
            '/tf_static',
        ],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
        startup_grace_sec=12.0,
    )

    perception_checker = _checker(
        module_name='perception',
        required_nodes=['/perception/obstacle_fusion'],
        required_topics=['/perception/obstacles'],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    sensor_kit_checker = _checker(
        module_name='sensor_kit',
        required_nodes=['/sensor_kit/robot_state_publisher'],
        required_topics=['/tf_static'],
        condition=IfCondition(enable_checkers),
        namespace=system_namespace,
    )

    diagnostics_aggregator = Node(
        package='camrod_system',
        executable='diagnostics_aggregator_node.py',
        name='diagnostics_aggregator',
        namespace=system_namespace,
        output='screen',
        condition=IfCondition(enable_diagnostics_aggregator),
        parameters=[{
            'source_topic': '/diagnostics',
            'output_topic': '/diagnostics_agg',
            'publish_period_s': 1.0,
            'stale_timeout_s': 3.0,
        }],
    )

    system_diagnostic = Node(
        package='camrod_system',
        executable='system_diagnostic_node.py',
        name='system_diagnostic',
        namespace=system_namespace,
        output='screen',
        condition=IfCondition(enable_system_diagnostic),
        parameters=[{
            'diagnostic_topic': '/diagnostics',
            'source_diagnostic_topic': '/diagnostics',
            'publish_period_s': 0.5,
            'known_modules': [
                'map',
                'sensing',
                'localization',
                'planning',
                'platform',
                'perception',
                'sensor_kit',
                'system',
            ],
        }],
    )

    return LaunchDescription([
        enable_checkers_arg,
        enable_diagnostics_aggregator_arg,
        enable_system_diagnostic_arg,
        system_namespace_arg,
        map_checker,
        sensing_checker,
        localization_checker,
        planning_checker,
        platform_checker,
        perception_checker,
        sensor_kit_checker,
        diagnostics_aggregator,
        system_diagnostic,
    ])
