from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _checker(
    module_name,
    required_nodes,
    required_topics,
    required_lifecycle_nodes=None,
    condition=None,
):
    params = {
        'module_name': module_name,
        'required_nodes': required_nodes,
        'required_topics': required_topics,
        'health_topic': f'/{module_name}/healthchecker',
        'check_period_s': 0.5,
        'warn_throttle_sec': 2.0,
        'publish_ok': True,
    }
    if required_lifecycle_nodes:
        params['required_lifecycle_nodes'] = required_lifecycle_nodes

    return Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name=f'{module_name}_checker',
        namespace='system',
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
    enable_aggregator_arg = DeclareLaunchArgument(
        'enable_aggregator',
        default_value='true',
        description='Launch system health aggregator node',
    )
    enable_checkers = LaunchConfiguration('enable_checkers')
    enable_aggregator = LaunchConfiguration('enable_aggregator')

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
            '/map/cost_grid/inflation_markers',
        ],
        condition=IfCondition(enable_checkers),
    )

    sensing_checker = _checker(
        module_name='sensing',
        required_nodes=[
            '/sensing/lidar_preprocessor',
            '/sensing/camera_preprocessor',
            '/sensing/platform_velocity_converter',
        ],
        required_topics=[
            '/sensing/lidar/near_cost_grid',
            '/sensing/radar/near_cost_grid',
        ],
        condition=IfCondition(enable_checkers),
    )

    localization_checker = _checker(
        module_name='localization',
        required_nodes=[
            '/localization/navsat_to_pose',
            '/localization/supervisor',
            '/localization/health_monitor',
        ],
        required_topics=[
            '/localization/pose',
            '/localization/pose_with_covariance',
        ],
        condition=IfCondition(enable_checkers),
    )

    planning_checker = _checker(
        module_name='planning',
        required_nodes=[
            '/planning/planner_server',
            '/planning/controller_server',
            '/planning/behavior_server',
            '/planning/bt_navigator',
            '/planning/local_path_extractor',
            '/planning/goal_replanner',
            '/planning/planning_state_machine',
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
    )

    platform_checker = _checker(
        module_name='platform',
        required_nodes=[
            '/platform/robot_visualization',
            '/sensor_kit/robot_state_publisher',
        ],
        required_topics=[
            '/tf',
            '/tf_static',
        ],
        condition=IfCondition(enable_checkers),
    )

    perception_checker = _checker(
        module_name='perception',
        required_nodes=['/perception/obstacle_fusion'],
        required_topics=['/perception/obstacles'],
        condition=IfCondition(enable_checkers),
    )

    aggregator = Node(
        package='camping_cart_system',
        executable='module_status_aggregator_node.py',
        name='module_status_aggregator',
        namespace='system',
        output='screen',
        condition=IfCondition(enable_aggregator),
        parameters=[{
            'input_topics': [
                '/map/healthchecker',
                '/sensing/healthchecker',
                '/localization/healthchecker',
                '/planning/healthchecker',
                '/platform/healthchecker',
                '/perception/healthchecker',
            ],
            'publish_period_s': 0.5,
            'stale_timeout_s': 3.0,
            'diagnostic_topic': '/system/diagnostic',
            'diagnostics_topic': '/system/diagnostics/modules',
        }],
    )

    return LaunchDescription([
        enable_checkers_arg,
        enable_aggregator_arg,
        map_checker,
        sensing_checker,
        localization_checker,
        planning_checker,
        platform_checker,
        perception_checker,
        aggregator,
    ])
