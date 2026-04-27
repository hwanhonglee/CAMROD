from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),
        DeclareLaunchArgument('enable_nav2_lifecycle_retry', default_value='false'),
        DeclareLaunchArgument('require_localization_ready', default_value='false'),

        Node(
            package='camrod_planning',
            executable='nav2_lifecycle_startup_retry_node',
            name='nav2_lifecycle_startup_retry',
            namespace=LaunchConfiguration('module_namespace'),
            output='screen',
            condition=IfCondition(PythonExpression([
                "('", LaunchConfiguration('enable_nav2_lifecycle_retry'), "' in ['true', 'True', '1']) or "
                "('", LaunchConfiguration('require_localization_ready'), "' in ['true', 'True', '1'])"
            ])),
            parameters=[{
                'manager_service': '/planning/lifecycle_manager_planning/manage_nodes',
                'is_active_service': '/planning/lifecycle_manager_planning/is_active',
                'retry_period_s': 0.5,
                'startup_cooldown_s': 1.0,
                'require_localization_ready': LaunchConfiguration('require_localization_ready'),
                'localization_ready_topic': '/localization/initial_match_ok',
                'localization_pose_cov_topic': '/localization/pose_with_covariance',
                'localization_pose_timeout_s': 1.0,
                'max_position_variance': 9.0,
                'max_yaw_variance': 1.0,
            }],
        ),
    ])
