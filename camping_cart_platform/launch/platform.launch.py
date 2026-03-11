import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Map frame id',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='robot_base_link',
        description='Base frame id',
    )
    # HH_260220: Keep sensor kit anchors under robot_base_link.
    sensor_kit_base_frame_arg = DeclareLaunchArgument(
        'sensor_kit_base_frame_id',
        default_value='sensor_kit_base_link',
        description='Sensor kit base frame id',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'sensor_kit', 'robot_params.yaml')),
        description='Sensor kit robot params',
    )
    robot_viz_params_arg = DeclareLaunchArgument(
        'robot_visualization_param_file',
        default_value=pkg_share('camping_cart_bringup', os.path.join('config', 'platform', 'robot_visualization.yaml')),
        description='Robot visualization parameters (platform namespace)',
    )
    base_link_alias_arg = DeclareLaunchArgument(
        'publish_base_link_alias',
        default_value='true',
        description='Publish static TF alias robot_base_link -> base_link for legacy consumers',
    )
    enable_module_checker_arg = DeclareLaunchArgument(
        'enable_module_checker',
        default_value='true',
        description='Enable platform module checker/diagnostics publisher',
    )
    enable_diagnostic_arg = DeclareLaunchArgument(
        'platform_enable_diagnostic',
        default_value='true',
        description='Enable /platform/diagnostic publisher',
    )
    diagnostic_publish_period_arg = DeclareLaunchArgument(
        'platform_diagnostic_publish_period_s',
        default_value='0.2',
        description='Publish period for /platform/diagnostic',
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camping_cart_sensor_kit', 'launch/sensor_kit.launch.py')),
        launch_arguments={
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'sensor_kit_base_frame_id': LaunchConfiguration('sensor_kit_base_frame_id'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # 2026-02-24: Launch robot visualization under platform module ownership.
    robot_visualization = Node(
        package='camping_cart_platform',
        executable='robot_visualization_node',
        name='robot_visualization',
        namespace='platform',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            LaunchConfiguration('robot_visualization_param_file'),
            {
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
            },
        ],
    )

    # HH_260307-00:00 Compatibility alias for Nav2 recovery / plugins that still request "base_link".
    # Keeps the canonical frame as robot_base_link while preventing lookup failures.
    base_link_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_alias_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_base_link_alias')),
    )

    platform_checker = Node(
        package='camping_cart_system',
        executable='module_checker_node.py',
        name='platform_checker',
        namespace='system',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_module_checker')),
        parameters=[{
            'module_name': 'platform',
            'required_nodes': [
                '/platform/robot_visualization',
                '/sensor_kit/robot_state_publisher',
            ],
            'required_topics': [
                '/tf',
                '/tf_static',
            ],
            'health_topic': '/platform/healthchecker',
            'check_period_s': 0.5,
            'warn_throttle_sec': 2.0,
            'publish_ok': True,
        }],
    )

    platform_diagnostic = Node(
        package='camping_cart_platform',
        executable='platform_diagnostic_node.py',
        name='platform_diagnostic',
        namespace='platform',
        output='screen',
        condition=IfCondition(LaunchConfiguration('platform_enable_diagnostic')),
        parameters=[{
            'msgs_topic': '/platform/messages',
            'diagnostic_topic': '/platform/diagnostic',
            'publish_period_s': LaunchConfiguration('platform_diagnostic_publish_period_s'),
        }],
    )

    return LaunchDescription([
        map_frame_arg,
        base_frame_arg,
        sensor_kit_base_frame_arg,
        params_arg,
        robot_viz_params_arg,
        base_link_alias_arg,
        enable_module_checker_arg,
        enable_diagnostic_arg,
        diagnostic_publish_period_arg,
        sensor_launch,
        robot_visualization,
        base_link_alias,
        platform_checker,
        platform_diagnostic,
    ])
