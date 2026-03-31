import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Implements `pkg_share` behavior.
def pkg_share(pkg, rel):
    return os.path.join(get_package_share_directory(pkg), rel)


# Implements `generate_launch_description` behavior.
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
        # HH_260330: Standalone platform launch uses package-local sensor_kit config by default.
        default_value=pkg_share('camrod_sensor_kit', os.path.join('config', 'robot_params.yaml')),
        description='Sensor kit robot params',
    )
    robot_viz_params_arg = DeclareLaunchArgument(
        'robot_visualization_param_file',
        # HH_260330: Standalone platform launch uses package-local platform config by default.
        default_value=pkg_share('camrod_platform', os.path.join('config', 'robot_visualization.yaml')),
        description='Robot visualization parameters (platform namespace)',
    )
    enable_module_validator_arg = DeclareLaunchArgument(
        'enable_module_validator',
        default_value='true',
        description='Enable platform module validator publisher',
    )
    module_namespace_arg = DeclareLaunchArgument(
        'module_namespace',
        default_value='platform',
        description='Namespace for platform module nodes',
    )
    system_namespace_arg = DeclareLaunchArgument(
        'system_namespace',
        default_value='system',
        description='Namespace for system validator nodes',
    )
    sensor_kit_namespace_arg = DeclareLaunchArgument(
        'sensor_kit_namespace',
        default_value='sensor_kit',
        description='Namespace for sensor_kit launch include',
    )
    cmd_vel_gate_enable_arg = DeclareLaunchArgument(
        'cmd_vel_gate_enable',
        default_value='true',
        description='Enable planning cmd_vel -> platform cmd_vel gate',
    )
    cmd_vel_in_topic_arg = DeclareLaunchArgument(
        'cmd_vel_in_topic',
        default_value='/planning/cmd_vel',
        description='Input cmd_vel topic from planning',
    )
    cmd_vel_out_topic_arg = DeclareLaunchArgument(
        'cmd_vel_out_topic',
        default_value='/platform/cmd_vel',
        description='Gated output cmd_vel topic for platform actuation',
    )
    drive_enable_topic_arg = DeclareLaunchArgument(
        'drive_enable_topic',
        default_value='/platform/drive_enable',
        description='Bool trigger topic to enable drive commands',
    )
    planning_engage_topic_arg = DeclareLaunchArgument(
        'planning_engage_topic',
        default_value='/planning/engage',
        description='Planning engage bool topic (alias of drive enable)',
    )
    use_planning_engage_topic_arg = DeclareLaunchArgument(
        'use_planning_engage_topic',
        default_value='true',
        description='Subscribe to planning engage topic for cmd_vel gating',
    )
    drive_state_topic_arg = DeclareLaunchArgument(
        'drive_state_topic',
        default_value='/platform/drive_enabled',
        description='Bool state topic published by cmd_vel_gate',
    )
    use_estop_topic_arg = DeclareLaunchArgument(
        'use_estop_topic',
        default_value='true',
        description='Use estop topic to force cmd_vel block',
    )
    estop_topic_arg = DeclareLaunchArgument(
        'estop_topic',
        default_value='/planning/state_machine/estop',
        description='Bool estop topic consumed by cmd_vel_gate',
    )
    drive_allow_on_start_arg = DeclareLaunchArgument(
        'drive_allow_on_start',
        default_value='false',
        description='Allow cmd_vel pass-through at startup before explicit enable',
    )
    module_namespace = LaunchConfiguration('module_namespace')
    system_namespace = LaunchConfiguration('system_namespace')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pkg_share('camrod_sensor_kit', 'launch/sensor_kit.launch.py')),
        launch_arguments={
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'sensor_kit_base_frame_id': LaunchConfiguration('sensor_kit_base_frame_id'),
            'params_file': LaunchConfiguration('params_file'),
            'module_namespace': LaunchConfiguration('sensor_kit_namespace'),
            # HH_260326: Disable sensor_kit status publisher from platform launch.
            'enable_status': 'false',
        }.items(),
    )

    # 2026-02-24: Launch robot visualization under platform module ownership.
    robot_visualization = Node(
        package='camrod_platform',
        executable='robot_visualization_node',
        name='robot_visualization',
        namespace=module_namespace,
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

    # HH_260326: Removed platform status/validator runtime nodes as requested.
    cmd_vel_gate = Node(
        package='camrod_platform',
        executable='cmd_vel_gate_node.py',
        name='cmd_vel_gate',
        namespace=module_namespace,
        output='screen',
        parameters=[{
            'input_cmd_vel_topic': LaunchConfiguration('cmd_vel_in_topic'),
            'output_cmd_vel_topic': LaunchConfiguration('cmd_vel_out_topic'),
            'enable_topic': LaunchConfiguration('drive_enable_topic'),
            'engage_topic': LaunchConfiguration('planning_engage_topic'),
            'use_engage_topic': LaunchConfiguration('use_planning_engage_topic'),
            'state_topic': LaunchConfiguration('drive_state_topic'),
            'use_estop_topic': LaunchConfiguration('use_estop_topic'),
            'estop_topic': LaunchConfiguration('estop_topic'),
            'allow_on_start': LaunchConfiguration('drive_allow_on_start'),
            'publish_zero_when_blocked': True,
        }],
        condition=IfCondition(LaunchConfiguration('cmd_vel_gate_enable')),
    )

    return LaunchDescription([
        map_frame_arg,
        base_frame_arg,
        sensor_kit_base_frame_arg,
        params_arg,
        robot_viz_params_arg,
        enable_module_validator_arg,
        module_namespace_arg,
        system_namespace_arg,
        sensor_kit_namespace_arg,
        cmd_vel_gate_enable_arg,
        cmd_vel_in_topic_arg,
        cmd_vel_out_topic_arg,
        drive_enable_topic_arg,
        planning_engage_topic_arg,
        use_planning_engage_topic_arg,
        drive_state_topic_arg,
        use_estop_topic_arg,
        estop_topic_arg,
        drive_allow_on_start_arg,
        robot_visualization,
        cmd_vel_gate,
        # HH_260327: Launch platform-owned nodes before sensor_kit include.
        # `sensor_kit.launch.py` also uses `module_namespace` argument name and can
        # overwrite LaunchConfiguration context for subsequent actions.
        sensor_launch,
    ])
