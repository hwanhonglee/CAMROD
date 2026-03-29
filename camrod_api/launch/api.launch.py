import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_frontend_dir() -> str:
    """Select frontend directory with legacy UI preference and package fallback."""
    share_dir = get_package_share_directory('camrod_api')
    legacy_frontend_dir = os.path.expanduser('~/camrod_ws/src/todo/camroad_ui/frontend/build')
    if os.path.isdir(legacy_frontend_dir):
        return legacy_frontend_dir
    return os.path.join(share_dir, 'web')


def _plugin_api_topics() -> dict:
    """Return common plugin API topic/service names for both nodes."""
    return {
        'api_set_engage_service': '/api/plugin/set/engage',
        'api_set_operation_mode_service': '/api/plugin/set/operation_mode',
        'api_set_auto_service': '/api/plugin/set/operation_mode/auto',
        'api_set_stop_service': '/api/plugin/set/operation_mode/stop',
        'api_get_engage_topic': '/api/plugin/get/engage',
        'api_ready_topic': '/api/plugin/get/ready',
        'api_operation_mode_topic': '/api/plugin/get/operation_mode',
        'api_module_states_topic': '/api/plugin/get/module_states',
        'api_ready_message_topic': '/api/plugin/get/ready_message',
    }


# Builds launch graph for CAMROD plugin API bridge + optional UI backend.
def generate_launch_description():
    # HH_260329: Remove user-specific hardcoded path and use HOME-relative lookup.
    default_frontend_dir = _resolve_default_frontend_dir()
    shared_api = _plugin_api_topics()

    enable_plugin_api_arg = DeclareLaunchArgument(
        'enable_plugin_api',
        default_value='true',
        description='Enable plugin API bridge node',
    )
    enable_ui_backend_arg = DeclareLaunchArgument(
        'enable_ui_backend',
        default_value='true',
        description='Enable API UI backend HTTP server',
    )
    ui_host_arg = DeclareLaunchArgument(
        'ui_host',
        default_value='0.0.0.0',
        description='UI backend bind host',
    )
    ui_port_arg = DeclareLaunchArgument(
        'ui_port',
        default_value='8010',
        description='UI backend bind port',
    )
    frontend_dir_arg = DeclareLaunchArgument(
        'frontend_dir',
        default_value=default_frontend_dir,
        description='Static frontend directory for UI backend',
    )

    plugin_api_bridge = Node(
        package='camrod_api',
        executable='plugin_api_bridge_node',
        name='plugin_api_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_plugin_api')),
        parameters=[{
            'engage_command_topic': '/planning/engage',
            'engage_state_topic': '/platform/drive_enabled',
            'diagnostics_topic': '/diagnostics',
            'required_modules': [
                'map',
                'sensing',
                'localization',
                'planning',
                'platform',
                'perception',
            ],
            'warn_is_ready': True,
            'status_stale_timeout_s': 3.0,
            'publish_period_s': 0.5,
            **shared_api,
        }],
    )

    ui_backend = Node(
        package='camrod_api',
        executable='ui_backend_node',
        name='ui_backend',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ui_backend')),
        parameters=[{
            'host': LaunchConfiguration('ui_host'),
            'port': LaunchConfiguration('ui_port'),
            'frontend_dir': LaunchConfiguration('frontend_dir'),
            'diagnostics_agg_topic': '/diagnostics_agg',
            **shared_api,
        }],
    )

    return LaunchDescription([
        enable_plugin_api_arg,
        enable_ui_backend_arg,
        ui_host_arg,
        ui_port_arg,
        frontend_dir_arg,
        plugin_api_bridge,
        ui_backend,
    ])
