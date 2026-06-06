import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_frontend_dir() -> str:
    """Resolve frontend build directory with package-share-first fallback."""
    share_dir = get_package_share_directory('camrod_ui')
    # File is at camrod_ui_robot/launch/ui.launch.py → go up 2 levels to package root.
    source_pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

    env_frontend_dir = os.environ.get('CAMROD_UI_FRONTEND_DIR', '').strip()
    if env_frontend_dir and os.path.isdir(env_frontend_dir):
        return env_frontend_dir

    source_frontend_dir = os.path.join(source_pkg_root, 'camrod_ui_robot', 'assets', 'frontend', 'build')
    if os.path.isdir(source_frontend_dir):
        return source_frontend_dir

    packaged_frontend_dir = os.path.join(share_dir, 'camrod_ui_robot', 'assets', 'frontend', 'build')
    if os.path.isdir(packaged_frontend_dir):
        return packaged_frontend_dir

    return os.path.join(share_dir, 'camrod_ui_robot', 'assets', 'web')


def _resolve_default_camping_sites_yaml() -> str:
    """Resolve planning camping-sites YAML for site->goal_pose dispatch."""
    try:
        planning_share = get_package_share_directory('camrod_planning')
        candidate = os.path.join(planning_share, 'config', 'camping_sites.yaml')
        if os.path.isfile(candidate):
            return candidate
    except PackageNotFoundError:
        pass

    # Source-workspace fallback.
    # Go up 3 levels: launch/ → camrod_ui_robot/ → camrod_ui/ → src/
    source_ws_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'camrod_planning', 'config', 'camping_sites.yaml')
    )
    if os.path.isfile(source_ws_candidate):
        return source_ws_candidate
    return ''


def generate_launch_description():
    default_frontend_dir = _resolve_default_frontend_dir()
    default_camping_sites_yaml = _resolve_default_camping_sites_yaml()

    enable_ui_backend_arg = DeclareLaunchArgument(
        'enable_ui_backend',
        default_value='true',
        description='Enable UI backend HTTP server and ROS topic bridge',
    )
    ui_host_arg = DeclareLaunchArgument(
        'ui_host',
        default_value='127.0.0.1',
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
    camping_sites_yaml_arg = DeclareLaunchArgument(
        'camping_sites_yaml',
        default_value=default_camping_sites_yaml,
        description='Camping site coordinates YAML used for destination->goal_pose dispatch',
    )

    ui_backend = Node(
        package='camrod_ui',
        executable='ui_backend_node',
        name='ui_backend',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ui_backend')),
        parameters=[{
            'host': LaunchConfiguration('ui_host'),
            'port': LaunchConfiguration('ui_port'),
            'frontend_dir': LaunchConfiguration('frontend_dir'),
            'diagnostics_agg_topic': '/diagnostics_agg',
            'site_names': [f'B{i}' for i in range(1, 14)],
            'ui_destination_topic': '/ui/selected_destination',
            'planning_engage_topic': '/planning/engage',
            'planning_goal_key_topic': '/planning/state_machine/goal_key',
            'planning_goal_pose_topic': '/goal_pose',
            'publish_goal_key': True,
            'publish_goal_pose': True,
            'publish_engage_from_destination': True,
            'default_goal_frame_id': 'map',
            'fallback_goal_key': 'camping_site_1',
            'fallback_to_first_known_goal': True,
            'camping_sites_yaml': LaunchConfiguration('camping_sites_yaml'),
        }],
    )

    return LaunchDescription([
        enable_ui_backend_arg,
        ui_host_arg,
        ui_port_arg,
        frontend_dir_arg,
        camping_sites_yaml_arg,
        ui_backend,
    ])
