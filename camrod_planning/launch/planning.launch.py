import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def lc_dict(*names: str) -> dict:
    return {name: LaunchConfiguration(name) for name in names}


def extract_map_ros_params(map_info_cfg: dict) -> dict:
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


def generate_launch_description():
    map_info_path = pkg_share('camrod_map', os.path.join('config', 'map_info.yaml'))
    map_path_default = ''
    origin_lat_default = '0.0'
    origin_lon_default = '0.0'
    origin_alt_default = '0.0'
    try:
        with open(map_info_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        params = extract_map_ros_params(data)
        map_path_default = str(params.get('map_path', map_path_default))
        origin_lat_default = str(params.get('offset_lat', origin_lat_default))
        origin_lon_default = str(params.get('offset_lon', origin_lon_default))
        origin_alt_default = str(params.get('offset_alt', origin_alt_default))
    except Exception:
        pass

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace', default_value='planning'),

        DeclareLaunchArgument('map_path', default_value=map_path_default),
        DeclareLaunchArgument('origin_lat', default_value=origin_lat_default),
        DeclareLaunchArgument('origin_lon', default_value=origin_lon_default),
        DeclareLaunchArgument('origin_alt', default_value=origin_alt_default),

        DeclareLaunchArgument('enable_path_cost_grids', default_value='true'),
        DeclareLaunchArgument('enable_goal_replanner', default_value='false'),
        DeclareLaunchArgument('enable_nav2_lifecycle_retry', default_value='false'),
        DeclareLaunchArgument('require_localization_ready', default_value='false'),
        DeclareLaunchArgument('enable_state_machine', default_value='false'),
        DeclareLaunchArgument('enable_tracking_error', default_value='true'),

        DeclareLaunchArgument('centerline_input_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_pose_topic', default_value='/localization/pose'),
        DeclareLaunchArgument('local_path_source', default_value='controller_then_slice'),
        DeclareLaunchArgument('tracking_error_topic', default_value='/planning/ltracking_error'),

        DeclareLaunchArgument('cmd_vel_gate_enable', default_value='true'),
        DeclareLaunchArgument('cmd_vel_raw_topic', default_value='/planning/cmd_vel_raw'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/planning/cmd_vel'),
        DeclareLaunchArgument('planning_engage_topic', default_value='/planning/engage'),
        DeclareLaunchArgument('planning_engaged_state_topic', default_value='/planning/engaged'),
        DeclareLaunchArgument('cmd_vel_gate_use_estop_topic', default_value='true'),
        DeclareLaunchArgument('cmd_vel_gate_estop_topic', default_value='/planning/state_machine/estop'),
        DeclareLaunchArgument('cmd_vel_gate_allow_on_start', default_value='false'),

        DeclareLaunchArgument(
            'nav2_base_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_base.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_vehicle_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_vehicle.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_lanelet_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_lanelet_overlay.yaml')),
        ),
        DeclareLaunchArgument(
            'nav2_behavior_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'nav2_behavior.yaml')),
        ),
        DeclareLaunchArgument('nav2_robot_base_frame', default_value='robot_base_link'),

        DeclareLaunchArgument(
            'local_path_extractor_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'local_path_extractor.yaml')),
        ),
        DeclareLaunchArgument(
            'path_cost_grids_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'path_cost_grids.yaml')),
        ),
        DeclareLaunchArgument(
            'goal_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'centerline_snapper_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'centerline_snapper.yaml')),
        ),
        DeclareLaunchArgument(
            'goal_replanner_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'goal_replanner.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_param_file',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'planning_state_machine.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_keypoints_yaml',
            default_value=pkg_share('camrod_map', os.path.join('config', 'drop_zones.yaml')),
        ),
        DeclareLaunchArgument(
            'planning_state_machine_camping_sites_yaml',
            default_value=pkg_share('camrod_planning', os.path.join('config', 'camping_sites.yaml')),
        ),

        # HH_260407: backward compatibility (no-op)
        DeclareLaunchArgument('system_namespace', default_value='system'),
        DeclareLaunchArgument('enable_module_validator', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'nav2_lanelet.launch.py'))
            ),
            launch_arguments=({
                **lc_dict(
                    'nav2_base_param_file',
                    'nav2_vehicle_param_file',
                    'nav2_lanelet_param_file',
                    'nav2_behavior_param_file',
                    'enable_path_cost_grids',
                    'path_cost_grids_param_file',
                    'map_path',
                    'origin_lat',
                    'origin_lon',
                    'origin_alt',
                    'nav2_robot_base_frame',
                    'module_namespace',
                ),
                'nav2_autostart': PythonExpression([
                    "'false' if ('", LaunchConfiguration('require_localization_ready'),
                    "' in ['true', 'True', '1']) else 'true'"
                ]),
            }).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'lanelet_tools.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'local_path.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'cmd_vel_gate.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'goal_replanner.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'state_machine.launch.py'))
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_share('camrod_planning', os.path.join('launch', 'lifecycle_retry.launch.py'))
            ),
        ),
    ])
