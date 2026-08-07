import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _inc(path, *through, condition=None, **overrides):
    args = {k: LaunchConfiguration(k) for k in through}
    args.update(overrides)
    kwargs = {"launch_arguments": args.items()}
    if condition is not None:
        kwargs["condition"] = condition
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), **kwargs)


def _truthy(raw: str) -> bool:
    return str(raw).strip().lower() in {"1", "true", "yes", "on"}


def _resolve_camera_pipeline(context, *args, **kwargs):
    mode_raw = context.perform_substitution(LaunchConfiguration('perception_mode')).strip().lower()
    camera_requested = _truthy(context.perform_substitution(LaunchConfiguration('enable_camera')))
    front_camera_requested = _truthy(
        context.perform_substitution(LaunchConfiguration('enable_front_camera')))
    sim = _truthy(context.perform_substitution(LaunchConfiguration('sim')))
    yolo_requested = _truthy(context.perform_substitution(LaunchConfiguration('enable_yolo')))
    camera_path = context.perform_substitution(LaunchConfiguration('camera_device_path')).strip()

    camera_requested = camera_requested and front_camera_requested and not sim
    if mode_raw == 'lidar_only':
        camera_requested = False
        yolo_requested = False
    elif mode_raw == 'camera_lidar':
        camera_requested = True
    elif mode_raw != 'auto':
        mode_raw = 'auto'

    camera_available = camera_requested and bool(camera_path) and os.path.exists(camera_path)
    yolo_effective = yolo_requested and camera_available
    fusion_effective = camera_available

    actions = [
        SetLaunchConfiguration('enable_camera_effective', 'true' if camera_available else 'false'),
        SetLaunchConfiguration('enable_yolo_effective', 'true' if yolo_effective else 'false'),
        SetLaunchConfiguration('enable_obstacle_fusion_effective', 'true' if fusion_effective else 'false'),
    ]
    if mode_raw == 'camera_lidar' and not camera_available:
        actions.append(
            LogInfo(
                msg=(
                    f"[perception.launch] perception_mode=camera_lidar requested, "
                    f"but camera device not found at '{camera_path}'. Falling back to LiDAR-only."
                )
            )
        )
    elif camera_requested and not camera_available:
        actions.append(
            LogInfo(
                msg=(
                    f"[perception.launch] camera pipeline disabled: device not found at '{camera_path}'. "
                    "Running LiDAR-only perception."
                )
            )
        )
    elif not camera_requested:
        actions.append(LogInfo(msg='[perception.launch] camera pipeline disabled by config; running LiDAR-only perception.'))
    return actions


def generate_launch_description():
    default_param = pkg_share('camrod_perception', os.path.join('config', 'perception_params.yaml'))
    # HH_260807 - Standalone campsite occupancy uses the same semantic site
    # source as bringup instead of silently receiving an empty path.
    default_camping_sites_yaml = pkg_share(
        'camrod_planning', os.path.join('config', 'camping_sites.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace',       default_value='perception'),
        DeclareLaunchArgument('perception_param_file',  default_value=default_param),
        DeclareLaunchArgument('enable_lidar_obstacle',  default_value='true'),
        DeclareLaunchArgument('enable_yolo',            default_value='true'),
        # HH_260522: unified selector for perception pipeline.
        #   auto: camera if available, fallback to lidar_only
        #   lidar_only: force LiDAR-only pipeline
        #   camera_lidar: require camera-lidar pipeline
        DeclareLaunchArgument('perception_mode',        default_value='auto'),
        # HH_260723 - Keep front-camera availability explicit so container
        # ownership cannot be mistaken for a disabled perception pipeline.
        DeclareLaunchArgument('enable_camera',          default_value='true'),
        DeclareLaunchArgument('enable_front_camera',    default_value='true'),
        DeclareLaunchArgument('sim',                    default_value='false'),
        DeclareLaunchArgument('camera_device_path',     default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camping_sites_yaml', default_value=default_camping_sites_yaml),

        SetLaunchConfiguration('enable_camera_effective', 'false'),
        SetLaunchConfiguration('enable_yolo_effective', 'false'),
        SetLaunchConfiguration('enable_obstacle_fusion_effective', 'false'),
        OpaqueFunction(function=_resolve_camera_pipeline),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_fusion.launch.py')),
             'module_namespace', 'perception_param_file',
             condition=IfCondition(LaunchConfiguration('enable_obstacle_fusion_effective'))),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_lidar.launch.py')),
             'module_namespace', 'perception_param_file', 'enable_lidar_obstacle'),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'yolo.launch.py')),
             'module_namespace', 'perception_param_file', enable_yolo=LaunchConfiguration('enable_yolo_effective')),

        # HH_260723 - Occupancy depends on semantic camera-LiDAR Detection3D.
        _inc(pkg_share('camrod_perception', os.path.join('launch', 'campsite_occupancy.launch.py')),
             'module_namespace', 'perception_param_file', 'camping_sites_yaml',
             condition=IfCondition(LaunchConfiguration('enable_obstacle_fusion_effective'))),
    ])
