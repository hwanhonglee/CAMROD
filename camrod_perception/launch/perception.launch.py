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
    front_camera_source_external = _truthy(
        context.perform_substitution(
            LaunchConfiguration('front_camera_source_external')))
    yolo_requested = _truthy(context.perform_substitution(LaunchConfiguration('enable_yolo')))
    camera_path = context.perform_substitution(LaunchConfiguration('camera_device_path')).strip()

    # Preserve develop's physical-camera decision exactly while the new
    # external-owner switch is false.  In particular, the explicit
    # camera_lidar mode historically overrides sim and probes the configured
    # device path; adding a CARLA source must not change that generic contract.
    camera_requested = camera_requested and front_camera_requested and not sim
    external_camera_requested = False
    if mode_raw == 'lidar_only':
        camera_requested = False
        yolo_requested = False
    elif mode_raw == 'camera_lidar':
        camera_requested = True
    elif mode_raw != 'auto':
        mode_raw = 'auto'

    if front_camera_source_external:
        # An explicit external owner supplies the canonical image and
        # CameraInfo topics.  camera_lidar retains develop's force-enable
        # meaning; auto still respects the ordinary camera/front selectors.
        external_camera_requested = (
            mode_raw == 'camera_lidar'
            or (
                mode_raw != 'lidar_only'
                and _truthy(context.perform_substitution(
                    LaunchConfiguration('enable_camera')))
                and front_camera_requested
            )
        )
        physical_camera_requested = False
    else:
        physical_camera_requested = camera_requested

    camera_available = external_camera_requested or (
        physical_camera_requested and bool(camera_path) and os.path.exists(camera_path))
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
                    "but neither an external canonical front camera nor "
                    f"a camera device at '{camera_path}' is available. "
                    "Falling back to LiDAR-only."
                )
            )
        )
    elif physical_camera_requested and not camera_available:
        actions.append(
            LogInfo(
                msg=(
                    f"[perception.launch] camera pipeline disabled: device not found at '{camera_path}'. "
                    "Running LiDAR-only perception."
                )
            )
        )
    elif external_camera_requested:
        actions.append(
            LogInfo(
                msg=(
                    '[perception.launch] external canonical front camera '
                    'selected; enabling production YOLO + camera-LiDAR fusion.'
                )
            )
        )
    elif not physical_camera_requested:
        actions.append(LogInfo(msg='[perception.launch] camera pipeline disabled by config; running LiDAR-only perception.'))
    return actions


def generate_launch_description():
    default_param = pkg_share('camrod_perception', os.path.join('config', 'perception_params.yaml'))
    default_runtime_override = pkg_share(
        'camrod_perception',
        os.path.join('config', 'perception_runtime_profiles', 'disabled.yaml'),
    )
    # HH_260807 - Standalone campsite occupancy uses the same semantic site
    # source as bringup instead of silently receiving an empty path.
    default_camping_sites_yaml = pkg_share(
        'camrod_planning', os.path.join('config', 'camping_sites.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('module_namespace',       default_value='perception'),
        DeclareLaunchArgument('perception_param_file',  default_value=default_param),
        DeclareLaunchArgument(
            'perception_runtime_override_param_file',
            default_value=default_runtime_override,
            description=(
                'Final sparse perception overlay; ordinary CAMROD uses an '
                'empty profile'
            ),
        ),
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
        DeclareLaunchArgument(
            'front_camera_source_external',
            default_value='false',
            description=(
                'Canonical front image and CameraInfo are owned by an external '
                'source; skip the local device check but retain YOLO/fusion'
            ),
        ),
        DeclareLaunchArgument('camera_device_path',     default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camping_sites_yaml', default_value=default_camping_sites_yaml),

        SetLaunchConfiguration('enable_camera_effective', 'false'),
        SetLaunchConfiguration('enable_yolo_effective', 'false'),
        SetLaunchConfiguration('enable_obstacle_fusion_effective', 'false'),
        OpaqueFunction(function=_resolve_camera_pipeline),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_fusion.launch.py')),
             'module_namespace', 'perception_param_file',
             'perception_runtime_override_param_file',
             condition=IfCondition(LaunchConfiguration('enable_obstacle_fusion_effective'))),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'obstacle_lidar.launch.py')),
             'module_namespace', 'perception_param_file', 'enable_lidar_obstacle'),

        _inc(pkg_share('camrod_perception', os.path.join('launch', 'yolo.launch.py')),
             'module_namespace', 'perception_param_file',
             'perception_runtime_override_param_file',
             enable_yolo=LaunchConfiguration('enable_yolo_effective')),

        # HH_260723 - Occupancy depends on semantic camera-LiDAR Detection3D.
        _inc(pkg_share('camrod_perception', os.path.join('launch', 'campsite_occupancy.launch.py')),
             'module_namespace', 'perception_param_file', 'camping_sites_yaml',
             condition=IfCondition(LaunchConfiguration('enable_obstacle_fusion_effective'))),
    ])
