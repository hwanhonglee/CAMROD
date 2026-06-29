# HH_260528: Econ dual-camera launch — front (/dev/video0) and rear (/dev/video1).
# Both cameras are the same econ product but run different nodes:
#   Front: camera_front_publisher_node  (VPI VIC fisheye undistortion + NvJPEG GPU encoding)
#   Rear:  camera_rear_publisher_node   (OpenCV GStreamer + CPU JPEG; publishes image_raw for Isaac ROS AprilTag)
#
# Topic layout:
#   /sensing/camera/econ_front/image_rect/compressed
#   /sensing/camera/econ_front/camera_info
#   /sensing/camera/econ_rear/image_raw
#   /sensing/camera/econ_rear/image_raw/compressed
#   /sensing/camera/econ_rear/camera_info
#
# Parameters:
#   config/camera/camera_params.yaml        — ROS2 node params (intrinsics, device path, etc.)
#   config/camera/camera_launch_config.yaml — launch-only enable flags (HJ_260529)
#     camrod_sensing_camera section was moved out of camera_params.yaml because
#     the non-namespace top-level key caused rcl params parser to SIGABRT the node.
#
# TF frames (from camrod_sensor_kit robot_state_publisher):
#   sensor_kit_base_link → camera_front_link → camera_front
#   sensor_kit_base_link → camera_rear_link  → camera_rear

import os
import yaml as _yaml

from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def _truthy(raw: str) -> bool:
    return str(raw).strip().lower() in {'1', 'true', 'yes', 'on'}


def _camera_executable_exists(executable_name: str) -> bool:
    prefix = get_package_prefix('camrod_sensing')
    return os.path.exists(os.path.join(prefix, 'lib', 'camrod_sensing', executable_name))


def _resolve_camera_enables(context, *args, **kwargs):
    """Read enable_front/rear from camera_launch_config.yaml unless already set by caller."""
    front_raw = context.perform_substitution(LaunchConfiguration('enable_front_camera'))
    rear_raw  = context.perform_substitution(LaunchConfiguration('enable_rear_camera'))

    if front_raw == '__yaml__' or rear_raw == '__yaml__':
        # HJ_260529: read from camera_launch_config.yaml (separate from ROS2 params file)
        launch_config_file = context.perform_substitution(
            LaunchConfiguration('camera_launch_config_file'))
        try:
            with open(launch_config_file, 'r', encoding='utf-8') as f:
                data = _yaml.safe_load(f) or {}
            cfg = data.get('camrod_sensing_camera', {})
        except Exception:
            cfg = {}

        if front_raw == '__yaml__':
            front_raw = 'true' if cfg.get('enable_front_camera', True) else 'false'
        if rear_raw == '__yaml__':
            rear_raw = 'true' if cfg.get('enable_rear_camera', True) else 'false'

    front_requested = _truthy(front_raw)
    rear_requested = _truthy(rear_raw)
    front_available = _camera_executable_exists('camera_front_publisher_node')
    rear_available = _camera_executable_exists('camera_rear_publisher_node')
    front_eff = front_requested and front_available
    rear_eff = rear_requested and rear_available
    actions = []
    if front_requested and not front_available:
        actions.append(LogInfo(
            msg='[camera.launch] front camera disabled: camera_front_publisher_node is not installed.'
        ))
    if rear_requested and not rear_available:
        actions.append(LogInfo(
            msg='[camera.launch] rear camera disabled: camera_rear_publisher_node is not installed.'
        ))
    # HH_260617: Camera publishers are installed only when their CMake dependencies
    # are available (Jetson/VPI/NvJPEG for the front camera). Keep standalone launch
    # from failing on x86_64 by disabling unavailable executables at launch time.
    return [
        *actions,
        SetLaunchConfiguration('_front_camera_eff', 'true' if front_eff else 'false'),
        SetLaunchConfiguration('_rear_camera_eff',  'true' if rear_eff else 'false'),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_sensing')

    default_config_file       = os.path.join(pkg_dir, 'config', 'camera', 'camera_params.yaml')
    default_launch_config     = os.path.join(pkg_dir, 'config', 'camera', 'camera_launch_config.yaml')
    cyclonedds_config         = os.path.join(pkg_dir, 'config', 'camera', 'cyclonedds.xml')

    # HH_260629: Standalone camera.launch has no outer /sensing namespace, so default
    # directly to /sensing/camera for topic compatibility with downstream consumers
    # (yolov9mit, obstacle_fusion, docking AprilTag, diagnostics) that hardcode
    # /sensing/camera/... topics. sensing.launch.py overrides this to 'camera' because
    # its PushRosNamespace("sensing") already adds the /sensing prefix.
    camera_namespace = LaunchConfiguration('camera_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=default_config_file,
            description='Camera parameter YAML (intrinsics + device paths for front + rear)',
        ),
        # HJ_260529: separate launch-only config (camera enable flags)
        DeclareLaunchArgument(
            'camera_launch_config_file',
            default_value=default_launch_config,
            description='Launch config YAML with camera enable flags (not passed to nodes)',
        ),
        # '__yaml__' sentinel: reads value from camera_launch_config.yaml (HJ_260529).
        # When called from bringup/sensing.launch.py, an explicit 'true'/'false' is passed
        # (overriding the yaml), so standalone and bringup behave consistently.
        DeclareLaunchArgument(
            'enable_front_camera',
            default_value='__yaml__',
            description='Enable front camera node. Default: read from camera_launch_config.yaml.',
        ),
        DeclareLaunchArgument(
            'enable_rear_camera',
            default_value='__yaml__',
            description='Enable rear camera node. Default: read from camera_launch_config.yaml.',
        ),
        # HH_260629: base namespace for camera nodes. Standalone default includes the
        # /sensing prefix; sensing.launch.py passes 'camera' (PushRosNamespace adds /sensing).
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='sensing/camera',
            description='Base namespace for camera nodes (standalone: sensing/camera; via sensing.launch: camera).',
        ),

        OpaqueFunction(function=_resolve_camera_enables),

        # HJ_260529: iox-roudi removed — CycloneDDS SharedMemory is disabled and
        # camera_front_publisher_node does not use iceoryx APIs directly.

        # ── Front camera node ───────────────────────────────────────────────────
        # FQN: /sensing/camera/econ_front/camera_front_publisher
        # camera_namespace defaults to sensing/camera (standalone); sensing.launch.py
        # passes 'camera' and its PushRosNamespace("sensing") adds the /sensing prefix.
        Node(
            package='camrod_sensing',
            executable='camera_front_publisher_node',
            name='camera_front_publisher',
            namespace=[camera_namespace, '/econ_front'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('_front_camera_eff')),
            parameters=[LaunchConfiguration('camera_params_file')],
            remappings=[
                ('~/image_rect/compressed', 'image_rect/compressed'),
                ('~/image_raw',             'image_raw'),
                ('~/camera_info',           'camera_info'),
            ],
            additional_env={'CYCLONEDDS_URI': cyclonedds_config},
        ),

        # HJ_260529: compressed→raw bridge for yolov9mit and obstacle_fusion.
        # Both nodes require sensor_msgs/Image (raw). camera_front_publisher_node
        # only outputs CompressedImage, so republish to processed/image.
        Node(
            package='image_transport',
            executable='republish',
            name='front_camera_republisher',
            namespace=camera_namespace,
            output='screen',
            condition=IfCondition(LaunchConfiguration('_front_camera_eff')),
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', 'econ_front/image_rect/compressed'),
                ('out',           'processed/image'),
            ],
            parameters=[{
                'qos_overrides./sensing/camera/econ_front/image_rect/compressed.subscription.reliability': 'best_effort',
                'qos_overrides./sensing/camera/econ_front/image_rect/compressed.subscription.history': 'keep_last',
                'qos_overrides./sensing/camera/econ_front/image_rect/compressed.subscription.depth': 5,
            }],
        ),

        # ── Rear camera node ────────────────────────────────────────────────────
        # FQN: /sensing/camera/econ_rear/camera_rear_publisher
        # Publishes image_raw (uncompressed) required by Isaac ROS AprilTag in docking.
        Node(
            package='camrod_sensing',
            executable='camera_rear_publisher_node',
            name='camera_rear_publisher',
            namespace=[camera_namespace, '/econ_rear'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('_rear_camera_eff')),
            parameters=[LaunchConfiguration('camera_params_file')],
            remappings=[
                ('~/image_raw',            'image_raw'),
                ('~/image_raw/compressed', 'image_raw/compressed'),
                ('~/camera_info',          'camera_info'),
            ],
        ),
    ])
