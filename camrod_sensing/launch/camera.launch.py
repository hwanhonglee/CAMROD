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
# Parameters: config/camera/camera_params.yaml
#   camrod_sensing_camera section controls which cameras are active (read at launch time).
#   Per-node sections keyed by FQN override the /** defaults.
#
# TF frames (from camrod_sensor_kit robot_state_publisher):
#   sensor_kit_base_link → camera_front_link → camera_front
#   sensor_kit_base_link → camera_rear_link  → camera_rear

import os
import yaml as _yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _resolve_camera_enables(context, *args, **kwargs):
    """Read enable_front/rear from camera_params.yaml unless already set by caller (bringup)."""
    front_raw = context.perform_substitution(LaunchConfiguration('enable_front_camera'))
    rear_raw  = context.perform_substitution(LaunchConfiguration('enable_rear_camera'))

    if front_raw == '__yaml__' or rear_raw == '__yaml__':
        param_file = context.perform_substitution(LaunchConfiguration('camera_params_file'))
        try:
            with open(param_file, 'r', encoding='utf-8') as f:
                data = _yaml.safe_load(f) or {}
            cfg = data.get('camrod_sensing_camera', {})
        except Exception:
            cfg = {}

        if front_raw == '__yaml__':
            front_raw = 'true' if cfg.get('enable_front_camera', True) else 'false'
        if rear_raw == '__yaml__':
            rear_raw = 'true' if cfg.get('enable_rear_camera', True) else 'false'

    return [
        SetLaunchConfiguration('_front_camera_eff', front_raw),
        SetLaunchConfiguration('_rear_camera_eff',  rear_raw),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_sensing')

    default_config_file = os.path.join(pkg_dir, 'config', 'camera', 'camera_params.yaml')
    cyclonedds_config   = os.path.join(pkg_dir, 'config', 'camera', 'cyclonedds.xml')
    roudi_config        = os.path.join(pkg_dir, 'config', 'camera', 'roudi_config.toml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=default_config_file,
            description='Camera parameter YAML (intrinsics + device paths for front + rear)',
        ),
        # '__yaml__' sentinel: camera.launch.py reads value from camera_params.yaml.
        # When called from bringup/sensing.launch.py, an explicit 'true'/'false' is passed
        # (overriding the yaml), so standalone and bringup behave consistently.
        DeclareLaunchArgument(
            'enable_front_camera',
            default_value='__yaml__',
            description='Enable front camera node. Default: read from camera_params.yaml.',
        ),
        DeclareLaunchArgument(
            'enable_rear_camera',
            default_value='__yaml__',
            description='Enable rear camera node. Default: read from camera_params.yaml.',
        ),

        OpaqueFunction(function=_resolve_camera_enables),

        # Iceoryx shared memory daemon — required by camera_front_publisher_node (VPI/NvJPEG).
        # camera_rear_publisher_node is CPU-based and does not use iceoryx.
        ExecuteProcess(
            cmd=['iox-roudi', '-l', 'warning', '-c', roudi_config],
            output='screen',
            name='iox-roudi',
            condition=IfCondition(LaunchConfiguration('_front_camera_eff')),
        ),

        # ── Front camera node ───────────────────────────────────────────────────
        # FQN: /sensing/camera/econ_front/camera_front_publisher
        # Relative namespace: sensing.launch.py wraps in PushRosNamespace("sensing")
        Node(
            package='camrod_sensing',
            executable='camera_front_publisher_node',
            name='camera_front_publisher',
            namespace='camera/econ_front',
            output='screen',
            condition=IfCondition(LaunchConfiguration('_front_camera_eff')),
            parameters=[LaunchConfiguration('camera_params_file')],
            remappings=[
                ('~/image_rect/compressed', 'image_rect/compressed'),
                ('~/camera_info', 'camera_info'),
            ],
            additional_env={'CYCLONEDDS_URI': cyclonedds_config},
        ),

        # ── Rear camera node ────────────────────────────────────────────────────
        # FQN: /sensing/camera/econ_rear/camera_rear_publisher
        # Publishes image_raw (uncompressed) required by Isaac ROS AprilTag in docking.
        Node(
            package='camrod_sensing',
            executable='camera_rear_publisher_node',
            name='camera_rear_publisher',
            namespace='camera/econ_rear',
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
