"""HH_260915 - Passive persistent recorder; no motion publishers or UI dependency."""
import os
import socket
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    state_root = os.environ.get('XDG_STATE_HOME', '').strip()
    default_root = str((Path(state_root) if state_root else Path.home() / '.local/state') / 'camrod/mission_records')
    defaults = {
        'storage_root': default_root,
        'robot_id': socket.gethostname(),
        'environment': os.environ.get('CAMROD_RECORDING_ENVIRONMENT', 'real'),
        'platform_status_topic': '/platform/status',
        'gate_status_topic': '/control/cmd_vel_safety_gate/status',
        'event_topic': '/ui/mission_recording/events',
        # Empty means explicitly no raw frames. Decoded CAN is still recorded.
        'raw_can_interface': '',
        'quota_bytes': '268435456',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='camrod_ui', executable='mission_recorder_node',
             name='mission_recorder', output='screen',
             parameters=[{name: (ParameterValue(LaunchConfiguration(name), value_type=int)
                                if name == 'quota_bytes' else LaunchConfiguration(name))
                          for name in defaults}]),
    ])
