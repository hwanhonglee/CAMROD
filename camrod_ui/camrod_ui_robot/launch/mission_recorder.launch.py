"""HH_261002 - Launch the passive mission/CAN journal independently of the UI."""

import os
import socket
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    records_root = str(
        (Path(state_root) if state_root else Path.home() / ".local/state")
        / "camrod/mission_records"
    )
    defaults = {
        "storage_root": os.environ.get("CAMROD_MISSION_RECORDS_ROOT", records_root),
        "robot_id": socket.gethostname(),
        "environment": os.environ.get("CAMROD_RECORDING_ENVIRONMENT", "real"),
        "platform_status_topic": "/platform/status",
        "gate_status_topic": "/control/cmd_vel_safety_gate/status",
        "event_topic": "/ui/mission_recording/events",
        # Empty is intentional: decoded CAN-derived platform telemetry is
        # always recorded; physical raw frames require an explicit interface.
        "raw_can_interface": os.environ.get("CAMROD_MISSION_RAW_CAN_INTERFACE", ""),
        "quota_bytes": "268435456",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    parameters = {
        name: (
            ParameterValue(LaunchConfiguration(name), value_type=int)
            if name == "quota_bytes" else LaunchConfiguration(name)
        )
        for name in defaults
    }
    return LaunchDescription([
        *arguments,
        Node(
            package="camrod_ui",
            executable="mission_recorder_node",
            name="mission_recorder",
            output="screen",
            parameters=[parameters],
        ),
    ])
