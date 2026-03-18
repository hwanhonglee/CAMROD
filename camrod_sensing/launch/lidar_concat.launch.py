#!/usr/bin/env python3
"""
Placeholder launch for LiDAR concatenation.

NOTE:
- `pointcloud_concat_node` is not implemented in `camrod_sensing` CMake targets.
- This launch intentionally does not start any runtime node to avoid misleading failures.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


# Implements `generate_launch_description` behavior.
def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[sensing/lidar_concat] Placeholder only: "
                    "pointcloud concatenation node is not implemented in this package yet."
                )
            ),
        ]
    )
