#!/usr/bin/env python3
"""Integration test: GNSS DR fallback and recovery.

Starts the full bringup in sim mode, then launches gnss_dr_test_node which:
  1. Waits for localization + Nav2 lifecycle to be ready
  2. Publishes a camping-site goal to /goal_pose
  3. Publishes /planning/engage = true
  4. Monitors through GNSS failure (DR_ONLY) and recovery (NORMAL)
  5. Verifies cmd_vel pause during gnss_recovery_hold_s window
  6. Prints PASS/FAIL summary

Usage:
    ros2 launch camrod_bringup gnss_dr_test.launch.py

GNSS failure timing is controlled by config/sim/fake_sensors.yaml:
    gnss_failure_after_s   seconds from fake_sensor startup when GNSS stops
    gnss_recovery_after_s  seconds from fake_sensor startup when GNSS resumes
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("camrod_bringup")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "sim": "true",
            "rviz": "false",
            "clean_before_launch": "false",
            "enable_gnss": "false",
            "enable_lidar_driver": "false",
            "enable_imu": "false",
            "enable_radar": "false",
            "enable_ntrip": "false",
            "planning_cmd_vel_gate_allow_on_start": "false",
        }.items(),
    )

    # Delay 1 s to let the launch graph wire up before the test node starts.
    # goal_delay_s=13: send goal 13 s after test node start (~14 s from bringup).
    # engage_delay_s=3: send engage 3 s after goal.
    # test_duration_s=85: covers failure at ~25 s and recovery at ~55 s.
    # gnss_recovery_hold_s must match planning_cmd_vel_gate's gnss_recovery_hold_s.
    test_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="camrod_bringup",
                executable="gnss_dr_test_node.py",
                name="gnss_dr_test",
                namespace="bringup",
                output="screen",
                parameters=[{
                    "goal_x": -13.1858,
                    "goal_y": -93.0608,
                    "goal_yaw_deg": -179.992,
                    "goal_delay_s": 13.0,
                    "engage_delay_s": 3.0,
                    "test_duration_s": 85.0,
                    "gnss_recovery_hold_s": 2.0,
                }],
            )
        ],
    )

    return LaunchDescription([bringup, test_node])
