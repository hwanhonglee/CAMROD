#!/usr/bin/env python3
# HH_260422: Launch file for GNSS DR fallback integration test.
# Starts the full bringup in sim mode, then after a delay launches
# gnss_dr_test_node which:
#   1. Waits for localization + nav2 to be ready
#   2. Publishes camping_site goal → /goal_pose
#   3. Publishes /planning/engage = true
#   4. Monitors through GNSS failure (DR_ONLY) and recovery
#   5. Prints PASS/FAIL summary
#
# Usage:
#   ros2 launch camrod_bringup gnss_dr_test.launch.py
#
# GNSS failure timing is configured in fake_sensors.yaml:
#   gnss_failure_after_s  (seconds from fake_sensor startup when GNSS stops)
#   gnss_recovery_after_s (seconds from fake_sensor startup when GNSS resumes)

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
            # Disable hardware drivers not needed for sim.
            "enable_gnss": "false",
            "enable_lidar_driver": "false",
            "enable_imu": "false",
            "enable_radar": "false",
            "enable_ntrip": "false",
            # Allow cmd_vel from start (platform gate).
            "planning_cmd_vel_gate_allow_on_start": "false",
        }.items(),
    )

    # HH_260422: Launch test node after 14s to allow bringup + nav2 lifecycle to settle.
    # goal_delay_s=13: send goal 13s after test node start (= ~14s from bringup start).
    # engage_delay_s=3: send engage 3s after goal is sent.
    # test_duration_s=85: covers failure at ~25s and recovery at ~55s from fake_sensor.
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
                }],
            )
        ],
    )

    return LaunchDescription([bringup, test_node])
