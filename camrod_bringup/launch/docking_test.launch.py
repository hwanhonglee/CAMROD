#!/usr/bin/env python3
"""도킹 테스트 런치 — 카메라 + GQ7 IMU + 도킹 모듈.

섀시(rmp401)는 별도 터미널에서 실행:
  ros2 launch segwayrmp rmp401.launch.py

이 런치가 담당하는 것:
  - sensor_kit  : imu_link, camera_rear_link 등 정적 TF
  - sensing      : 후방 카메라(AprilTag용) + GQ7 IMU → /sensing/imu/data
  - docking      : odom_yaw_corrector(GQ7 yaw + rmp401 position) + AprilTag + DockingServer

도킹 실행:
  ros2 action send_goal /docking/manual_dock avg_msgs/action/ManualDock \
    "{dock_id: 'home_dock', max_staging_time: 120.0, dock_timeout_sec: 180.0}" --feedback
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _inc(path, **overrides):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=overrides.items(),
    )


def generate_launch_description():
    bringup_share    = get_package_share_directory('camrod_bringup')
    sensing_share    = get_package_share_directory('camrod_sensing')
    sensor_kit_share = get_package_share_directory('camrod_sensor_kit')
    docking_share    = get_package_share_directory('camrod_docking')

    params_file = os.path.join(bringup_share, 'config', 'sensor_kit', 'robot_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_model', default_value='gq7',
            description='IMU 모델: gq7 | cv7',
        ),

        # ── 1. sensor_kit: 정적 TF (imu_link, camera_rear_link …) ──────────────
        # base_frame_id=base_link: odom_yaw_corrector(odom→base_link)와 URDF 루트 일치
        _inc(
            os.path.join(sensor_kit_share, 'launch', 'sensor_kit.launch.py'),
            params_file=params_file,
            base_frame_id='base_link',
        ),

        # ── 2. sensing: 후방 카메라 + GQ7 IMU (불필요한 센서 전부 OFF) ──────────
        _inc(
            os.path.join(sensing_share, 'launch', 'sensing.launch.py'),
            enable_camera='true',
            enable_front_camera='false',
            enable_rear_camera='true',
            enable_imu='true',
            imu_model=LaunchConfiguration('imu_model'),
            use_ntrip='false',
            enable_gnss='false',
            enable_ntrip='false',
            enable_lidar_driver='false',
            enable_lidar_cost_grid='false',
            enable_radar='false',
            enable_radar_cost_grid='false',
            enable_inflation_cost_grid='false',
        ),

        # ── 3. docking: odom_yaw_corrector + AprilTag + DockingServer ──────────
        #   enable_odom_corrector=true : GQ7 yaw + rmp401 position → odom→base_link TF
        #   (풀스택 ESKF 연동 시에는 false로 변경)
        _inc(
            os.path.join(docking_share, 'launch', 'docking.launch.py'),
            enable_odom_corrector='true',
            enable_manual_docking='true',
            enable_auto_docking='false',
        ),
    ])
