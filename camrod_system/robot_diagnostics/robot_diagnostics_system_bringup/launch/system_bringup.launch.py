"""
Robot Diagnostics System Bringup

모든 진단 체커 + Aggregator 를 시스템 config 기반으로 실행한다.

사용법 (기본 — default config):
  ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py

로봇별 config 적용:
  ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py robot:=ranger_a

config 위치
-----------
  config/default/          ← 공통 기본 설정
  config/robots/{robot}/   ← 로봇별 오버라이드 (해당 yaml만 배치)

포함 노드
---------
[Aggregator]
  - aggregator_node              : /diagnostics → /diagnostics_agg  (C++ custom)

[Hardware]
  - hw_checker_node      : CPU / Memory / Disk / CPU 온도
  - gpu_checker_node     : GPU 사용률 / 온도
  - network_checker_node : WiFi 연결 / 신호 / 패킷 품질

[Sensing]
  - gnss_checker_node                : /sensing/gnss/navsatfix
  - imu_checker_node                 : /sensing/imu/data
  - lidar_checker_node               : /sensing/lidar/points
  - radar_checker_node               : /sensing/radar/*/range (6채널)
  - camera_checker_node              : /sensing/camera/front/image_raw
  - wheel_odometry_checker_node      : /platform/wheel/odometry
  - cost_grid_checker_node           : /sensing/lidar/near_cost_grid
  - velocity_converter_checker_node  : /sensing/platform_velocity_converter/...

[Localization]
  - localization_gnss_checker_node    : /sensing/gnss/pose_with_covariance
  - localization_mode_checker_node    : /localization/status
  - localization_pose_checker_node    : /localization/pose_with_covariance
  - localization_init_checker_node    : /localization/initial_match_*
  - localization_source_checker_node  : /localization/pose_source
  - localization_lanelet_checker_node : /localization/lanelet_pose

[Map]
  - map_cost_grid_checker_node : /map/cost_grid/lanelet

[Perception]
  - perception_obstacle_checker_node : /perception/camera/detections_2d
                                       /perception/obstacles

[Planning]
  - planning_lifecycle_checker_node  : Nav2 lifecycle 상태
  - planning_costmap_checker_node    : /planning/*/costmap
  - planning_nav_status_checker_node : /planning/navigate_to_pose/_action/status
  - planning_path_checker_node       : /planning/global_path, /planning/local_path

[Platform]
  - ranger_platform_checker_node : /system_state, /battery_state,
                                   /actuator_state, /odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PKG = 'robot_diagnostics_system_bringup'
COMPONENTS_BASE = ['hw', 'sensing', 'localization', 'map', 'perception', 'planning']


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    enable_platform = _as_bool(LaunchConfiguration('enable_platform').perform(context))
    pkg_share = get_package_share_directory(PKG)

    # robot 프로파일 config 경로 (없으면 default 사용)
    robot_config = os.path.join(pkg_share, 'config', 'robots', robot)
    default_config = os.path.join(pkg_share, 'config', 'default')
    config_dir = robot_config if os.path.isdir(robot_config) else default_config

    components_dir = os.path.join(pkg_share, 'launch', 'components')

    actions = []

    # ── Aggregator (C++ custom) ────────────────────────────────────────
    actions.append(Node(
        package='robot_diagnostics_agg',
        executable='aggregator_node',
        name='diagnostics_agg',
        parameters=[{'config_file': os.path.join(
            pkg_share, 'config', 'default', 'aggregator', 'diagnostics_config.yaml')}],
        output='screen',
    ))

    # HH_260330: platform checker needs ranger_msgs in this workspace.
    # Keep it optional so copied diagnostics stack can run without ranger dependency.
    components = list(COMPONENTS_BASE)
    if enable_platform:
        components.append('platform')
    else:
        actions.append(
            LogInfo(msg='[robot_diagnostics_system_bringup] HH_260330: platform checkers disabled')
        )

    # ── 카테고리별 component launch include ────────────────────────────
    for cat in components:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(components_dir, f'{cat}.launch.py')),
            launch_arguments={'config_dir': config_dir}.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='default',
            description='로봇 프로파일명 (config/robots/{robot}/ 이 없으면 config/default/ 사용)',
        ),
        DeclareLaunchArgument(
            'enable_platform',
            default_value='false',
            description='플랫폼 checker 실행 여부 (ranger_msgs 의존성 필요)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
