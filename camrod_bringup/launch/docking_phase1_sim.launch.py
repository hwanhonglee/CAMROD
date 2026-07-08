#!/usr/bin/env python3
"""Phase 1 도킹 RViz 시뮬레이터 전용 경량 런치.

YH_260703 — full bringup 은 x86 개발 PC(WSL2)에서 과부하로 Nav2 lifecycle 이
flapping 하므로, Phase 1(opennav_docking navigate_to_staging_pose) 검증에 필요한
최소 구성만 올린다.

포함:
  - bringup sim(경량): 진단/UI/state_machine/progress/path_viz/replan 은 OFF,
    localization(EKF→TF) · map · planning(Nav2) · cost grid 는 ON(컨트롤러 필요).
  - opennav_docking DockingServer + lifecycle_manager (Phase 1 엔진).
    · navigate_to_pose  -> /planning/navigate_to_pose  (launch remap = 내부 navigator 까지 적용)
    · cmd_vel           -> /platform/cmd_vel
    · base_frame        -> robot_base_link  (docking_server.yaml 의 base_link 오버라이드)
    · config 는 camrod_docking 소스 트리에서 직접 참조(x86 에선 camrod_docking 미빌드).

사용:
  ros2 launch camrod_bringup docking_phase1_sim.launch.py
  # 약 30초 후 Nav2 active + TF 연결(unconnected 경고 멈춤) 확인 뒤:
  ros2 topic pub -r 5 /planning/engage        std_msgs/msg/Bool "{data: true}" &
  ros2 topic pub -r 5 /platform/drive_enable  std_msgs/msg/Bool "{data: true}" &
  ros2 action send_goal /docking/dock_robot opennav_docking_msgs/action/DockRobot \
    "{use_dock_id: true, dock_id: 'home_dock', navigate_to_staging_pose: true}" --feedback

인자:
  rviz:=false           RViz 끄기(헤드리스)
  docking_delay:=25.0   opennav 기동 지연(Nav2 가 먼저 뜨도록)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# camrod_docking 은 x86 에서 빌드 스킵되어 share 가 install 되지 않으므로 소스 트리 config 사용.
DOCK_CFG = '/home/nvidia/camrod_docking_test_Ws/src/camrod_docking/config'


def generate_launch_description():
    bringup_share = get_package_share_directory('camrod_bringup')

    rviz = LaunchConfiguration('rviz')
    docking_delay = LaunchConfiguration('docking_delay')

    # ── 경량 bringup sim ────────────────────────────────────────────────────────
    #   cost grid 는 유지(MPPI/RPP 컨트롤러가 costmap 없으면 유효 궤적을 못 만든다).
    #   부하가 큰 진단(체커 ~30노드)/UI/보조 노드만 끈다.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'rviz': rviz,
            'enable_docking': 'false',            # 내장 도킹모듈(목서버+Isaac) 비활성; 아래서 opennav 직접 기동
            'enable_module_validators': 'false',  # 진단 체커 ~30노드 OFF (최대 부하 요인)
            'enable_state_machine': 'false',
            'enable_progress': 'false',
            'enable_path_visualization': 'false',
            'enable_obstacle_replan_monitor': 'false',
        }.items(),
    )

    # ── opennav_docking Phase 1 엔진 ────────────────────────────────────────────
    docking = GroupAction([
        PushRosNamespace('docking'),
        # YH_260706: opennav 내부 navigator 는 노드 remap 을 무시하고
        # /docking/navigate_to_pose 를 부른다(서버 없음 → Phase1 903). 이 릴레이가
        # /docking/navigate_to_pose 서버를 제공하고 /planning/navigate_to_pose 로 중계.
        Node(
            package='camrod_docking',
            executable='navigate_to_pose_relay',
            name='navigate_to_pose_relay',
            output='screen',
            parameters=[
                {'input_action': 'navigate_to_pose'},
                {'output_action': '/planning/navigate_to_pose'},
            ],
        ),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[
                os.path.join(DOCK_CFG, 'docking_server.yaml'),
                os.path.join(DOCK_CFG, 'controller.yaml'),
                {'dock_database': os.path.join(DOCK_CFG, 'docks.yaml')},
                {'base_frame': 'robot_base_link'},   # base_link → robot_base_link (CAMROD 프레임)
            ],
            remappings=[
                ('cmd_vel', '/platform/cmd_vel'),
                # 내부 navigator 가 상대명 navigate_to_pose 로 클라이언트를 만들므로
                # launch remap 으로 /planning/navigate_to_pose(bt_navigator) 에 연결.
                ('navigate_to_pose', '/planning/navigate_to_pose'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[
                os.path.join(DOCK_CFG, 'lifecycle_manager.yaml'),
                # WSL2 저사양 + cost grid 부하에서 configure/activate 서비스 응답이
                # 기본 bond_timeout(4s) 안에 안 돌아와 docking lifecycle 이 멈추는 것을 방지.
                {'bond_timeout': 60.0},
            ],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2 표시'),
        DeclareLaunchArgument(
            'docking_delay', default_value='45.0',
            description='opennav_docking 기동 지연(초) — Nav2/costmap 초기 부하가 가라앉은 뒤 뜨도록',
        ),
        bringup,
        # Nav2(/planning/navigate_to_pose)가 뜬 뒤 opennav 을 올린다.
        TimerAction(period=docking_delay, actions=[docking]),
    ])
