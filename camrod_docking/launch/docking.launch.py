import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_docking')

    # ── config 파일 경로 ────────────────────────────────────────────────────────
    cfg = lambda f: os.path.join(pkg_dir, 'config', f)  # noqa: E731

    # ── 토픽 이름 — 단일 소스 ──────────────────────────────────────────────────
    # AprilTagNode remapping 과 bridge input_detection_topic 이 반드시 일치해야 함.
    # 한 곳에서만 변경하면 무음 실패하므로 아래 상수로 두 곳을 동시에 제어한다.
    TOPIC_RAW_DETECTIONS = '/docking/apriltag/detections_raw'

    docking_ns = LaunchConfiguration('docking_ns')
    enable_auto_docking = LaunchConfiguration('enable_auto_docking')
    enable_manual_docking = LaunchConfiguration('enable_manual_docking')
    enable_apriltag = OrSubstitution(enable_auto_docking, enable_manual_docking)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    nav_to_pose_action = LaunchConfiguration('nav_to_pose_action')

    return LaunchDescription([
        DeclareLaunchArgument(
            'docking_ns',
            default_value='docking',
            description='Top-level namespace for docking nodes'
        ),
        DeclareLaunchArgument(
            'enable_auto_docking',
            default_value='false',
            description='Enable battery-triggered automatic docking'
        ),
        DeclareLaunchArgument(
            'enable_manual_docking',
            default_value='true',
            description='Enable UI-triggered manual docking server'
        ),
        # HH_260618 - Full CAMROD bringup overrides this to /planning/cmd_vel_raw
        # so marker docking and rule-based parking share the same safety gates.
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/platform/cmd_vel',
            description='Velocity command topic used by opennav_docking'
        ),
        # Standalone docking: true (publish odom→base_link TF directly).
        # HH_260617 - CAMROD full-stack integration: false. Localization owns odom→base_link
        # TF for the selected EKF/ESKF backend; duplicate TF publication conflicts.
        DeclareLaunchArgument(
            'enable_odom_corrector',
            default_value='true',
            description='Enable odom_yaw_corrector TF broadcaster. '
                        'Set false when running with CAMROD full stack localization TF'
        ),
        # YH_260702 (Stage 1-3) - opennav_docking navigator action remap.
        # navigator.cpp 가 상대경로 "navigate_to_pose" 로 클라이언트를 생성 → docking 네임스페이스
        # 하에서 /docking/navigate_to_pose 를 탐색하나 실제 서버는 /planning/navigate_to_pose 에 있음.
        # remap 없이는 Phase 1 staging 이 FailedToStage 로 즉시 실패.
        DeclareLaunchArgument(
            'nav_to_pose_action',
            default_value='/planning/navigate_to_pose',
            description='Nav2 navigate_to_pose action name that opennav_docking calls for Phase 1 staging'
        ),

        # 카메라 TF: camrod_sensor_kit URDF 가 담당
        # (camera_rear_link → camera_rear, camera_front_link → camera_front)
        # CAMROD 통합 환경에서는 static_transform_publisher 불필요

        Node(
            package='camrod_docking',
            executable='odom_yaw_corrector',
            name='odom_yaw_corrector',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_odom_corrector')),
            parameters=[cfg('odom_yaw_corrector.yaml')],
        ),

        GroupAction([
            PushRosNamespace(docking_ns),

            # ── Isaac ROS GPU 이미지 보정 + AprilTag (NITROS zero-copy) ───────────
            ComposableNodeContainer(
                package='rclcpp_components',
                name='apriltag_container',
                namespace='',
                executable='component_container_mt',
                condition=IfCondition(enable_apriltag),
                composable_node_descriptions=[
                    ComposableNode(
                        package='isaac_ros_image_proc',
                        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                        name='rectify_node',
                        namespace='',
                        parameters=[cfg('apriltag.yaml')],
                        remappings=[
                            ('image_raw',   '/sensing/camera/econ_rear/image_raw'),
                            ('camera_info', '/sensing/camera/econ_rear/camera_info'),
                            ('image_rect',  '/sensing/camera/econ_rear/image_rect'),
                        ],
                    ),
                    ComposableNode(
                        package='isaac_ros_apriltag',
                        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                        name='apriltag',
                        namespace='',
                        parameters=[cfg('apriltag.yaml')],
                        remappings=[
                            ('image',         '/sensing/camera/econ_rear/image_rect'),
                            ('camera_info',   '/sensing/camera/econ_rear/camera_info'),
                            ('tag_detections', TOPIC_RAW_DETECTIONS),
                            ('tf', '/tf'),
                        ],
                    ),
                ],
                output='screen',
            ),

            # ── AprilTag Bridge ─────────────────────────────────────────────────
            # camera frame PoseStamped 직접 발행 → SimpleChargingDock 이 TF 변환 처리
            Node(
                package='camrod_docking',
                executable='docking_apriltag_bridge',
                name='docking_apriltag_bridge',
                output='screen',
                condition=IfCondition(enable_apriltag),
                parameters=[cfg('bridge_params.yaml'),
                            {'input_detection_topic': TOPIC_RAW_DETECTIONS}],
            ),

            # ── DockingServer ────────────────────────────────────────────────────
            # docking_server.yaml  : 서버 전역 설정 + SimpleChargingDock 플러그인
            # controller.yaml      : EgoPolar 게인
            # dock_database        : 런타임 경로이므로 인라인 주입
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                parameters=[
                    cfg('docking_server.yaml'),
                    cfg('controller.yaml'),
                    {'dock_database': cfg('docks.yaml')},
                    {'cmd_vel_topic': cmd_vel_topic},
                ],
                remappings=[
                    ('cmd_vel', cmd_vel_topic),
                    # YH_260702 (Stage 1-3) - Phase 1 staging navigation 을 CAMROD Nav2
                    # (/planning/navigate_to_pose) 로 연결. 기본 BT 사용 (navigator_bt_xml="").
                    ('navigate_to_pose', nav_to_pose_action),
                ],
                respawn=True,
                respawn_delay=2.0,
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_docking',
                output='screen',
                parameters=[cfg('lifecycle_manager.yaml')],
            ),

            # ── 수동 도킹 서버 ──────────────────────────────────────────────────
            Node(
                package='camrod_docking',
                executable='manual_dock_server_node',
                name='manual_dock_server',
                output='screen',
                condition=IfCondition(enable_manual_docking),
                parameters=[cfg('manual_dock_server.yaml')],
            ),
        ]),
    ])
