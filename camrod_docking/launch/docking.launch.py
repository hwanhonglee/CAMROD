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
        # 도킹 단독 실행: true (odom→base_link TF 직접 발행)
        # CAMROD 풀스택(ESKF) 연동: false (ESKF가 TF 담당 — 중복 발행 시 TF tree 충돌)
        DeclareLaunchArgument(
            'enable_odom_corrector',
            default_value='true',
            description='Enable odom_yaw_corrector TF broadcaster. '
                        'Set false when running with CAMROD full stack (ESKF publishes odom TF)'
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
                ],
                remappings=[('cmd_vel', '/platform/cmd_vel')],
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
