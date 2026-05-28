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

    docking_param = os.path.join(pkg_dir, 'config', 'docking_server.yaml')
    manual_dock_param = os.path.join(pkg_dir, 'config', 'manual_dock_server.yaml')

    docking_ns = LaunchConfiguration('docking_ns')
    enable_auto_docking = LaunchConfiguration('enable_auto_docking')
    enable_manual_docking = LaunchConfiguration('enable_manual_docking')
    # AprilTag + bridge는 auto/manual 어느 쪽이든 활성화되면 실행
    enable_apriltag = OrSubstitution(enable_auto_docking, enable_manual_docking)

    return LaunchDescription([
        DeclareLaunchArgument(
            'docking_ns',
            default_value='docking',
            description='Top-level namespace for docking feature'
        ),
        DeclareLaunchArgument(
            'enable_auto_docking',
            default_value='false',
            description='Enable battery-triggered automatic docking (state machine integration)'
        ),
        DeclareLaunchArgument(
            'enable_manual_docking',
            default_value='true',
            description='Enable UI-triggered manual docking server'
        ),

        # 후면 카메라: base_link 기준 전방 10cm, 높이 46cm, 후방을 향함 (실측)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_tf_publisher',
            arguments=['0.10', '0.0', '0.46', '3.1416', '0.0', '0.0',
                       'base_link', 'econ_rear_camera_frame'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_optical_tf_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'econ_rear_camera_frame', 'econ_rear_camera_optical_frame'],
            output='screen',
        ),
        # 전면 카메라: base_link 기준 전방 40cm, 높이 46cm, 전방을 향함 (실측)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_tf_publisher',
            arguments=['0.40', '0.0', '0.46', '0.0', '0.0', '0.0',
                       'base_link', 'econ_front_camera_frame'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_optical_tf_publisher',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'econ_front_camera_frame', 'econ_front_camera_optical_frame'],
            output='screen',
        ),
        Node(
            package='camrod_docking',
            executable='odom_yaw_corrector',
            name='odom_yaw_corrector',
            output='screen',
        ),

        GroupAction([
            PushRosNamespace(docking_ns),

            # Isaac ROS GPU 이미지 보정 + AprilTag — 동일 container에서 NITROS zero-copy
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
                        remappings=[
                            ('image_raw', '/rear/econ_camera/image_raw'),
                            ('camera_info', '/rear/econ_camera/camera_info'),
                            ('image_rect', '/rear/econ_camera/image_rect'),
                        ],
                    ),
                    ComposableNode(
                        package='isaac_ros_apriltag',
                        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                        name='apriltag',
                        namespace='',
                        parameters=[{
                            'size': 0.200,
                            'max_tags': 64,
                            'tag_family': 'tag36h11',
                            'backends': 'CUDA',
                        }],
                        remappings=[
                            ('image', '/rear/econ_camera/image_rect'),
                            ('camera_info', '/rear/econ_camera/camera_info'),
                            ('tag_detections', '/docking/apriltag/detections_raw'),
                            ('tf', '/tf'),
                        ],
                    ),
                ],
                output='screen',
            ),

            Node(
                package='camrod_docking',
                executable='docking_apriltag_bridge',
                name='docking_apriltag_bridge',
                output='screen',
                condition=IfCondition(enable_apriltag),
                parameters=[{
                    'input_detection_topic': '/docking/apriltag/detections_raw',
                    'output_avg_detection_topic': '/docking/apriltag/detections',
                    'output_avg_pose_topic': '/docking/apriltag/pose',
                    'output_detected_dock_pose_topic': '/docking/detected_dock_pose',
                    'fixed_frame': 'odom',
                    'tag_frame': 'tag36h11:3',
                    'family': 'tag36h11',
                    'target_tag_id': 3,
                    'publish_rate_hz': 10.0,
                    'ema_alpha': 0.3,            # EMA 필터 계수 (낮을수록 강한 필터)
                    'max_jump_m': 0.3,          # 이 거리 이상 위치 급변 시 무시 (outlier rejection)
                    'detection_timeout': 0.5,   # 이 시간 이내 검출 없으면 publish 중단
                    'negate_y': True,           # 후면 카메라 Y 부호 반전 보정 (전면 카메라 시 False)
                }]
            ),

            Node(
                package='camrod_docking',
                executable='tag_distance_node',
                name='tag_distance_node',
                output='screen',
                condition=IfCondition(enable_apriltag),
                parameters=[{
                    'camera_frame': 'econ_rear_camera_optical_frame',
                    'tag_frame': 'tag36h11:3',
                    'publish_rate_hz': 10.0,
                }]
            ),

            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                parameters=[
                    docking_param,
                    {
                        'dock_database': os.path.join(pkg_dir, 'config', 'docks.yaml'),
                        # PushRosNamespace 안에서 YAML key 매칭이 깨지는 경우 대비 — 모두 명시적 주입
                        'controller_frequency': 20.0,
                        'initial_perception_timeout': 5.0,
                        'wait_charge_timeout': 10.0,
                        'dock_approach_timeout': 120.0,
                        'undock_linear_tolerance': 0.05,
                        'undock_angular_tolerance': 0.10,
                        'max_retries': 3,
                        'base_frame': 'base_link',
                        'fixed_frame': 'odom',
                        'cmd_vel_topic': '/rmp401/cmd_vel',
                        'dock_backwards': True,
                        'dock_prestaging_tolerance': 0.5,
                        'dock_plugins': ['apriltag_dock'],
                        'apriltag_dock.plugin': 'camrod_docking::CamrodDockingPlugin',
                        'apriltag_dock.docking_threshold': 0.50,
                        'apriltag_dock.staging_x_offset': -0.50,
                        'apriltag_dock.staging_yaw_offset': 0.0,
                        'apriltag_dock.external_detection_timeout': 1.0,
                        'apriltag_dock.external_detection_translation_x': -0.20,
                        'apriltag_dock.external_detection_translation_y': 0.0,
                        'apriltag_dock.detected_dock_pose_topic': '/docking/detected_dock_pose',
                        'apriltag_dock.base_frame': 'base_link',
                        'apriltag_dock.filter_coef': 0.5,
                        'controller.k_phi': 1.5,
                        'controller.k_delta': 1.5,
                        'controller.beta': 0.4,
                        'controller.lambda': 2.0,
                        'controller.v_linear_min': 0.05,
                        'controller.v_linear_max': 0.15,
                        'controller.v_angular_max': 0.3,
                        'controller.slowdown_radius': 0.5,
                        'controller.use_collision_detection': False,
                    },
                ],
                remappings=[
                    ('cmd_vel', '/rmp401/cmd_vel'),
                ],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_docking',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['docking_server']
                }]
            ),

            Node(
                package='camrod_docking',
                executable='manual_dock_server_node',
                name='manual_dock_server',
                output='screen',
                condition=IfCondition(enable_manual_docking),
                parameters=[manual_dock_param, {
                    'navigate_to_staging_pose': False,
                }],
            ),
        ])
    ])
