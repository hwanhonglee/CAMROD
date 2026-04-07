import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_parking')

    apriltag_param = os.path.join(pkg_dir, 'config', 'apriltag.yaml')
    docking_param = os.path.join(pkg_dir, 'config', 'docking_server.yaml')

    parking_ns = LaunchConfiguration('parking_ns')
    docking_ns = LaunchConfiguration('docking_ns')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parking_ns',
            default_value='parking',
            description='Top-level namespace for parking features'
        ),
        DeclareLaunchArgument(
            'docking_ns',
            default_value='docking',
            description='Sub-namespace for docking feature'
        ),

        GroupAction([
            PushRosNamespace(parking_ns),

            GroupAction([
                PushRosNamespace(docking_ns),

                Node(
                    package='apriltag_ros',
                    executable='apriltag_node',
                    name='apriltag',
                    output='screen',
                    parameters=[apriltag_param],
                    remappings=[
                        ('image_rect', '/camera/color/image_rect'),
                        ('camera_info', '/camera/color/camera_info'),
                        ('detections', 'apriltag/detections_raw'),
                    ]
                ),

                Node(
                    package='camrod_parking',
                    executable='parking_apriltag_bridge',
                    name='parking_apriltag_bridge',
                    output='screen',
                    parameters=[{
                        'input_detection_topic': '/parking/docking/apriltag/detections_raw',
                        'output_avg_detection_topic': '/parking/docking/apriltag/detections',
                        'output_avg_pose_topic': '/parking/docking/apriltag/pose',
                        'output_detected_dock_pose_topic': '/parking/docking/detected_dock_pose',
                        'fixed_frame': 'odom',
                        'tag_frame': 'dock_tag',
                        'family': '36h11',
                        'target_tag_id': 0,
                        'publish_rate_hz': 10.0,
                    }]
                ),

                Node(
                    package='opennav_docking',
                    executable='opennav_docking',
                    name='docking_server',
                    output='screen',
                    parameters=[docking_param]
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
            ])
        ])
    ])
