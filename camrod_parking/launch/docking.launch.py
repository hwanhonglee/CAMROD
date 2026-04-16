import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_dir = get_package_share_directory('camrod_parking')
    planning_pkg_dir = get_package_share_directory('camrod_planning')

    apriltag_param = os.path.join(pkg_dir, 'config', 'apriltag.yaml')
    docking_param = os.path.join(pkg_dir, 'config', 'docking_server.yaml')
    docks_file = os.path.join(pkg_dir, 'config', 'docks.yaml')
    docking_bt_xml = os.path.join(
        planning_pkg_dir, 'config', 'bt', 'navigate_to_pose_w_planner_selector_grid.xml'
    )

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
        DeclareLaunchArgument(
            'docking_param_file',
            default_value=docking_param,
            description='Parameter file for opennav docking server'
        ),
        DeclareLaunchArgument(
            'docks_file',
            default_value=docks_file,
            description='Dock database YAML used by opennav docking server'
        ),
        DeclareLaunchArgument(
            'navigator_bt_xml',
            default_value=docking_bt_xml,
            description='BT XML used by docking navigator when staging with NavigateToPose'
        ),
        DeclareLaunchArgument(
            'controller_costmap_topic',
            default_value='/planning/local_costmap/costmap_raw',
            description='Costmap topic used by docking controller collision checker'
        ),
        DeclareLaunchArgument(
            'controller_footprint_topic',
            default_value='/planning/local_costmap/published_footprint',
            description='Footprint topic used by docking controller collision checker'
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
                    parameters=[
                        LaunchConfiguration('docking_param_file'),
                        {
                            # HH_260410: Remove hardcoded absolute path and make dock DB portable.
                            'dock_database': LaunchConfiguration('docks_file'),
                            # HH_260410: Expose docking navigator BT as launch-level override.
                            'navigator_bt_xml': LaunchConfiguration('navigator_bt_xml'),
                            # HH_260410: Ensure docking controller consumes active Nav2 local costmap.
                            'controller.costmap_topic': LaunchConfiguration('controller_costmap_topic'),
                            'controller.footprint_topic': LaunchConfiguration('controller_footprint_topic'),
                        },
                    ],
                    remappings=[
                        # HH_260410: Docking navigator internally uses relative action name
                        # "navigate_to_pose"; remap it to planning Nav2 action server.
                        ('navigate_to_pose', '/planning/navigate_to_pose'),
                        ('/navigate_to_pose', '/planning/navigate_to_pose'),
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
            ])
        ])
    ])
