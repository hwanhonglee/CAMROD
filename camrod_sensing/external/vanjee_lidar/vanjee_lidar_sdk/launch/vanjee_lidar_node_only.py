from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('vanjee_lidar_sdk')
    config_path = os.path.join(pkg_share, 'config', 'config.yaml')

    return LaunchDescription([
        # 1) Vanjee LiDAR driver
        Node(
            namespace='vanjee_lidar_sdk',
            package='vanjee_lidar_sdk',
            executable='vanjee_lidar_sdk_node',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),

        # 2) Static TF: base_link -> vanjee_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_vanjee',
            # x y z roll pitch yaw parent child  (rpy는 rad)
            arguments=['0', '0', '0.9', '0', '0.65', '0', 'base_link', 'vanjee_lidar'],
            output='screen'
        ),
    ])
