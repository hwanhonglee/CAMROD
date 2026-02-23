# groups | grep dialout
# sudo usermod -aG dialout $USER
# sudo -E env "PATH=$PATH" bash -lc 'source /opt/ros/humble/setup.bash && ros2 run ublox_gps ublox_gps_node --ros-args --params-file /home/hong/cart_test_ws/install/camping_cart_sensing/share/camping_cart_sensing/config/zed_f9p_rover.yaml'
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('camping_cart_sensing')

    ublox_yaml = os.path.join(pkg_share, 'config', 'zed_f9p_rover.yaml')
    ntrip_yaml  = os.path.join(pkg_share, 'config', 'ntrip_client.yaml')

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[ublox_yaml],
    )

    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        output='screen',
        parameters=[ntrip_yaml],
        # IMPORTANT: publish to /rtcm so ublox_gps_node can subscribe
        remappings=[('rtcm', '/rtcm')],
    )

    return LaunchDescription([ublox_node, ntrip_node])

