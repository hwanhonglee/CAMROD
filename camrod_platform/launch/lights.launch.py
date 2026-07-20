import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 260708: Exterior lights — light_controller (lamp mode decision) and
# mcu_serial_bridge (JSON serial to the Arduino Nano light MCU).
# Decision rules and wire protocol: docs/lights-design-doc.html.
def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('camrod_platform'), 'config', 'lights.yaml')

    return LaunchDescription([
        # HH_260720 - Use a dedicated argument name so sensor-kit includes cannot overwrite it.
        DeclareLaunchArgument('lights_namespace', default_value='platform'),
        DeclareLaunchArgument('lights_enable', default_value='true'),
        # The serial bridge is separate so sim / bench runs without the MCU
        # can keep the controller (and its /platform/lights/command output)
        # while skipping the port-retry loop.
        DeclareLaunchArgument('lights_mcu_bridge_enable', default_value='true'),
        DeclareLaunchArgument('lights_param_file', default_value=default_params),

        Node(
            package='camrod_platform',
            executable='light_controller_node',
            name='light_controller',
            namespace=LaunchConfiguration('lights_namespace'),
            output='screen',
            parameters=[LaunchConfiguration('lights_param_file')],
            condition=IfCondition(LaunchConfiguration('lights_enable')),
        ),
        Node(
            package='camrod_platform',
            executable='mcu_serial_bridge_node',
            name='mcu_serial_bridge',
            namespace=LaunchConfiguration('lights_namespace'),
            output='screen',
            parameters=[LaunchConfiguration('lights_param_file')],
            condition=IfCondition(LaunchConfiguration('lights_mcu_bridge_enable')),
        ),
    ])
