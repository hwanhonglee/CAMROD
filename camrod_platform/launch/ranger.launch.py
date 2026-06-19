#!/usr/bin/env python3
# HH_260428: Ranger platform launch — starts ranger_base_node and ranger_platform_bridge_node.
# ranger_base_node  : reads CAN bus (port_name) and publishes /odom, /system_state,
#                     /actuator_state, /battery_state. Subscribes to /platform/cmd_vel
#                     (remapped from its internal /cmd_vel) for motion commands.
# ranger_platform_bridge_node : normalises ranger topics to /platform/status/* interface.
#   Primary odom  : /odom (ranger_base output, hardcoded here so it never conflicts
#                   with other platforms publishing on their own odom topics).
#   Fallback odom : odom_fallback_topic param (default /rmp401/odom for substitute platform).
#   Actuator state: /actuator_state -> /platform/status/wheel (CAN 0x281/0x271, id-based).
#   System state  : /system_state  -> /platform/status/estop  (CAN 0x211).
#   HH_260617: Battery state : /battery_state -> /platform/status/battery_state and /docking/is_charging
#                   (BMS CAN 0x361 current-based charger contact).

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def pkg_share(pkg: str, rel: str) -> str:
    return os.path.join(get_package_share_directory(pkg), rel)


def _load_ros_params(params_path: str) -> dict:
    with open(params_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('/**', {}).get('ros__parameters', {})


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    p = _load_ros_params(params_file)

    # HH_260428: ranger_base odom output is hardcoded to /odom so it never collides
    # with substitute-platform topics (e.g. /rmp401/odom).
    # The bridge subscribes to odom_topic_name from params_file (default /odom) and
    # falls back to odom_fallback_topic (default /rmp401/odom) when /odom is silent.
    ranger_base_node = Node(
        package='ranger_base',
        executable='ranger_base_node',
        output='screen',
        emulate_tty=True,
        # HH_260528: Ranger CAN node runs only for ranger platform type
        # and when explicitly enabled.
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("platform_type"),
            "'.strip().lower() == 'ranger' and '",
            LaunchConfiguration("enable_ranger_base_node"),
            "'.strip().lower() in ['1','true','yes','on']",
        ])),
        parameters=[{
            'use_sim_time':    p.get('use_sim_time', False),
            'port_name':       p.get('port_name', 'can0'),
            'odom_frame':      p.get('odom_frame', 'odom'),
            'base_frame':      p.get('base_frame', 'robot_base_link'),
            'odom_topic_name': '/odom',          # HH_260428: hardcoded; not from params_file
            'simulated_robot': p.get('simulated_robot', False),
            'publish_odom_tf': p.get('publish_odom_tf', False),
            'update_rate':     int(p.get('update_rate', 50)),
            'robot_model':     p.get('robot_model', 'ranger'),
        }],
        remappings=[('/cmd_vel', '/platform/cmd_vel')],  # HH_260428: receives gated cmd_vel
    )

    bridge_node = Node(
        package='camrod_platform',
        executable='ranger_platform_bridge_node',
        output='screen',
        # HH_260528: Keep bridge independently controllable from Ranger base.
        condition=IfCondition(LaunchConfiguration("enable_ranger_bridge_node")),
        parameters=[params_file],  # HH_260428: reads odom_topic_name, odom_fallback_topic, etc.
    )

    return [ranger_base_node, bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=pkg_share(
                'camrod_platform', os.path.join('config', 'ranger_driver.yaml')),
            description='YAML parameter file for ranger_base and platform bridge nodes',
        ),
        # HH_260528: Platform type selector propagated from platform.launch.py.
        DeclareLaunchArgument('platform_type', default_value='ranger'),
        # HH_260528: Independent toggles for Ranger CAN and bridge.
        DeclareLaunchArgument('enable_ranger_base_node', default_value='true'),
        DeclareLaunchArgument('enable_ranger_bridge_node', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
