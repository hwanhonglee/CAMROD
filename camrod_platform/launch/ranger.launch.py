#!/usr/bin/env python3
# HH_260428: Ranger platform launch — starts ranger_base_node and ranger_platform_bridge_node.
# ranger_base_node  : reads CAN bus (port_name) and publishes /odom, /system_state,
#                     /actuator_state, /battery_state. Subscribes to /control/cmd_vel_ros
#                     (remapped from its internal /cmd_vel) for final motion commands.
# ranger_platform_bridge_node : normalises ranger topics to /platform/status/* interface.
#   Primary odom  : /odom (ranger_base output, hardcoded here so it never conflicts
#                   with other platforms publishing on their own odom topics).
#   Fallback odom : odom_fallback_topic param (default /rmp401/odom for substitute platform).
#   Actuator state: /actuator_state -> /platform/status/wheel (CAN 0x281/0x271, id-based).
#   HH_260720 - System/BMS state: raw /system_state and /battery_state are normalized
#                   into the single generated /platform/status contract.

import os
import shlex
import subprocess
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


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _can_ready(port_name: str, bitrate: int, restart_ms: int) -> bool:
    result = subprocess.run(
        ['ip', '-details', 'link', 'show', port_name],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return False
    text = result.stdout + result.stderr
    return (
        'state UP' in text and
        f'bitrate {bitrate}' in text and
        f'restart-ms {restart_ms}' in text
    )


def _can_interface_exists(port_name: str) -> bool:
    # HH_260721 - Distinguish missing CAN hardware from a present interface that only needs privileges.
    result = subprocess.run(
        ['ip', 'link', 'show', port_name],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.returncode == 0


def _setup_can_interface(context, params: dict) -> None:
    platform_type = LaunchConfiguration('platform_type').perform(context)
    ranger_enabled = LaunchConfiguration('enable_ranger_base_node').perform(context)
    auto_setup_can = LaunchConfiguration('auto_setup_can').perform(context)
    if platform_type.strip().lower() != 'ranger' or not _truthy(ranger_enabled) or not _truthy(auto_setup_can):
        return

    port_name = str(params.get('port_name', 'can0')).strip()
    bitrate = int(LaunchConfiguration('can_bitrate').perform(context))
    restart_ms = int(LaunchConfiguration('can_restart_ms').perform(context))
    if _can_ready(port_name, bitrate, restart_ms):
        print(f'[ranger.launch] CAN {port_name} already up: bitrate={bitrate} restart-ms={restart_ms}')
        return

    # HH_260721 - Sudo can configure an existing SocketCAN link but cannot create absent hardware.
    if not _can_interface_exists(port_name):
        raise RuntimeError(
            f'CAN interface {port_name} does not exist. Connect and power the SocketCAN adapter, '
            f'then verify `ip link show {port_name}`. For a non-hardware run, launch with `sim:=true`.'
        )

    # HH_260629: Bring SocketCAN up before ranger_base_node opens it. `sudo -n`
    # avoids a hidden password prompt inside ros2 launch; configure sudoers or
    # run `sudo -v` before launch if this fails on a freshly booted system.
    quoted_port = shlex.quote(port_name)
    setup_script = (
        f'ip link set {quoted_port} down || true; '
        f'ip link set {quoted_port} type can bitrate {bitrate} restart-ms {restart_ms}; '
        f'ip link set {quoted_port} up'
    )
    cmd = ['bash', '-lc', setup_script] if os.geteuid() == 0 else ['sudo', '-n', 'bash', '-lc', setup_script]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or '').strip()
        raise RuntimeError(
            f'Failed to setup CAN {port_name}. '
            f'The interface exists but requires network-administration privileges. '
            f'Run sudo -v before launch or install camrod-can0.service once. detail={detail}'
        )
    if not _can_ready(port_name, bitrate, restart_ms):
        raise RuntimeError(f'CAN {port_name} setup finished but interface is not UP at {bitrate}bps')
    print(f'[ranger.launch] CAN {port_name} setup complete: bitrate={bitrate} restart-ms={restart_ms}')


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    p = _load_ros_params(params_file)
    _setup_can_interface(context, p)

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
        # HH_260720 - Ranger consumes the single final output from camrod_control directly.
        # HH_260720 - Ranger requires geometry_msgs/Twist at the explicit driver boundary.
        remappings=[('/cmd_vel', '/control/cmd_vel_ros')],
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
        DeclareLaunchArgument('auto_setup_can', default_value='true'),
        DeclareLaunchArgument('can_bitrate', default_value='500000'),
        DeclareLaunchArgument('can_restart_ms', default_value='100'),
        OpaqueFunction(function=_launch_setup),
    ])
