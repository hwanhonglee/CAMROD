"""GNSS rover + NTRIP client + converter integrated launch"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg = get_package_share_directory('ublox_dgnss')

    device_family = LaunchConfiguration('device_family')
    device_serial_string = LaunchConfiguration('device_serial_string')

    device_family_arg = DeclareLaunchArgument(
        'device_family', default_value=TextSubstitution(text='F9P'))
    device_serial_string_arg = DeclareLaunchArgument(
        'device_serial_string', default_value='')

    # ── ublox GNSS rover ──────────────────────────────────────────────────
    rover_params = [
        {'DEVICE_FAMILY': device_family},
        {'DEVICE_SERIAL_STRING': device_serial_string},
        {'FRAME_ID': 'ublox_dgnss'},
        {'CFG_RATE_MEAS': 0xc8},   # 200 ms = 5 Hz
        {'CFG_RATE_NAV': 0x1},
        {'CFG_UART1INPROT_NMEA': False},
        {'CFG_UART1INPROT_RTCM3X': False},
        {'CFG_UART1INPROT_UBX': False},
        {'CFG_UART1OUTPROT_NMEA': False},
        {'CFG_UART1OUTPROT_RTCM3X': False},
        {'CFG_UART1OUTPROT_UBX': False},
        {'CFG-UART2-BAUDRATE': 0x70800},   # 460800
        {'CFG_UART2INPROT_NMEA': False},
        {'CFG_UART2INPROT_RTCM3X': True},
        {'CFG_UART2INPROT_UBX': False},
        {'CFG_UART2OUTPROT_NMEA': False},
        {'CFG_UART2OUTPROT_RTCM3X': False},
        {'CFG_UART2OUTPROT_UBX': False},
        {'CFG_USBINPROT_NMEA': False},
        {'CFG_USBINPROT_RTCM3X': False},
        {'CFG_USBINPROT_UBX': True},
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_USBOUTPROT_RTCM3X': False},
        {'CFG_USBOUTPROT_UBX': True},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_PVT_USB': 0x1},
        {'CFG_MSGOUT_UBX_NAV_RELPOSNED_USB': 0x1},
    ]

    container_rover = ComposableNodeContainer(
        name='ublox_dgnss_rover',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', 'warn'],
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                namespace='ublox_dgnss',
                parameters=rover_params,
            )
        ],
        output='screen',
    )

    # ── NTRIP client ──────────────────────────────────────────────────────
    config_path = os.path.join(pkg, 'config', 'ntrip_params.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    ntrip_params = config['/ntrip_client']['ros__parameters']

    if 'NTRIP_USERNAME' in os.environ:
        ntrip_params['username'] = os.environ['NTRIP_USERNAME']
    if 'NTRIP_PASSWORD' in os.environ:
        ntrip_params['password'] = os.environ['NTRIP_PASSWORD']

    container_ntrip = ComposableNodeContainer(
        name='ntrip_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', 'warn'],
        composable_node_descriptions=[
            ComposableNode(
                package='ntrip_client_node',
                plugin='ublox_dgnss::NTRIPClientNode',
                name='ntrip_client',
                parameters=[ntrip_params],
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        device_family_arg,
        device_serial_string_arg,
        container_rover,
        container_ntrip,
    ])
