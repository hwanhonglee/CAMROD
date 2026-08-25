"""ROS-boundary tests for the simulated Ranger platform heartbeat."""

import math
import os

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, UInt16

from camrod_carla_adapter.carla_platform_heartbeat_node import (
    CarlaPlatformHeartbeatNode,
)


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_ros_messages_and_runtime_overrides_follow_raw_ranger_contract():
    suffix = str(os.getpid())
    rclpy.init()
    node = CarlaPlatformHeartbeatNode(parameter_overrides=[
        Parameter("publish_rate_hz", value=10.0),
        Parameter(
            "system_state_topic",
            value="/camrod_carla/test/system_state_" + suffix,
        ),
        Parameter(
            "battery_state_topic",
            value="/camrod_carla/test/battery_state_" + suffix,
        ),
        Parameter(
            "charging_topic",
            value="/camrod_carla/test/charging_" + suffix,
        ),
        Parameter(
            "soc_topic",
            value="/camrod_carla/test/soc_" + suffix,
        ),
        Parameter(
            "estop_topic",
            value="/camrod_carla/test/estop_" + suffix,
        ),
        Parameter(
            "error_code_topic",
            value="/camrod_carla/test/error_code_" + suffix,
        ),
    ])
    system_capture = _CapturePublisher()
    battery_capture = _CapturePublisher()
    node.system_state_publisher = system_capture
    node.battery_state_publisher = battery_capture

    try:
        assert node.timer.timer_period_ns == 100_000_000
        node._publish_heartbeat()
        initial_system = system_capture.messages[-1]
        initial_battery = battery_capture.messages[-1]
        assert initial_system.vehicle_state == initial_system.VEHICLE_STATE_NORMAL
        assert initial_system.control_mode == initial_system.CONTROL_MODE_CAN
        assert initial_system.error_code == 0
        assert initial_system.motion_mode == (
            initial_system.MOTION_MODE_DUAL_ACKERMAN)
        assert initial_system.battery_voltage == 48.0
        assert initial_battery.percentage == pytest.approx(0.8)
        assert initial_battery.current < 0.0
        assert initial_battery.power_supply_status == (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)

        node._on_charging(Bool(data=True))
        node._on_soc(Float32(data=1.2))
        node._on_estop(Bool(data=True))
        node._on_error_code(UInt16(data=0x1234))
        node._publish_heartbeat()
        fault_system = system_capture.messages[-1]
        charging_battery = battery_capture.messages[-1]
        assert fault_system.vehicle_state == fault_system.VEHICLE_STATE_ESTOP
        assert fault_system.error_code == 0x1234
        assert charging_battery.percentage == 1.0
        assert charging_battery.current > 0.0
        assert charging_battery.power_supply_status == (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING)

        node._on_soc(Float32(data=math.nan))
        node._publish_heartbeat()
        assert battery_capture.messages[-1].percentage == 0.0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
