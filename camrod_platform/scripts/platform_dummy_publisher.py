#!/usr/bin/env python3
"""Publish an explicitly non-drivable Ranger boundary when CAN is disabled."""

import math

import rclpy
from avg_msgs.msg import AvgBool
from nav_msgs.msg import Odometry
from ranger_msgs.msg import ActuatorState, ActuatorStateArray, SystemState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState


ODOM_TOPIC = "/odom"
ACTUATOR_STATE_TOPIC = "/actuator_state"
SYSTEM_STATE_TOPIC = "/system_state"
BATTERY_STATE_TOPIC = "/battery_state"
DUMMY_ACTIVE_TOPIC = "/platform/dummy_active"

ACTUATOR_COUNT = 8
DUMMY_ERROR_CODE = 0xFFFF
DUMMY_DRIVER_STATE = 0xFFFFFFFF
UNKNOWN_COVARIANCE = 1.0e6


def _unknown_covariance():
    """Return a 6x6 covariance with deliberately high diagonal uncertainty."""
    covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = UNKNOWN_COVARIANCE
    return covariance


def build_safe_odometry(stamp, odom_frame_id, base_frame_id):
    """Build stationary odometry that cannot imply measured localization."""
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = odom_frame_id
    message.child_frame_id = base_frame_id
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance = _unknown_covariance()
    message.twist.covariance = _unknown_covariance()
    return message


def build_safe_actuator_state(stamp, base_frame_id):
    """Build eight stopped actuator records with an explicit fault bitmap."""
    message = ActuatorStateArray()
    message.header.stamp = stamp
    message.header.frame_id = base_frame_id
    for actuator_id in range(ACTUATOR_COUNT):
        state = ActuatorState()
        state.id = actuator_id
        state.motor.rpm = 0
        state.motor.current = 0.0
        state.motor.pulse_count = 0
        state.motor.motor_angles = 0.0
        state.motor.motor_speeds = 0.0
        state.driver.driver_voltage = 0.0
        state.driver.driver_temperature = math.nan
        state.driver.motor_temperature = math.nan
        state.driver.driver_state = DUMMY_DRIVER_STATE
        message.states.append(state)
    return message


def build_safe_system_state(stamp, base_frame_id):
    """Build an ESTOP/RC/error state; dummy mode must never look drivable."""
    message = SystemState()
    message.header.stamp = stamp
    message.header.frame_id = base_frame_id
    message.vehicle_state = SystemState.VEHICLE_STATE_ESTOP
    message.control_mode = SystemState.CONTROL_MODE_RC
    message.error_code = DUMMY_ERROR_CODE
    message.battery_voltage = 0.0
    message.motion_mode = SystemState.MOTION_MODE_DUAL_ACKERMAN
    return message


def build_safe_battery_state(stamp, base_frame_id):
    """Build an absent/unknown battery record instead of fake healthy data."""
    message = BatteryState()
    message.header.stamp = stamp
    message.header.frame_id = base_frame_id
    message.voltage = math.nan
    message.temperature = math.nan
    message.current = math.nan
    message.charge = math.nan
    message.capacity = math.nan
    message.design_capacity = math.nan
    message.percentage = math.nan
    message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    message.power_supply_technology = (
        BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    )
    message.present = False
    message.location = "dummy:no-physical-platform"
    return message


def validate_configuration(publish_rate_hz, odom_frame_id, base_frame_id):
    """Validate low-load timing and required frame identifiers."""
    rate = float(publish_rate_hz)
    if not math.isfinite(rate) or rate < 1.0 or rate > 20.0:
        raise ValueError("publish_rate_hz must be finite and in [1, 20]")
    odom_frame = str(odom_frame_id).strip()
    base_frame = str(base_frame_id).strip()
    if not odom_frame or not base_frame:
        raise ValueError("odom_frame_id and base_frame_id must not be empty")
    return rate, odom_frame, base_frame


class PlatformDummyPublisher(Node):
    """Maintain raw Ranger topic shape while forcing a safe stopped state."""

    def __init__(self):
        super().__init__("platform_dummy_publisher")
        rate, self._odom_frame_id, self._base_frame_id = (
            validate_configuration(
                self.declare_parameter("publish_rate_hz", 5.0).value,
                self.declare_parameter("odom_frame_id", "odom").value,
                self.declare_parameter(
                    "base_frame_id", "robot_base_link"
                ).value,
            )
        )

        # HH_260729 - Reliable volatile QoS matches the raw Ranger boundary.
        # Five hertz keeps the bridge heartbeat fresh with negligible CPU load.
        raw_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        dummy_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._odom_pub = self.create_publisher(
            Odometry, ODOM_TOPIC, raw_qos
        )
        self._actuator_pub = self.create_publisher(
            ActuatorStateArray, ACTUATOR_STATE_TOPIC, raw_qos
        )
        self._system_pub = self.create_publisher(
            SystemState, SYSTEM_STATE_TOPIC, raw_qos
        )
        self._battery_pub = self.create_publisher(
            BatteryState, BATTERY_STATE_TOPIC, raw_qos
        )
        self._dummy_active_pub = self.create_publisher(
            AvgBool, DUMMY_ACTIVE_TOPIC, dummy_qos
        )
        self._timer = self.create_timer(1.0 / rate, self._publish_safe_state)
        self.get_logger().warning(
            "HH_260729 DUMMY PLATFORM ACTIVE: Ranger CAN is disabled; "
            f"publishing ESTOP/non-drivable zero state at {rate:.1f} Hz"
        )

    def _publish_safe_state(self):
        stamp = self.get_clock().now().to_msg()
        self._odom_pub.publish(
            build_safe_odometry(
                stamp, self._odom_frame_id, self._base_frame_id
            )
        )
        self._actuator_pub.publish(
            build_safe_actuator_state(stamp, self._base_frame_id)
        )
        self._system_pub.publish(
            build_safe_system_state(stamp, self._base_frame_id)
        )
        self._battery_pub.publish(
            build_safe_battery_state(stamp, self._base_frame_id)
        )
        dummy_active = AvgBool()
        dummy_active.data = True
        self._dummy_active_pub.publish(dummy_active)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlatformDummyPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(
                f"invalid platform dummy configuration: {exc}"
            )
        else:
            rclpy.logging.get_logger("platform_dummy_publisher").fatal(
                f"invalid platform dummy configuration: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
