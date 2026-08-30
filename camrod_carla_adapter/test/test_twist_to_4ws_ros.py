"""ROS-level contract test for the command adapter and its watchdog."""

import os
import time

from avg_msgs.msg import ModuleState
from carla_extended_ackermann_msgs.msg import ExtendedAckermannDrive
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from camrod_carla_adapter.command_mapping import DriveMode
from camrod_carla_adapter.twist_to_4ws_node import TwistToFourWSNode


def _spin_until(executor, predicate, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return False


def test_ros_message_mapping_invalid_input_and_timeout_stop():
    suffix = str(os.getpid())
    input_topic = "/camrod_carla/test/input_" + suffix
    output_topic = "/camrod_carla/test/output_" + suffix
    recovery_status_topic = "/camrod_carla/test/recovery_status_" + suffix
    rclpy.init()
    adapter = TwistToFourWSNode(parameter_overrides=[
        Parameter("input_topic", value=input_topic),
        Parameter("output_topic", value=output_topic),
        Parameter("status_topic", value=(
            "/camrod_carla/test/status_" + suffix)),
        Parameter(
            "recovery_breakaway_status_topic",
            value=recovery_status_topic,
        ),
        Parameter("recovery_breakaway_status_timeout_sec", value=0.05),
        Parameter("input_timeout_sec", value=0.08),
        Parameter("watchdog_rate_hz", value=200.0),
        Parameter("zero_publish_rate_hz", value=100.0),
    ])
    probe = Node("camrod_carla_adapter_test_probe_" + suffix)
    received = []
    probe.create_subscription(
        ExtendedAckermannDrive,
        output_topic,
        lambda message: received.append((time.monotonic(), message)),
        10,
    )
    publisher = probe.create_publisher(Twist, input_topic, 10)
    recovery_qos = QoSProfile(depth=1)
    recovery_qos.reliability = ReliabilityPolicy.RELIABLE
    recovery_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    recovery_status_publisher = probe.create_publisher(
        ModuleState, recovery_status_topic, recovery_qos)
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)

    try:
        assert _spin_until(
            executor,
            lambda: (
                publisher.get_subscription_count() == 1
                and recovery_status_publisher.get_subscription_count() == 1
            ),
        )

        active = Twist()
        active.linear.x = 0.8
        active.angular.z = 0.2
        publisher.publish(active)
        assert _spin_until(
            executor,
            lambda: any(abs(item[1].speed - 0.8) < 1.0e-6
                        for item in received),
        )
        active_index = max(
            index for index, item in enumerate(received)
            if abs(item[1].speed - 0.8) < 1.0e-6)
        active_message = received[active_index][1]
        assert active_message.drive_mode.mode == DriveMode.ACKERMANN
        assert active_message.steering_angle > 0.0
        assert active_message.recovery_breakaway_authorized is False

        recovery_status = ModuleState()
        recovery_status.module_name = "route_safety_recovery_controller"
        recovery_status.level = ModuleState.OK
        recovery_status.operating_state = "CRAB_LEFT"
        recovery_status_publisher.publish(recovery_status)
        assert _spin_until(
            executor,
            lambda: adapter._last_recovery_operating_state == "CRAB_LEFT",
        )
        recovery = Twist()
        recovery.linear.y = 0.05
        publisher.publish(recovery)
        assert _spin_until(
            executor,
            lambda: any(
                item[1].drive_mode.mode == DriveMode.CRAB
                and abs(item[1].speed - 0.05) < 1.0e-6
                and item[1].recovery_breakaway_authorized
                for item in received
            ),
        )

        time.sleep(0.06)
        publisher.publish(recovery)
        assert _spin_until(
            executor,
            lambda: any(
                item[1].drive_mode.mode == DriveMode.CRAB
                and abs(item[1].speed - 0.05) < 1.0e-6
                and not item[1].recovery_breakaway_authorized
                for item in received
            ),
        )

        received.clear()
        recovery_status.operating_state = "CRAB_RIGHT"
        recovery_status_publisher.publish(recovery_status)
        assert _spin_until(
            executor,
            lambda: adapter._last_recovery_operating_state == "CRAB_RIGHT",
        )
        # The still-left Twist must not reuse a fresh right-recovery status.
        publisher.publish(recovery)
        assert _spin_until(
            executor,
            lambda: any(
                item[1].drive_mode.mode == DriveMode.CRAB
                and abs(item[1].speed - 0.05) < 1.0e-6
                and not item[1].recovery_breakaway_authorized
                for item in received
            ),
        )

        invalid = Twist()
        invalid.linear.x = 0.8
        invalid.angular.x = 0.1
        invalid_publish_time = time.monotonic()
        publisher.publish(invalid)
        assert _spin_until(
            executor,
            lambda: any(
                item[0] >= invalid_publish_time and item[1].speed == 0.0
                for item in received
            ),
        )

        received.clear()
        publisher.publish(active)
        assert _spin_until(
            executor,
            lambda: any(abs(item[1].speed - 0.8) < 1.0e-6
                        for item in received),
        )
        active_time = max(
            item[0] for item in received
            if abs(item[1].speed - 0.8) < 1.0e-6)
        assert _spin_until(
            executor,
            lambda: any(
                item[0] > active_time
                and item[1].drive_mode.mode == DriveMode.ACKERMANN
                and item[1].speed == 0.0
                and item[1].steering_angle == 0.0
                and item[1].crab_angle == 0.0
                and item[1].yaw_rate_cmd == 0.0
                and not item[1].recovery_breakaway_authorized
                for item in received
            ),
            timeout_sec=0.5,
        )
    finally:
        executor.remove_node(probe)
        executor.remove_node(adapter)
        probe.destroy_node()
        adapter.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
