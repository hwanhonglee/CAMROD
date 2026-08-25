"""ROS-level contract test for the command adapter and its watchdog."""

import os
import time

from carla_extended_ackermann_msgs.msg import ExtendedAckermannDrive
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

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
    rclpy.init()
    adapter = TwistToFourWSNode(parameter_overrides=[
        Parameter("input_topic", value=input_topic),
        Parameter("output_topic", value=output_topic),
        Parameter("status_topic", value=(
            "/camrod_carla/test/status_" + suffix)),
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
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)

    try:
        assert _spin_until(
            executor, lambda: publisher.get_subscription_count() == 1)

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

        invalid = Twist()
        invalid.linear.x = 0.8
        invalid.angular.x = 0.1
        publisher.publish(invalid)
        assert _spin_until(
            executor,
            lambda: any(
                index > active_index and item[1].speed == 0.0
                for index, item in enumerate(received)
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
