"""ROS-boundary health tests for CARLA feedback relays."""

import math
import os
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from camrod_carla_adapter.feedback_bridge_node import CarlaFeedbackBridgeNode


def test_invalid_imu_is_not_accepted_or_erased_by_valid_odometry():
    suffix = str(os.getpid())
    rclpy.init()
    bridge = CarlaFeedbackBridgeNode(parameter_overrides=[
        Parameter(
            "input_odometry_topic",
            value="/camrod_carla/test/feedback_odom_" + suffix,
        ),
        Parameter(
            "input_imu_topic",
            value="/camrod_carla/test/feedback_imu_" + suffix,
        ),
        Parameter(
            "output_imu_topic",
            value="/camrod_carla/test/feedback_imu_out_" + suffix,
        ),
        Parameter("publish_platform_odometry", value=False),
        Parameter("publish_metric_pose", value=False),
        Parameter("publish_ground_truth_localization", value=False),
        Parameter("publish_ground_truth_tf", value=False),
        Parameter("relay_imu", value=True),
        Parameter("relay_gnss", value=False),
        Parameter("feedback_timeout_sec", value=0.05),
    ])

    try:
        invalid_imu = Imu()
        invalid_imu.orientation.w = 1.0
        invalid_imu.angular_velocity.x = math.nan
        bridge._on_imu(invalid_imu)
        assert bridge._last_imu_monotonic is None
        assert bridge._stream_errors["imu"]

        valid_odometry = Odometry()
        valid_odometry.pose.pose.orientation.w = 1.0
        bridge._on_odometry(valid_odometry)
        assert bridge._last_odometry_monotonic is not None
        assert bridge._stream_errors["odometry"] == ""
        assert bridge._stream_errors["imu"]

        _, errors, _ = bridge._stream_health(time.monotonic())
        assert any(error.startswith("imu:") for error in errors)

        valid_imu = Imu()
        valid_imu.orientation.w = 1.0
        bridge._on_imu(valid_imu)
        assert bridge._last_imu_monotonic is not None
        assert bridge._stream_errors["imu"] == ""

        bridge._last_imu_monotonic = time.monotonic() - 0.1
        _, errors, stale = bridge._stream_health(time.monotonic())
        assert errors == []
        assert "imu" in stale
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
