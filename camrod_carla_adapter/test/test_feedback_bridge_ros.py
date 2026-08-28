"""ROS-boundary health tests for CARLA feedback relays."""

import math
import os
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from camrod_carla_adapter.feedback_bridge_node import CarlaFeedbackBridgeNode
from camrod_carla_adapter.gnss_compat import EARTH_SEMIMAJOR_AXIS_M


class _Collector:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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


def test_actual_dual_carla_gnss_populates_every_ui_gnss_boundary():
    suffix = str(os.getpid())
    rclpy.init()
    bridge = CarlaFeedbackBridgeNode(parameter_overrides=[
        Parameter(
            "input_odometry_topic",
            value="/camrod_carla/test/compat_odom_" + suffix,
        ),
        Parameter(
            "input_gnss_topic",
            value="/camrod_carla/test/compat_gnss_left_" + suffix,
        ),
        Parameter(
            "input_gnss_right_topic",
            value="/camrod_carla/test/compat_gnss_right_" + suffix,
        ),
        Parameter(
            "output_gnss_topic",
            value="/camrod_carla/test/compat_fix_" + suffix,
        ),
        Parameter(
            "output_gnss_navpvt_topic",
            value="/camrod_carla/test/compat_navpvt_" + suffix,
        ),
        Parameter(
            "output_gnss_navcov_topic",
            value="/camrod_carla/test/compat_navcov_" + suffix,
        ),
        Parameter(
            "output_gnss_relpos_topic",
            value="/camrod_carla/test/compat_relpos_" + suffix,
        ),
        Parameter("publish_platform_odometry", value=False),
        Parameter("publish_metric_pose", value=False),
        Parameter("publish_ground_truth_localization", value=False),
        Parameter("publish_ground_truth_tf", value=False),
        Parameter("relay_imu", value=False),
        Parameter("relay_gnss", value=True),
        Parameter("publish_gnss_ui_compat", value=True),
    ])

    try:
        fix_output = _Collector()
        navpvt_output = _Collector()
        navcov_output = _Collector()
        relpos_output = _Collector()
        bridge.gnss_publisher = fix_output
        bridge.gnss_navpvt_publisher = navpvt_output
        bridge.gnss_navcov_publisher = navcov_output
        bridge.gnss_relpos_publisher = relpos_output

        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 0.4
        bridge._on_odometry(odometry)

        right = NavSatFix()
        right.status.status = NavSatStatus.STATUS_FIX
        right.latitude = 37.0
        right.longitude = 127.0
        right.altitude = 100.0
        left = NavSatFix()
        left.status.status = NavSatStatus.STATUS_FIX
        left.latitude = right.latitude + math.degrees(
            0.9 / EARTH_SEMIMAJOR_AXIS_M
        )
        left.longitude = right.longitude
        left.altitude = right.altitude

        bridge._on_gnss(left)
        bridge._on_gnss_right(right)

        assert len(fix_output.messages) == 1
        assert len(navpvt_output.messages) == 1
        assert len(navcov_output.messages) == 1
        assert len(relpos_output.messages) == 1
        assert navpvt_output.messages[0].g_speed == 400
        assert relpos_output.messages[0].rel_pos_length == 90
        assert bridge._last_gnss_monotonic is not None
        assert bridge._last_gnss_right_monotonic is not None
        _, errors, stale = bridge._stream_health(time.monotonic())
        assert errors == []
        assert "gnss" not in stale
        assert "gnss_right" not in stale
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
