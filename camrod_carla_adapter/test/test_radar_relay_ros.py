"""ROS message-boundary tests for the standard CARLA radar relay."""

import os

from builtin_interfaces.msg import Time
from camrod_carla_adapter.radar_relay_node import (
    CarlaRadarRelayNode,
    ranges_from_carla_cloud,
)
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


CARLA_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='Range', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='Velocity', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='AzimuthAngle', offset=20, datatype=PointField.FLOAT32, count=1),
    PointField(name='ElevationAngle', offset=28, datatype=PointField.FLOAT32, count=1),
]


class CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _cloud(ranges):
    header = Header()
    header.stamp = Time(sec=12, nanosec=34)
    header.frame_id = 'ego_vehicle/radar_front1'
    points = [
        (value, 0.0, 0.0, value, -0.5, 0.0, 0.0)
        for value in ranges
    ]
    return point_cloud2.create_cloud(header, CARLA_FIELDS, points)


def _one_channel_node():
    suffix = str(os.getpid())
    return CarlaRadarRelayNode(parameter_overrides=[
        Parameter('channel_names', value=['front1']),
        Parameter(
            'input_topics', value=[f'/camrod_carla/test/radar_in_{suffix}']
        ),
        Parameter(
            'output_topics', value=[f'/camrod_carla/test/range_{suffix}']
        ),
        Parameter(
            'standard_output_topics',
            value=[f'/camrod_carla/test/range_ros_{suffix}'],
        ),
        Parameter('frame_ids', value=['radar_front1_link']),
        Parameter('min_ranges_m', value=[0.02]),
        Parameter('max_ranges_m', value=[1.5]),
        Parameter('field_of_views_rad', value=[0.26]),
        Parameter('stamp_with_reception_time', value=False),
    ])


def test_standard_bridge_range_field_is_extracted_case_insensitively():
    assert ranges_from_carla_cloud(_cloud([0.7, 0.3])) == pytest.approx(
        [0.7, 0.3]
    )


def test_cloud_callback_publishes_generated_and_standard_range_contracts():
    rclpy.init()
    node = _one_channel_node()
    avg_capture = CapturePublisher()
    ros_capture = CapturePublisher()
    dummy_capture = CapturePublisher()
    global_dummy_capture = CapturePublisher()
    node._avg_publishers = [avg_capture]
    node._ros_publishers = [ros_capture]
    node._dummy_publishers = [dummy_capture]
    node._global_dummy_publisher = global_dummy_capture
    try:
        node._on_cloud(0, _cloud([0.8, 0.25, 1.7]))

        assert len(avg_capture.messages) == 1
        assert len(ros_capture.messages) == 1
        avg_message = avg_capture.messages[0]
        ros_message = ros_capture.messages[0]
        assert avg_message.header.frame_id == 'radar_front1_link'
        assert avg_message.header.stamp.sec == 12
        assert avg_message.range == pytest.approx(0.25)
        assert ros_message.range == pytest.approx(0.25)
        assert ros_message.max_range == pytest.approx(1.5)
        assert dummy_capture.messages[-1].data is False
        assert global_dummy_capture.messages[-1].data is False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_empty_cloud_stays_fresh_without_creating_an_obstacle():
    rclpy.init()
    node = _one_channel_node()
    capture = CapturePublisher()
    node._avg_publishers = [capture]
    node._ros_publishers = [CapturePublisher()]
    node._dummy_publishers = [CapturePublisher()]
    node._global_dummy_publisher = CapturePublisher()
    try:
        node._on_cloud(0, _cloud([]))
        assert capture.messages[-1].range == pytest.approx(1.501)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
