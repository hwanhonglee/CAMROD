"""Unit contracts for in-process CARLA image compression."""

import math

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from camrod_carla_adapter.sensor_relay_node import (
    CarlaSensorRelayNode,
    encode_image_jpeg,
)


class _Publisher:
    def __init__(self, subscription_count):
        self.subscription_count = subscription_count
        self.messages = []

    def get_subscription_count(self):
        return self.subscription_count

    def publish(self, message):
        self.messages.append(message)


def _relay_without_ros_init(monkeypatch, monotonic_values):
    relay = CarlaSensorRelayNode.__new__(CarlaSensorRelayNode)
    relay.jpeg_quality = 80
    relay.compressed_image_max_rate_hz = 10.0
    relay.raw_image_max_rate_hz = 10.0
    relay._compression_errors = {}
    relay._last_seen = {
        "front_image": 1.0,
        "front_compressed": None,
        "rear_compressed": None,
    }
    relay._last_compressed_publish_monotonic = {
        "front_compressed": None,
        "rear_compressed": None,
    }
    relay._last_raw_publish_monotonic = {
        "front_image": None,
        "rear_image": None,
    }
    clock = iter(monotonic_values)
    monkeypatch.setattr(
        "camrod_carla_adapter.sensor_relay_node.time.monotonic",
        lambda: next(clock),
    )
    return relay


def _valid_bgra_image():
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "bgra8"
    message.step = 8
    message.data = bytes(16)
    return message


@pytest.mark.parametrize(
    ("encoding", "channels"),
    (("mono8", 1), ("bgr8", 3), ("rgb8", 3), ("bgra8", 4), ("rgba8", 4)),
)
def test_common_carla_image_layouts_encode_as_jpeg(encoding, channels):
    message = Image()
    message.height = 2
    message.width = 3
    message.encoding = encoding
    message.step = message.width * channels
    message.data = bytes(range(message.height * message.step))

    payload = encode_image_jpeg(message, 80)
    decoded = cv2.imdecode(
        np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED
    )

    assert payload.startswith(b"\xff\xd8")
    assert decoded is not None
    assert decoded.shape[:2] == (message.height, message.width)


def test_row_padding_is_removed_before_jpeg_encoding():
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "bgr8"
    message.step = 8
    message.data = bytes(
        [0, 0, 255, 0, 255, 0, 99, 99, 255, 0, 0, 255, 255, 255, 88, 88]
    )

    payload = encode_image_jpeg(message, 90)
    decoded = cv2.imdecode(
        np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
    )

    assert decoded.shape == (2, 2, 3)


@pytest.mark.parametrize(
    "mutate",
    (
        lambda message: setattr(message, "encoding", "32FC1"),
        lambda message: setattr(message, "step", 1),
        lambda message: setattr(message, "data", bytes(2)),
    ),
)
def test_invalid_or_unsupported_images_fail_closed(mutate):
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "bgra8"
    message.step = 8
    message.data = bytes(16)
    mutate(message)

    with pytest.raises(ValueError):
        encode_image_jpeg(message, 80)


def test_compression_does_no_jpeg_work_without_a_subscriber(monkeypatch):
    relay = _relay_without_ros_init(monkeypatch, [])
    publisher = _Publisher(subscription_count=0)
    encode_calls = []
    monkeypatch.setattr(
        "camrod_carla_adapter.sensor_relay_node.encode_image_jpeg",
        lambda *_args: encode_calls.append(True),
    )

    relay._publish_compressed(
        _valid_bgra_image(), publisher, "front_compressed"
    )

    assert encode_calls == []
    assert publisher.messages == []
    assert relay._last_seen["front_compressed"] is None


def test_compression_is_capped_by_wall_clock_and_resumes_immediately(monkeypatch):
    relay = _relay_without_ros_init(monkeypatch, [1.0, 1.05, 1.10, 1.11])
    publisher = _Publisher(subscription_count=1)
    monkeypatch.setattr(
        "camrod_carla_adapter.sensor_relay_node.encode_image_jpeg",
        lambda *_args: b"jpeg",
    )
    image = _valid_bgra_image()

    relay._publish_compressed(image, publisher, "front_compressed")
    relay._publish_compressed(image, publisher, "front_compressed")
    relay._publish_compressed(image, publisher, "front_compressed")
    publisher.subscription_count = 0
    relay._publish_compressed(image, publisher, "front_compressed")
    publisher.subscription_count = 1
    relay._publish_compressed(image, publisher, "front_compressed")

    assert len(publisher.messages) == 3
    assert relay._last_seen["front_compressed"] == pytest.approx(1.11)


def test_inactive_compressed_streams_are_excluded_from_diagnostic_ages():
    relay = CarlaSensorRelayNode.__new__(CarlaSensorRelayNode)
    relay._last_seen = {
        "front_image": 10.0,
        "front_compressed": None,
        "rear_compressed": None,
    }
    relay._compressed_publishers = {
        "front_compressed": _Publisher(subscription_count=1),
        "rear_compressed": _Publisher(subscription_count=0),
    }

    ages = relay.active_stream_ages(11.0)

    assert ages["front_image"] == pytest.approx(1.0)
    assert math.isinf(ages["front_compressed"])
    assert "rear_compressed" not in ages


def test_raw_camera_publish_skips_idle_endpoints_and_is_wall_rate_limited():
    relay = CarlaSensorRelayNode.__new__(CarlaSensorRelayNode)
    relay.raw_image_max_rate_hz = 10.0
    relay._last_raw_publish_monotonic = {"front_image": None}
    idle = _Publisher(subscription_count=0)
    active = _Publisher(subscription_count=1)
    image = _valid_bgra_image()

    relay._publish_raw_image(image, [idle], "front_image", 1.0)
    relay._publish_raw_image(image, [idle, active], "front_image", 1.01)
    relay._publish_raw_image(image, [idle, active], "front_image", 1.05)
    relay._publish_raw_image(image, [idle, active], "front_image", 1.11)

    assert idle.messages == []
    assert active.messages == [image, image]
    assert relay._last_raw_publish_monotonic["front_image"] == pytest.approx(
        1.11
    )
