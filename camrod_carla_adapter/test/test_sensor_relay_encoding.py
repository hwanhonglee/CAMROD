"""Unit contracts for in-process CARLA image compression."""

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from camrod_carla_adapter.sensor_relay_node import encode_image_jpeg


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
