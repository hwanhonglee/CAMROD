"""Unit tests for disabled-hardware sensing placeholder contracts."""

import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / (
    "scripts/sensing_dummy_publisher.py"
)
SPEC = importlib.util.spec_from_file_location(
    "sensing_dummy_publisher", SCRIPT_PATH
)
SENSING_DUMMY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SENSING_DUMMY)


def _stamp():
    return SENSING_DUMMY.NavSatFix().header.stamp


def test_public_topics_and_dummy_markers_are_absolute():
    data_topics = (
        SENSING_DUMMY.GNSS_FIX_TOPIC,
        SENSING_DUMMY.GNSS_HEADING_TOPIC,
        SENSING_DUMMY.IMU_RAW_TOPIC,
        SENSING_DUMMY.LIDAR_RAW_TOPIC,
        SENSING_DUMMY.LIDAR_FILTERED_TOPIC,
        SENSING_DUMMY.FRONT_CAMERA_RAW_TOPIC,
        SENSING_DUMMY.FRONT_CAMERA_COMPRESSED_TOPIC,
        SENSING_DUMMY.FRONT_CAMERA_INFO_TOPIC,
        SENSING_DUMMY.REAR_CAMERA_RAW_TOPIC,
        SENSING_DUMMY.REAR_CAMERA_COMPRESSED_TOPIC,
        SENSING_DUMMY.REAR_CAMERA_INFO_TOPIC,
        *SENSING_DUMMY.DUMMY_ACTIVE_TOPICS.values(),
    )

    assert all(topic.startswith("/") for topic in data_topics)
    assert SENSING_DUMMY.DUMMY_ACTIVE_TOPICS == {
        "gnss": "/sensing/gnss/dummy_active",
        "imu": "/sensing/imu/dummy_active",
        "lidar": "/sensing/lidar/dummy_active",
        "front_camera": "/sensing/camera/econ_front/dummy_active",
        "rear_camera": "/sensing/camera/econ_rear/dummy_active",
    }


def test_enabled_groups_have_stable_order_and_independent_switches():
    assert SENSING_DUMMY.enabled_groups(
        True, False, True, False, True
    ) == ("gnss", "lidar", "rear_camera")
    assert SENSING_DUMMY.enabled_groups(
        False, False, False, False, False
    ) == ()


@pytest.mark.parametrize("invalid_rate", (0.0, -1.0, math.nan, math.inf, 20.1))
def test_invalid_publish_rate_is_rejected(invalid_rate):
    with pytest.raises(ValueError, match="publish_rate_hz"):
        SENSING_DUMMY.validate_publish_rate(invalid_rate)


def test_gnss_dummy_is_explicitly_not_a_fix():
    message = SENSING_DUMMY.make_gnss_fix(_stamp())

    assert message.header.frame_id == "gps"
    assert message.status.status == SENSING_DUMMY.NavSatStatus.STATUS_NO_FIX
    assert message.status.service == SENSING_DUMMY.NavSatStatus.SERVICE_GPS
    assert math.isnan(message.latitude)
    assert math.isnan(message.longitude)
    assert math.isnan(message.altitude)
    assert (
        message.position_covariance_type
        == SENSING_DUMMY.NavSatFix.COVARIANCE_TYPE_UNKNOWN
    )
    assert message.position_covariance[0] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )
    assert message.position_covariance[4] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )
    assert message.position_covariance[8] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )
    assert all(
        message.position_covariance[index] == 0.0
        for index in (1, 2, 3, 5, 6, 7)
    )


def test_gnss_heading_cannot_be_accepted_as_precise():
    message = SENSING_DUMMY.make_gnss_heading(_stamp())

    assert message.header.frame_id == "gps"
    assert message.orientation.w == 1.0
    assert message.orientation_covariance[0] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )
    assert message.orientation_covariance[4] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )
    assert message.orientation_covariance[8] == (
        SENSING_DUMMY.HIGH_UNCERTAINTY
    )


def test_imu_dummy_is_finite_stationary_and_high_uncertainty():
    message = SENSING_DUMMY.make_imu_raw(_stamp())

    assert message.header.frame_id == "imu_link"
    assert message.orientation.x == 0.0
    assert message.orientation.y == 0.0
    assert message.orientation.z == 0.0
    assert message.orientation.w == 1.0
    assert message.angular_velocity.x == 0.0
    assert message.angular_velocity.y == 0.0
    assert message.angular_velocity.z == 0.0
    assert message.linear_acceleration.x == 0.0
    assert message.linear_acceleration.y == 0.0
    assert message.linear_acceleration.z == 0.0
    for covariance in (
        message.orientation_covariance,
        message.angular_velocity_covariance,
        message.linear_acceleration_covariance,
    ):
        assert covariance[0] == SENSING_DUMMY.HIGH_UNCERTAINTY
        assert covariance[4] == SENSING_DUMMY.HIGH_UNCERTAINTY
        assert covariance[8] == SENSING_DUMMY.HIGH_UNCERTAINTY


@pytest.mark.parametrize(
    "frame_id", ("camera_front", "camera_rear")
)
def test_camera_dummy_is_a_valid_one_pixel_schema(frame_id):
    raw = SENSING_DUMMY.make_raw_camera_image(_stamp(), frame_id)
    compressed = SENSING_DUMMY.make_compressed_camera_image(
        _stamp(), frame_id
    )
    info = SENSING_DUMMY.make_camera_info(_stamp(), frame_id)

    assert raw.header.frame_id == frame_id
    assert (raw.width, raw.height, raw.encoding, raw.step) == (
        1, 1, "bgr8", 3
    )
    assert bytes(raw.data) == b"\x00\x00\x00"

    assert compressed.header.frame_id == frame_id
    assert compressed.format == "jpeg"
    assert bytes(compressed.data).startswith(b"\xff\xd8")
    assert bytes(compressed.data).endswith(b"\xff\xd9")
    assert len(compressed.data) > 100
    decoded = cv2.imdecode(
        np.frombuffer(bytes(compressed.data), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    assert decoded is not None
    assert decoded.shape == (1, 1, 3)
    assert np.array_equal(decoded, np.zeros((1, 1, 3), dtype=np.uint8))

    assert info.header.frame_id == frame_id
    assert (info.width, info.height) == (1, 1)
    assert len(info.k) == 9
    assert len(info.r) == 9
    assert len(info.p) == 12


def test_lidar_dummy_is_empty_xyz_cloud_in_lidar_frame():
    message = SENSING_DUMMY.make_empty_point_cloud(_stamp())

    assert message.header.frame_id == "lidar_link"
    assert (message.width, message.height) == (0, 1)
    assert [field.name for field in message.fields] == ["x", "y", "z"]
    assert [field.offset for field in message.fields] == [0, 4, 8]
    assert all(
        field.datatype == SENSING_DUMMY.PointField.FLOAT32
        for field in message.fields
    )
    assert message.point_step == 12
    assert message.row_step == 0
    assert bytes(message.data) == b""
    assert message.is_dense
