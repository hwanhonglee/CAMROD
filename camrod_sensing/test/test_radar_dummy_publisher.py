"""Unit tests for the disabled-hardware radar heartbeat contract."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / (
    "scripts/radar_dummy_publisher.py"
)
SPEC = importlib.util.spec_from_file_location(
    "radar_dummy_publisher", SCRIPT_PATH
)
RADAR_DUMMY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RADAR_DUMMY)


def _valid_arguments():
    return (
        RADAR_DUMMY.EXPECTED_SENSOR_NAMES,
        RADAR_DUMMY.DEFAULT_FRAME_IDS,
        RADAR_DUMMY.DEFAULT_TOPICS,
        RADAR_DUMMY.DEFAULT_STANDARD_ROS_TOPICS,
        0.02,
        RADAR_DUMMY.DEFAULT_MAX_RANGES_M,
        0.26,
        10.0,
        0.001,
    )


def test_common_yaml_topics_resolve_to_seven_canonical_boundaries():
    names, frames, topics, ros_topics, max_ranges = (
        RADAR_DUMMY.validate_radar_configuration(*_valid_arguments())
    )

    assert names == list(RADAR_DUMMY.EXPECTED_SENSOR_NAMES)
    assert frames == list(RADAR_DUMMY.DEFAULT_FRAME_IDS)
    assert topics == [
        f"/sensing/radar/{name.lower()}/range" for name in names
    ]
    assert ros_topics == [f"{topic}_ros" for topic in topics]
    assert max_ranges == list(RADAR_DUMMY.DEFAULT_MAX_RANGES_M)


def test_all_seven_per_channel_dummy_markers_are_canonical_and_unique():
    marker_topics = [
        RADAR_DUMMY.per_channel_dummy_active_topic(topic)
        for topic in RADAR_DUMMY.DEFAULT_TOPICS
    ]

    assert marker_topics == [
        f"/sensing/radar/{name.lower()}/dummy_active"
        for name in RADAR_DUMMY.EXPECTED_SENSOR_NAMES
    ]
    assert len(set(marker_topics)) == len(RADAR_DUMMY.EXPECTED_SENSOR_NAMES)


@pytest.mark.parametrize(
    "invalid_topic",
    (
        "/sensing/radar/front1/range_ros",
        "/sensing/radar/front1/status",
        "",
    ),
)
def test_per_channel_dummy_marker_rejects_non_range_topic(invalid_topic):
    with pytest.raises(ValueError):
        RADAR_DUMMY.per_channel_dummy_active_topic(invalid_topic)


def test_reordered_sensor_without_matching_topic_is_rejected():
    arguments = list(_valid_arguments())
    arguments[0] = (
        "FRONT2",
        "FRONT1",
        "LEFT1",
        "LEFT2",
        "RIGHT1",
        "RIGHT2",
        "REAR",
    )

    with pytest.raises(ValueError, match=r"topics\[0\].*FRONT2"):
        RADAR_DUMMY.validate_radar_configuration(*arguments)


@pytest.mark.parametrize(
    ("argument_index", "invalid_value", "error_match"),
    (
        (5, (1.5, 1.5), "exactly 7 entries"),
        (5, (1.5, 1.5, 0.8, 0.8, 0.8, 0.8, 0.01), "greater than"),
        (6, 0.0, "field_of_view"),
        (7, 0.0, "publish_rate_hz"),
        (8, 0.0, "no_target_epsilon_m"),
    ),
)
def test_unsafe_or_incomplete_configuration_is_rejected(
    argument_index, invalid_value, error_match
):
    arguments = list(_valid_arguments())
    arguments[argument_index] = invalid_value

    with pytest.raises(ValueError, match=error_match):
        RADAR_DUMMY.validate_radar_configuration(*arguments)
