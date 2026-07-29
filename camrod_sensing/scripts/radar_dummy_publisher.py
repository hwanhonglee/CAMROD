#!/usr/bin/env python3
"""Publish no-target radar heartbeats while physical radar is disabled."""

import math
from typing import Iterable, List, Sequence

import rclpy
from avg_msgs.msg import AvgBool, AvgRange
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range as RosRange


EXPECTED_SENSOR_NAMES = (
    "FRONT1",
    "FRONT2",
    "LEFT1",
    "LEFT2",
    "RIGHT1",
    "RIGHT2",
    "REAR",
)
DEFAULT_FRAME_IDS = (
    "radar_front1_link",
    "radar_front2_link",
    "radar_left1_link",
    "radar_left2_link",
    "radar_right1_link",
    "radar_right2_link",
    "radar_rear_link",
)
DEFAULT_TOPICS = (
    "front1/range",
    "front2/range",
    "left1/range",
    "left2/range",
    "right1/range",
    "right2/range",
    "rear/range",
)
DEFAULT_STANDARD_ROS_TOPICS = tuple(
    f"{topic}_ros" for topic in DEFAULT_TOPICS
)
DEFAULT_MAX_RANGES_M = (1.50, 1.50, 0.80, 0.80, 0.80, 0.80, 0.50)
CANONICAL_RADAR_NAMESPACE = "/sensing/radar"


def _as_list(values: Iterable, parameter_name: str) -> List:
    """Return a concrete list and reject scalar strings early."""
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{parameter_name} must be an array, not a string")
    try:
        return list(values)
    except TypeError as exc:
        raise ValueError(f"{parameter_name} must be an array") from exc


def _validate_nonempty_unique_strings(
    values: Sequence, parameter_name: str
) -> List[str]:
    result = [str(value).strip() for value in _as_list(values, parameter_name)]
    if any(not value for value in result):
        raise ValueError(f"{parameter_name} entries must not be empty")
    if len(set(result)) != len(result):
        raise ValueError(f"{parameter_name} entries must be unique")
    return result


def _canonical_radar_topic(topic: str) -> str:
    """Resolve a SEN0592 YAML suffix into the fixed public boundary."""
    topic = str(topic).strip()
    if not topic:
        raise ValueError("radar topic must not be empty")
    if any(character.isspace() for character in topic):
        raise ValueError(f"radar topic contains whitespace: {topic!r}")
    if topic.startswith("/"):
        canonical = topic
    else:
        canonical = f"{CANONICAL_RADAR_NAMESPACE}/{topic.lstrip('/')}"
    if "//" in canonical or canonical.endswith("/"):
        raise ValueError(f"invalid radar topic: {canonical!r}")
    return canonical


def per_channel_dummy_active_topic(range_topic: str) -> str:
    """Return the marker beside one canonical radar AvgRange topic."""
    canonical_range_topic = _canonical_radar_topic(range_topic)
    suffix = "/range"
    if not canonical_range_topic.endswith(suffix):
        raise ValueError(
            "radar range topic must end with /range: "
            f"{canonical_range_topic}"
        )
    return canonical_range_topic[: -len(suffix)] + "/dummy_active"


def validate_radar_configuration(
    sensor_names: Sequence,
    frame_ids: Sequence,
    topics: Sequence,
    standard_ros_topics: Sequence,
    software_min_range_m: float,
    software_max_ranges_m: Sequence,
    field_of_view_rad: float,
    publish_rate_hz: float,
    no_target_epsilon_m: float,
) -> tuple[List[str], List[str], List[str], List[str], List[float]]:
    """Validate and normalize the common SEN0592/dummy configuration."""
    names = _validate_nonempty_unique_strings(sensor_names, "sensor_names")
    frames = _validate_nonempty_unique_strings(frame_ids, "frame_ids")
    raw_topics = _validate_nonempty_unique_strings(topics, "topics")
    raw_ros_topics = _validate_nonempty_unique_strings(
        standard_ros_topics, "standard_ros_topics"
    )
    max_ranges = [
        float(value)
        for value in _as_list(
            software_max_ranges_m, "software_max_ranges_m"
        )
    ]

    expected_count = len(EXPECTED_SENSOR_NAMES)
    lengths = {
        "sensor_names": len(names),
        "frame_ids": len(frames),
        "topics": len(raw_topics),
        "standard_ros_topics": len(raw_ros_topics),
        "software_max_ranges_m": len(max_ranges),
    }
    wrong_lengths = {
        name: length
        for name, length in lengths.items()
        if length != expected_count
    }
    if wrong_lengths:
        details = ", ".join(
            f"{name}={length}" for name, length in wrong_lengths.items()
        )
        raise ValueError(
            "dummy radar requires exactly "
            f"{expected_count} entries per array; {details}"
        )
    if set(names) != set(EXPECTED_SENSOR_NAMES):
        raise ValueError(
            "sensor_names must contain exactly "
            + ", ".join(EXPECTED_SENSOR_NAMES)
        )

    normalized_topics = [
        _canonical_radar_topic(topic) for topic in raw_topics
    ]
    normalized_ros_topics = [
        _canonical_radar_topic(topic) for topic in raw_ros_topics
    ]
    if len(set(normalized_topics)) != expected_count:
        raise ValueError("topics resolve to duplicate canonical names")
    if len(set(normalized_ros_topics)) != expected_count:
        raise ValueError(
            "standard_ros_topics resolve to duplicate canonical names"
        )

    for index, sensor_name in enumerate(names):
        sensor_suffix = sensor_name.lower()
        expected_topic = f"{CANONICAL_RADAR_NAMESPACE}/{sensor_suffix}/range"
        expected_ros_topic = f"{expected_topic}_ros"
        if normalized_topics[index] != expected_topic:
            raise ValueError(
                f"topics[{index}] for {sensor_name} must resolve to "
                f"{expected_topic}, "
                f"got {normalized_topics[index]}"
            )
        if normalized_ros_topics[index] != expected_ros_topic:
            raise ValueError(
                f"standard_ros_topics[{index}] for {sensor_name} must "
                "resolve to "
                f"{expected_ros_topic}, got {normalized_ros_topics[index]}"
            )

    min_range = float(software_min_range_m)
    if not math.isfinite(min_range) or min_range < 0.0:
        raise ValueError("software_min_range_m must be finite and >= 0")
    for index, max_range in enumerate(max_ranges):
        if not math.isfinite(max_range) or max_range <= min_range:
            raise ValueError(
                f"software_max_ranges_m[{index}] must be finite and "
                "greater than "
                "software_min_range_m"
            )

    fov = float(field_of_view_rad)
    if not math.isfinite(fov) or fov <= 0.0 or fov > 2.0 * math.pi:
        raise ValueError(
            "range_message_field_of_view_rad must be in (0, 2*pi]"
        )
    rate = float(publish_rate_hz)
    if not math.isfinite(rate) or rate <= 0.0 or rate > 100.0:
        raise ValueError("publish_rate_hz must be in (0, 100]")
    epsilon = float(no_target_epsilon_m)
    if not math.isfinite(epsilon) or epsilon <= 0.0 or epsilon > 0.10:
        raise ValueError("no_target_epsilon_m must be in (0, 0.10]")

    return (
        names,
        frames,
        normalized_topics,
        normalized_ros_topics,
        max_ranges,
    )


class RadarDummyPublisher(Node):
    """Publish seven transport heartbeats that explicitly contain no hit."""

    def __init__(self) -> None:
        super().__init__("radar_dummy_publisher")

        sensor_names = self.declare_parameter(
            "sensor_names", list(EXPECTED_SENSOR_NAMES)
        ).value
        frame_ids = self.declare_parameter(
            "frame_ids", list(DEFAULT_FRAME_IDS)
        ).value
        topics = self.declare_parameter("topics", list(DEFAULT_TOPICS)).value
        standard_ros_topics = self.declare_parameter(
            "standard_ros_topics", list(DEFAULT_STANDARD_ROS_TOPICS)
        ).value
        self._min_range_m = float(
            self.declare_parameter("software_min_range_m", 0.02).value
        )
        default_max_range_m = float(
            self.declare_parameter("software_default_max_range_m", 4.50).value
        )
        configured_max_ranges = list(
            self.declare_parameter(
                "software_max_ranges_m", list(DEFAULT_MAX_RANGES_M)
            ).value
        )
        # HH_260729 - An empty per-channel list uses the same fallback as the
        # real SEN0592 driver; otherwise all seven entries are required.
        if not configured_max_ranges:
            configured_max_ranges = [
                default_max_range_m
            ] * len(EXPECTED_SENSOR_NAMES)
        self._field_of_view_rad = float(
            self.declare_parameter(
                "range_message_field_of_view_rad", 0.26
            ).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self._no_target_epsilon_m = float(
            self.declare_parameter("no_target_epsilon_m", 0.001).value
        )
        self._dummy_active_topic = str(
            self.declare_parameter(
                "dummy_active_topic", "/sensing/radar/dummy_active"
            ).value
        ).strip()
        if self._dummy_active_topic != "/sensing/radar/dummy_active":
            raise ValueError(
                "dummy_active_topic must remain "
                "/sensing/radar/dummy_active so "
                "diagnostics cannot silently miss dummy mode"
            )

        (
            self._sensor_names,
            self._frame_ids,
            self._topics,
            self._standard_ros_topics,
            self._max_ranges_m,
        ) = validate_radar_configuration(
            sensor_names,
            frame_ids,
            topics,
            standard_ros_topics,
            self._min_range_m,
            configured_max_ranges,
            self._field_of_view_rad,
            self._publish_rate_hz,
            self._no_target_epsilon_m,
        )

        # HH_260729 - Match the real driver/cost-grid SensorDataQoS boundary.
        # The dummy carries only current no-target state, so queued history
        # adds latency without diagnostic value.
        self._range_publishers = [
            self.create_publisher(AvgRange, topic, qos_profile_sensor_data)
            for topic in self._topics
        ]
        self._standard_ros_publishers = [
            self.create_publisher(RosRange, topic, qos_profile_sensor_data)
            for topic in self._standard_ros_topics
        ]
        dummy_state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._dummy_active_publisher = self.create_publisher(
            AvgBool, self._dummy_active_topic, dummy_state_qos
        )
        # HH_260729 - The full-radar dummy owns all seven channels, so publish
        # both the legacy group marker and one explicit marker per channel.
        # Per-channel diagnostics can then identify each replacement without
        # treating transport-only no-target data as physical radar input.
        self._per_channel_dummy_active_topics = [
            per_channel_dummy_active_topic(topic) for topic in self._topics
        ]
        self._per_channel_dummy_active_publishers = [
            self.create_publisher(AvgBool, topic, dummy_state_qos)
            for topic in self._per_channel_dummy_active_topics
        ]
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz, self._publish_heartbeats
        )
        self.get_logger().warning(
            "HH_260729 DUMMY RADAR ACTIVE: physical SEN0592 polling is "
            "disabled; publishing seven no-target heartbeats at "
            f"{self._publish_rate_hz:.1f} Hz"
        )

    @staticmethod
    def _to_standard_ros(message: AvgRange) -> RosRange:
        output = RosRange()
        output.header.stamp = message.header.stamp
        output.header.frame_id = message.header.frame_id
        output.radiation_type = message.radiation_type
        output.field_of_view = message.field_of_view
        output.min_range = message.min_range
        output.max_range = message.max_range
        output.range = message.range
        return output

    def _publish_heartbeats(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for index, publisher in enumerate(self._range_publishers):
            message = AvgRange()
            message.header.stamp = stamp
            message.header.frame_id = self._frame_ids[index]
            message.radiation_type = AvgRange.ULTRASOUND
            message.field_of_view = self._field_of_view_rad
            message.min_range = self._min_range_m
            message.max_range = self._max_ranges_m[index]
            # HH_260729 - Match the real driver's no-target heartbeat.
            # range > max_range keeps topic-health checks alive while every
            # obstacle/cost consumer deterministically discards this sample.
            message.range = (
                self._max_ranges_m[index] + self._no_target_epsilon_m
            )
            publisher.publish(message)
            self._standard_ros_publishers[index].publish(
                self._to_standard_ros(message)
            )

        dummy_active = AvgBool()
        dummy_active.data = True
        for publisher in self._per_channel_dummy_active_publishers:
            publisher.publish(dummy_active)
        self._dummy_active_publisher.publish(dummy_active)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RadarDummyPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(
                f"invalid radar dummy configuration: {exc}"
            )
        else:
            rclpy.logging.get_logger("radar_dummy_publisher").fatal(
                f"invalid radar dummy configuration: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
