"""Convert standard CARLA radar PointCloud2 streams to CAMROD ranges."""

import math
import time

from avg_msgs.msg import AvgBool, AvgRange
from camrod_carla_adapter.radar_mapping import (
    CarlaRadarDetection,
    build_channel_specs,
    build_co_moving_near_field_exclusions,
    filter_co_moving_near_field_detections,
    select_nearest_range,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2


CHANNEL_NAMES = (
    'front1',
    'front2',
    'left1',
    'left2',
    'right1',
    'right2',
    'rear',
)
DEFAULT_OUTPUT_TOPICS = tuple(
    f'/sensing/radar/{name}/range' for name in CHANNEL_NAMES
)
DEFAULT_STANDARD_OUTPUT_TOPICS = tuple(
    f'/sensing/radar/{name}/range_ros' for name in CHANNEL_NAMES
)
DEFAULT_FRAME_IDS = tuple(f'radar_{name}_link' for name in CHANNEL_NAMES)
DEFAULT_MIN_RANGES_M = (0.02,) * len(CHANNEL_NAMES)
DEFAULT_MAX_RANGES_M = (1.5, 1.5, 0.8, 0.8, 0.8, 0.8, 0.5)
DEFAULT_FIELD_OF_VIEWS_RAD = (0.26,) * len(CHANNEL_NAMES)


def ranges_from_carla_cloud(message: PointCloud2) -> list[float]:
    """Extract the standard bridge's case-sensitive ``Range`` field."""
    names = {field.name.lower(): field.name for field in message.fields}
    range_field = names.get('range')
    if range_field is None:
        raise ValueError('CARLA radar PointCloud2 has no Range field')
    try:
        points = point_cloud2.read_points(
            message,
            field_names=[range_field],
            skip_nans=False,
        )
    except (AssertionError, TypeError, ValueError) as error:
        raise ValueError(f'invalid CARLA radar PointCloud2: {error}') from error
    return [float(value) for value in points[range_field]]


def detections_from_carla_cloud(
    message: PointCloud2,
) -> list[CarlaRadarDetection]:
    """Extract CARLA range, Doppler and ray angles without scalarizing first."""
    names = {field.name.lower(): field.name for field in message.fields}
    required = ('range', 'velocity', 'azimuthangle', 'elevationangle')
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(
            'CARLA radar PointCloud2 lacks raw filter fields: '
            + ','.join(missing)
        )
    field_names = [names[name] for name in required]
    try:
        points = point_cloud2.read_points(
            message,
            field_names=field_names,
            skip_nans=False,
        )
    except (AssertionError, TypeError, ValueError) as error:
        raise ValueError(f'invalid CARLA radar PointCloud2: {error}') from error
    return [
        CarlaRadarDetection(
            range_m=float(point[field_names[0]]),
            velocity_mps=float(point[field_names[1]]),
            azimuth_rad=float(point[field_names[2]]),
            elevation_rad=float(point[field_names[3]]),
        )
        for point in points
    ]


class CarlaRadarRelayNode(Node):
    """Own the sole CARLA PointCloud2 to CAMROD seven-range boundary."""

    def __init__(self, **node_kwargs):
        super().__init__('carla_radar_relay', **node_kwargs)
        role_name = str(self.declare_parameter('role_name', 'ego_vehicle').value)
        role_name = role_name.strip().strip('/')
        if not role_name:
            raise ValueError('role_name must be non-empty')

        names = tuple(self.declare_parameter(
            'channel_names', list(CHANNEL_NAMES)
        ).value)
        default_inputs = [f'/carla/{role_name}/radar_{name}' for name in names]
        input_topics = tuple(self.declare_parameter(
            'input_topics', default_inputs
        ).value)
        output_topics = tuple(self.declare_parameter(
            'output_topics', list(DEFAULT_OUTPUT_TOPICS)
        ).value)
        standard_output_topics = tuple(self.declare_parameter(
            'standard_output_topics', list(DEFAULT_STANDARD_OUTPUT_TOPICS)
        ).value)
        frame_ids = tuple(self.declare_parameter(
            'frame_ids', list(DEFAULT_FRAME_IDS)
        ).value)
        min_ranges_m = tuple(self.declare_parameter(
            'min_ranges_m', list(DEFAULT_MIN_RANGES_M)
        ).value)
        max_ranges_m = tuple(self.declare_parameter(
            'max_ranges_m', list(DEFAULT_MAX_RANGES_M)
        ).value)
        field_of_views_rad = tuple(self.declare_parameter(
            'field_of_views_rad', list(DEFAULT_FIELD_OF_VIEWS_RAD)
        ).value)
        self._no_return_epsilon_m = float(self.declare_parameter(
            'no_return_epsilon_m', 0.001
        ).value)
        if (
            not math.isfinite(self._no_return_epsilon_m)
            or self._no_return_epsilon_m <= 0.0
        ):
            raise ValueError('no_return_epsilon_m must be finite and positive')
        self._stamp_with_reception_time = bool(self.declare_parameter(
            'stamp_with_reception_time', True
        ).value)
        self._stream_timeout_sec = float(self.declare_parameter(
            'stream_timeout_sec', 3.0
        ).value)
        if (
            not math.isfinite(self._stream_timeout_sec)
            or self._stream_timeout_sec <= 0.0
        ):
            raise ValueError('stream_timeout_sec must be finite and positive')

        self._co_moving_near_field_filter_enable = bool(
            self.declare_parameter(
                'co_moving_near_field_filter_enable', False
            ).value
        )
        self._co_moving_near_field_max_abs_velocity_mps = float(
            self.declare_parameter(
                'co_moving_near_field_filter_max_abs_velocity_mps', 0.05
            ).value
        )
        raw_exclusions = tuple(self.declare_parameter(
            'co_moving_near_field_filter_exclusions', []
        ).value)
        self._co_moving_near_field_exclusions = (
            build_co_moving_near_field_exclusions(raw_exclusions, names)
        )
        # Validate the velocity bound even when no cloud has arrived.  A bad
        # safety profile must fail at startup rather than silently stop filtering.
        filter_co_moving_near_field_detections(
            names[0] if names else '',
            (),
            self._co_moving_near_field_exclusions,
            self._co_moving_near_field_max_abs_velocity_mps,
        )
        if (
            self._co_moving_near_field_filter_enable
            and not self._co_moving_near_field_exclusions
        ):
            raise ValueError(
                'co-moving near-field filter enabled without exclusions'
            )

        self._specs = build_channel_specs(
            names,
            input_topics,
            output_topics,
            standard_output_topics,
            frame_ids,
            min_ranges_m,
            max_ranges_m,
            field_of_views_rad,
        )

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        dummy_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._avg_publishers = [
            self.create_publisher(AvgRange, spec.output_topic, reliable)
            for spec in self._specs
        ]
        self._ros_publishers = [
            self.create_publisher(Range, spec.standard_output_topic, reliable)
            for spec in self._specs
        ]
        self._dummy_publishers = [
            self.create_publisher(
                AvgBool,
                spec.output_topic.removesuffix('/range') + '/dummy_active',
                dummy_qos,
            )
            for spec in self._specs
        ]
        self._global_dummy_publisher = self.create_publisher(
            AvgBool, '/sensing/radar/dummy_active', dummy_qos
        )
        self._subscriptions = [
            self.create_subscription(
                PointCloud2,
                spec.input_topic,
                lambda message, index=index: self._on_cloud(index, message),
                reliable,
            )
            for index, spec in enumerate(self._specs)
        ]

        self._last_seen = [None] * len(self._specs)
        self._last_ranges = [math.nan] * len(self._specs)
        self._last_input_counts = [0] * len(self._specs)
        self._last_valid_counts = [0] * len(self._specs)
        self._last_filtered_counts = [0] * len(self._specs)
        self._stream_errors = [''] * len(self._specs)
        self._status_publisher = self.create_publisher(
            DiagnosticArray, '/camrod_carla/radar_relay/status', 10
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self._status_timer = self.create_timer(0.5, self._publish_status)

        physical = AvgBool()
        physical.data = False
        self._global_dummy_publisher.publish(physical)
        for publisher in self._dummy_publishers:
            publisher.publish(physical)
        self.get_logger().info(
            'CARLA radar relay configured for '
            f'{len(self._specs)} physical-simulation channels'
        )
        if self._co_moving_near_field_filter_enable:
            self.get_logger().warn(
                'CARLA co-moving near-field return filter active: '
                f'exclusions={len(self._co_moving_near_field_exclusions)} '
                'range-only filtering is forbidden'
            )

    def _on_cloud(self, index, message):
        spec = self._specs[index]
        try:
            filtered_count = 0
            if self._co_moving_near_field_filter_enable:
                raw_detections = detections_from_carla_cloud(message)
                detections, filtered_count = (
                    filter_co_moving_near_field_detections(
                        spec.name,
                        raw_detections,
                        self._co_moving_near_field_exclusions,
                        self._co_moving_near_field_max_abs_velocity_mps,
                    )
                )
                detection_ranges = [
                    detection.range_m for detection in detections
                ]
                input_detection_count = len(raw_detections)
            else:
                detection_ranges = ranges_from_carla_cloud(message)
                input_detection_count = len(detection_ranges)
            selection = select_nearest_range(
                detection_ranges,
                spec.min_range_m,
                spec.max_range_m,
                self._no_return_epsilon_m,
            )
        except ValueError as error:
            text = str(error)
            if self._stream_errors[index] != text:
                self.get_logger().error(f'{spec.name} radar conversion failed: {text}')
            self._stream_errors[index] = text
            return

        stamp = (
            self.get_clock().now().to_msg()
            if self._stamp_with_reception_time
            else message.header.stamp
        )
        avg_message = AvgRange()
        avg_message.header.stamp = stamp
        avg_message.header.frame_id = spec.frame_id
        avg_message.radiation_type = AvgRange.ULTRASOUND
        avg_message.field_of_view = spec.field_of_view_rad
        avg_message.min_range = spec.min_range_m
        avg_message.max_range = spec.max_range_m
        avg_message.range = selection.range_m
        self._avg_publishers[index].publish(avg_message)

        ros_message = Range()
        ros_message.header.stamp = stamp
        ros_message.header.frame_id = spec.frame_id
        ros_message.radiation_type = Range.ULTRASOUND
        ros_message.field_of_view = spec.field_of_view_rad
        ros_message.min_range = spec.min_range_m
        ros_message.max_range = spec.max_range_m
        ros_message.range = selection.range_m
        self._ros_publishers[index].publish(ros_message)

        physical = AvgBool()
        physical.data = False
        self._dummy_publishers[index].publish(physical)
        self._global_dummy_publisher.publish(physical)
        self._last_seen[index] = time.monotonic()
        self._last_ranges[index] = selection.range_m
        self._last_input_counts[index] = input_detection_count
        self._last_valid_counts[index] = selection.valid_detection_count
        self._last_filtered_counts[index] = filtered_count
        self._stream_errors[index] = ''

    def _publish_status(self):
        now = time.monotonic()
        stale = []
        errors = []
        values = []
        for index, spec in enumerate(self._specs):
            seen = self._last_seen[index]
            age = math.inf if seen is None else now - seen
            if self._stream_errors[index]:
                errors.append(spec.name)
            elif age < 0.0 or age > self._stream_timeout_sec:
                stale.append(spec.name)
            values.extend((
                KeyValue(key=f'{spec.name}_age_sec', value=str(age)),
                KeyValue(
                    key=f'{spec.name}_range_m',
                    value=str(self._last_ranges[index]),
                ),
                KeyValue(
                    key=f'{spec.name}_detections',
                    value=(
                        f'{self._last_valid_counts[index]}/'
                        f'{self._last_input_counts[index]}'
                    ),
                ),
                KeyValue(
                    key=f'{spec.name}_filtered_co_moving_near_field',
                    value=str(self._last_filtered_counts[index]),
                ),
            ))
            if self._stream_errors[index]:
                values.append(KeyValue(
                    key=f'{spec.name}_error',
                    value=self._stream_errors[index],
                ))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'camrod_carla/radar_relay'
        status.hardware_id = 'vehicle.ranger.default'
        status.values = values
        if errors:
            status.level = DiagnosticStatus.ERROR
            status.message = 'CARLA radar conversion failed: ' + ','.join(errors)
        elif stale:
            status.level = DiagnosticStatus.WARN
            status.message = 'CARLA radar streams unavailable/stale: ' + ','.join(stale)
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'CARLA radar streams active'
        array.status = [status]
        self._status_publisher.publish(array)
        self._diagnostics_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaRadarRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
