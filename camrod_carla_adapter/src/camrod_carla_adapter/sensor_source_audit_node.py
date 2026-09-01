"""Live ROS/CARLA audit for every sensor shown by the CAMROD operator UI."""

import argparse
import importlib
import json
import math
import time

from camrod_carla_adapter.sensor_source_audit import (
    ActorObservation,
    build_actor_contracts,
    build_stream_contracts,
    evaluate_actors,
    evaluate_streams,
    PublisherEndpoint,
    StreamObservation,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    Imu,
    NavSatFix,
    PointCloud2,
    PointField,
    Range,
)
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED9


MESSAGE_TYPES = {
    'sensor_msgs/msg/CameraInfo': CameraInfo,
    'sensor_msgs/msg/CompressedImage': CompressedImage,
    'sensor_msgs/msg/Image': Image,
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/NavSatFix': NavSatFix,
    'sensor_msgs/msg/PointCloud2': PointCloud2,
    'sensor_msgs/msg/Range': Range,
    'ublox_msgs/msg/NavCOV': NavCOV,
    'ublox_msgs/msg/NavPVT': NavPVT,
    'ublox_msgs/msg/NavRELPOSNED9': NavRELPOSNED9,
}


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _validate_lidar_cloud(contract, message: PointCloud2) -> str:
    """Validate nonempty source clouds and valid empty obstacle heartbeats."""
    if not str(message.header.frame_id).strip():
        return 'LiDAR frame_id is empty'
    if message.height <= 0 or message.point_step <= 0:
        return 'LiDAR height/point_step are not positive'

    fields = {field.name.lower(): field for field in message.fields}
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if (
            field is None
            or field.datatype != PointField.FLOAT32
            or field.count != 1
            or field.offset < 0
            or field.offset + 4 > message.point_step
        ):
            return f'LiDAR field {name!r} is not one in-record FLOAT32'

    minimum_row_step = int(message.width) * int(message.point_step)
    if message.row_step < minimum_row_step:
        return 'LiDAR row_step is shorter than width * point_step'
    minimum_payload_size = (
        (int(message.height) - 1) * int(message.row_step)
        + minimum_row_step
    )
    if len(message.data) < minimum_payload_size:
        return 'LiDAR payload is shorter than its declared geometry'

    if message.width == 0:
        if not contract.allow_empty_pointcloud:
            return 'LiDAR point cloud is empty'
        if message.row_step != 0 or message.data:
            return 'empty LiDAR point cloud has nonempty row/payload storage'
    return ''


def validate_message(contract, message) -> str:
    """Return an empty string for a structurally useful live UI payload."""
    if isinstance(message, CameraInfo):
        if message.width <= 0 or message.height <= 0:
            return 'camera calibration dimensions are not positive'
        if len(message.k) != 9 or not _finite(message.k):
            return 'camera calibration K matrix is invalid'
        if float(message.k[0]) <= 0.0 or float(message.k[4]) <= 0.0:
            return 'camera calibration focal lengths are not positive'
        if not _finite(message.d):
            return 'camera calibration distortion contains non-finite values'
    elif isinstance(message, CompressedImage):
        if not message.data:
            return 'compressed image payload is empty'
        if not str(message.format).strip():
            return 'compressed image format is empty'
    elif isinstance(message, Image):
        if message.width <= 0 or message.height <= 0 or message.step <= 0:
            return 'raw image dimensions/step are not positive'
        if not message.data:
            return 'raw image payload is empty'
    elif isinstance(message, PointCloud2):
        is_radar_source = contract.key.startswith('source.radar.')
        if is_radar_source:
            fields = {field.name.lower() for field in message.fields}
            if 'range' not in fields:
                return 'CARLA radar cloud has no Range field'
            # A fresh zero-detection radar cloud is a valid physical sample.
        else:
            return _validate_lidar_cloud(contract, message)
    elif isinstance(message, NavSatFix):
        if not _finite((message.latitude, message.longitude, message.altitude)):
            return 'GNSS coordinates are not finite'
        if not -90.0 <= message.latitude <= 90.0:
            return 'GNSS latitude is outside [-90, 90]'
        if not -180.0 <= message.longitude <= 180.0:
            return 'GNSS longitude is outside [-180, 180]'
    elif isinstance(message, Imu):
        values = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        if not _finite(values):
            return 'IMU payload contains non-finite values'
    elif isinstance(message, Range):
        if not _finite((
            message.field_of_view,
            message.min_range,
            message.max_range,
            message.range,
        )):
            return 'radar range payload contains non-finite values'
        if message.min_range < 0.0 or message.max_range <= message.min_range:
            return 'radar range limits are invalid'
        if message.range < 0.0:
            return 'radar range is negative'
    elif isinstance(message, NavPVT):
        latitude = float(message.lat) * 1.0e-7
        longitude = float(message.lon) * 1.0e-7
        if not -90.0 <= latitude <= 90.0:
            return 'NAV-PVT latitude is outside [-90, 90]'
        if not -180.0 <= longitude <= 180.0:
            return 'NAV-PVT longitude is outside [-180, 180]'
        if int(message.fix_type) == int(NavPVT.FIX_TYPE_NO_FIX):
            return 'NAV-PVT reports no actual GNSS fix'
        if not int(message.flags) & int(NavPVT.FLAGS_GNSS_FIX_OK):
            return 'NAV-PVT GNSS_FIX_OK is false'
    elif isinstance(message, NavCOV):
        values = (
            message.pos_cov_nn,
            message.pos_cov_ne,
            message.pos_cov_nd,
            message.pos_cov_ee,
            message.pos_cov_ed,
            message.pos_cov_dd,
        )
        if not _finite(values):
            return 'NAV-COV position covariance contains non-finite values'
        if not message.pos_cov_valid:
            return 'NAV-COV position covariance is invalid'
    elif isinstance(message, NavRELPOSNED9):
        flags = int(message.flags)
        required = (
            int(NavRELPOSNED9.FLAGS_GNSS_FIX_OK)
            | int(NavRELPOSNED9.FLAGS_REL_POS_VALID)
            | int(NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID)
        )
        if flags & required != required:
            return 'NAV-RELPOS actual dual-GNSS baseline/heading is invalid'
        baseline_m = (
            float(message.rel_pos_length)
            + float(message.rel_pos_hp_length) * 0.01
        ) * 0.01
        if not math.isfinite(baseline_m) or baseline_m <= 0.0:
            return 'NAV-RELPOS baseline is not positive'
    return ''


class SensorSourceAuditNode(Node):
    """Collect graph ownership and fresh receive evidence in one bounded run."""

    def __init__(self, role_name: str):
        super().__init__('carla_sensor_source_audit')
        self.contracts = build_stream_contracts(role_name)
        self.sample_counts = {contract.key: 0 for contract in self.contracts}
        self.last_received = {contract.key: None for contract in self.contracts}
        self.invalid_reasons = {contract.key: '' for contract in self.contracts}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._audit_subscriptions = []
        for contract in self.contracts:
            message_type = MESSAGE_TYPES[contract.type_name]
            self._audit_subscriptions.append(self.create_subscription(
                message_type,
                contract.topic,
                lambda message, item=contract: self._on_message(item, message),
                qos,
            ))

    def _on_message(self, contract, message) -> None:
        self.sample_counts[contract.key] += 1
        self.last_received[contract.key] = time.monotonic()
        self.invalid_reasons[contract.key] = validate_message(contract, message)

    def observations(self):
        """Take a final graph/sample snapshot suitable for pure evaluation."""
        observations = {}
        for contract in self.contracts:
            endpoints = tuple(
                PublisherEndpoint(
                    node_name=info.node_name,
                    node_namespace=info.node_namespace,
                    topic_type=info.topic_type,
                )
                for info in self.get_publishers_info_by_topic(contract.topic)
            )
            observations[contract.key] = StreamObservation(
                publishers=endpoints,
                sample_count=self.sample_counts[contract.key],
                last_received_monotonic=self.last_received[contract.key],
                invalid_reason=self.invalid_reasons[contract.key],
            )
        return observations

    def all_topics_observed(self) -> bool:
        """Return true once all streams have data and at least one publisher."""
        if not all(self.sample_counts.values()):
            return False
        return all(
            self.get_publishers_info_by_topic(contract.topic)
            for contract in self.contracts
        )


def inspect_carla_actors(host, port, timeout_seconds, role_name):
    """Return live CARLA actor evidence, or an availability warning."""
    try:
        carla = importlib.import_module('carla')
    except (ImportError, OSError) as error:
        return (), f'CARLA Python API unavailable: {error}'
    try:
        client = carla.Client(host, int(port))
        client.set_timeout(float(timeout_seconds))
        world = client.get_world()
        # A fresh Python client starts with snapshot frame 0 even while the
        # ROS bridge owns and ticks a synchronous CARLA world. In that state
        # get_actors() returns an empty cached inventory and falsely reports
        # every live sensor as missing. wait_for_tick() is passive (it does not
        # advance the simulation) and hydrates the client's first snapshot.
        world.wait_for_tick(float(timeout_seconds))
        actors = world.get_actors()
    except Exception as error:  # CARLA raises version-specific RPC classes.
        return (), f'CARLA actor query failed at {host}:{port}: {error}'

    observations = []
    for actor in actors:
        type_id = str(getattr(actor, 'type_id', ''))
        if not type_id.startswith('sensor.'):
            continue
        attributes = getattr(actor, 'attributes', {}) or {}
        actor_role = str(
            attributes.get('role_name', attributes.get('ros_name', ''))
        )
        parent = getattr(actor, 'parent', None)
        parent_attributes = getattr(parent, 'attributes', {}) or {}
        parent_role = str(parent_attributes.get('role_name', ''))
        observations.append(ActorObservation(
            actor_id=int(actor.id),
            role_name=actor_role,
            type_id=type_id,
            parent_role_name=parent_role,
        ))
    results = evaluate_actors(
        build_actor_contracts(), observations, vehicle_role_name=role_name
    )
    return results, ''


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Prove that every CAMROD UI sensor is fresh and solely owned by '
            'the expected CARLA adapter chain.'
        )
    )
    parser.add_argument('--role-name', default='ego_vehicle')
    parser.add_argument('--timeout-seconds', type=float, default=15.0)
    parser.add_argument('--min-observation-seconds', type=float, default=1.0)
    parser.add_argument('--max-sample-age-seconds', type=float, default=3.0)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--actor-timeout-seconds', type=float, default=2.0)
    parser.add_argument(
        '--actor-policy',
        choices=('auto', 'require', 'skip'),
        default='require',
        help=(
            'require (default) fails unless the actual CARLA actor inventory '
            'is reachable; auto audits actors only when the API is reachable'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='emit one machine-readable JSON document instead of text',
    )
    args = parser.parse_args()
    if args.timeout_seconds <= 0.0:
        parser.error('--timeout-seconds must be positive')
    if args.min_observation_seconds < 0.0:
        parser.error('--min-observation-seconds must be non-negative')
    if args.min_observation_seconds > args.timeout_seconds:
        parser.error('--min-observation-seconds cannot exceed timeout')
    if args.max_sample_age_seconds <= 0.0:
        parser.error('--max-sample-age-seconds must be positive')
    if args.actor_timeout_seconds <= 0.0:
        parser.error('--actor-timeout-seconds must be positive')
    if not 1 <= args.port <= 65535:
        parser.error('--port must be in [1, 65535]')
    return args


def _build_document(
    stream_results,
    actor_results,
    actor_warning,
    graph_error,
    elapsed,
):
    stream_failures = sum(not result.passed for result in stream_results)
    actor_failures = sum(not result.passed for result in actor_results)
    passed = stream_failures == 0 and actor_failures == 0
    return {
        'schema_version': 1,
        'status': 'PASS' if passed else 'FAIL',
        'passed': passed,
        'elapsed_seconds': elapsed,
        'summary': {
            'streams_checked': len(stream_results),
            'stream_failures': stream_failures,
            'actors_checked': len(actor_results),
            'actor_failures': actor_failures,
        },
        'graph_error': graph_error,
        'actor_warning': actor_warning,
        'streams': [result.as_dict() for result in stream_results],
        'actors': [result.as_dict() for result in actor_results],
    }


def _print_text(document):
    summary = document['summary']
    print(
        'CARLA_SENSOR_SOURCE_AUDIT %s streams=%d failed=%d actors=%d '
        'actor_failed=%d elapsed_s=%.3f'
        % (
            document['status'],
            summary['streams_checked'],
            summary['stream_failures'],
            summary['actors_checked'],
            summary['actor_failures'],
            document['elapsed_seconds'],
        )
    )
    if document['graph_error']:
        print('[FAIL] ' + document['graph_error'])
    for stream in document['streams']:
        marker = 'PASS' if stream['passed'] else 'FAIL'
        age = stream['sample_age_seconds']
        age_text = 'none' if age is None else f'{age:.3f}s'
        publishers = ','.join(stream['publisher_identities']) or 'none'
        print(
            f"[{marker}] {stream['label']}: {stream['topic']} "
            f"publisher={publishers} samples={stream['sample_count']} "
            f'age={age_text}'
        )
        for error in stream['errors']:
            print(f'  - {error}')
    if document['actor_warning']:
        print('[WARN] ' + document['actor_warning'])
    for actor in document['actors']:
        marker = 'PASS' if actor['passed'] else 'FAIL'
        ids = ','.join(str(item) for item in actor['actor_ids']) or 'none'
        print(
            f"[{marker}] CARLA actor {actor['role_name']}: "
            f"type={actor['type_id']} ids={ids}"
        )
        for error in actor['errors']:
            print(f'  - {error}')


def main() -> int:
    args = _parse_args()
    rclpy.init(args=[])
    node = SensorSourceAuditNode(args.role_name)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout_seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if (
                elapsed >= args.min_observation_seconds
                and node.all_topics_observed()
            ):
                break
    except KeyboardInterrupt:
        interrupted = True

    now = time.monotonic()
    observations = node.observations()
    stream_results = evaluate_streams(
        node.contracts,
        observations,
        now_monotonic=now,
        max_sample_age_seconds=args.max_sample_age_seconds,
    )
    graph_error = ''
    if not any(item.publishers for item in observations.values()):
        graph_error = (
            'ROS graph has no sensor publishers; start CARLA bridge, spawn, '
            'and CAMROD before running this audit'
        )
    node.destroy_node()
    rclpy.shutdown()

    actor_results = ()
    actor_warning = ''
    if args.actor_policy != 'skip':
        actor_results, actor_warning = inspect_carla_actors(
            args.host,
            args.port,
            args.actor_timeout_seconds,
            args.role_name,
        )
        if actor_warning and args.actor_policy == 'require':
            actor_results = evaluate_actors(
                build_actor_contracts(), (), args.role_name
            )

    document = _build_document(
        stream_results,
        actor_results,
        actor_warning,
        graph_error,
        elapsed=now - started,
    )
    if interrupted:
        document['status'] = 'INTERRUPTED'
        document['passed'] = False
    if args.json:
        print(json.dumps(document, sort_keys=True, separators=(',', ':')))
    else:
        _print_text(document)
    if interrupted:
        return 130
    return 0 if document['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
