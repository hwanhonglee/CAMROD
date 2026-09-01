"""
Pure contracts for proving that CAMROD UI sensors originate in CARLA.

The live ROS node is intentionally kept in ``sensor_source_audit_node``.  This
module contains only immutable contracts and report evaluation, which makes the
fail-closed ownership rules unit-testable without a running ROS graph.
"""

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Mapping, Sequence


RADAR_CHANNELS = (
    'front1',
    'front2',
    'left1',
    'left2',
    'right1',
    'right2',
    'rear',
)


@dataclass(frozen=True)
class StreamContract:
    """One source or canonical stream that must be physically accounted for."""

    key: str
    label: str
    topic: str
    type_name: str
    expected_publisher: str
    layer: str
    # Clear-scene obstacle products are live, meaningful heartbeats even when
    # they contain zero points. Source/raw LiDAR contracts leave this false so
    # they still prove that the CARLA ray-cast sensor produced measurements.
    allow_empty_pointcloud: bool = False


@dataclass(frozen=True)
class PublisherEndpoint:
    """ROS graph identity reduced to the fields used by this audit."""

    node_name: str
    node_namespace: str = '/'
    topic_type: str = ''

    @property
    def identity(self) -> str:
        namespace = self.node_namespace.strip()
        if not namespace or namespace == '/':
            return '/' + self.node_name.strip('/')
        return '/' + '/'.join(
            (namespace.strip('/'), self.node_name.strip('/'))
        )


@dataclass(frozen=True)
class StreamObservation:
    """Graph and receive evidence collected for a stream."""

    publishers: tuple[PublisherEndpoint, ...] = ()
    sample_count: int = 0
    last_received_monotonic: float | None = None
    invalid_reason: str = ''


@dataclass(frozen=True)
class StreamAuditResult:
    """Evaluation result for one stream contract."""

    key: str
    label: str
    layer: str
    topic: str
    expected_publisher: str
    publisher_identities: tuple[str, ...]
    sample_count: int
    sample_age_seconds: float | None
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def as_dict(self) -> dict:
        result = asdict(self)
        result['passed'] = self.passed
        return result


@dataclass(frozen=True)
class ActorContract:
    """Expected CARLA actor role/type attached to the Ranger vehicle."""

    role_name: str
    type_id: str


@dataclass(frozen=True)
class ActorObservation:
    """Relevant fields from one live CARLA actor."""

    actor_id: int
    role_name: str
    type_id: str
    parent_role_name: str = ''


@dataclass(frozen=True)
class ActorAuditResult:
    """Evaluation result for one actor contract."""

    role_name: str
    type_id: str
    actor_ids: tuple[int, ...]
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def as_dict(self) -> dict:
        result = asdict(self)
        result['passed'] = self.passed
        return result


def build_stream_contracts(role_name: str = 'ego_vehicle') -> tuple[StreamContract, ...]:
    """Build the complete CARLA-input and CAMROD-UI stream inventory."""
    normalized_role = str(role_name).strip().strip('/')
    if not normalized_role:
        raise ValueError('role_name must be non-empty')
    carla_prefix = f'/carla/{normalized_role}'
    contracts = [
        StreamContract(
            'source.camera.front',
            'CARLA front camera',
            f'{carla_prefix}/rgb_view/image',
            'sensor_msgs/msg/Image',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'source.camera.front.info',
            'CARLA front camera calibration',
            f'{carla_prefix}/rgb_view/camera_info',
            'sensor_msgs/msg/CameraInfo',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.camera.front',
            'UI front camera',
            '/sensing/camera/econ_front/image_rect/compressed',
            'sensor_msgs/msg/CompressedImage',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'ui.camera.front.raw',
            'UI front camera raw fallback',
            '/sensing/camera/econ_front/image_raw',
            'sensor_msgs/msg/Image',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'ui.camera.front.info',
            'CAMROD front camera calibration',
            '/sensing/camera/econ_front/camera_info',
            'sensor_msgs/msg/CameraInfo',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'source.camera.rear',
            'CARLA rear camera',
            f'{carla_prefix}/rgb_rear/image',
            'sensor_msgs/msg/Image',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'source.camera.rear.info',
            'CARLA rear camera calibration',
            f'{carla_prefix}/rgb_rear/camera_info',
            'sensor_msgs/msg/CameraInfo',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.camera.rear',
            'UI rear camera',
            '/sensing/camera/econ_rear/image_raw/compressed',
            'sensor_msgs/msg/CompressedImage',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'ui.camera.rear.raw',
            'UI rear camera raw fallback',
            '/sensing/camera/econ_rear/image_raw',
            'sensor_msgs/msg/Image',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'ui.camera.rear.info',
            'CAMROD rear camera calibration',
            '/sensing/camera/econ_rear/camera_info',
            'sensor_msgs/msg/CameraInfo',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'source.lidar.front',
            'CARLA front LiDAR',
            f'{carla_prefix}/lidar_front',
            'sensor_msgs/msg/PointCloud2',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.lidar.raw',
            'UI raw LiDAR',
            '/sensing/lidar/vanjee/points_raw',
            'sensor_msgs/msg/PointCloud2',
            'carla_sensor_relay',
            'canonical',
        ),
        StreamContract(
            'ui.lidar.filtered',
            'UI filtered LiDAR',
            '/sensing/lidar/points_filtered',
            'sensor_msgs/msg/PointCloud2',
            'carla_lidar_filter',
            'canonical',
            allow_empty_pointcloud=True,
        ),
        StreamContract(
            'ui.perception.obstacles',
            'UI fused semantic obstacle cloud',
            '/perception/obstacles',
            'sensor_msgs/msg/PointCloud2',
            'obstacle_fusion',
            'canonical',
            allow_empty_pointcloud=True,
        ),
        StreamContract(
            'source.gnss',
            'CARLA GNSS',
            f'{carla_prefix}/gnss',
            'sensor_msgs/msg/NavSatFix',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.gnss',
            'UI GNSS fix',
            '/sensing/gnss/ublox_gps_node/fix',
            'sensor_msgs/msg/NavSatFix',
            'carla_feedback_bridge',
            'canonical',
        ),
        StreamContract(
            'source.gnss.right',
            'CARLA right GNSS',
            f'{carla_prefix}/gnss_right',
            'sensor_msgs/msg/NavSatFix',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.gnss.navpvt',
            'UI GNSS NAV-PVT compatibility view',
            '/sensing/gnss/ublox_gps_node/navpvt',
            'ublox_msgs/msg/NavPVT',
            'carla_feedback_bridge',
            'canonical',
        ),
        StreamContract(
            'ui.gnss.navcov',
            'UI GNSS NAV-COV compatibility view',
            '/sensing/gnss/navcov',
            'ublox_msgs/msg/NavCOV',
            'carla_feedback_bridge',
            'canonical',
        ),
        StreamContract(
            'ui.gnss.relpos',
            'UI dual-GNSS moving baseline',
            '/sensing/gnss/navrelposned',
            'ublox_msgs/msg/NavRELPOSNED9',
            'carla_feedback_bridge',
            'canonical',
        ),
        StreamContract(
            'source.imu',
            'CARLA IMU',
            f'{carla_prefix}/imu',
            'sensor_msgs/msg/Imu',
            'carla_ros_bridge',
            'source',
        ),
        StreamContract(
            'ui.imu',
            'UI IMU',
            '/sensing/imu/data_ros',
            'sensor_msgs/msg/Imu',
            'carla_feedback_bridge',
            'canonical',
        ),
    ]
    for channel in RADAR_CHANNELS:
        contracts.extend((
            StreamContract(
                f'source.radar.{channel}',
                f'CARLA radar {channel}',
                f'{carla_prefix}/radar_{channel}',
                'sensor_msgs/msg/PointCloud2',
                'carla_ros_bridge',
                'source',
            ),
            StreamContract(
                f'ui.radar.{channel}',
                f'UI radar {channel}',
                f'/sensing/radar/{channel}/range_ros',
                'sensor_msgs/msg/Range',
                'carla_radar_relay',
                'canonical',
            ),
        ))
    return tuple(contracts)


def build_actor_contracts() -> tuple[ActorContract, ...]:
    """Return physical CARLA sensors required by the UI stream contract."""
    actors = [
        ActorContract('rgb_view', 'sensor.camera.rgb'),
        ActorContract('rgb_rear', 'sensor.camera.rgb'),
        ActorContract('lidar_front', 'sensor.lidar.ray_cast'),
        ActorContract('gnss', 'sensor.other.gnss'),
        ActorContract('gnss_right', 'sensor.other.gnss'),
        ActorContract('imu', 'sensor.other.imu'),
    ]
    actors.extend(
        ActorContract(f'radar_{channel}', 'sensor.other.radar')
        for channel in RADAR_CHANNELS
    )
    return tuple(actors)


def _is_forbidden_publisher(endpoint: PublisherEndpoint) -> bool:
    identity = endpoint.identity.lower()
    return 'fake' in identity or 'dummy_publisher' in identity


def evaluate_streams(
    contracts: Sequence[StreamContract],
    observations: Mapping[str, StreamObservation],
    now_monotonic: float,
    max_sample_age_seconds: float,
) -> tuple[StreamAuditResult, ...]:
    """Fail closed on missing, stale, fake, duplicate, or wrong publishers."""
    if not math.isfinite(max_sample_age_seconds) or max_sample_age_seconds <= 0.0:
        raise ValueError('max_sample_age_seconds must be finite and positive')
    results = []
    for contract in contracts:
        observation = observations.get(contract.key, StreamObservation())
        publishers = tuple(observation.publishers)
        identities = tuple(sorted(item.identity for item in publishers))
        errors = []
        if not publishers:
            errors.append('no publisher discovered')
        else:
            forbidden = [
                endpoint.identity
                for endpoint in publishers
                if _is_forbidden_publisher(endpoint)
            ]
            if forbidden:
                errors.append(
                    'fake/dummy publisher owns topic: ' + ', '.join(forbidden)
                )
            unexpected = [
                endpoint.identity
                for endpoint in publishers
                if endpoint.node_name.strip('/') != contract.expected_publisher
            ]
            if unexpected:
                errors.append(
                    'unexpected publisher (expected /%s): %s'
                    % (contract.expected_publisher, ', '.join(unexpected))
                )
            if len(publishers) != 1:
                errors.append(
                    f'expected exactly one publisher, found {len(publishers)}'
                )
            type_mismatches = [
                endpoint.topic_type or '<unknown>'
                for endpoint in publishers
                if endpoint.topic_type != contract.type_name
            ]
            if type_mismatches:
                errors.append(
                    'publisher type mismatch (expected %s): %s'
                    % (contract.type_name, ', '.join(type_mismatches))
                )

        age = None
        if observation.sample_count <= 0 or observation.last_received_monotonic is None:
            errors.append('no live message received during audit window')
        else:
            age = max(0.0, now_monotonic - observation.last_received_monotonic)
            if age > max_sample_age_seconds:
                errors.append(
                    'latest message is stale: %.3fs > %.3fs'
                    % (age, max_sample_age_seconds)
                )
        if observation.invalid_reason:
            errors.append('latest message invalid: ' + observation.invalid_reason)

        results.append(StreamAuditResult(
            key=contract.key,
            label=contract.label,
            layer=contract.layer,
            topic=contract.topic,
            expected_publisher=contract.expected_publisher,
            publisher_identities=identities,
            sample_count=max(0, int(observation.sample_count)),
            sample_age_seconds=age,
            errors=tuple(errors),
        ))
    return tuple(results)


def _role_matches(observed: str, expected: str) -> bool:
    normalized = str(observed).strip().strip('/')
    return normalized == expected or normalized.endswith('/' + expected)


def evaluate_actors(
    contracts: Sequence[ActorContract],
    observations: Iterable[ActorObservation],
    vehicle_role_name: str,
) -> tuple[ActorAuditResult, ...]:
    """Require one correctly typed sensor attached to the requested vehicle."""
    actors = tuple(observations)
    results = []
    for contract in contracts:
        matches = [
            actor
            for actor in actors
            if _role_matches(actor.role_name, contract.role_name)
        ]
        errors = []
        if not matches:
            errors.append('actor not found')
        elif len(matches) != 1:
            errors.append(f'expected exactly one actor, found {len(matches)}')
        for actor in matches:
            if actor.type_id != contract.type_id:
                errors.append(
                    f'actor {actor.actor_id} has type {actor.type_id}; '
                    f'expected {contract.type_id}'
                )
            if not _role_matches(actor.parent_role_name, vehicle_role_name):
                errors.append(
                    f'actor {actor.actor_id} parent role is '
                    f"{actor.parent_role_name or '<none>'}; "
                    f'expected {vehicle_role_name}'
                )
        results.append(ActorAuditResult(
            role_name=contract.role_name,
            type_id=contract.type_id,
            actor_ids=tuple(sorted(actor.actor_id for actor in matches)),
            errors=tuple(errors),
        ))
    return tuple(results)
