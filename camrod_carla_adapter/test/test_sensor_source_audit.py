"""Pure tests for the fail-closed CARLA-to-UI sensor source audit."""

from types import SimpleNamespace

from camrod_carla_adapter import sensor_source_audit_node
from camrod_carla_adapter.sensor_source_audit import (
    ActorObservation,
    build_actor_contracts,
    build_stream_contracts,
    evaluate_actors,
    evaluate_streams,
    PublisherEndpoint,
    StreamObservation,
)
from sensor_msgs.msg import PointCloud2, PointField


def _healthy_observations(contracts, now=100.0):
    return {
        contract.key: StreamObservation(
            publishers=(PublisherEndpoint(
                node_name=contract.expected_publisher,
                topic_type=contract.type_name,
            ),),
            sample_count=3,
            last_received_monotonic=now - 0.1,
        )
        for contract in contracts
    }


def test_inventory_covers_exact_ui_topics_and_all_carla_sources():
    contracts = build_stream_contracts('ranger_test')

    assert len(contracts) == 32
    assert sum(item.layer == 'source' for item in contracts) == 13
    assert sum(item.layer == 'canonical' for item in contracts) == 19
    topics = {item.topic for item in contracts}
    assert '/carla/ranger_test/rgb_view/image' in topics
    assert '/sensing/camera/econ_front/image_rect/compressed' in topics
    assert '/sensing/camera/econ_front/image_raw' in topics
    assert '/sensing/camera/econ_rear/image_raw/compressed' in topics
    assert '/sensing/camera/econ_rear/image_raw' in topics
    assert '/sensing/lidar/vanjee/points_raw' in topics
    assert '/sensing/lidar/points_filtered' in topics
    assert '/perception/obstacles' in topics
    contracts_by_key = {item.key: item for item in contracts}
    assert contracts_by_key['ui.lidar.raw'].expected_publisher == (
        'carla_sensor_relay'
    )
    assert contracts_by_key['ui.lidar.filtered'].expected_publisher == (
        'carla_lidar_filter'
    )
    assert contracts_by_key['ui.lidar.obstacles'].expected_publisher == (
        'obstacle_lidar'
    )
    assert contracts_by_key['source.lidar.front'].allow_empty_pointcloud is False
    assert contracts_by_key['ui.lidar.raw'].allow_empty_pointcloud is False
    assert contracts_by_key['ui.lidar.filtered'].allow_empty_pointcloud is True
    assert contracts_by_key['ui.lidar.obstacles'].allow_empty_pointcloud is True
    assert '/sensing/gnss/ublox_gps_node/fix' in topics
    assert '/carla/ranger_test/gnss_right' in topics
    assert '/sensing/gnss/ublox_gps_node/navpvt' in topics
    assert '/sensing/gnss/navcov' in topics
    assert '/sensing/gnss/navrelposned' in topics
    assert '/sensing/imu/data_ros' in topics
    for channel in (
        'front1', 'front2', 'left1', 'left2', 'right1', 'right2', 'rear'
    ):
        assert f'/carla/ranger_test/radar_{channel}' in topics
        assert f'/sensing/radar/{channel}/range_ros' in topics


def test_healthy_unique_adapter_ownership_and_fresh_samples_pass():
    contracts = build_stream_contracts()
    results = evaluate_streams(
        contracts,
        _healthy_observations(contracts),
        now_monotonic=100.0,
        max_sample_age_seconds=1.0,
    )

    assert results
    assert all(result.passed for result in results)


def _empty_lidar_heartbeat() -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = 'lidar_link'
    message.height = 1
    message.width = 0
    message.fields = [
        PointField(
            name=name,
            offset=offset,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for name, offset in (('x', 0), ('y', 4), ('z', 8))
    ]
    message.point_step = 16
    message.row_step = 0
    message.data = b''
    message.is_dense = True
    return message


def test_clear_scene_empty_cloud_is_valid_only_for_processed_lidar_products():
    contracts = {
        item.key: item for item in build_stream_contracts('ego_vehicle')
    }
    message = _empty_lidar_heartbeat()

    assert sensor_source_audit_node.validate_message(
        contracts['ui.lidar.filtered'], message
    ) == ''
    assert sensor_source_audit_node.validate_message(
        contracts['ui.lidar.obstacles'], message
    ) == ''
    assert sensor_source_audit_node.validate_message(
        contracts['source.lidar.front'], message
    ) == 'LiDAR point cloud is empty'
    assert sensor_source_audit_node.validate_message(
        contracts['ui.lidar.raw'], message
    ) == 'LiDAR point cloud is empty'


def test_empty_processed_lidar_still_requires_a_valid_frame_and_schema():
    contract = next(
        item
        for item in build_stream_contracts()
        if item.key == 'ui.lidar.filtered'
    )

    missing_frame = _empty_lidar_heartbeat()
    missing_frame.header.frame_id = ''
    assert 'frame_id is empty' in sensor_source_audit_node.validate_message(
        contract, missing_frame
    )

    missing_xyz = _empty_lidar_heartbeat()
    missing_xyz.fields = []
    assert "field 'x'" in sensor_source_audit_node.validate_message(
        contract, missing_xyz
    )

    inconsistent_storage = _empty_lidar_heartbeat()
    inconsistent_storage.row_step = 16
    assert 'nonempty row/payload storage' in (
        sensor_source_audit_node.validate_message(
            contract, inconsistent_storage
        )
    )


def test_fake_or_duplicate_canonical_publisher_fails_closed():
    contracts = build_stream_contracts()
    observations = _healthy_observations(contracts)
    key = 'ui.gnss'
    contract = next(item for item in contracts if item.key == key)
    observations[key] = StreamObservation(
        publishers=(
            PublisherEndpoint(
                contract.expected_publisher, topic_type=contract.type_name
            ),
            PublisherEndpoint(
                'fake_sensor_publisher',
                '/bringup',
                contract.type_name,
            ),
        ),
        sample_count=4,
        last_received_monotonic=99.9,
    )

    result = next(item for item in evaluate_streams(
        contracts,
        observations,
        now_monotonic=100.0,
        max_sample_age_seconds=1.0,
    ) if item.key == key)
    joined = '; '.join(result.errors)
    assert not result.passed
    assert 'fake/dummy publisher' in joined
    assert 'unexpected publisher' in joined
    assert 'exactly one publisher' in joined


def test_missing_graph_stale_data_and_invalid_latest_payload_are_reported():
    contracts = build_stream_contracts()
    target = contracts[0]
    observations = _healthy_observations(contracts)
    observations[target.key] = StreamObservation(
        publishers=(),
        sample_count=2,
        last_received_monotonic=90.0,
        invalid_reason='raw image payload is empty',
    )

    result = evaluate_streams(
        contracts,
        observations,
        now_monotonic=100.0,
        max_sample_age_seconds=1.0,
    )[0]

    assert not result.passed
    assert result.sample_age_seconds == 10.0
    assert 'no publisher discovered' in result.errors
    assert any('stale' in error for error in result.errors)
    assert any('payload is empty' in error for error in result.errors)


def test_actor_inventory_requires_exact_type_and_ranger_parent():
    contracts = build_actor_contracts()
    observations = tuple(
        ActorObservation(
            actor_id=index,
            role_name=contract.role_name,
            type_id=contract.type_id,
            parent_role_name='ego_vehicle',
        )
        for index, contract in enumerate(contracts, start=1)
    )

    assert all(result.passed for result in evaluate_actors(
        contracts, observations, 'ego_vehicle'
    ))

    broken = list(observations)
    broken[0] = ActorObservation(
        actor_id=1,
        role_name=contracts[0].role_name,
        type_id='sensor.other.radar',
        parent_role_name='other_vehicle',
    )
    result = evaluate_actors(contracts, broken, 'ego_vehicle')[0]
    assert not result.passed
    assert any('has type' in error for error in result.errors)
    assert any('parent role' in error for error in result.errors)


def test_live_actor_inventory_waits_for_first_synchronous_snapshot(monkeypatch):
    events = []
    parent = SimpleNamespace(attributes={'role_name': 'ego_vehicle'})
    actors = [
        SimpleNamespace(
            id=index,
            type_id=contract.type_id,
            attributes={'role_name': contract.role_name},
            parent=parent,
        )
        for index, contract in enumerate(build_actor_contracts(), start=1)
    ]

    class FakeWorld:
        def wait_for_tick(self, timeout_seconds):
            events.append(('wait_for_tick', timeout_seconds))
            return SimpleNamespace(frame=1)

        def get_actors(self):
            events.append(('get_actors', None))
            assert events[-2][0] == 'wait_for_tick'
            return actors

    class FakeClient:
        def __init__(self, host, port):
            events.append(('client', (host, port)))

        def set_timeout(self, timeout_seconds):
            events.append(('set_timeout', timeout_seconds))

        def get_world(self):
            events.append(('get_world', None))
            return FakeWorld()

    monkeypatch.setattr(
        sensor_source_audit_node.importlib,
        'import_module',
        lambda name: SimpleNamespace(Client=FakeClient),
    )
    results, warning = sensor_source_audit_node.inspect_carla_actors(
        '127.0.0.1', 2000, 2.0, 'ego_vehicle'
    )

    assert warning == ''
    assert all(result.passed for result in results)
    assert ('wait_for_tick', 2.0) in events
