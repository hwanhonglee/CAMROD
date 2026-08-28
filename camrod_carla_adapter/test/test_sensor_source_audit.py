"""Pure tests for the fail-closed CARLA-to-UI sensor source audit."""

from camrod_carla_adapter.sensor_source_audit import (
    ActorObservation,
    build_actor_contracts,
    build_stream_contracts,
    evaluate_actors,
    evaluate_streams,
    PublisherEndpoint,
    StreamObservation,
)


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
