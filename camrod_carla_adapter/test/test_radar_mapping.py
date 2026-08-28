"""Pure contracts for reducing CARLA radar detections to CAMROD ranges."""

import math

from camrod_carla_adapter.radar_mapping import (
    build_channel_specs,
    select_nearest_range,
)
import pytest


def test_nearest_finite_detection_inside_channel_limits_wins():
    selection = select_nearest_range(
        [0.9, math.nan, 0.42, math.inf, 1.51, 0.01],
        min_range_m=0.02,
        max_range_m=1.5,
    )

    assert selection.range_m == pytest.approx(0.42)
    assert selection.input_detection_count == 6
    assert selection.valid_detection_count == 2
    assert selection.has_target is True


def test_empty_or_invalid_cloud_uses_physical_driver_no_target_heartbeat():
    selection = select_nearest_range(
        [math.nan, -1.0, 0.81],
        min_range_m=0.02,
        max_range_m=0.8,
    )

    assert selection.range_m == pytest.approx(0.801)
    assert selection.input_detection_count == 3
    assert selection.valid_detection_count == 0
    assert selection.has_target is False


@pytest.mark.parametrize(
    ('minimum', 'maximum', 'epsilon'),
    (
        (-0.1, 1.0, 0.001),
        (1.0, 1.0, 0.001),
        (0.0, math.inf, 0.001),
        (0.0, 1.0, 0.0),
    ),
)
def test_invalid_scalar_contract_is_rejected(minimum, maximum, epsilon):
    with pytest.raises(ValueError):
        select_nearest_range((), minimum, maximum, epsilon)


def test_parallel_channel_vectors_are_validated_and_immutable():
    specs = build_channel_specs(
        names=('front1',),
        input_topics=('/carla/ego_vehicle/radar_front1',),
        output_topics=('/sensing/radar/front1/range',),
        standard_output_topics=('/sensing/radar/front1/range_ros',),
        frame_ids=('radar_front1_link',),
        min_ranges_m=(0.02,),
        max_ranges_m=(1.5,),
        field_of_views_rad=(0.26,),
    )

    assert len(specs) == 1
    assert specs[0].name == 'front1'
    assert specs[0].max_range_m == pytest.approx(1.5)

    with pytest.raises(ValueError, match='output_topics'):
        build_channel_specs(
            names=('front1',),
            input_topics=('/carla/ego_vehicle/radar_front1',),
            output_topics=(),
            standard_output_topics=('/sensing/radar/front1/range_ros',),
            frame_ids=('radar_front1_link',),
            min_ranges_m=(0.02,),
            max_ranges_m=(1.5,),
            field_of_views_rad=(0.26,),
        )
