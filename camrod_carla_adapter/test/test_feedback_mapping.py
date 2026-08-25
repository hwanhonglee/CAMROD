import math

import pytest

from camrod_carla_adapter.feedback_mapping import (
    PlanarTransform,
    diagonal_covariance,
    multiply_quaternions,
    normalize_quaternion,
    quaternion_from_yaw,
    transform_pose,
    validate_motion_values,
    yaw_from_quaternion,
)


def _angle_difference(left, right):
    return math.atan2(math.sin(left - right), math.cos(left - right))


def test_lane_anchor_maps_accepted_spawn_to_lanelet_centerline():
    transform = PlanarTransform(
        x_m=7.855611455645338,
        y_m=13.676196558868874,
        yaw_rad=-0.05402607389329672,
    )
    source_yaw = math.radians(32.45613098144531)
    position, orientation = transform_pose(
        (38.00654220581055, -36.45695114135742, 7.181118011474609),
        quaternion_from_yaw(source_yaw),
        transform,
    )
    assert position[0] == pytest.approx(43.83803217408370, abs=1.0e-12)
    assert position[1] == pytest.approx(-24.779907458655806, abs=1.0e-12)
    assert position[2] == pytest.approx(7.181118011474609)
    expected_yaw = math.radians(29.360664963697495)
    assert _angle_difference(
        yaw_from_quaternion(orientation), expected_yaw) == pytest.approx(
            0.0, abs=1.0e-12)


def test_transform_rotates_translation_and_orientation():
    transform = PlanarTransform(x_m=2.0, y_m=-3.0, yaw_rad=math.pi / 2.0)
    position, orientation = transform_pose(
        (1.0, 0.0, 4.0), quaternion_from_yaw(-math.pi / 4.0), transform)
    assert position == pytest.approx((2.0, -2.0, 4.0))
    assert yaw_from_quaternion(orientation) == pytest.approx(math.pi / 4.0)


def test_quaternion_helpers_normalize_and_multiply():
    normalized = normalize_quaternion(0.0, 0.0, 2.0, 2.0)
    assert sum(value * value for value in normalized) == pytest.approx(1.0)
    product = multiply_quaternions(
        quaternion_from_yaw(math.radians(20.0)),
        quaternion_from_yaw(math.radians(25.0)),
    )
    assert yaw_from_quaternion(product) == pytest.approx(math.radians(45.0))


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0)],
)
def test_invalid_quaternion_is_rejected(quaternion):
    with pytest.raises(ValueError):
        normalize_quaternion(*quaternion)


def test_covariance_builder_has_expected_row_major_diagonal():
    covariance = diagonal_covariance([1.0, 2.0, 3.0])
    assert covariance == [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]
    with pytest.raises(ValueError):
        diagonal_covariance([1.0, 0.0, 3.0])


def test_motion_validation_rejects_malformed_values():
    validate_motion_values((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    with pytest.raises(ValueError):
        validate_motion_values((1.0, 2.0), (3.0, 4.0, 5.0))
    with pytest.raises(ValueError):
        validate_motion_values((1.0, 2.0, math.inf), (4.0, 5.0, 6.0))
