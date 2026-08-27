import math
from pathlib import Path

import pytest
import yaml

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


def test_lane_anchor_maps_settled_physical_body_to_lanelet_centerline():
    parameters = yaml.safe_load(
        (
            Path(__file__).resolve().parents[1]
            / "config"
            / "woraksan_lane_anchor_alignment.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    transform = PlanarTransform(
        x_m=parameters["map_offset_x_m"],
        y_m=parameters["map_offset_y_m"],
        yaw_rad=parameters["map_yaw_offset_rad"],
    )
    # The current split-rigid Blueprint reports the chassis pose below after
    # settling. Its actor spawn root is deliberately not used as localization.
    source_yaw = 0.561320571169091
    position, orientation = transform_pose(
        (38.726207733154297, -37.281283416748046, 3.179900169372559),
        quaternion_from_yaw(source_yaw),
        transform,
    )
    assert position[0] == pytest.approx(43.83803217408370, abs=1.0e-12)
    assert position[1] == pytest.approx(-24.779907458655806, abs=1.0e-12)
    assert position[2] == pytest.approx(3.179900169372559)
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
