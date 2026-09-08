import math
import json
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


def _woraksan_transform():
    parameters = yaml.safe_load(
        (
            Path(__file__).resolve().parents[1]
            / "config"
            / "woraksan_lane_anchor_alignment.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    return PlanarTransform(
        x_m=parameters["map_offset_x_m"],
        y_m=parameters["map_offset_y_m"],
        yaw_rad=parameters["map_yaw_offset_rad"],
    )


def test_woraksan_alignment_maps_xodr_origin_from_georeferences():
    transform = _woraksan_transform()
    position, orientation = transform_pose(
        (0.0, 0.0, 0.0), quaternion_from_yaw(0.0), transform)
    assert position == pytest.approx(
        (6.952184129722062, 9.490185712639594, 0.0), abs=1.0e-12)
    assert yaw_from_quaternion(orientation) == pytest.approx(
        8.142811119961662e-7, abs=1.0e-15)


def test_woraksan_spawn_maps_to_current_develop_drop_zone_center_and_yaw():
    transform = _woraksan_transform()
    package = Path(__file__).resolve().parents[1]
    drop_zones = yaml.safe_load(
        (package.parent / "camrod_bringup/config/map/drop_zones.yaml").read_text(encoding="utf-8")
    )["drop_zones"]
    drop_zone = next(item for item in drop_zones if item["type"] == "drop_zone")
    parking = yaml.safe_load(
        (package.parent / "camrod_control/config/parking.yaml").read_text(encoding="utf-8")
    )["/parking/reverse_parking_controller"]["ros__parameters"]
    body_yaw_deg = drop_zone["yaw_deg"] + (
        180.0 if parking["station_yaw_represents_reverse_axis"] else 0.0
    )
    # Start inside the authored station so the first real UI dispatch must
    # exercise the production Drop Zone departure state machine.
    for name in ("ranger_spawn_camrod_control_only.json", "ranger_spawn_camrod_full_sensors.json"):
        objects = json.loads((package / "config" / name).read_text(encoding="utf-8"))["objects"]
        spawn = next(item for item in objects if item["type"] == "vehicle.ranger.default")["spawn_point"]
        position, orientation = transform_pose(
            (spawn["x"], spawn["y"], spawn["z"]),
            quaternion_from_yaw(math.radians(spawn["yaw"])), transform,
        )
        assert position[:2] == pytest.approx((drop_zone["x"], drop_zone["y"]), abs=1.0e-8)
        assert _angle_difference(
            yaw_from_quaternion(orientation), math.radians(body_yaw_deg)
        ) == pytest.approx(0.0, abs=1.0e-8)


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
