"""Regression tests for the shared drop-zone reverse-axis contract."""

# HH_260720 - Lock the C-track station geometry that exposed the reversed yaw interpretation.

import math
import sys
from pathlib import Path

import pytest

# HH_260720 - Import flattened parking geometry without a redundant package wrapper.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from parking_geometry import (
    body_yaw_for_reverse_axis,
    reverse_axis_yaw_for_body,
    signed_distance_along_axis_m,
)


def test_drop_zone_yaw_produces_reverse_motion_toward_station():
    station_yaw_rad = math.radians(-82.2127)
    body_yaw_rad = body_yaw_for_reverse_axis(station_yaw_rad)
    reverse_axis_yaw_rad = reverse_axis_yaw_for_body(body_yaw_rad)

    distance_m = signed_distance_along_axis_m(
        origin_x_m=-13.88,
        origin_y_m=43.14,
        target_x_m=-13.5777,
        target_y_m=40.7412,
        axis_yaw_rad=reverse_axis_yaw_rad,
    )

    assert math.degrees(body_yaw_rad) == pytest.approx(97.7873)
    assert distance_m == pytest.approx(2.4177, abs=0.01)


def test_using_station_yaw_as_body_front_points_reverse_away_from_station():
    station_yaw_rad = math.radians(-82.2127)
    reverse_axis_yaw_rad = reverse_axis_yaw_for_body(station_yaw_rad)

    distance_m = signed_distance_along_axis_m(
        origin_x_m=-13.88,
        origin_y_m=43.14,
        target_x_m=-13.5777,
        target_y_m=40.7412,
        axis_yaw_rad=reverse_axis_yaw_rad,
    )

    assert distance_m < -2.4
