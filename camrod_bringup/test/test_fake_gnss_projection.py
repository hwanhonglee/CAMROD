#!/usr/bin/env python3

"""Regression tests for the full-sim GNSS local/WGS84 projection."""

import importlib.util
import math
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "fake_sensor_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("fake_sensor_publisher", SCRIPT_PATH)
FAKE_SENSOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FAKE_SENSOR)

ORIGIN_LAT = 36.8435737
ORIGIN_LON = 128.0925646
ORIGIN_ALT = 0.0


def _project_back_to_enu(x, y):
    latitude, longitude = FAKE_SENSOR.local_enu_xy_to_latlon(
        x,
        y,
        ORIGIN_LAT,
        ORIGIN_LON,
        ORIGIN_ALT,
    )
    origin_lat_rad = FAKE_SENSOR.deg2rad(ORIGIN_LAT)
    origin_lon_rad = FAKE_SENSOR.deg2rad(ORIGIN_LON)
    reference = FAKE_SENSOR.llh_to_ecef(
        origin_lat_rad,
        origin_lon_rad,
        ORIGIN_ALT,
    )
    projected = FAKE_SENSOR.llh_to_ecef(
        math.radians(latitude),
        math.radians(longitude),
        ORIGIN_ALT,
    )
    return FAKE_SENSOR.ecef_to_enu(
        reference,
        projected,
        origin_lat_rad,
        origin_lon_rad,
    )


def test_origin_round_trip_is_exact():
    east, north, _ = _project_back_to_enu(0.0, 0.0)

    assert abs(east) < 1.0e-8
    assert abs(north) < 1.0e-8


def test_gnss_ab_location_round_trip_is_submillimetre():
    expected_x = -13.8802
    expected_y = 43.5384

    east, north, _ = _project_back_to_enu(expected_x, expected_y)
    residual = math.hypot(east - expected_x, north - expected_y)

    assert residual < 1.0e-3
