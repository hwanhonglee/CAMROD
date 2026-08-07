"""Pure-helper and live-config tests for the physical radar dashboard."""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts/radar_status_gui.py"
SPEC = importlib.util.spec_from_file_location("radar_status_gui", SCRIPT_PATH)
RADAR_GUI = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RADAR_GUI)


def test_sample_classification_preserves_real_no_target_contract():
    classify = RADAR_GUI.classify_sample

    assert classify(None, None, 0.8, None, 1.0) == "no_data"
    assert classify(0.4, 0.02, 0.8, 1.1, 1.0) == "stale"
    assert classify(0.4, 0.02, 0.8, 0.1, 1.0) == "hit"
    assert classify(0.8, 0.02, 0.8, 0.1, 1.0) == "hit"
    assert classify(0.801, 0.02, 0.8, 0.1, 1.0) == "no_target"
    assert classify(math.inf, 0.02, 0.8, 0.1, 1.0) == "no_target"
    assert classify(math.nan, 0.02, 0.8, 0.1, 1.0) == "invalid"
    assert classify(0.01, 0.02, 0.8, 0.1, 1.0) == "invalid"
    front1_body = ((0.020, 0.120), (0.120, 0.220))
    assert (
        classify(0.220, 0.02, 0.5, 0.1, 1.0, front1_body, 0.320)
        == "filtered"
    )
    assert (
        classify(0.221, 0.02, 0.5, 0.1, 1.0, front1_body, 0.320)
        == "hit"
    )
    assert classify(0.320, 0.02, 0.5, 0.1, 1.0, front1_body, 0.320) == "hit"
    assert (
        classify(0.321, 0.02, 0.5, 0.1, 1.0, front1_body, 0.320)
        == "outside_stop"
    )

    rear_body = ((0.020, 0.106),)
    assert classify(0.106, 0.02, 0.5, 0.1, 1.0, rear_body, 0.206) == "filtered"
    assert classify(0.107, 0.02, 0.5, 0.1, 1.0, rear_body, 0.206) == "hit"
    assert classify(0.206, 0.02, 0.5, 0.1, 1.0, rear_body, 0.206) == "hit"
    assert classify(0.207, 0.02, 0.5, 0.1, 1.0, rear_body, 0.206) == "outside_stop"


def test_rolling_rate_uses_intervals_not_sample_count():
    assert RADAR_GUI.rolling_hz([]) is None
    assert RADAR_GUI.rolling_hz([1.0]) is None
    assert RADAR_GUI.rolling_hz([1.0, 1.1, 1.2]) == pytest.approx(10.0)


def test_canvas_axes_match_robot_forward_and_left_convention():
    center = RADAR_GUI.world_to_canvas(0.0, 0.0, 100.0, 100.0, 10.0)
    forward = RADAR_GUI.world_to_canvas(1.0, 0.0, 100.0, 100.0, 10.0)
    left = RADAR_GUI.world_to_canvas(0.0, 1.0, 100.0, 100.0, 10.0)

    assert center == (100.0, 100.0)
    assert forward == (100.0, 90.0)
    assert left == (90.0, 100.0)


def test_side_radar_mounts_match_center_referenced_field_measurements():
    poses = RADAR_GUI.MOUNT_POSES

    assert poses["LEFT1"][:2] == pytest.approx((0.38, 0.53))
    assert poses["LEFT2"][:2] == pytest.approx((-0.38, 0.53))
    assert poses["RIGHT1"][:2] == pytest.approx((0.38, -0.53))
    assert poses["RIGHT2"][:2] == pytest.approx((-0.38, -0.53))


def test_front_beam_arc_is_centered_on_positive_robot_x():
    spec = RADAR_GUI.ChannelSpec(
        name="FRONT1",
        frame_id="radar_front1_link",
        port="/dev/ttyCH9344USB0",
        topic="/sensing/radar/front1/range_ros",
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        configured_max_m=1.5,
        configured_enabled=True,
        field_of_view_rad=0.26,
    )
    points = RADAR_GUI.beam_arc_points(spec, 1.0, 0.26, steps=4)

    assert points[2][0] == pytest.approx(1.0)
    assert points[2][1] == pytest.approx(0.0)
    assert points[0][1] < 0.0 < points[-1][1]


def test_active_yaml_supplies_seven_physical_standard_range_topics():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "radar"
        / "sen0592_radar.yaml"
    )
    specs = RADAR_GUI.load_channel_specs(str(config_path), "/sensing/radar")

    assert tuple(spec.name for spec in specs) == RADAR_GUI.SENSOR_ORDER
    assert all(spec.configured_enabled for spec in specs)
    assert specs[0].topic == "/sensing/radar/front1/range_ros"
    assert specs[3].port == "/dev/ttyCH9344USB5"
    assert specs[5].port == "/dev/ttyCH9344USB3"
    assert all(spec.configured_max_m == pytest.approx(0.50) for spec in specs)


def test_live_cost_profile_uses_body_edge_plus_ten_centimeter_stop_windows():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "radar"
        / "cost_grid.yaml"
    )
    bands = RADAR_GUI.load_fixed_return_bands(str(config_path))
    stop_max = RADAR_GUI.load_stop_candidate_max_ranges_m(str(config_path))

    assert bands["FRONT1"][:2] == ((0.020, 0.120), (0.120, 0.220))
    assert bands["FRONT2"] == ((0.020, 0.117),)
    assert bands["REAR"] == ((0.020, 0.106),)
    assert tuple(stop_max[name] for name in RADAR_GUI.SENSOR_ORDER) == pytest.approx(
        (0.320, 0.217, 0.100, 0.100, 0.100, 0.100, 0.206)
    )
    assert stop_max["FRONT1"] - bands["FRONT1"][-1][1] == pytest.approx(0.10)
    assert stop_max["FRONT2"] - bands["FRONT2"][-1][1] == pytest.approx(0.10)
    assert stop_max["REAR"] - bands["REAR"][-1][1] == pytest.approx(0.10)
