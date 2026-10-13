"""HH_261007 - Offline guards only; never execute a live CRAB probe."""

import argparse
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


TOOLS = Path(__file__).resolve().parents[1]
PROBE_PATH = TOOLS / "probe_crab_yaw_hold.py"
EXPECTED_PROFILE = {
    "/carla_extended_ackermann_control": {
        "experimental_crab_yaw_hold_enabled": True,
        "extreme_crab_curb_boost_enable": True,
        "experimental_crab_yaw_hold_heading_gain_nm_per_rad": 8.0,
        "experimental_crab_yaw_hold_rate_gain_nm_per_radps": 16.0,
        "experimental_crab_yaw_hold_maximum_reduction_fraction": 0.4,
        "experimental_crab_yaw_hold_maximum_heading_error_deg": 12.0,
        "experimental_crab_yaw_hold_maximum_yaw_rate_degps": 18.0,
        "experimental_crab_yaw_hold_stall_timeout_sec": 4.0,
        "experimental_crab_yaw_hold_stall_progress_mps": 0.05,
        "experimental_crab_yaw_hold_odometry_timeout_sec": 0.25,
        "experimental_crab_yaw_hold_torque_limit_nm": 16.0,
    },
    "/physical_four_wheel_bridge": {
        "experimental_crab_yaw_hold_enabled": True,
        "wheel_torque_safety_cap_nm": 20.0,
    },
}


@pytest.fixture
def probe(monkeypatch):
    # HH_261007 - Loading pure guards must not initialize ROS, CARLA or a UI.
    monkeypatch.syspath_prepend(str(TOOLS))
    for module_name in ("carla", "rclpy", "manual_4ws_evidence"):
        monkeypatch.setitem(sys.modules, module_name, None)
    spec = importlib.util.spec_from_file_location("crab_yaw_hold_probe_test", PROBE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(*, actor_id=42, speed=0.0, yaw=0.0):
    return {
        "actor": {
            "id": actor_id,
            "transform": {"location_m": {"x": 0.0, "y": 0.0, "z": 2.0}},
            "linear_velocity_mps": {"x": speed, "y": 0.0, "z": 0.0},
            "yaw_degrees": yaw,
            "angular_velocity_degrees_per_second": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "wheels": [{"is_in_air": False} for _ in range(4)],
    }


@pytest.mark.parametrize("value", ["0.0001", "0.5", "1", "2.0"])
def test_duration_accepts_only_positive_at_most_two_seconds(probe, value):
    assert probe.duration(value) == float(value)


@pytest.mark.parametrize("value", ["0", "-0.1", "2.000001", "nan", "inf", "-inf"])
def test_duration_rejects_outside_envelope(probe, value):
    with pytest.raises(argparse.ArgumentTypeError):
        probe.duration(value)


@pytest.mark.parametrize("speed,yaw", [(0.0, 0.0), (0.35, 3.0), (-0.35, -3.0)])
def test_bounded_accepts_grounded_same_actor_at_limits(probe, speed, yaw):
    probe.bounded(sample(speed=speed, yaw=yaw), sample())


@pytest.mark.parametrize("speed", [-0.350001, 0.350001, math.nan, math.inf])
def test_bounded_rejects_speed_excess_or_nonfinite_feedback(probe, speed):
    with pytest.raises(RuntimeError):
        probe.bounded(sample(speed=speed), sample())


@pytest.mark.parametrize("yaw", [-3.000001, 3.000001, math.nan, math.inf])
def test_bounded_rejects_heading_drift_or_nonfinite_feedback(probe, yaw):
    with pytest.raises(RuntimeError):
        probe.bounded(sample(yaw=yaw), sample())


def test_bounded_wraps_heading_across_180_degrees(probe):
    probe.bounded(sample(yaw=-179.0), sample(yaw=179.0))


def test_bounded_rejects_actor_replacement(probe):
    with pytest.raises(RuntimeError):
        probe.bounded(sample(actor_id=43), sample())


@pytest.mark.parametrize("count", [0, 1, 3, 5])
def test_bounded_requires_exactly_four_wheels(probe, count):
    actual = sample()
    actual["wheels"] = [{"is_in_air": False} for _ in range(count)]
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample())


@pytest.mark.parametrize("wheel", range(4))
def test_bounded_rejects_each_ungrounded_wheel(probe, wheel):
    actual = sample()
    actual["wheels"][wheel]["is_in_air"] = True
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample())


@pytest.mark.parametrize("value", [None, 0, "false", ""])
def test_bounded_requires_known_boolean_ground_contact(probe, value):
    actual = sample()
    actual["wheels"][0]["is_in_air"] = value
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample())


def test_bounded_rejects_missing_ground_contact(probe):
    actual = sample()
    actual["wheels"][0].clear()
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample())


@pytest.mark.parametrize("distance", [-0.6, 0.6])
def test_bounded_accepts_distance_at_limit(probe, distance):
    actual = sample()
    actual["actor"]["transform"]["location_m"]["x"] = distance
    probe.bounded(actual, sample())


@pytest.mark.parametrize("distance", [-0.600001, 0.600001, math.nan, math.inf])
def test_bounded_rejects_distance_excess_or_nonfinite_feedback(probe, distance):
    actual = sample()
    actual["actor"]["transform"]["location_m"]["x"] = distance
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample())


@pytest.mark.parametrize("speed", [-0.02, 0.0, 0.02])
def test_stationary_accepts_only_low_speed(probe, speed):
    probe.bounded(sample(speed=speed), sample(), stationary=True)


@pytest.mark.parametrize("speed", [-0.020001, 0.020001, 0.1])
def test_stationary_rejects_already_moving_actor(probe, speed):
    with pytest.raises(RuntimeError):
        probe.bounded(sample(speed=speed), sample(), stationary=True)


@pytest.mark.parametrize("yaw_rate", [-1.0, 0.0, 1.0])
def test_stationary_accepts_yaw_rate_at_limit(probe, yaw_rate):
    actual = sample()
    actual["actor"]["angular_velocity_degrees_per_second"]["z"] = yaw_rate
    probe.bounded(actual, sample(), stationary=True)


@pytest.mark.parametrize("yaw_rate", [-1.000001, 1.000001, math.nan, math.inf])
def test_stationary_rejects_rotating_or_unknown_rate(probe, yaw_rate):
    actual = sample()
    actual["actor"]["angular_velocity_degrees_per_second"]["z"] = yaw_rate
    with pytest.raises(RuntimeError):
        probe.bounded(actual, sample(), stationary=True)


def test_profile_explicitly_binds_both_nodes_and_active_experiment(probe):
    expected = probe.PROFILE
    assert expected == EXPECTED_PROFILE
    assert set(expected) == {"/carla_extended_ackermann_control", "/physical_four_wheel_bridge"}
    controller = expected["/carla_extended_ackermann_control"]
    bridge = expected["/physical_four_wheel_bridge"]
    assert controller["experimental_crab_yaw_hold_enabled"] is True
    assert controller["extreme_crab_curb_boost_enable"] is True
    assert controller["experimental_crab_yaw_hold_heading_gain_nm_per_rad"] == 8.0
    assert bridge["experimental_crab_yaw_hold_enabled"] is True
    assert bridge["wheel_torque_safety_cap_nm"] == 20.0
    probe.require_profile(copy.deepcopy(expected))


def test_profile_rejects_each_missing_node_and_extra_node(probe):
    for node in probe.PROFILE:
        actual = copy.deepcopy(probe.PROFILE)
        del actual[node]
        with pytest.raises(RuntimeError):
            probe.require_profile(actual)
    actual = copy.deepcopy(probe.PROFILE)
    actual["/unknown_node"] = {}
    with pytest.raises(RuntimeError):
        probe.require_profile(actual)


def test_profile_rejects_each_missing_or_additional_parameter(probe):
    for node, parameters in probe.PROFILE.items():
        for name in parameters:
            actual = copy.deepcopy(probe.PROFILE)
            del actual[node][name]
            with pytest.raises(RuntimeError):
                probe.require_profile(actual)
        actual = copy.deepcopy(probe.PROFILE)
        actual[node]["unknown_parameter"] = True
        with pytest.raises(RuntimeError):
            probe.require_profile(actual)


def test_profile_rejects_every_parameter_mismatch(probe):
    for node, parameters in probe.PROFILE.items():
        for name, value in parameters.items():
            actual = copy.deepcopy(probe.PROFILE)
            actual[node][name] = not value if type(value) is bool else value + 0.01
            with pytest.raises(RuntimeError):
                probe.require_profile(actual)


def test_profile_rejects_truthy_integer_and_string_boolean_coercion(probe):
    for node, parameters in probe.PROFILE.items():
        for name, value in parameters.items():
            if type(value) is not bool:
                continue
            for substitute in (int(value), str(value), None):
                actual = copy.deepcopy(probe.PROFILE)
                actual[node][name] = substitute
                with pytest.raises(RuntimeError):
                    probe.require_profile(actual)


@pytest.mark.parametrize("actual", [None, [], True, "profile"])
def test_profile_rejects_malformed_root(probe, actual):
    with pytest.raises(RuntimeError):
        probe.require_profile(actual)


def test_reserve_output_creates_exclusive_json_without_running_probe(probe, tmp_path):
    output = tmp_path / "evidence.json"
    payload = {"kind": "offline-test", "motion_executed": False, "rows": []}
    with probe.reserve_output(output) as stream:
        json.dump(payload, stream)
    assert json.loads(output.read_text(encoding="utf-8")) == payload
    original = output.read_bytes()
    with pytest.raises(FileExistsError):
        probe.reserve_output(output)
    assert output.read_bytes() == original


@pytest.mark.parametrize("name", ["evidence.txt", "evidence.json.gz", "evidence"])
def test_reserve_output_rejects_non_json_suffix(probe, tmp_path, name):
    output = tmp_path / name
    with pytest.raises((RuntimeError, ValueError)):
        probe.reserve_output(output)
    assert not output.exists()


def test_reserve_output_requires_existing_parent(probe, tmp_path):
    output = tmp_path / "absent" / "evidence.json"
    with pytest.raises((RuntimeError, ValueError, FileNotFoundError)):
        probe.reserve_output(output)
    assert not output.parent.exists()


def test_source_only_commands_real_manual_browser_and_no_direct_actuation():
    source = PROBE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    attributes = {call.func.attr for call in calls if isinstance(call.func, ast.Attribute)}
    forbidden = {
        "create_publisher", "new_publisher", "publish", "apply_control",
        "set_transform", "set_target_velocity", "set_target_angular_velocity",
        "set_simulate_physics", "set_wheel_steer_direction",
        "set_wheel_physics_steer_angles", "set_wheel_physics_steer_angles_and_drive_torques",
        "set_parameters", "set_parameters_atomically", "spawn_actor", "try_spawn_actor",
        "load_world", "reload_world", "tick",
    }
    assert attributes.isdisjoint(forbidden)
    assert any(isinstance(call.func, ast.Name) and call.func.id == "ManualDriveBrowser"
               for call in calls)
    assert "dispatch_key" in attributes
    assert "zero" in attributes
    # DOM reads are legitimate; synthetic DOM input writes/events are not.
    for marker in (".value =", ".value=", ".checked =", ".checked=",
                   "dispatchEvent(", "setAttribute(", ".innerHTML ="):
        assert marker not in source


def test_probe_stays_small_and_has_no_runtime_entry_on_import():
    source = PROBE_PATH.read_text(encoding="utf-8")
    assert len(source.splitlines()) <= 200
    tree = ast.parse(source)
    main_calls = [node for node in tree.body if isinstance(node, ast.If)
                  and isinstance(node.test, ast.Compare)
                  and isinstance(node.test.left, ast.Name)
                  and node.test.left.id == "__name__"]
    assert len(main_calls) == 1
    assert isinstance(main_calls[0].test.comparators[0], ast.Constant)
    assert main_calls[0].test.comparators[0].value == "__main__"


def test_main_reserves_exclusive_output_before_any_run():
    tree = ast.parse(PROBE_PATH.read_text(encoding="utf-8"))
    main = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == "main")
    reservations = [node for node in ast.walk(main) if isinstance(node, ast.With)
                    and any(isinstance(item.context_expr, ast.Call)
                            and isinstance(item.context_expr.func, ast.Name)
                            and item.context_expr.func.id == "reserve_output"
                            for item in node.items)]
    assert len(reservations) == 1
    runs = [node for node in ast.walk(main) if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name) and node.func.id == "run"]
    assert len(runs) == 1
    assert runs[0] in list(ast.walk(reservations[0]))
