"""Focused tests for the passive physical-wheel evidence recorder."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "record_physical_wheel_telemetry.py"
)
SPEC = importlib.util.spec_from_file_location("record_physical_wheel_telemetry", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def transform(x=1.0, y=2.0, z=0.5, pitch=0.0, yaw=15.0, roll=0.0):
    return SimpleNamespace(
        location=vector(x, y, z),
        rotation=SimpleNamespace(pitch=pitch, yaw=yaw, roll=roll),
    )


def substep():
    return SimpleNamespace(
        threshold_longitudinal_speed_mps=5.0,
        low_forward_speed_substep_count=3,
        high_forward_speed_substep_count=1,
    )


def wheel(index):
    values = {
        "wheel_index": index,
        "wheel_location": MODULE.EXPECTED_WHEEL_LOCATIONS[index],
        "contact_surface_type": 0,
        "contact_material_path": "/Engine/TestMaterial",
        "contact_material_class": "/Script/PhysicsCore.PhysicalMaterial",
    }
    values.update({name: False for name in MODULE.WHEEL_BOOL_FIELDS})
    values.update({name: 0.25 + index for name in MODULE.WHEEL_FLOAT_FIELDS})
    values.update(
        {
            name: vector(index, index + 1, index + 2)
            for name in MODULE.WHEEL_VECTOR_FIELDS
        }
    )
    return SimpleNamespace(**values)


def telemetry(frame=101):
    values = {
        "api_version": 1,
        "body_transform": transform(),
        "body_linear_velocity_mps": vector(0.2, 0.1, 0.0),
        "body_angular_velocity_rad_per_second": vector(0.0, 0.0, 0.3),
        "component_substep_control": substep(),
        "live_substep_control": substep(),
        "wheels": [wheel(index) for index in range(4)],
    }
    values.update({name: "physical_scope" for name in MODULE.TELEMETRY_STRING_FIELDS})
    values.update({name: True for name in MODULE.TELEMETRY_BOOL_FIELDS})
    values.update(
        {
            "world_frame": frame,
            "engine_frame_counter": frame,
            "wheel_query_engine_frame_counter": frame - 1,
            "active_vehicle_substep_count": 3,
            "last_vehicle_substep_index": 2,
            "external_drive_torque_wheel_mask": 15,
        }
    )
    values.update({name: 0.05 for name in MODULE.TELEMETRY_FLOAT_FIELDS})
    return SimpleNamespace(**values)


class Actor:

    def __init__(self, actor_id=17, role="ego_vehicle", sample=None):
        self.id = actor_id
        self.type_id = "vehicle.ranger.default"
        self.attributes = {"role_name": role}
        self.is_alive = True
        self.sample = sample or telemetry()
        self.read_calls = []

    def get_wheel_physics_telemetry(self):
        self.read_calls.append("telemetry")
        return self.sample

    def get_wheel_steer_angle(self, location):
        self.read_calls.append(("steer", location))
        return {10: 11.0, 11: -12.0, 12: 13.0, 13: -14.0}[location]

    def get_transform(self):
        self.read_calls.append("transform")
        return transform(yaw=42.5)

    def get_velocity(self):
        self.read_calls.append("velocity")
        return vector(1.0, 2.0, 3.0)

    def get_angular_velocity(self):
        self.read_calls.append("angular_velocity")
        return vector(0.0, 0.0, 4.0)


class World:

    def __init__(self, inventories):
        self.inventories = list(inventories)
        self.inventory_calls = 0
        self.wait_for_tick_calls = []

    def wait_for_tick(self, timeout):
        self.wait_for_tick_calls.append(timeout)
        return SimpleNamespace(frame=100)

    def get_actors(self):
        index = min(self.inventory_calls, len(self.inventories) - 1)
        self.inventory_calls += 1
        return self.inventories[index]


class Client:

    def __init__(self, world):
        self.world = world
        self.timeout = None

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_world(self):
        return self.world


def carla_module(world):
    client = Client(world)
    module = SimpleNamespace(
        VehicleWheelLocation=SimpleNamespace(
            FL_Wheel=10, FR_Wheel=11, BL_Wheel=12, BR_Wheel=13
        ),
        Client=lambda host, port: client,
    )
    return module, client


def args(output, **overrides):
    values = {
        "output": str(output),
        "manifest": None,
        "host": "127.0.0.1",
        "port": 2000,
        "timeout_s": 3.0,
        "rate_hz": 10.0,
        "duration_s": 0.0,
        "max_samples": 1,
        "fsync_every_samples": 1,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def increasing_clock(step=0.01):
    value = [-step]

    def read():
        value[0] += step
        return value[0]

    return read


def increasing_wall_clock():
    value = [1_780_000_000_000_000_000]

    def read():
        value[0] += 1_000_000
        return value[0]

    return read


def test_capture_preserves_all_physical_fields_and_canonical_steering():
    actor = Actor()
    module, _client = carla_module(World([[actor]]))
    sample = MODULE.capture_sample(module, actor, 0, 0.25, 1_780_000_000_000_000_000)

    assert sample["actor"]["yaw_degrees"] == 42.5
    assert sample["actor"]["linear_velocity_mps"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert sample["world_frame"] == 101
    assert sample["telemetry"]["world_frame"] == 101
    assert [item["canonical_location"] for item in sample["wheels"]] == [
        "FL",
        "FR",
        "RL",
        "RR",
    ]
    assert [item["steer_angle_degrees"] for item in sample["wheels"]] == [
        11.0,
        -12.0,
        13.0,
        -14.0,
    ]
    for item in sample["wheels"]:
        assert set(MODULE.WHEEL_BOOL_FIELDS).issubset(item)
        assert set(MODULE.WHEEL_FLOAT_FIELDS).issubset(item)
        assert set(MODULE.WHEEL_STRING_FIELDS).issubset(item)
        assert set(MODULE.WHEEL_VECTOR_FIELDS).issubset(item)


def test_run_writes_exclusive_jsonl_and_sha256_manifest(tmp_path):
    actor = Actor()
    world = World([[actor]])
    module, client = carla_module(world)
    output = tmp_path / "physical_wheels.jsonl"
    result = MODULE.run(
        args(output),
        carla_module=module,
        monotonic=increasing_clock(),
        wall_time_ns=increasing_wall_clock(),
        sleeper=lambda _seconds: None,
    )

    assert result["status"] == "COMPLETED"
    assert result["stop_reason"] == "max_samples"
    assert result["sample_count"] == 1
    assert result["first_world_frame"] == result["last_world_frame"] == 101
    assert client.timeout == 3.0
    assert world.wait_for_tick_calls == [3.0]
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [item["record_type"] for item in records] == ["header", "sample", "footer"]
    assert records[0]["read_only"] is True
    expected_sha = hashlib.sha256(output.read_bytes()).hexdigest()
    manifest = json.loads((tmp_path / "physical_wheels.manifest.json").read_text())
    assert manifest["output"]["sha256"] == expected_sha
    assert manifest["output"]["bytes"] == output.stat().st_size

    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        MODULE.validate_args(args(output))


def test_signal_stop_finalizes_a_clean_stopped_manifest(tmp_path):
    actor = Actor()
    module, _client = carla_module(World([[actor]]))
    stop_calls = [0]

    def should_stop():
        stop_calls[0] += 1
        return stop_calls[0] >= 2

    result = MODULE.run(
        args(tmp_path / "stopped.jsonl", max_samples=0),
        carla_module=module,
        should_stop=should_stop,
        monotonic=increasing_clock(),
        wall_time_ns=increasing_wall_clock(),
        sleeper=lambda _seconds: None,
    )
    assert result["status"] == "STOPPED"
    assert result["stop_reason"] == "signal"
    assert result["sample_count"] == 1
    assert result["error"] is None


def test_identity_drift_fails_closed_and_records_error_manifest(tmp_path):
    original = Actor(actor_id=17)
    replacement = Actor(actor_id=18)
    module, _client = carla_module(World([[original], [replacement]]))
    output = tmp_path / "drift.jsonl"
    with pytest.raises(RuntimeError, match="identity drifted"):
        MODULE.run(
            args(output),
            carla_module=module,
            monotonic=increasing_clock(),
            wall_time_ns=increasing_wall_clock(),
            sleeper=lambda _seconds: None,
        )
    manifest = json.loads((tmp_path / "drift.manifest.json").read_text())
    assert manifest["status"] == "ERROR"
    assert manifest["sample_count"] == 0
    assert manifest["error"]["type"] == "RuntimeError"


def test_nonfinite_or_incomplete_patched_telemetry_fails_closed():
    broken = telemetry()
    broken.wheels[2].external_drive_torque_nm = float("nan")
    actor = Actor(sample=broken)
    module, _client = carla_module(World([[actor]]))
    with pytest.raises(RuntimeError, match="non-finite"):
        MODULE.capture_sample(module, actor, 0, 0.0, 1)

    broken.wheels[2].external_drive_torque_nm = 2.25
    del broken.wheels[2].contact_normal
    with pytest.raises(RuntimeError, match="missing field"):
        MODULE.capture_sample(module, actor, 0, 0.0, 1)


def test_exact_ranger_cardinality_and_local_endpoint_are_mandatory(tmp_path):
    with pytest.raises(RuntimeError, match="found 0"):
        MODULE.select_unique_ranger(World([[]]))
    duplicate = Actor()
    with pytest.raises(RuntimeError, match="found 2"):
        MODULE.select_unique_ranger(World([[duplicate, Actor(actor_id=18)]]))
    with pytest.raises(ValueError, match="localhost:2000"):
        MODULE.validate_args(args(tmp_path / "remote.jsonl", host="10.0.0.1"))
    with pytest.raises(ValueError, match="localhost:2000"):
        MODULE.validate_args(args(tmp_path / "wrong_port.jsonl", port=2001))


def test_source_has_no_simulation_mutation_calls():
    tree = ast.parse(SCRIPT.read_text())
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint(
        {
            "tick",
            "apply_settings",
            "load_world",
            "reload_world",
            "spawn_actor",
            "try_spawn_actor",
            "destroy",
            "apply_control",
            "set_transform",
            "set_target_velocity",
            "set_target_angular_velocity",
            "set_wheel_physics_steer_angles",
            "set_wheel_physics_steer_angles_and_drive_torques",
        }
    )
