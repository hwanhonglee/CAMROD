"""Contracts for streaming physical-wheel mission evidence summaries."""
# flake8: noqa

import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tracemalloc

from PIL import Image
import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "summarize_physical_wheel_telemetry.py"
)
SPEC = importlib.util.spec_from_file_location(
    "summarize_physical_wheel_telemetry", SCRIPT
)
assert SPEC and SPEC.loader
wheel_summary = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = wheel_summary
SPEC.loader.exec_module(wheel_summary)


SCHEMA = wheel_summary.SOURCE_SCHEMA
ACTOR = {
    "id": 27,
    "type_id": "vehicle.ranger.default",
    "role_name": "ego_vehicle",
}
WHEEL_IDENTITIES = {
    "FL": (0, "front_left"),
    "FR": (1, "front_right"),
    "RL": (2, "rear_left"),
    "RR": (3, "rear_right"),
}


def _header():
    return {
        "record_type": "header",
        "schema": SCHEMA,
        "endpoint": "127.0.0.1:2000",
        "actor": dict(ACTOR),
        "rate_hz": 10.0,
        "read_only": True,
        "started_timestamp_unix_ns": 1_000_000_000,
    }


def _wheel(name, sequence):
    index, location = WHEEL_IDENTITIES[name]
    scale = float(index + 1)
    return {
        "canonical_location": name,
        "wheel_index": index,
        "wheel_location": location,
        "steer_angle_degrees": (-1.0 if index % 2 else 1.0) * scale * sequence,
        "combined_wheel_torque_nm": scale * (sequence + 1.0),
        "external_drive_torque_nm": scale * 0.5,
        "raw_solver_rotation_speed_rad_per_second": -scale * (sequence + 2.0),
        "corrected_angle_integration_speed_rad_per_second": scale * (sequence + 1.5),
        "longitudinal_slip": -scale * 0.1 * sequence,
        "lateral_slip_radians": scale * 0.01 * sequence,
        "is_in_air": name == "RR" and sequence == 1,
        "current_tire_load_n": 200.0 + scale * 10.0 + sequence,
        "contact_material_path": "/Game/PhysicalMaterials/Test",
        "contact_material_class": "/Script/PhysicsCore.PhysicalMaterial",
        "contact_surface_type": index,
    }


def _sample(sequence, *, actor=None, wheels=None):
    locations = ((0.0, 0.0, 1.0), (3.0, 4.0, 1.5), (6.0, 8.0, 0.5))
    x, y, z = locations[sequence % len(locations)]
    return {
        "record_type": "sample",
        "schema": SCHEMA,
        "sequence": sequence,
        "timestamp_unix_ns": 1_000_000_000 + sequence * 100_000_000,
        "elapsed_wall_seconds": sequence * 0.1,
        "world_frame": 100 + sequence,
        "actor": {
            **(actor or ACTOR),
            "transform": {
                "location_m": {"x": x, "y": y, "z": z},
                "rotation_degrees": {
                    "pitch": (-1.0 if sequence % 2 else 1.0) * sequence * 2.0,
                    "yaw": 0.0,
                    "roll": sequence * -3.0,
                },
            },
        },
        "wheels": wheels
        or [_wheel(name, sequence) for name in wheel_summary.CANONICAL_WHEELS],
    }


def _footer(sample_count=3, **updates):
    footer = {
        "record_type": "footer",
        "schema": SCHEMA,
        "status": "STOPPED",
        "stop_reason": "signal",
        "sample_count": sample_count,
        "first_world_frame": 100,
        "last_world_frame": 100 + sample_count - 1,
        "error": None,
        "ended_timestamp_unix_ns": 1_000_000_000 + sample_count * 100_000_000,
    }
    footer.update(updates)
    return footer


def _write(path, records):
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
    return path


def _valid_source(path, sample_count=3):
    return _write(
        path,
        [_header()]
        + [_sample(sequence) for sequence in range(sample_count)]
        + [_footer(sample_count)],
    )


def test_stream_summary_validates_and_measures_motion_and_all_four_wheels(tmp_path):
    source = _valid_source(tmp_path / "physical_wheels.jsonl")
    result = wheel_summary.analyze(source)

    assert result["schema"] == wheel_summary.SUMMARY_SCHEMA
    assert result["status"] == "PASS"
    assert result["actor"] == ACTOR
    assert result["samples"]["count"] == 3
    assert result["samples"]["observed_rate_hz"] == pytest.approx(10.0)
    assert result["motion"]["planar_distance_m"] == pytest.approx(10.0)
    assert result["motion"]["spatial_distance_m"] == pytest.approx(
        (25.25 ** 0.5) + (26.0 ** 0.5)
    )
    assert result["motion"]["max_abs_z_delta_from_initial_m"] == 0.5
    assert result["motion"]["max_abs_pitch_degrees"] == 4.0
    assert result["motion"]["max_abs_roll_degrees"] == 6.0
    assert tuple(result["wheels"]) == wheel_summary.CANONICAL_WHEELS
    assert result["wheels"]["RR"]["in_air_count"] == 1
    assert result["wheels"]["RR"]["in_air_ratio"] == pytest.approx(1 / 3)
    assert result["wheels"]["FL"]["max_abs_combined_wheel_torque_nm"] == 3.0
    assert result["wheels"]["RR"]["max_abs_omega_rad_per_second"] == 14.0
    assert result["source"]["sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()


def test_new_output_contains_json_csv_large_png_and_verified_hashes(tmp_path):
    summary = wheel_summary.analyze(_valid_source(tmp_path / "source.jsonl"))
    output = tmp_path / "evidence"
    assert wheel_summary.write_outputs(output, summary) == output.resolve()

    assert {item.name for item in output.iterdir()} == {
        "wheel_summary.json",
        "wheel_measurements.csv",
        "wheel_summary.png",
        "SHA256SUMS",
    }
    decoded = json.loads((output / "wheel_summary.json").read_text())
    assert decoded["source"]["sha256"] == summary["source"]["sha256"]
    rows = list(csv.DictReader((output / "wheel_measurements.csv").open()))
    assert [row["wheel"] for row in rows] == list(wheel_summary.CANONICAL_WHEELS)
    assert {row["source_sha256"] for row in rows} == {summary["source"]["sha256"]}
    with Image.open(output / "wheel_summary.png") as image:
        assert image.width >= 1600
        assert image.height > 0
        assert image.format == "PNG"
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ", 1)
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == digest
    with pytest.raises(wheel_summary.WheelSummaryError, match="already exists"):
        wheel_summary.write_outputs(output, summary)


def test_actor_identity_drift_is_rejected(tmp_path):
    records = [_header(), _sample(0), _sample(1, actor={**ACTOR, "id": 28}), _footer(2)]
    source = _write(tmp_path / "actor_drift.jsonl", records)
    with pytest.raises(wheel_summary.WheelSummaryError, match="identity drifted"):
        wheel_summary.analyze(source)


def test_wheels_must_be_exact_unique_fl_fr_rl_rr_with_matching_identity(tmp_path):
    duplicate = [_wheel(name, 0) for name in wheel_summary.CANONICAL_WHEELS]
    duplicate[-1] = copy.deepcopy(duplicate[-2])
    source = _write(tmp_path / "duplicate.jsonl", [_header(), _sample(0, wheels=duplicate), _footer(1)])
    with pytest.raises(wheel_summary.WheelSummaryError, match="duplicates wheel RL"):
        wheel_summary.analyze(source)

    wrong_identity = [_wheel(name, 0) for name in wheel_summary.CANONICAL_WHEELS]
    wrong_identity[0]["wheel_index"] = 3
    source = _write(tmp_path / "wrong_identity.jsonl", [_header(), _sample(0, wheels=wrong_identity), _footer(1)])
    with pytest.raises(wheel_summary.WheelSummaryError, match="identity must be"):
        wheel_summary.analyze(source)


@pytest.mark.parametrize(
    "footer,pattern",
    [
        (_footer(2), "sample_count 2 does not match"),
        (_footer(3, status="ERROR", error={"message": "failed"}), "status must be"),
        (_footer(3, schema="wrong.schema"), "schema mismatch"),
    ],
)
def test_invalid_footer_contracts_fail_closed(tmp_path, footer, pattern):
    source = _write(
        tmp_path / "bad_footer.jsonl",
        [_header(), _sample(0), _sample(1), _sample(2), footer],
    )
    with pytest.raises(wheel_summary.WheelSummaryError, match=pattern):
        wheel_summary.analyze(source)


def test_missing_footer_and_records_after_footer_are_rejected(tmp_path):
    missing = _write(tmp_path / "missing.jsonl", [_header(), _sample(0)])
    with pytest.raises(wheel_summary.WheelSummaryError, match="footer is missing"):
        wheel_summary.analyze(missing)

    after = _write(
        tmp_path / "after.jsonl",
        [_header(), _sample(0), _footer(1), _sample(1)],
    )
    with pytest.raises(wheel_summary.WheelSummaryError, match="appears after the footer"):
        wheel_summary.analyze(after)


def test_duplicate_json_keys_are_rejected(tmp_path):
    source = tmp_path / "ambiguous.jsonl"
    source.write_text(
        '{"record_type":"header","schema":"%s","schema":"%s"}\n'
        % (SCHEMA, SCHEMA),
        encoding="utf-8",
    )
    with pytest.raises(wheel_summary.WheelSummaryError, match="duplicate JSON key"):
        wheel_summary.analyze(source)


def test_analyzer_memory_does_not_scale_with_recording_size(tmp_path):
    sample_count = 3000
    source = tmp_path / "long.jsonl"
    with source.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(_header(), separators=(",", ":")) + "\n")
        for sequence in range(sample_count):
            sample = _sample(sequence % 3)
            sample["sequence"] = sequence
            sample["timestamp_unix_ns"] = 1_000_000_000 + sequence * 100_000_000
            sample["elapsed_wall_seconds"] = sequence * 0.1
            sample["world_frame"] = 100 + sequence
            stream.write(json.dumps(sample, separators=(",", ":")) + "\n")
        stream.write(
            json.dumps(
                _footer(
                    sample_count,
                    last_world_frame=100 + sample_count - 1,
                    ended_timestamp_unix_ns=1_000_000_000
                    + sample_count * 100_000_000,
                ),
                separators=(",", ":"),
            )
            + "\n"
        )
    assert source.stat().st_size > 5_000_000

    tracemalloc.start()
    result = wheel_summary.analyze(source)
    _, peak_bytes = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    assert result["samples"]["count"] == sample_count
    assert peak_bytes < 12_000_000
