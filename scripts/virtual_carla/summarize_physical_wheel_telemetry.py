#!/usr/bin/env python3
"""Validate and summarize a physical-wheel telemetry JSONL evidence file.

The input is consumed one line at a time so long mission recordings do not
need to fit in memory.  The command is offline/read-only with respect to CARLA
and writes a new, self-contained evidence directory atomically.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any, Iterable, Mapping


SOURCE_SCHEMA = "camrod.virtual_carla.physical_wheel_telemetry.v1"
SUMMARY_SCHEMA = "camrod.virtual_carla.physical_wheel_summary.v1"
EXPECTED_TYPE_ID = "vehicle.ranger.default"
EXPECTED_ROLE_NAME = "ego_vehicle"
CANONICAL_WHEELS = ("FL", "FR", "RL", "RR")
EXPECTED_WHEELS = {
    "FL": (0, "front_left"),
    "FR": (1, "front_right"),
    "RL": (2, "rear_left"),
    "RR": (3, "rear_right"),
}
VALID_FOOTER_STATUSES = frozenset(("COMPLETED", "STOPPED"))
OUTPUT_NAMES = (
    "wheel_summary.json",
    "wheel_measurements.csv",
    "wheel_summary.png",
)


class WheelSummaryError(RuntimeError):
    """The source evidence or requested output violates the contract."""


def _reject_duplicate_keys(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise WheelSummaryError("duplicate JSON key {!r}".format(key))
        result[key] = value
    return result


def _decode_record(raw: bytes, line_number: int) -> dict[str, Any]:
    if not raw.strip():
        raise WheelSummaryError("line {} is blank".format(line_number))
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise WheelSummaryError(
            "line {} is not valid UTF-8".format(line_number)
        ) from exc
    try:
        value = json.loads(text, object_pairs_hook=_reject_duplicate_keys)
    except (json.JSONDecodeError, WheelSummaryError) as exc:
        raise WheelSummaryError(
            "line {} is not valid unambiguous JSON: {}".format(line_number, exc)
        ) from exc
    if not isinstance(value, dict):
        raise WheelSummaryError("line {} must contain a JSON object".format(line_number))
    return value


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise WheelSummaryError("{} must be an object".format(label))
    return value


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise WheelSummaryError("{}.{} is required".format(label, key))
    return mapping[key]


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise WheelSummaryError("{} must be a string".format(label))
    return value


def _integer(value: Any, label: str, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise WheelSummaryError("{} must be an exact integer".format(label))
    if minimum is not None and value < minimum:
        raise WheelSummaryError("{} must be >= {}".format(label, minimum))
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WheelSummaryError("{} must be a finite number".format(label))
    result = float(value)
    if not math.isfinite(result):
        raise WheelSummaryError("{} must be a finite number".format(label))
    return result


def _identity(value: Any, label: str) -> tuple[int, str, str]:
    actor = _mapping(value, label)
    identity = (
        _integer(_required(actor, "id", label), "{}.id".format(label), 1),
        _string(_required(actor, "type_id", label), "{}.type_id".format(label)),
        _string(_required(actor, "role_name", label), "{}.role_name".format(label)),
    )
    if identity[1:] != (EXPECTED_TYPE_ID, EXPECTED_ROLE_NAME):
        raise WheelSummaryError(
            "{} must identify type={} role={}, got {}".format(
                label, EXPECTED_TYPE_ID, EXPECTED_ROLE_NAME, identity
            )
        )
    return identity


def _schema_and_type(record: Mapping[str, Any], expected_type: str, label: str) -> None:
    schema = _string(_required(record, "schema", label), "{}.schema".format(label))
    if schema != SOURCE_SCHEMA:
        raise WheelSummaryError(
            "{}.schema mismatch: expected {!r}, got {!r}".format(
                label, SOURCE_SCHEMA, schema
            )
        )
    record_type = _string(
        _required(record, "record_type", label), "{}.record_type".format(label)
    )
    if record_type != expected_type:
        raise WheelSummaryError(
            "{}.record_type must be {!r}, got {!r}".format(
                label, expected_type, record_type
            )
        )


@dataclass
class WheelAccumulator:
    sample_count: int = 0
    max_abs_steer_angle_degrees: float = 0.0
    max_abs_combined_wheel_torque_nm: float = 0.0
    max_abs_external_drive_torque_nm: float = 0.0
    max_abs_raw_solver_omega_rad_per_second: float = 0.0
    max_abs_omega_rad_per_second: float = 0.0
    max_abs_longitudinal_slip: float = 0.0
    max_abs_lateral_slip_radians: float = 0.0
    in_air_count: int = 0
    tire_load_min_n: float = math.inf
    tire_load_max_n: float = -math.inf
    contact_materials: Counter[tuple[str, str, int]] = field(default_factory=Counter)

    def update(self, wheel: Mapping[str, Any], label: str) -> None:
        self.sample_count += 1
        metrics = (
            ("steer_angle_degrees", "max_abs_steer_angle_degrees"),
            (
                "combined_wheel_torque_nm",
                "max_abs_combined_wheel_torque_nm",
            ),
            (
                "external_drive_torque_nm",
                "max_abs_external_drive_torque_nm",
            ),
            (
                "raw_solver_rotation_speed_rad_per_second",
                "max_abs_raw_solver_omega_rad_per_second",
            ),
            (
                "corrected_angle_integration_speed_rad_per_second",
                "max_abs_omega_rad_per_second",
            ),
            ("longitudinal_slip", "max_abs_longitudinal_slip"),
            ("lateral_slip_radians", "max_abs_lateral_slip_radians"),
        )
        for source_field, accumulator_field in metrics:
            value = abs(
                _finite(
                    _required(wheel, source_field, label),
                    "{}.{}".format(label, source_field),
                )
            )
            setattr(self, accumulator_field, max(getattr(self, accumulator_field), value))

        in_air = _required(wheel, "is_in_air", label)
        if not isinstance(in_air, bool):
            raise WheelSummaryError("{}.is_in_air must be boolean".format(label))
        self.in_air_count += int(in_air)

        load = _finite(
            _required(wheel, "current_tire_load_n", label),
            "{}.current_tire_load_n".format(label),
        )
        self.tire_load_min_n = min(self.tire_load_min_n, load)
        self.tire_load_max_n = max(self.tire_load_max_n, load)

        material_path = _string(
            _required(wheel, "contact_material_path", label),
            "{}.contact_material_path".format(label),
        )
        material_class = _string(
            _required(wheel, "contact_material_class", label),
            "{}.contact_material_class".format(label),
        )
        surface_type = _integer(
            _required(wheel, "contact_surface_type", label),
            "{}.contact_surface_type".format(label),
            0,
        )
        self.contact_materials[(material_path, material_class, surface_type)] += 1

    def result(self) -> dict[str, Any]:
        return {
            "sample_count": self.sample_count,
            "max_abs_steer_angle_degrees": self.max_abs_steer_angle_degrees,
            "max_abs_combined_wheel_torque_nm": self.max_abs_combined_wheel_torque_nm,
            "max_abs_external_drive_torque_nm": self.max_abs_external_drive_torque_nm,
            "max_abs_raw_solver_omega_rad_per_second": self.max_abs_raw_solver_omega_rad_per_second,
            "max_abs_omega_rad_per_second": self.max_abs_omega_rad_per_second,
            "max_abs_longitudinal_slip": self.max_abs_longitudinal_slip,
            "max_abs_lateral_slip_radians": self.max_abs_lateral_slip_radians,
            "in_air_count": self.in_air_count,
            "in_air_ratio": self.in_air_count / self.sample_count,
            "current_tire_load_n": {
                "min": self.tire_load_min_n,
                "max": self.tire_load_max_n,
            },
            "contact_materials": [
                {
                    "path": path,
                    "class": material_class,
                    "surface_type": surface_type,
                    "sample_count": count,
                    "sample_ratio": count / self.sample_count,
                }
                for (path, material_class, surface_type), count in sorted(
                    self.contact_materials.items()
                )
            ],
        }


@dataclass
class StreamAccumulator:
    actor_identity: tuple[int, str, str]
    header_rate_hz: float
    header_started_ns: int
    wheel_stats: dict[str, WheelAccumulator] = field(
        default_factory=lambda: {
            wheel: WheelAccumulator() for wheel in CANONICAL_WHEELS
        }
    )
    sample_count: int = 0
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    first_elapsed_s: float | None = None
    last_elapsed_s: float | None = None
    first_world_frame: int | None = None
    last_world_frame: int | None = None
    first_location: tuple[float, float, float] | None = None
    last_location: tuple[float, float, float] | None = None
    planar_distance_m: float = 0.0
    spatial_distance_m: float = 0.0
    z_min_m: float = math.inf
    z_max_m: float = -math.inf
    max_abs_z_delta_from_initial_m: float = 0.0
    max_abs_pitch_degrees: float = 0.0
    max_abs_roll_degrees: float = 0.0

    def update(self, sample: Mapping[str, Any], line_number: int) -> None:
        label = "line {} sample".format(line_number)
        expected_sequence = self.sample_count
        sequence = _integer(
            _required(sample, "sequence", label), "{}.sequence".format(label), 0
        )
        if sequence != expected_sequence:
            raise WheelSummaryError(
                "{}.sequence must be {}, got {}".format(
                    label, expected_sequence, sequence
                )
            )
        identity = _identity(_required(sample, "actor", label), "{}.actor".format(label))
        if identity != self.actor_identity:
            raise WheelSummaryError(
                "{} actor identity drifted from {} to {}".format(
                    label, self.actor_identity, identity
                )
            )

        timestamp_ns = _integer(
            _required(sample, "timestamp_unix_ns", label),
            "{}.timestamp_unix_ns".format(label),
            1,
        )
        elapsed_s = _finite(
            _required(sample, "elapsed_wall_seconds", label),
            "{}.elapsed_wall_seconds".format(label),
        )
        if elapsed_s < 0.0:
            raise WheelSummaryError("{}.elapsed_wall_seconds must be non-negative".format(label))
        if self.last_timestamp_ns is not None and timestamp_ns < self.last_timestamp_ns:
            raise WheelSummaryError("{} timestamp moved backwards".format(label))
        if self.last_elapsed_s is not None and elapsed_s < self.last_elapsed_s:
            raise WheelSummaryError("{} elapsed time moved backwards".format(label))

        world_frame = _integer(
            _required(sample, "world_frame", label), "{}.world_frame".format(label), 0
        )
        if self.last_world_frame is not None and world_frame < self.last_world_frame:
            raise WheelSummaryError("{} world frame moved backwards".format(label))

        actor = _mapping(sample["actor"], "{}.actor".format(label))
        transform = _mapping(
            _required(actor, "transform", "{}.actor".format(label)),
            "{}.actor.transform".format(label),
        )
        location = _mapping(
            _required(transform, "location_m", "{}.actor.transform".format(label)),
            "{}.actor.transform.location_m".format(label),
        )
        rotation = _mapping(
            _required(
                transform,
                "rotation_degrees",
                "{}.actor.transform".format(label),
            ),
            "{}.actor.transform.rotation_degrees".format(label),
        )
        point = tuple(
            _finite(
                _required(location, axis, "{}.actor.transform.location_m".format(label)),
                "{}.actor.transform.location_m.{}".format(label, axis),
            )
            for axis in ("x", "y", "z")
        )
        pitch = _finite(
            _required(rotation, "pitch", "{}.actor.transform.rotation_degrees".format(label)),
            "{}.actor.transform.rotation_degrees.pitch".format(label),
        )
        roll = _finite(
            _required(rotation, "roll", "{}.actor.transform.rotation_degrees".format(label)),
            "{}.actor.transform.rotation_degrees.roll".format(label),
        )

        wheels = _required(sample, "wheels", label)
        if not isinstance(wheels, list) or len(wheels) != 4:
            raise WheelSummaryError("{}.wheels must contain exactly four entries".format(label))
        by_name: dict[str, Mapping[str, Any]] = {}
        for index, raw_wheel in enumerate(wheels):
            wheel_label = "{}.wheels[{}]".format(label, index)
            wheel = _mapping(raw_wheel, wheel_label)
            name = _string(
                _required(wheel, "canonical_location", wheel_label),
                "{}.canonical_location".format(wheel_label),
            )
            if name not in EXPECTED_WHEELS:
                raise WheelSummaryError(
                    "{}.canonical_location is unexpected: {!r}".format(wheel_label, name)
                )
            if name in by_name:
                raise WheelSummaryError("{} duplicates wheel {}".format(label, name))
            expected_index, expected_location = EXPECTED_WHEELS[name]
            actual_index = _integer(
                _required(wheel, "wheel_index", wheel_label),
                "{}.wheel_index".format(wheel_label),
                0,
            )
            actual_location = _string(
                _required(wheel, "wheel_location", wheel_label),
                "{}.wheel_location".format(wheel_label),
            )
            if (actual_index, actual_location) != (expected_index, expected_location):
                raise WheelSummaryError(
                    "{} identity must be index={} location={}, got index={} location={}".format(
                        name,
                        expected_index,
                        expected_location,
                        actual_index,
                        actual_location,
                    )
                )
            by_name[name] = wheel
        if set(by_name) != set(CANONICAL_WHEELS):
            raise WheelSummaryError("{}.wheels are not exactly FL/FR/RL/RR".format(label))

        if self.first_timestamp_ns is None:
            self.first_timestamp_ns = timestamp_ns
            self.first_elapsed_s = elapsed_s
            self.first_world_frame = world_frame
            self.first_location = point
        if self.last_location is not None:
            dx = point[0] - self.last_location[0]
            dy = point[1] - self.last_location[1]
            dz = point[2] - self.last_location[2]
            self.planar_distance_m += math.hypot(dx, dy)
            self.spatial_distance_m += math.sqrt(dx * dx + dy * dy + dz * dz)
        initial_z = self.first_location[2] if self.first_location is not None else point[2]
        self.z_min_m = min(self.z_min_m, point[2])
        self.z_max_m = max(self.z_max_m, point[2])
        self.max_abs_z_delta_from_initial_m = max(
            self.max_abs_z_delta_from_initial_m, abs(point[2] - initial_z)
        )
        self.max_abs_pitch_degrees = max(self.max_abs_pitch_degrees, abs(pitch))
        self.max_abs_roll_degrees = max(self.max_abs_roll_degrees, abs(roll))
        for name in CANONICAL_WHEELS:
            self.wheel_stats[name].update(by_name[name], "{}.wheel.{}".format(label, name))

        self.sample_count += 1
        self.last_timestamp_ns = timestamp_ns
        self.last_elapsed_s = elapsed_s
        self.last_world_frame = world_frame
        self.last_location = point


def _validate_header(record: Mapping[str, Any]) -> StreamAccumulator:
    _schema_and_type(record, "header", "header")
    identity = _identity(_required(record, "actor", "header"), "header.actor")
    read_only = _required(record, "read_only", "header")
    if read_only is not True:
        raise WheelSummaryError("header.read_only must be true")
    rate_hz = _finite(_required(record, "rate_hz", "header"), "header.rate_hz")
    if rate_hz <= 0.0:
        raise WheelSummaryError("header.rate_hz must be positive")
    started_ns = _integer(
        _required(record, "started_timestamp_unix_ns", "header"),
        "header.started_timestamp_unix_ns",
        1,
    )
    return StreamAccumulator(identity, rate_hz, started_ns)


def _validate_footer(
    footer: Mapping[str, Any], accumulator: StreamAccumulator, line_number: int
) -> dict[str, Any]:
    label = "line {} footer".format(line_number)
    _schema_and_type(footer, "footer", label)
    status = _string(_required(footer, "status", label), "{}.status".format(label))
    if status not in VALID_FOOTER_STATUSES:
        raise WheelSummaryError(
            "{}.status must be one of {}, got {!r}".format(
                label, sorted(VALID_FOOTER_STATUSES), status
            )
        )
    if _required(footer, "error", label) is not None:
        raise WheelSummaryError("{}.error must be null for valid evidence".format(label))
    sample_count = _integer(
        _required(footer, "sample_count", label), "{}.sample_count".format(label), 0
    )
    if sample_count != accumulator.sample_count:
        raise WheelSummaryError(
            "footer sample_count {} does not match streamed sample count {}".format(
                sample_count, accumulator.sample_count
            )
        )
    if sample_count <= 0:
        raise WheelSummaryError("source must contain at least one sample")
    first_frame = _integer(
        _required(footer, "first_world_frame", label),
        "{}.first_world_frame".format(label),
        0,
    )
    last_frame = _integer(
        _required(footer, "last_world_frame", label),
        "{}.last_world_frame".format(label),
        0,
    )
    if (first_frame, last_frame) != (
        accumulator.first_world_frame,
        accumulator.last_world_frame,
    ):
        raise WheelSummaryError(
            "footer world-frame bounds do not match streamed samples"
        )
    ended_ns = _integer(
        _required(footer, "ended_timestamp_unix_ns", label),
        "{}.ended_timestamp_unix_ns".format(label),
        1,
    )
    if ended_ns < accumulator.header_started_ns:
        raise WheelSummaryError("footer ended timestamp precedes header start")
    return {
        "status": status,
        "stop_reason": _string(
            _required(footer, "stop_reason", label), "{}.stop_reason".format(label)
        ),
        "ended_timestamp_unix_ns": ended_ns,
    }


def analyze(source: Path | str) -> dict[str, Any]:
    """Stream, validate, and summarize ``source`` without buffering its samples."""
    source_path = Path(source).expanduser().resolve()
    if not source_path.is_file():
        raise WheelSummaryError("source is not a regular file: {}".format(source_path))

    digest = hashlib.sha256()
    source_bytes = 0
    line_count = 0
    accumulator: StreamAccumulator | None = None
    footer_result: dict[str, Any] | None = None
    with source_path.open("rb") as stream:
        for raw in stream:
            line_count += 1
            source_bytes += len(raw)
            digest.update(raw)
            record = _decode_record(raw, line_count)
            if accumulator is None:
                accumulator = _validate_header(record)
                continue
            if footer_result is not None:
                raise WheelSummaryError(
                    "line {} appears after the footer; footer must be last".format(line_count)
                )
            record_type = record.get("record_type")
            if record_type == "sample":
                _schema_and_type(record, "sample", "line {} sample".format(line_count))
                accumulator.update(record, line_count)
            elif record_type == "footer":
                footer_result = _validate_footer(record, accumulator, line_count)
            else:
                raise WheelSummaryError(
                    "line {} has unexpected record_type {!r}".format(
                        line_count, record_type
                    )
                )

    if accumulator is None:
        raise WheelSummaryError("source is empty; header is missing")
    if footer_result is None:
        raise WheelSummaryError("source footer is missing")
    assert accumulator.first_timestamp_ns is not None
    assert accumulator.last_timestamp_ns is not None
    assert accumulator.first_elapsed_s is not None
    assert accumulator.last_elapsed_s is not None
    assert accumulator.first_location is not None
    assert accumulator.last_location is not None

    sample_span_s = (
        accumulator.last_timestamp_ns - accumulator.first_timestamp_ns
    ) / 1_000_000_000.0
    recording_duration_s = (
        footer_result["ended_timestamp_unix_ns"] - accumulator.header_started_ns
    ) / 1_000_000_000.0
    observed_rate_hz = (
        (accumulator.sample_count - 1) / sample_span_s
        if accumulator.sample_count > 1 and sample_span_s > 0.0
        else None
    )
    actor_id, actor_type, role_name = accumulator.actor_identity
    return {
        "schema": SUMMARY_SCHEMA,
        "status": "PASS",
        "source": {
            "path": str(source_path),
            "bytes": source_bytes,
            "sha256": digest.hexdigest(),
            "schema": SOURCE_SCHEMA,
            "line_count": line_count,
        },
        "validation": {
            "streaming_reader": True,
            "header_valid": True,
            "footer_valid": True,
            "footer_status": footer_result["status"],
            "footer_stop_reason": footer_result["stop_reason"],
            "footer_sample_count_matches": True,
            "actor_identity_consistent": True,
            "canonical_wheels_exact": list(CANONICAL_WHEELS),
        },
        "actor": {
            "id": actor_id,
            "type_id": actor_type,
            "role_name": role_name,
        },
        "samples": {
            "count": accumulator.sample_count,
            "header_requested_rate_hz": accumulator.header_rate_hz,
            "observed_rate_hz": observed_rate_hz,
            "recording_duration_seconds": recording_duration_s,
            "sample_span_seconds": sample_span_s,
            "first_elapsed_wall_seconds": accumulator.first_elapsed_s,
            "last_elapsed_wall_seconds": accumulator.last_elapsed_s,
            "first_timestamp_unix_ns": accumulator.first_timestamp_ns,
            "last_timestamp_unix_ns": accumulator.last_timestamp_ns,
            "first_world_frame": accumulator.first_world_frame,
            "last_world_frame": accumulator.last_world_frame,
        },
        "motion": {
            "planar_distance_m": accumulator.planar_distance_m,
            "spatial_distance_m": accumulator.spatial_distance_m,
            "initial_location_m": dict(
                zip(("x", "y", "z"), accumulator.first_location)
            ),
            "final_location_m": dict(
                zip(("x", "y", "z"), accumulator.last_location)
            ),
            "z_min_m": accumulator.z_min_m,
            "z_max_m": accumulator.z_max_m,
            "max_abs_z_delta_from_initial_m": accumulator.max_abs_z_delta_from_initial_m,
            "max_abs_pitch_degrees": accumulator.max_abs_pitch_degrees,
            "max_abs_roll_degrees": accumulator.max_abs_roll_degrees,
        },
        "metric_definitions": {
            "distance": "sum of consecutive actor-transform location deltas",
            "z_delta": "absolute actor Z difference from the first sample",
            "torque": "combined_wheel_torque_nm; external drive torque is also reported",
            "omega": "corrected_angle_integration_speed_rad_per_second; raw solver omega is also reported",
            "slip": "absolute longitudinal_slip and lateral_slip_radians maxima",
            "tire_load": "current_tire_load_n minimum and maximum",
        },
        "wheels": {
            name: accumulator.wheel_stats[name].result()
            for name in CANONICAL_WHEELS
        },
    }


def _write_json(path: Path, summary: Mapping[str, Any]) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(
            summary,
            stream,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")


def _contact_material_csv(materials: list[Mapping[str, Any]]) -> str:
    return "; ".join(
        "{}|{}|surface={}|samples={}".format(
            item["path"] or "<none>",
            item["class"] or "<none>",
            item["surface_type"],
            item["sample_count"],
        )
        for item in materials
    )


def _write_csv(path: Path, summary: Mapping[str, Any]) -> None:
    fieldnames = (
        "wheel",
        "sample_count",
        "max_abs_steer_angle_degrees",
        "max_abs_combined_wheel_torque_nm",
        "max_abs_external_drive_torque_nm",
        "max_abs_omega_rad_per_second",
        "max_abs_raw_solver_omega_rad_per_second",
        "max_abs_longitudinal_slip",
        "max_abs_lateral_slip_radians",
        "in_air_count",
        "in_air_ratio",
        "current_tire_load_min_n",
        "current_tire_load_max_n",
        "contact_materials",
        "source_sha256",
    )
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for name in CANONICAL_WHEELS:
            wheel = summary["wheels"][name]
            writer.writerow(
                {
                    "wheel": name,
                    "sample_count": wheel["sample_count"],
                    "max_abs_steer_angle_degrees": wheel["max_abs_steer_angle_degrees"],
                    "max_abs_combined_wheel_torque_nm": wheel[
                        "max_abs_combined_wheel_torque_nm"
                    ],
                    "max_abs_external_drive_torque_nm": wheel[
                        "max_abs_external_drive_torque_nm"
                    ],
                    "max_abs_omega_rad_per_second": wheel[
                        "max_abs_omega_rad_per_second"
                    ],
                    "max_abs_raw_solver_omega_rad_per_second": wheel[
                        "max_abs_raw_solver_omega_rad_per_second"
                    ],
                    "max_abs_longitudinal_slip": wheel[
                        "max_abs_longitudinal_slip"
                    ],
                    "max_abs_lateral_slip_radians": wheel[
                        "max_abs_lateral_slip_radians"
                    ],
                    "in_air_count": wheel["in_air_count"],
                    "in_air_ratio": wheel["in_air_ratio"],
                    "current_tire_load_min_n": wheel["current_tire_load_n"]["min"],
                    "current_tire_load_max_n": wheel["current_tire_load_n"]["max"],
                    "contact_materials": _contact_material_csv(
                        wheel["contact_materials"]
                    ),
                    "source_sha256": summary["source"]["sha256"],
                }
            )


def _load_font(size: int, bold: bool = False):
    from PIL import ImageFont

    candidates = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    )
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def _draw_metric_bar(draw, box, label, values, color, font, small_font) -> None:
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=18, fill="#f7f9fb", outline="#d9e1e8", width=2)
    draw.text((x0 + 22, y0 + 15), label, font=font, fill="#15324a")
    maximum = max(values.values()) if values else 0.0
    bar_left = x0 + 78
    bar_right = x1 - 150
    row_y = y0 + 66
    for name in CANONICAL_WHEELS:
        value = values[name]
        draw.text((x0 + 22, row_y + 4), name, font=small_font, fill="#334e5c")
        draw.rounded_rectangle(
            (bar_left, row_y, bar_right, row_y + 24),
            radius=8,
            fill="#e3eaf0",
        )
        fraction = value / maximum if maximum > 0.0 else 0.0
        fill_right = bar_left + max(2, int((bar_right - bar_left) * fraction))
        draw.rounded_rectangle(
            (bar_left, row_y, fill_right, row_y + 24),
            radius=8,
            fill=color,
        )
        draw.text(
            (bar_right + 14, row_y + 2),
            "{:.3g}".format(value),
            font=small_font,
            fill="#334e5c",
        )
        row_y += 42


def _write_png(path: Path, summary: Mapping[str, Any]) -> None:
    from PIL import Image, ImageDraw

    width, height = 1800, 1200
    image = Image.new("RGB", (width, height), "#eef2f5")
    draw = ImageDraw.Draw(image)
    title_font = _load_font(46, bold=True)
    heading_font = _load_font(28, bold=True)
    body_font = _load_font(23)
    small_font = _load_font(19)
    draw.rectangle((0, 0, width, 112), fill="#123249")
    draw.text((54, 25), "CAMROD Physical 4-Wheel Mission Summary", font=title_font, fill="white")
    draw.text(
        (54, 128),
        "Validated source: {}  |  actor {}  |  {} samples".format(
            Path(summary["source"]["path"]).name,
            summary["actor"]["id"],
            summary["samples"]["count"],
        ),
        font=body_font,
        fill="#29485d",
    )

    cards = (
        (
            "Duration",
            "{:.2f} s".format(summary["samples"]["recording_duration_seconds"]),
        ),
        ("Observed rate", "{:.2f} Hz".format(summary["samples"]["observed_rate_hz"] or 0.0)),
        ("Planar distance", "{:.3f} m".format(summary["motion"]["planar_distance_m"])),
        (
            "Body attitude",
            "P {:.2f}°  R {:.2f}°".format(
                summary["motion"]["max_abs_pitch_degrees"],
                summary["motion"]["max_abs_roll_degrees"],
            ),
        ),
        (
            "Max |Z - Z0|",
            "{:.3f} m".format(
                summary["motion"]["max_abs_z_delta_from_initial_m"]
            ),
        ),
    )
    card_y = 182
    card_gap = 18
    card_width = (width - 108 - card_gap * 4) // 5
    for index, (label, value) in enumerate(cards):
        x0 = 54 + index * (card_width + card_gap)
        x1 = x0 + card_width
        draw.rounded_rectangle(
            (x0, card_y, x1, card_y + 122),
            radius=18,
            fill="white",
            outline="#d4dde4",
            width=2,
        )
        draw.text((x0 + 18, card_y + 18), label, font=small_font, fill="#5b7180")
        draw.text((x0 + 18, card_y + 61), value, font=heading_font, fill="#17384d")

    wheels = summary["wheels"]
    panels = (
        (
            (54, 332, 872, 590),
            "Max |steer| (degrees)",
            {name: wheels[name]["max_abs_steer_angle_degrees"] for name in CANONICAL_WHEELS},
            "#247ba0",
        ),
        (
            (928, 332, 1746, 590),
            "Max |combined torque| (N·m)",
            {name: wheels[name]["max_abs_combined_wheel_torque_nm"] for name in CANONICAL_WHEELS},
            "#e76f51",
        ),
        (
            (54, 616, 872, 874),
            "Max |wheel omega| (rad/s)",
            {name: wheels[name]["max_abs_omega_rad_per_second"] for name in CANONICAL_WHEELS},
            "#43aa8b",
        ),
        (
            (928, 616, 1746, 874),
            "Max |longitudinal slip|",
            {name: wheels[name]["max_abs_longitudinal_slip"] for name in CANONICAL_WHEELS},
            "#f4a261",
        ),
    )
    for box, label, values, color in panels:
        _draw_metric_bar(draw, box, label, values, color, heading_font, small_font)

    draw.rounded_rectangle(
        (54, 902, 1746, 1136),
        radius=18,
        fill="white",
        outline="#d4dde4",
        width=2,
    )
    draw.text((76, 918), "Ground contact and tire load", font=heading_font, fill="#15324a")
    table_y = 968
    headers = ("Wheel", "In-air samples", "In-air ratio", "Tire load min", "Tire load max", "Materials")
    columns = (80, 245, 485, 690, 920, 1150)
    for x, header in zip(columns, headers):
        draw.text((x, table_y), header, font=small_font, fill="#627683")
    table_y += 39
    for name in CANONICAL_WHEELS:
        wheel = wheels[name]
        material_count = len(wheel["contact_materials"])
        cells = (
            name,
            str(wheel["in_air_count"]),
            "{:.4%}".format(wheel["in_air_ratio"]),
            "{:.1f} N".format(wheel["current_tire_load_n"]["min"]),
            "{:.1f} N".format(wheel["current_tire_load_n"]["max"]),
            "{} unique (see JSON/CSV)".format(material_count),
        )
        for x, cell in zip(columns, cells):
            draw.text((x, table_y), cell, font=small_font, fill="#243f50")
        table_y += 29
    draw.text(
        (56, 1164),
        "source sha256: {}".format(summary["source"]["sha256"]),
        font=small_font,
        fill="#546b78",
    )
    image.save(path, format="PNG", optimize=True)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_outputs(output_dir: Path | str, summary: Mapping[str, Any]) -> Path:
    """Write the four evidence artifacts to a new directory, or write nothing."""
    destination = Path(output_dir).expanduser().resolve()
    if destination.exists() or destination.is_symlink():
        raise WheelSummaryError(
            "output directory already exists; refusing to overwrite: {}".format(
                destination
            )
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(
            prefix=".{}-tmp-".format(destination.name), dir=str(destination.parent)
        )
    )
    try:
        _write_json(temporary / "wheel_summary.json", summary)
        _write_csv(temporary / "wheel_measurements.csv", summary)
        _write_png(temporary / "wheel_summary.png", summary)
        with (temporary / "SHA256SUMS").open(
            "x", encoding="utf-8", newline="\n"
        ) as stream:
            for name in OUTPUT_NAMES:
                stream.write("{}  {}\n".format(_sha256(temporary / name), name))
        if destination.exists() or destination.is_symlink():
            raise WheelSummaryError(
                "output directory appeared while writing; refusing to overwrite: {}".format(
                    destination
                )
            )
        os.rename(temporary, destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return destination


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="physical_wheels.jsonl source")
    parser.add_argument(
        "--output-dir", required=True, help="new evidence output directory"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        summary = analyze(args.input)
        output = write_outputs(args.output_dir, summary)
    except (WheelSummaryError, OSError, ValueError) as exc:
        print(
            "[physical_wheel_summary] ERROR: {}: {}".format(type(exc).__name__, exc),
            file=sys.stderr,
            flush=True,
        )
        return 1
    print(
        "[physical_wheel_summary] PASS samples={} distance_m={:.3f} output={} source_sha256={}".format(
            summary["samples"]["count"],
            summary["motion"]["planar_distance_m"],
            output,
            summary["source"]["sha256"],
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
