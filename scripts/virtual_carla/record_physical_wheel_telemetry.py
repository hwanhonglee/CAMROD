#!/usr/bin/env python3
"""Passively record physical Ranger wheel telemetry from a live CARLA server.

This process is deliberately an observer.  Its CARLA calls are limited to
client/world inventory reads and vehicle state/physics readback.  It has no
simulation-clock, actor-lifecycle, or vehicle-command authority.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Any, Callable


SCHEMA = "camrod.virtual_carla.physical_wheel_telemetry.v1"
MANIFEST_SCHEMA = "camrod.virtual_carla.physical_wheel_telemetry_manifest.v1"
EXPECTED_HOSTS = frozenset(("127.0.0.1", "localhost", "::1"))
EXPECTED_PORT = 2000
EXPECTED_TYPE_ID = "vehicle.ranger.default"
EXPECTED_ROLE_NAME = "ego_vehicle"
EXPECTED_API_VERSION = 1

CANONICAL_WHEELS = ("FL", "FR", "RL", "RR")
EXPECTED_WHEEL_LOCATIONS = {
    0: "front_left",
    1: "front_right",
    2: "rear_left",
    3: "rear_right",
}

WHEEL_BOOL_FIELDS = (
    "is_in_air",
    "tire_force_shader_invoked",
    "tire_force_sample_valid",
    "sticky_forward_constraint_active",
    "external_drive_torque_masked",
)
WHEEL_FLOAT_FIELDS = (
    "raw_solver_rotation_speed_rad_per_second",
    "corrected_angle_integration_speed_rad_per_second",
    "ue_rotation_angle_degrees",
    "ue_angle_convention_corrected_speed_degrees_per_second",
    "wheel_radius_m",
    "tire_longitudinal_speed_mps",
    "longitudinal_slip",
    "lateral_slip_radians",
    "rest_tire_load_n",
    "normalized_tire_load",
    "current_tire_load_n",
    "tire_friction",
    "tire_longitudinal_force_n",
    "tire_lateral_force_n",
    "combined_wheel_torque_nm",
    "external_drive_torque_nm",
    "tire_reaction_torque_nm",
    "suspension_jounce_m",
    "suspension_spring_force_n",
)
WHEEL_STRING_FIELDS = (
    "contact_material_path",
    "contact_material_class",
)
WHEEL_VECTOR_FIELDS = (
    "contact_point_m",
    "contact_normal",
    "tire_longitudinal_direction",
    "tire_lateral_direction",
)
TELEMETRY_STRING_FIELDS = (
    "collection_scope",
    "wheel_query_scope",
    "wheel_dynamics_scope",
    "body_scope",
    "tire_force_scope",
)
TELEMETRY_BOOL_FIELDS = (
    "rigid_body_sleeping",
    "wheel_query_current",
    "vehicle_substep_metadata_valid",
    "component_live_substep_control_match",
)
TELEMETRY_INT_FIELDS = (
    "world_frame",
    "engine_frame_counter",
    "wheel_query_engine_frame_counter",
    "active_vehicle_substep_count",
    "last_vehicle_substep_index",
    "external_drive_torque_wheel_mask",
)
TELEMETRY_FLOAT_FIELDS = (
    "world_delta_seconds",
    "vehicle_substep_delta_seconds",
    "min_long_slip_denominator_mps",
)
SUBSTEP_FIELDS = (
    "threshold_longitudinal_speed_mps",
    "low_forward_speed_substep_count",
    "high_forward_speed_substep_count",
)


def _required_attr(value: Any, name: str) -> Any:
    if not hasattr(value, name):
        raise RuntimeError("patched CARLA telemetry is missing field {!r}".format(name))
    return getattr(value, name)


def _finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise RuntimeError("{} is boolean, expected finite number".format(label))
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("{} is not numeric".format(label)) from exc
    if not math.isfinite(result):
        raise RuntimeError("{} is non-finite".format(label))
    return result


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise RuntimeError("{} is boolean, expected integer".format(label))
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("{} is not an integer".format(label)) from exc
    if result < 0 or result != value:
        raise RuntimeError("{} is not a non-negative exact integer".format(label))
    return result


def _vector_record(value: Any, label: str) -> dict[str, float]:
    return {
        axis: _finite_float(_required_attr(value, axis), "{}.{}".format(label, axis))
        for axis in ("x", "y", "z")
    }


def _transform_record(value: Any, label: str) -> dict[str, dict[str, float]]:
    location = _required_attr(value, "location")
    rotation = _required_attr(value, "rotation")
    return {
        "location_m": _vector_record(location, "{}.location".format(label)),
        "rotation_degrees": {
            axis: _finite_float(
                _required_attr(rotation, axis), "{}.rotation.{}".format(label, axis)
            )
            for axis in ("pitch", "yaw", "roll")
        },
    }


def _actor_identity(actor: Any) -> tuple[int, str, str]:
    attributes = getattr(actor, "attributes", {}) or {}
    return (
        _nonnegative_int(getattr(actor, "id", -1), "actor.id"),
        str(getattr(actor, "type_id", "")),
        str(attributes.get("role_name", "")),
    )


def select_unique_ranger(world: Any) -> Any:
    matches = []
    for actor in world.get_actors():
        if (
            str(getattr(actor, "type_id", "")) == EXPECTED_TYPE_ID
            and str((getattr(actor, "attributes", {}) or {}).get("role_name", ""))
            == EXPECTED_ROLE_NAME
        ):
            matches.append(actor)
    if len(matches) != 1:
        identities = []
        for actor in matches:
            try:
                identities.append(_actor_identity(actor))
            except RuntimeError:
                identities.append(("invalid", EXPECTED_TYPE_ID, EXPECTED_ROLE_NAME))
        raise RuntimeError(
            "expected exactly one type={} role={} actor, found {}: {}".format(
                EXPECTED_TYPE_ID, EXPECTED_ROLE_NAME, len(matches), identities
            )
        )
    return matches[0]


def _substep_record(value: Any, label: str) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for field in SUBSTEP_FIELDS:
        raw = _required_attr(value, field)
        if field.endswith("count"):
            result[field] = _nonnegative_int(raw, "{}.{}".format(label, field))
        else:
            result[field] = _finite_float(raw, "{}.{}".format(label, field))
    return result


def _wheel_enums(carla_module: Any) -> tuple[Any, Any, Any, Any]:
    locations = _required_attr(carla_module, "VehicleWheelLocation")
    try:
        return (
            locations.FL_Wheel,
            locations.FR_Wheel,
            locations.BL_Wheel,
            locations.BR_Wheel,
        )
    except AttributeError as exc:
        raise RuntimeError("CARLA physical wheel-location enums are incomplete") from exc


def _wheel_record(item: Any, steer_angle_degrees: float) -> dict[str, Any]:
    index = _nonnegative_int(_required_attr(item, "wheel_index"), "wheel_index")
    if index not in EXPECTED_WHEEL_LOCATIONS:
        raise RuntimeError("unexpected physical wheel index {}".format(index))
    location = str(_required_attr(item, "wheel_location"))
    if location != EXPECTED_WHEEL_LOCATIONS[index]:
        raise RuntimeError(
            "wheel {} location mismatch: expected {!r}, got {!r}".format(
                index, EXPECTED_WHEEL_LOCATIONS[index], location
            )
        )
    result: dict[str, Any] = {
        "canonical_location": CANONICAL_WHEELS[index],
        "wheel_index": index,
        "wheel_location": location,
        "steer_angle_degrees": _finite_float(
            steer_angle_degrees, "wheel[{}].steer_angle_degrees".format(index)
        ),
        "contact_surface_type": _nonnegative_int(
            _required_attr(item, "contact_surface_type"),
            "wheel[{}].contact_surface_type".format(index),
        ),
    }
    for field in WHEEL_BOOL_FIELDS:
        raw = _required_attr(item, field)
        if not isinstance(raw, bool):
            raise RuntimeError("wheel[{}].{} is not boolean".format(index, field))
        result[field] = raw
    for field in WHEEL_FLOAT_FIELDS:
        result[field] = _finite_float(
            _required_attr(item, field), "wheel[{}].{}".format(index, field)
        )
    for field in WHEEL_STRING_FIELDS:
        result[field] = str(_required_attr(item, field))
    for field in WHEEL_VECTOR_FIELDS:
        result[field] = _vector_record(
            _required_attr(item, field), "wheel[{}].{}".format(index, field)
        )
    return result


def capture_sample(
    carla_module: Any,
    actor: Any,
    sequence: int,
    elapsed_s: float,
    wall_time_ns: int,
) -> dict[str, Any]:
    telemetry_reader = getattr(actor, "get_wheel_physics_telemetry", None)
    steer_reader = getattr(actor, "get_wheel_steer_angle", None)
    if not callable(telemetry_reader) or not callable(steer_reader):
        raise RuntimeError(
            "vehicle lacks patched physical telemetry or wheel steer readback API"
        )
    telemetry = telemetry_reader()
    api_version = _nonnegative_int(
        _required_attr(telemetry, "api_version"), "telemetry.api_version"
    )
    if api_version != EXPECTED_API_VERSION:
        raise RuntimeError(
            "telemetry API version mismatch: expected {}, got {}".format(
                EXPECTED_API_VERSION, api_version
            )
        )

    steer_angles = tuple(
        _finite_float(steer_reader(location), "steer_readback[{}]".format(index))
        for index, location in enumerate(_wheel_enums(carla_module))
    )
    raw_wheels = list(_required_attr(telemetry, "wheels"))
    if len(raw_wheels) != 4:
        raise RuntimeError(
            "physical telemetry must contain exactly four wheels, got {}".format(
                len(raw_wheels)
            )
        )
    by_index: dict[int, Any] = {}
    for item in raw_wheels:
        index = _nonnegative_int(_required_attr(item, "wheel_index"), "wheel_index")
        if index in by_index:
            raise RuntimeError("duplicate physical wheel index {}".format(index))
        by_index[index] = item
    if set(by_index) != set(EXPECTED_WHEEL_LOCATIONS):
        raise RuntimeError("physical wheel indices are not exactly 0,1,2,3")

    actor_transform = _transform_record(actor.get_transform(), "actor.transform")
    actor_velocity = _vector_record(actor.get_velocity(), "actor.velocity")
    angular_reader = getattr(actor, "get_angular_velocity", None)
    if not callable(angular_reader):
        raise RuntimeError("vehicle lacks angular velocity readback API")
    actor_angular_velocity = _vector_record(
        angular_reader(), "actor.angular_velocity"
    )
    timestamp_utc = datetime.fromtimestamp(
        wall_time_ns / 1_000_000_000.0, tz=timezone.utc
    ).isoformat(timespec="microseconds").replace("+00:00", "Z")

    sample: dict[str, Any] = {
        "record_type": "sample",
        "schema": SCHEMA,
        "sequence": _nonnegative_int(sequence, "sequence"),
        "timestamp_utc": timestamp_utc,
        "timestamp_unix_ns": _nonnegative_int(wall_time_ns, "wall_time_ns"),
        "elapsed_wall_seconds": _finite_float(elapsed_s, "elapsed_s"),
        "api_version": api_version,
        "actor": {
            "id": _actor_identity(actor)[0],
            "type_id": EXPECTED_TYPE_ID,
            "role_name": EXPECTED_ROLE_NAME,
            "transform": actor_transform,
            "yaw_degrees": actor_transform["rotation_degrees"]["yaw"],
            "linear_velocity_mps": actor_velocity,
            "angular_velocity_degrees_per_second": actor_angular_velocity,
        },
        "telemetry": {
            "body_transform": _transform_record(
                _required_attr(telemetry, "body_transform"),
                "telemetry.body_transform",
            ),
            "body_linear_velocity_mps": _vector_record(
                _required_attr(telemetry, "body_linear_velocity_mps"),
                "telemetry.body_linear_velocity_mps",
            ),
            "body_angular_velocity_rad_per_second": _vector_record(
                _required_attr(telemetry, "body_angular_velocity_rad_per_second"),
                "telemetry.body_angular_velocity_rad_per_second",
            ),
            "component_substep_control": _substep_record(
                _required_attr(telemetry, "component_substep_control"),
                "telemetry.component_substep_control",
            ),
            "live_substep_control": _substep_record(
                _required_attr(telemetry, "live_substep_control"),
                "telemetry.live_substep_control",
            ),
        },
        "wheels": [
            _wheel_record(by_index[index], steer_angles[index]) for index in range(4)
        ],
    }
    metadata = sample["telemetry"]
    for field in TELEMETRY_STRING_FIELDS:
        metadata[field] = str(_required_attr(telemetry, field))
    for field in TELEMETRY_BOOL_FIELDS:
        raw = _required_attr(telemetry, field)
        if not isinstance(raw, bool):
            raise RuntimeError("telemetry.{} is not boolean".format(field))
        metadata[field] = raw
    for field in TELEMETRY_INT_FIELDS:
        metadata[field] = _nonnegative_int(
            _required_attr(telemetry, field), "telemetry.{}".format(field)
        )
    for field in TELEMETRY_FLOAT_FIELDS:
        metadata[field] = _finite_float(
            _required_attr(telemetry, field), "telemetry.{}".format(field)
        )
    sample["world_frame"] = metadata["world_frame"]
    return sample


def _json_line(value: dict[str, Any]) -> str:
    return json.dumps(
        value, allow_nan=False, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ) + "\n"


def _open_exclusive(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    return os.fdopen(fd, "w", encoding="utf-8", newline="\n")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)


def _default_manifest_path(output: Path) -> Path:
    if output.suffix:
        return output.with_suffix(".manifest.json")
    return output.with_name(output.name + ".manifest.json")


def _never_stop() -> bool:
    return False


def validate_args(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.host not in EXPECTED_HOSTS or args.port != EXPECTED_PORT:
        raise ValueError("recorder endpoint is restricted to localhost:2000")
    if args.timeout_s <= 0.0 or args.rate_hz <= 0.0:
        raise ValueError("timeout and rate must be positive")
    if args.duration_s < 0.0 or args.max_samples < 0 or args.fsync_every_samples <= 0:
        raise ValueError("duration/max-samples must be non-negative; fsync interval positive")
    output = Path(args.output).expanduser().resolve()
    manifest = (
        Path(args.manifest).expanduser().resolve()
        if args.manifest
        else _default_manifest_path(output)
    )
    if output == manifest:
        raise ValueError("output and manifest paths must differ")
    for path in (output, manifest):
        if path.exists() or path.is_symlink():
            raise FileExistsError("refusing to overwrite existing output: {}".format(path))
    return output, manifest


def run(
    args: argparse.Namespace,
    carla_module: Any = None,
    should_stop: Callable[[], bool] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    wall_time_ns: Callable[[], int] = time.time_ns,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    output_path, manifest_path = validate_args(args)
    if carla_module is None:
        import carla as carla_module
    if should_stop is None:
        should_stop = _never_stop

    client = carla_module.Client(args.host, args.port)
    client.set_timeout(args.timeout_s)
    world = client.get_world()
    # A newly connected CARLA client initially exposes an unpopulated frame-0
    # actor cache until it receives its first world snapshot.  Waiting for the
    # already-running pacer/bridge tick is passive: it neither advances the
    # simulator nor changes world settings, actors, or vehicle control.
    wait_for_tick = getattr(world, "wait_for_tick", None)
    if not callable(wait_for_tick):
        raise RuntimeError("CARLA world is missing passive wait_for_tick synchronization")
    wait_for_tick(args.timeout_s)
    actor = select_unique_ranger(world)
    initial_identity = _actor_identity(actor)
    if initial_identity[0] <= 0:
        raise RuntimeError("Ranger actor id must be positive")
    for method in (
        "get_wheel_physics_telemetry",
        "get_wheel_steer_angle",
        "get_transform",
        "get_velocity",
        "get_angular_velocity",
    ):
        if not callable(getattr(actor, method, None)):
            raise RuntimeError("vehicle is missing required readback method {}".format(method))
    _wheel_enums(carla_module)

    data_stream = _open_exclusive(output_path)
    try:
        manifest_stream = _open_exclusive(manifest_path)
    except BaseException:
        data_stream.close()
        raise

    started_ns = wall_time_ns()
    started_mono = monotonic()
    sample_count = 0
    first_frame = None
    last_frame = None
    status = "COMPLETED"
    stop_reason = "limit_reached"
    error_record = None
    pending_error: BaseException | None = None
    period_s = 1.0 / args.rate_hz
    next_poll = started_mono

    header = {
        "record_type": "header",
        "schema": SCHEMA,
        "endpoint": "{}:{}".format(args.host, args.port),
        "actor": {
            "id": initial_identity[0],
            "type_id": initial_identity[1],
            "role_name": initial_identity[2],
        },
        "rate_hz": args.rate_hz,
        "read_only": True,
        "started_timestamp_unix_ns": started_ns,
    }
    try:
        data_stream.write(_json_line(header))
        data_stream.flush()
        while True:
            if should_stop():
                status = "STOPPED"
                stop_reason = "signal"
                break
            now = monotonic()
            if args.duration_s > 0.0 and now - started_mono >= args.duration_s:
                stop_reason = "duration"
                break
            if args.max_samples > 0 and sample_count >= args.max_samples:
                stop_reason = "max_samples"
                break
            if now < next_poll:
                sleeper(next_poll - now)
                continue

            current_actor = select_unique_ranger(world)
            if not bool(getattr(current_actor, "is_alive", False)):
                raise RuntimeError("bound Ranger actor is no longer alive")
            if _actor_identity(current_actor) != initial_identity:
                raise RuntimeError(
                    "Ranger actor identity drifted from {} to {}".format(
                        initial_identity, _actor_identity(current_actor)
                    )
                )
            sample = capture_sample(
                carla_module,
                current_actor,
                sample_count,
                now - started_mono,
                wall_time_ns(),
            )
            frame = sample["telemetry"]["world_frame"]
            if last_frame is not None and frame < last_frame:
                raise RuntimeError(
                    "telemetry world frame moved backwards: {} -> {}".format(
                        last_frame, frame
                    )
                )
            if first_frame is None:
                first_frame = frame
            last_frame = frame
            data_stream.write(_json_line(sample))
            sample_count += 1
            if sample_count % args.fsync_every_samples == 0:
                data_stream.flush()
                os.fsync(data_stream.fileno())
            next_poll += period_s
            after_sample = monotonic()
            if next_poll < after_sample:
                next_poll = after_sample + period_s
    except BaseException as exc:
        status = "ERROR"
        stop_reason = "exception"
        error_record = {"type": type(exc).__name__, "message": str(exc)}
        pending_error = exc
    finally:
        ended_ns = wall_time_ns()
        footer = {
            "record_type": "footer",
            "schema": SCHEMA,
            "status": status,
            "stop_reason": stop_reason,
            "sample_count": sample_count,
            "first_world_frame": first_frame,
            "last_world_frame": last_frame,
            "error": error_record,
            "ended_timestamp_unix_ns": ended_ns,
        }
        try:
            data_stream.write(_json_line(footer))
            data_stream.flush()
            os.fsync(data_stream.fileno())
        finally:
            data_stream.close()

        digest = _sha256_file(output_path)
        manifest = {
            "schema": MANIFEST_SCHEMA,
            "status": status,
            "stop_reason": stop_reason,
            "read_only": True,
            "endpoint": "{}:{}".format(args.host, args.port),
            "actor": {
                "id": initial_identity[0],
                "type_id": initial_identity[1],
                "role_name": initial_identity[2],
            },
            "rate_hz": args.rate_hz,
            "sample_count": sample_count,
            "first_world_frame": first_frame,
            "last_world_frame": last_frame,
            "started_timestamp_unix_ns": started_ns,
            "ended_timestamp_unix_ns": ended_ns,
            "output": {
                "path": str(output_path),
                "bytes": output_path.stat().st_size,
                "sha256": digest,
            },
            "error": error_record,
        }
        try:
            manifest_stream.write(
                json.dumps(
                    manifest,
                    allow_nan=False,
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                + "\n"
            )
            manifest_stream.flush()
            os.fsync(manifest_stream.fileno())
        finally:
            manifest_stream.close()

    if pending_error is not None:
        raise pending_error
    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="new JSONL output path")
    parser.add_argument(
        "--manifest",
        help="new manifest path (default: output stem + .manifest.json)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=EXPECTED_PORT, type=int)
    parser.add_argument("--timeout-s", default=3.0, type=float)
    parser.add_argument("--rate-hz", default=10.0, type=float)
    parser.add_argument("--duration-s", default=0.0, type=float, help="0 records until signalled")
    parser.add_argument("--max-samples", default=0, type=int, help="0 means unlimited")
    parser.add_argument("--fsync-every-samples", default=10, type=int)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        manifest = run(args, should_stop=stop_event.is_set)
    except Exception as exc:
        print(
            "[physical_wheel_telemetry] ERROR: {}: {}".format(
                type(exc).__name__, exc
            ),
            file=sys.stderr,
            flush=True,
        )
        return 1
    print(
        "[physical_wheel_telemetry] {} samples={} frames={}..{} sha256={}".format(
            manifest["status"],
            manifest["sample_count"],
            manifest["first_world_frame"],
            manifest["last_world_frame"],
            manifest["output"]["sha256"],
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
