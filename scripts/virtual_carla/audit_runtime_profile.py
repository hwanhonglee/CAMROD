#!/usr/bin/env python3
"""Fail-closed provenance audit for a live CAMROD/CARLA mission profile."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Sequence


SCHEMA = "camrod.virtual_carla.runtime_profile_audit.v1"
EXACT_RANGER_BLUEPRINT = "vehicle.ranger.default"
PROFILE_LAUNCHES = {
    "camrod_carla_full.launch.py": "develop-parity",
    "camrod_carla_develop_site_geometry.launch.py": (
        "develop-plus-carla-site-geometry-v26"
    ),
    "camrod_carla_woraksan_tuned.launch.py": "woraksan-tuned",
}
SITE_GEOMETRY_ALLOWED_CARLA_MAPS = (
    "map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13/"
    "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13",
    "map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12/"
    "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12",
    "map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11/"
    "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11",
)
DEVELOP_PARITY_PARAMETERS: dict[str, dict[str, Any]] = {
    "/camrod_twist_to_4ws": {
        "input_topic": "/control/cmd_vel_ros",
        "output_topic": "/carla/ego_vehicle/extended_ackermann_cmd",
        "minimum_turn_radius_m": 0.810330349,
        "max_crab_angle_rad": 1.53588974175501,
        "recovery_breakaway_enable": False,
        "rotation_recovery_breakaway_enable": False,
        "rotation_recovery_breakaway_status_timeout_sec": 0.30,
    },
    "/control/cmd_vel_safety_gate": {
        "speed_scale": 0.5,
        "navigation_minimum_ackermann_turn_radius_m": 0.0,
        "cost_stop_threshold": 85,
        "lanelet_safety_enable": True,
        "lanelet_safety_threshold": 85,
        "lanelet_safety_body_hard_stop_enable": True,
        "lanelet_safety_body_hard_stop_threshold": 100,
        "lanelet_safety_footprint_enable": True,
        "side_cost_threshold": 85,
        "rear_cost_threshold": 85,
        "lanelet_safety_check_reverse": False,
        "cost_stop_require_dynamic_source": True,
        "cost_stop_latch_use_trigger_source_for_merged_clear": False,
        "cost_stop_merged_dynamic_source_labels": "",
        "cost_stop_classified_source_labels": "fusion",
        "route_safety_path_relative_recovery_enable": False,
        "route_safety_path_center_reentry_m": 0.08,
    },
    "/planning/controller_server": {
        "RPP.desired_linear_vel": 1.111111,
        "RPP.min_approach_linear_velocity": 0.277778,
        "RPP.regulated_linear_scaling_min_speed": 0.333333,
        "RotationShim.desired_linear_vel": 1.111111,
        "RotationShim.min_approach_linear_velocity": 0.277778,
        "RotationShim.regulated_linear_scaling_min_speed": 0.333333,
    },
    "/control/camping_site_maneuver_controller": {
        "max_angular_speed_radps": 0.35,
        "roadside_reverse_return_enable": False,
        "roadside_reverse_handoff_distance_m": 0.03,
        "crab_approach_slowdown_distance_m": 0.0,
        "crab_approach_min_speed_mps": 0.0,
        "rotate_180_timeout_s": 0.0,
        "entry_position_tolerance_m": 0.15,
        "rotate_entry_max_position_error_m": 0.0,
        "rotate_entry_centering_max_initial_error_m": 0.30,
        "entry_anchor_centering_max_initial_error_m": 0.0,
        "entry_anchor_centering_max_speed_mps": 0.12,
        "entry_anchor_centering_timeout_s": 15.0,
        "entry_anchor_centering_tolerance_m": 0.05,
        "enable_live_lanelet_return_handoff": True,
        "return_lanelet_handoff_distance_m": 0.15,
        "return_lanelet_handoff_hold_s": 1.20,
        "return_lateral_transition_tolerance_m": 0.02,
        "return_lateral_hysteresis_m": 0.10,
        "crab_entry_max_heading_drift_deg": 0.0,
        "crab_entry_max_cross_track_error_m": 0.0,
        "crab_entry_body_yaw_compensation_deg": 0.0,
        "crab_entry_body_yaw_alignment_tolerance_deg": 0.5,
        "crab_entry_body_yaw_alignment_timeout_s": 15.0,
        "crab_out_yaw_recovery_enable": False,
        "crab_out_yaw_recovery_trigger_deg": 8.0,
        "crab_out_yaw_recovery_max_attempts": 3,
        "crab_out_yaw_recovery_global_timeout_s": 60.0,
    },
    "/control/route_safety_recovery_controller": {
        "zero_hold_pauses_limits": False,
        "allow_corrective_yaw_beyond_limit": False,
    },
    "/planning/goal_snapper": {
        "reverse_auxiliary_input_goal_topic": "",
        "pose_jump_check_topic": "",
        "reissue_active_goal_after_route_recovery_when_nav_active": False,
    },
    "/parking/apriltag_parking_controller": {
        "heading_gain": 1.5,
        "lateral_to_heading_gain": 2.5,
        # The latest develop node defaults are the effective values in the
        # AprilTag composition because the nested detector launch scopes its
        # generic `parameter_file` argument over the controller include.
        "reverse_approach_speed_mps": 0.2,
        "final_insertion_speed_mps": 0.05,
        "translation_stop_tag_distance_m": 0.40,
        "final_lateral_tolerance_m": 0.03,
        "minimum_approach_turn_radius_m": 0.85,
        "tag_wait_timeout_s": 60.0,
        "enable_bounded_lateral_retry": False,
        "retry_forward_distance_m": 1.0,
        "retry_forward_speed_mps": 0.10,
        "retry_forward_timeout_s": 25.0,
        "retry_yaw_alignment_timeout_s": 8.0,
        "retry_maximum_lateral_error_m": 0.15,
        "retry_maximum_heading_error_rad": 0.35,
        "retry_maximum_forward_exit_lateral_drift_m": 0.15,
        "retry_maximum_odometry_step_m": 0.10,
        "retry_minimum_tag_distance_m": 0.35,
        "retry_maximum_tag_distance_m": 0.45,
        "maximum_retries": 5,
    },
    # Plain run.sh camrod keeps the latest develop controller parameters and
    # opts in only to the CARLA rear-camera and charger/BMS plant boundary.
    # Requiring the nodes and their exact effective parameters makes a missing
    # launch argument, stale install, or production detector config fail closed.
    "/perception/apriltag_parking_detector": {
        "image_topic": "/sensing/camera/econ_rear/image_rect",
        "camera_info_topic": "/sensing/camera/econ_rear/camera_info",
        "camera_frame_id": "camera_rear",
        "tag_family": "tag36h11",
        "target_tag_id": 3,
        "tag_size": 0.16,
        "quad_decimate": 1.0,
        "n_threads": 2,
        "roi_scale": 3.0,
        "roi_full_search_interval": 30,
        "max_reproj_error_px": 2.0,
        "publish_tf": True,
        "publish_debug_image": True,
        "debug_jpeg_quality": 80,
    },
    "/carla_charging_contact_emulator": {
        "drop_zone_id": "drop_zone",
        "pose_topic": "/localization/pose",
        "odometry_topic": "/odom",
        "parking_status_topic": (
            "/parking/apriltag_parking_controller/status"
        ),
        "planning_state_topic": "/planning/state_machine/state",
        "charging_topic": "/camrod_carla/platform_heartbeat/charging",
        "position_tolerance_m": 0.35,
        "speed_tolerance_mps": 0.05,
        "pose_timeout_s": 0.5,
        "odometry_timeout_s": 0.5,
        "state_timeout_s": 2.0,
        "dwell_s": 1.0,
        "publish_rate_hz": 10.0,
    },
    "/planning/nav2_selector_latch": {
        "controller_id": "RPP",
        "reverse_controller_id": "RPP",
    },
    "/localization/pose_selector": {
        "primary_pose_cov_topic": "/localization/primary/pose_with_covariance",
        "fallback_pose_cov_topic": "",
        "primary_odom_topic": "/localization/primary/odometry",
        "fallback_odom_topic": "",
    },
    "/ui_backend": {
        "enable_operator_telemetry": True,
        "telemetry_raw_lidar_bbox_overlay_enabled": False,
        "telemetry_obstacle_cloud_topic": "/perception/obstacles",
        "telemetry_front_camera_topic": "/sensing/camera/econ_front/image_rect/compressed",
        "telemetry_rear_camera_topic": "/sensing/camera/econ_rear/image_raw/compressed",
        "charging_departure_delay_s": 7.0,
    },
    "/sensing/lidar/lidar_cost_grid": {
        "raw_lidar_cost_enabled": False,
        "input_topics": ["/perception/obstacles"],
        "max_cost": 95,
    },
    "/perception/yolov9mit": {
        "min_confidence": 0.95,
    },
}


def _with_site_geometry(
    base: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Apply the proven site geometry and scoped CARLA recovery lease."""
    result = {node: dict(parameters) for node, parameters in base.items()}
    result["/control/cmd_vel_safety_gate"]["speed_scale"] = 1.0
    result["/planning/controller_server"].update({
        "RPP.desired_linear_vel": 0.555556,
        "RPP.min_approach_linear_velocity": 0.138889,
        "RPP.regulated_linear_scaling_min_speed": 0.166667,
        "RotationShim.desired_linear_vel": 0.555556,
        "RotationShim.min_approach_linear_velocity": 0.138889,
        "RotationShim.regulated_linear_scaling_min_speed": 0.166667,
    })
    result["/ui_backend"]["return_site_exit_rearm_enabled"] = True
    result["/camrod_twist_to_4ws"]["recovery_breakaway_enable"] = True
    result["/camrod_twist_to_4ws"][
        "rotation_recovery_breakaway_enable"
    ] = True
    result["/camrod_twist_to_4ws"][
        "rotation_recovery_breakaway_status_timeout_sec"
    ] = 1.25
    result["/control/cmd_vel_safety_gate"][
        "route_safety_path_relative_recovery_enable"
    ] = True
    result["/control/cmd_vel_safety_gate"][
        "route_safety_path_center_reentry_m"
    ] = 0.15
    result["/control/cmd_vel_safety_gate"][
        "lanelet_safety_footprint_enable"
    ] = False
    result["/control/cmd_vel_safety_gate"][
        "cost_stop_latch_use_trigger_source_for_merged_clear"
    ] = True
    result["/control/cmd_vel_safety_gate"][
        "cost_stop_merged_dynamic_source_labels"
    ] = "radar"
    result["/control/route_safety_recovery_controller"].update({
        "zero_hold_pauses_limits": True,
        "allow_corrective_yaw_beyond_limit": True,
    })
    result["/planning/goal_snapper"][
        "reissue_active_goal_after_route_recovery_when_nav_active"
    ] = True
    result["/planning/goal_snapper"][
        "pose_jump_check_topic"
    ] = "/localization/pose"
    result["/parking/apriltag_parking_controller"].update({
        "heading_gain": 1.5,
        "lateral_to_heading_gain": 2.7,
        "reverse_approach_speed_mps": 0.2,
        "final_insertion_speed_mps": 0.05,
        "enable_bounded_lateral_retry": True,
        "retry_forward_distance_m": 0.8,
        "retry_forward_speed_mps": 0.20,
        "retry_forward_timeout_s": 30.0,
        "retry_yaw_alignment_timeout_s": 8.0,
        "retry_maximum_lateral_error_m": 0.15,
        "retry_maximum_heading_error_rad": 0.35,
        "retry_maximum_forward_exit_lateral_drift_m": 0.15,
        "retry_maximum_odometry_step_m": 0.10,
        "retry_minimum_tag_distance_m": 0.35,
        "retry_maximum_tag_distance_m": 0.45,
        "maximum_retries": 2,
    })
    result["/control/camping_site_maneuver_controller"].update({
        "max_angular_speed_radps": 0.45,
        "crab_approach_slowdown_distance_m": 1.0,
        "crab_approach_min_speed_mps": 0.12,
        "rotate_180_timeout_s": 90.0,
        "entry_position_tolerance_m": 0.05,
        "rotate_entry_max_position_error_m": 0.05,
        "rotate_entry_centering_max_initial_error_m": 0.65,
        "entry_anchor_centering_max_initial_error_m": 0.0,
        "entry_anchor_centering_max_speed_mps": 0.12,
        "entry_anchor_centering_timeout_s": 15.0,
        "entry_anchor_centering_tolerance_m": 0.05,
        "crab_entry_max_heading_drift_deg": 0.0,
        "crab_entry_body_yaw_compensation_deg": 2.0,
        "crab_entry_body_yaw_alignment_tolerance_deg": 1.5,
        "crab_entry_body_yaw_alignment_timeout_s": 15.0,
        "crab_out_yaw_recovery_enable": True,
        "crab_out_yaw_recovery_trigger_deg": 8.0,
        "crab_out_yaw_recovery_max_attempts": 8,
        "crab_out_yaw_recovery_global_timeout_s": 90.0,
    })
    return result


DEVELOP_SITE_GEOMETRY_PARAMETERS = _with_site_geometry(
    DEVELOP_PARITY_PARAMETERS
)
AUDITED_PROFILE_PARAMETERS = {
    "develop-parity": DEVELOP_PARITY_PARAMETERS,
    "develop-plus-carla-site-geometry-v26": DEVELOP_SITE_GEOMETRY_PARAMETERS,
}


class AuditError(RuntimeError):
    """A live identity or parameter did not satisfy the requested profile."""


def utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z")
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise AuditError(f"cannot hash {path}: {error}") from None
    return digest.hexdigest()


def normalize_carla_map(value: str) -> str:
    """Normalize CARLA's `/Game/...` and Python API map-name spellings."""
    result = str(value).strip()
    if result.startswith("/Game/"):
        result = result[len("/Game/") :]
    return result.strip("/")


def validate_audited_map(profile: str, normalized_map: str) -> None:
    """Restrict the CARLA-only site profile to exact built map identities."""
    if (
        profile == "develop-plus-carla-site-geometry-v26"
        and normalized_map not in SITE_GEOMETRY_ALLOWED_CARLA_MAPS
    ):
        raise AuditError(
            "site-geometry profile map is not in the v13/v12/v11 allowlist: "
            f"{normalized_map}"
        )


def _read_cmdline(path: Path) -> tuple[bytes, list[str]]:
    raw = path.read_bytes()
    argv = [part.decode("utf-8", "surrogateescape") for part in raw.split(b"\0") if part]
    return raw, argv


def discover_camrod_launch(proc_root: Path = Path("/proc")) -> dict[str, Any]:
    """Require exactly one live CARLA CAMROD launch and classify its profile."""
    candidates: list[dict[str, Any]] = []
    try:
        entries = tuple(proc_root.iterdir())
    except OSError as error:
        raise AuditError(f"cannot enumerate process table {proc_root}: {error}") from None
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            raw, argv = _read_cmdline(entry / "cmdline")
        except (OSError, UnicodeError):
            continue
        launch_names = [Path(token).name for token in argv if Path(token).name in PROFILE_LAUNCHES]
        if len(launch_names) != 1:
            continue
        if "launch" not in argv or "camrod_carla_adapter" not in argv:
            continue
        lanelet_tokens = [
            token.split(":=", 1)[1]
            for token in argv
            if token.startswith("camrod_map_path:=")
        ]
        if len(lanelet_tokens) != 1:
            raise AuditError(
                f"CAMROD launch pid {entry.name} must contain exactly one camrod_map_path"
            )
        launch_name = launch_names[0]
        candidates.append(
            {
                "pid": int(entry.name),
                "launch_file": launch_name,
                "profile": PROFILE_LAUNCHES[launch_name],
                "lanelet_map_argument": str(Path(lanelet_tokens[0]).expanduser().resolve()),
                "cmdline_sha256": hashlib.sha256(raw).hexdigest(),
                "argv": argv,
            }
        )
    if len(candidates) != 1:
        identities = [
            f"pid={item['pid']}:{item['profile']}" for item in candidates
        ]
        raise AuditError(
            "expected exactly one live CAMROD CARLA launch, found "
            f"{len(candidates)} ({', '.join(identities)})"
        )
    return candidates[0]


def resolve_audited_profile(
    requested_profile: str, live_profile: str
) -> str:
    """Bind auto detection to one known launch and an exact parameter set."""
    if requested_profile != "auto" and live_profile != requested_profile:
        raise AuditError(
            f"live profile is {live_profile!r}, expected {requested_profile!r}"
        )
    if live_profile not in AUDITED_PROFILE_PARAMETERS:
        raise AuditError(
            f"live profile {live_profile!r} has no motion-authorized audit "
            "contract"
        )
    return live_profile


def dump_ros_parameters(
    node: str,
    *,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> dict[str, Any]:
    """Read one node's actual parameter service through the ROS CLI."""
    try:
        completed = runner(
            ["ros2", "param", "dump", node],
            check=False,
            capture_output=True,
            text=True,
            timeout=15.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise AuditError(f"cannot dump ROS parameters for {node}: {error}") from None
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise AuditError(
            f"ROS parameter dump failed for {node}: status={completed.returncode}: {detail}"
        )
    try:
        import yaml

        document = yaml.safe_load(completed.stdout)
    except (ImportError, ValueError) as error:
        raise AuditError(f"cannot parse ROS parameter dump for {node}: {error}") from None
    if not isinstance(document, Mapping):
        raise AuditError(f"ROS parameter dump root is not a mapping for {node}")
    node_document = document.get(node)
    if not isinstance(node_document, Mapping):
        raise AuditError(f"ROS parameter dump is missing exact node {node}")
    parameters = node_document.get("ros__parameters")
    if not isinstance(parameters, Mapping):
        raise AuditError(f"ROS parameter dump is missing ros__parameters for {node}")
    # ``ros2 param dump`` serializes declared parameters containing dots as
    # nested YAML mappings (for example ``RPP.desired_linear_vel`` becomes
    # ``RPP: {desired_linear_vel: ...}``).  The ROS parameter service and our
    # runtime contract use the declared dotted name, so normalize only mapping
    # nodes back to that exact representation. Lists remain parameter values.
    flattened: dict[str, Any] = {}

    def flatten(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping):
            for child_name, child_value in value.items():
                child_prefix = f"{prefix}.{child_name}" if prefix else str(child_name)
                flatten(child_prefix, child_value)
            return
        flattened[prefix] = value

    for parameter_name, parameter_value in parameters.items():
        flatten(str(parameter_name), parameter_value)
    return flattened


def select_and_validate_parameters(
    actual_by_node: Mapping[str, Mapping[str, Any]],
    expected_by_node: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Return the selected live values only when every exact value matches."""
    selected: dict[str, dict[str, Any]] = {}
    mismatches: list[str] = []
    for node, expected_parameters in expected_by_node.items():
        actual_parameters = actual_by_node.get(node)
        if not isinstance(actual_parameters, Mapping):
            mismatches.append(f"missing node parameter dump: {node}")
            continue
        selected[node] = {}
        for name, expected in expected_parameters.items():
            if name not in actual_parameters:
                mismatches.append(f"{node}.{name}=MISSING expected {expected!r}")
                continue
            actual = actual_parameters[name]
            selected[node][name] = actual
            # Python considers True == 1 and False == 0.  ROS parameter
            # identity must remain type-exact so a malformed YAML overlay can
            # never satisfy a boolean/int contract by accident.
            if type(actual) is not type(expected) or actual != expected:
                mismatches.append(
                    f"{node}.{name}={actual!r} ({type(actual).__name__}) "
                    f"expected {expected!r} ({type(expected).__name__})"
                )
    if mismatches:
        raise AuditError("runtime parameter profile mismatch: " + "; ".join(mismatches))
    return selected


def _module_is_inside_egg(module_path: str, python_egg: Path) -> bool:
    module_real = os.path.realpath(module_path)
    egg_real = os.path.realpath(str(python_egg))
    return module_real.startswith(egg_real + os.sep)


def read_live_carla_world(
    *,
    host: str,
    port: int,
    timeout_s: float,
    python_egg: Path,
    role_name: str,
    expected_actor_id: int,
    expected_fixed_delta_seconds: float,
    expected_no_rendering_mode: bool,
) -> dict[str, Any]:
    """Passively hydrate and bind the live CARLA world and exact Ranger.

    A newly connected client can initially expose snapshot frame 0 and an
    empty cached actor inventory even though the synchronous bridge is already
    ticking.  ``wait_for_tick`` is passive; unlike ``tick`` it never advances
    or mutates the simulation.
    """
    if not python_egg.is_file():
        raise AuditError(f"CARLA Python egg is missing: {python_egg}")
    sys.path.insert(0, str(python_egg))
    try:
        import carla
    except Exception as error:
        raise AuditError(f"cannot import CARLA Python API from {python_egg}: {error}") from None
    module_origin = str(getattr(carla, "__file__", "") or "")
    if not module_origin or not _module_is_inside_egg(module_origin, python_egg):
        raise AuditError(
            "CARLA Python API resolved outside the requested gate-bound egg: "
            f"{module_origin or '<unknown>'} expected under {python_egg}"
        )

    deadline = time.monotonic() + timeout_s
    last_detail = "no CARLA world sample received"
    try:
        client = carla.Client(host, port)
        client.set_timeout(min(2.0, timeout_s))
        world = client.get_world()
    except Exception as error:
        raise AuditError(f"cannot read live CARLA world at {host}:{port}: {error}") from None

    while time.monotonic() < deadline:
        remaining = max(0.001, deadline - time.monotonic())
        try:
            client.set_timeout(min(2.0, remaining))
            snapshot = world.wait_for_tick(min(2.0, remaining))
            frame = int(getattr(snapshot, "frame", 0))
            actors = list(world.get_actors())
            settings = world.get_settings()
            map_name = str(world.get_map().name)
        except Exception as error:
            last_detail = f"{type(error).__name__}: {error}"
            continue

        identities: list[dict[str, Any]] = []
        for actor in actors:
            attributes = getattr(actor, "attributes", {}) or {}
            identities.append(
                {
                    "actor_id": int(getattr(actor, "id", 0) or 0),
                    "type_id": str(getattr(actor, "type_id", "")),
                    "role_name": str(attributes.get("role_name", "")),
                }
            )
        if frame <= 0 or not any(
            item["actor_id"] > 0 and item["type_id"] for item in identities
        ):
            last_detail = (
                f"unsynchronized snapshot frame={frame}, actor_count={len(actors)}"
            )
            continue

        role_actors = [item for item in identities if item["role_name"] == role_name]
        if len(role_actors) != 1:
            last_detail = (
                f"frame={frame} has {len(role_actors)} actors for "
                f"role_name={role_name!r} (actor_count={len(actors)})"
            )
            continue
        ego = role_actors[0]
        if ego["type_id"] != EXACT_RANGER_BLUEPRINT:
            raise AuditError(
                f"live ego type is {ego['type_id']!r}, expected "
                f"{EXACT_RANGER_BLUEPRINT!r}"
            )
        if ego["actor_id"] != expected_actor_id:
            raise AuditError(
                f"live Ranger actor_id={ego['actor_id']} differs from the "
                f"preflight/physical bridge actor_id={expected_actor_id}"
            )

        synchronous_mode = bool(getattr(settings, "synchronous_mode", False))
        no_rendering_mode = bool(getattr(settings, "no_rendering_mode", False))
        fixed_delta = float(getattr(settings, "fixed_delta_seconds", 0.0) or 0.0)
        if synchronous_mode is not True:
            raise AuditError("live CARLA world is not synchronous")
        if not math.isclose(
            fixed_delta,
            expected_fixed_delta_seconds,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise AuditError(
                f"live CARLA fixed_delta_seconds={fixed_delta!r}, expected "
                f"{expected_fixed_delta_seconds!r}"
            )
        if no_rendering_mode is not expected_no_rendering_mode:
            raise AuditError(
                f"live CARLA no_rendering_mode={no_rendering_mode!r}, expected "
                f"{expected_no_rendering_mode!r}"
            )

        child_sensor_ids: list[int] = []
        for actor in actors:
            if not str(getattr(actor, "type_id", "")).startswith("sensor."):
                continue
            parent = getattr(actor, "parent", None)
            if int(getattr(parent, "id", 0) or 0) == expected_actor_id:
                child_sensor_ids.append(int(actor.id))
        return {
            "endpoint": f"{host}:{port}",
            "map_name": map_name,
            "normalized_map_name": normalize_carla_map(map_name),
            "snapshot_frame": frame,
            "actor_inventory_count": len(actors),
            "ego_actor": ego,
            "child_sensor_actor_ids": sorted(child_sensor_ids),
            "world_settings": {
                "synchronous_mode": synchronous_mode,
                "fixed_delta_seconds": fixed_delta,
                "no_rendering_mode": no_rendering_mode,
            },
            "python_api_origin": str(Path(module_origin).resolve()),
            "python_egg": str(python_egg.resolve()),
            "python_egg_sha256": sha256_file(python_egg),
        }

    raise AuditError(
        "live CARLA actor inventory did not synchronize within "
        f"{timeout_s:g}s at {host}:{port}; last sample: {last_detail}"
    )


def write_json_create_only(path: Path, document: Mapping[str, Any]) -> None:
    path = path.expanduser().resolve()
    if not path.is_absolute():
        raise AuditError("output path must be absolute")
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags, 0o644)
    except FileExistsError:
        raise AuditError(f"refusing to overwrite runtime profile report: {path}") from None
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise


def read_physical_bridge_actor_id(
    script: Path,
    *,
    role_name: str,
    timeout_s: float,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> int:
    """Require one fresh ready physical-4WS status and return its actor id."""
    resolved = script.expanduser().resolve()
    if not resolved.is_file():
        raise AuditError(f"physical bridge status checker is missing: {resolved}")
    try:
        completed = runner(
            [
                sys.executable,
                str(resolved),
                "--role-name",
                role_name,
                "--timeout-seconds",
                str(min(10.0, timeout_s)),
                "--actor-id-only",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=min(15.0, timeout_s + 3.0),
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise AuditError(f"cannot read physical 4WS bridge status: {error}") from None
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise AuditError(
            "physical 4WS bridge status check failed: "
            f"status={completed.returncode}: {detail}"
        )
    try:
        actor_id = int(completed.stdout.strip())
    except ValueError:
        raise AuditError(
            f"physical 4WS bridge returned malformed actor id: {completed.stdout!r}"
        ) from None
    if actor_id <= 0:
        raise AuditError(f"physical 4WS bridge returned invalid actor id: {actor_id}")
    return actor_id


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--expected-profile",
        choices=("auto", *AUDITED_PROFILE_PARAMETERS),
        required=True,
        help=(
            "required profile, or auto to bind the exact live launch identity; "
            "unsupported/tuned launches remain rejected"
        ),
    )
    result.add_argument("--expected-carla-map", required=True)
    result.add_argument("--expected-carla-town", required=True)
    result.add_argument("--expected-lanelet-map", type=Path, required=True)
    result.add_argument("--expected-umap", type=Path, required=True)
    result.add_argument("--expected-actor-id", type=int, required=True)
    result.add_argument("--role-name", required=True)
    result.add_argument(
        "--expected-fixed-delta-seconds", type=float, default=0.05
    )
    result.add_argument(
        "--expected-no-rendering-mode",
        choices=("true", "false"),
        default="false",
    )
    result.add_argument(
        "--physical-status-check-script", type=Path, required=True
    )
    result.add_argument("--carla-python-egg", type=Path, required=True)
    result.add_argument("--host", default="127.0.0.1")
    result.add_argument("--port", type=int, default=2000)
    result.add_argument("--timeout", type=float, default=10.0)
    result.add_argument("--output", type=Path, required=True)
    return result


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    if not args.host.strip() or not (1 <= args.port <= 65535):
        raise AuditError("CARLA endpoint is invalid")
    if not args.role_name.strip():
        raise AuditError("CARLA role name is empty")
    if args.expected_actor_id <= 0:
        raise AuditError("expected Ranger actor id must be positive")
    if not math.isfinite(args.timeout) or not (0.1 <= args.timeout <= 60.0):
        raise AuditError("timeout must be finite and in [0.1, 60.0]")
    if not math.isfinite(args.expected_fixed_delta_seconds) or not (
        0.001 <= args.expected_fixed_delta_seconds <= 1.0
    ):
        raise AuditError("expected fixed delta must be finite and in [0.001, 1.0]")
    expected_lanelet = args.expected_lanelet_map.expanduser().resolve()
    if not expected_lanelet.is_file():
        raise AuditError(f"expected lanelet map is missing: {expected_lanelet}")
    expected_umap = args.expected_umap.expanduser().resolve()
    if not expected_umap.is_file():
        raise AuditError(f"expected CARLA UE map asset is missing: {expected_umap}")
    expected_carla = normalize_carla_map(args.expected_carla_map)
    expected_town = normalize_carla_map(args.expected_carla_town)
    if not expected_carla or expected_carla != expected_town:
        raise AuditError(
            "CARLA_UE_MAP and CARLA_TOWN do not identify the same normalized map: "
            f"{expected_carla!r} != {expected_town!r}"
        )

    launch = discover_camrod_launch()
    audited_profile = resolve_audited_profile(
        args.expected_profile, launch["profile"]
    )
    validate_audited_map(audited_profile, expected_carla)
    if Path(launch["lanelet_map_argument"]) != expected_lanelet:
        raise AuditError(
            "live launch lanelet map differs from the requested map: "
            f"{launch['lanelet_map_argument']} != {expected_lanelet}"
        )

    expected_parameters = {
        node: dict(parameters)
        for node, parameters in AUDITED_PROFILE_PARAMETERS[
            audited_profile
        ].items()
    }
    lanelet_parameter_nodes = (
        "/map/lanelet_map_provider",
        "/map/lanelet_boundary_cost_grid",
        "/map/lanelet_safety_cost_grid",
    )
    for node in lanelet_parameter_nodes:
        expected_parameters[node] = {"map_path": str(expected_lanelet)}
    expected_parameters["/planning/goal_snapper"]["map_path"] = str(
        expected_lanelet
    )
    expected_parameters["/carla_ros_bridge"] = {
        "host": args.host,
        "port": args.port,
        "town": args.expected_carla_town,
        "ego_vehicle_role_name": args.role_name,
        "passive": False,
        "synchronous_mode": True,
        "synchronous_mode_wait_for_vehicle_control_command": False,
        "fixed_delta_seconds": args.expected_fixed_delta_seconds,
        "register_all_sensors": True,
        "use_sim_time": True,
    }
    expected_parameters["/physical_four_wheel_bridge"] = {
        "host": args.host,
        "port": args.port,
        "role_name": args.role_name,
        "expected_blueprint_id": EXACT_RANGER_BLUEPRINT,
        "extended_mode_backend": "PHYSX_FOUR_WHEEL_STEERING",
        "use_sim_time": True,
    }
    actual_parameters = {
        node: dump_ros_parameters(node) for node in expected_parameters
    }
    selected_parameters = select_and_validate_parameters(
        actual_parameters, expected_parameters
    )
    physical_actor_id = read_physical_bridge_actor_id(
        args.physical_status_check_script,
        role_name=args.role_name,
        timeout_s=args.timeout,
    )
    if physical_actor_id != args.expected_actor_id:
        raise AuditError(
            f"physical 4WS bridge actor_id={physical_actor_id} differs from "
            f"preflight actor_id={args.expected_actor_id}"
        )

    carla_world = read_live_carla_world(
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        python_egg=args.carla_python_egg.expanduser().resolve(),
        role_name=args.role_name,
        expected_actor_id=args.expected_actor_id,
        expected_fixed_delta_seconds=args.expected_fixed_delta_seconds,
        expected_no_rendering_mode=(
            args.expected_no_rendering_mode == "true"
        ),
    )
    if carla_world["normalized_map_name"] != expected_carla:
        raise AuditError(
            "live CARLA map differs from the requested map: "
            f"{carla_world['normalized_map_name']!r} != {expected_carla!r}"
        )

    return {
        "accepted": True,
        "profile": audited_profile,
        "launch": launch,
        "carla": {
            **carla_world,
            "expected_ue_map": args.expected_carla_map,
            "expected_town": args.expected_carla_town,
            "ue_map_asset": {
                "path": str(expected_umap),
                "sha256": sha256_file(expected_umap),
            },
        },
        "physical_four_wheel_bridge": {
            "status": "READY",
            "actor_id": physical_actor_id,
            "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
            "actor_binding_match": True,
        },
        "lanelet_map": {
            "path": str(expected_lanelet),
            "sha256": sha256_file(expected_lanelet),
            "launch_argument_match": True,
            "parameter_nodes": [
                *lanelet_parameter_nodes,
                "/planning/goal_snapper",
            ],
            "all_parameter_paths_match": True,
        },
        "selected_live_parameters": selected_parameters,
        "profile_contract": {
            "requested_profile": args.expected_profile,
            "launch_identity_auto_detected": args.expected_profile == "auto",
            "audited_profiles": list(AUDITED_PROFILE_PARAMETERS),
            "site_geometry_allowed_carla_maps": list(
                SITE_GEOMETRY_ALLOWED_CARLA_MAPS
            ),
            "fake_sensor_ownership": "bound separately by the 36-stream/13-actor sensor-source audit",
            "raw_lidar_cost_enabled": False,
            "semantic_lidar_input": "/perception/obstacles",
            "ordinary_camrod_defaults_modified": False,
            "carla_site_geometry_overlay": (
                "v20-current-pose-return-wide-b2-access-apriltag-two-retries-"
                "charging-site-yaw-tolerance-bounded-rotation-torque-"
                "saturation-crab-out-yaw-recovery"
                if audited_profile ==
                "develop-plus-carla-site-geometry-v26" else "disabled"
            ),
            "carla_recovery_breakaway_lease": (
                "bounded_controller_authenticated"
                if audited_profile ==
                "develop-plus-carla-site-geometry-v26"
                else "disabled"
            ),
            "woraksan_tuned_profile_allowed": False,
        },
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "FAIL",
        "accepted": False,
        "started_at_utc": utc_now(),
        "errors": [],
    }
    exit_status = 0
    try:
        report.update(run_audit(args))
        report["status"] = "PASS"
    except AuditError as error:
        report["errors"].append(str(error))
        exit_status = 1
    except Exception as error:
        report["errors"].append(f"unexpected {type(error).__name__}: {error}")
        exit_status = 1
    report["completed_at_utc"] = utc_now()
    try:
        write_json_create_only(args.output, report)
    except AuditError as error:
        print(f"RUNTIME_PROFILE_AUDIT ERROR: {error}", file=sys.stderr)
        return 1
    if exit_status:
        print(
            "RUNTIME_PROFILE_AUDIT FAIL: " + "; ".join(report["errors"]),
            file=sys.stderr,
        )
    else:
        print(
            "RUNTIME_PROFILE_AUDIT PASS "
            f"profile={report['profile']} map={report['carla']['normalized_map_name']} "
            f"lanelet_sha256={report['lanelet_map']['sha256']}",
            flush=True,
        )
    return exit_status


if __name__ == "__main__":
    raise SystemExit(main())
