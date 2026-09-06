#!/usr/bin/env python3
# flake8: noqa
"""Run (or plan) a CAMROD camping-site round-trip matrix in CARLA.

The default action is deliberately read-only: it loads the active CAMROD map
configuration and writes a plan.  ``--run`` is required before this program
can call the selected production UI authority (operator REST, the visible
Robot UI browser, or the visible Guest browser page).  The runner never
publishes a motion command, a goal
pose, an initial pose, or a CARLA teleport.  Movement is therefore the same
UI -> CAMROD -> controller path a real frontend uses.

ROS imports are intentionally deferred until the live path is selected.  The
configuration, state-machine, and report helpers can consequently be tested
on a plain Python installation.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import hashlib
import sys
import tempfile
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen


DEFAULT_SITES = tuple(f"B{i}" for i in range(1, 14))
RECALL_TO_SITE_ROAD = 7
GUEST_LOADING_WAIT = 8
RETURN_WITH_CARGO = 9
WAITING_FOR_RETURN_REQUEST = 11
WAITING_FOR_CHARGING = 12
CHARGING = 13
DROP_ZONE_PARKING = 10
FAILURE_STATES = {16}
RETURN_OPERATION = 3
UI_RETURN_TOKEN_RE = re.compile(r"^g[1-9][0-9]*-s[1-9][0-9]*-[0-9a-f]+$")
GUEST_RETURN_SOURCE_RE = re.compile(
    r"^guest:usage_complete:site=(B(?:[1-9]|1[0-3])):g=([1-9][0-9]*)$"
)
GUEST_UI_TITLE = "국립공원 로봇 서비스"
OPERATOR_UI_TITLE = "Robot UI"
COLLISION_SAMPLE_LIMIT = 256
COLLISION_FIRST_SAMPLE_LIMIT = 64
FINAL_OBSERVATION_DRAIN_SECONDS = 0.25
SITE_TYPE_RE = re.compile(r"^camping_site_(\d+)$", re.IGNORECASE)
STATIONARY_DROP_ZONE_STATES = {0, WAITING_FOR_CHARGING, CHARGING}
SITE_PHASES = {
    "turnaround": (
        "CRAB_IN", "ROTATE_180", "UNLOAD_WAIT", "WAIT_RETURN",
        "ALIGN_RETRACE_YAW", "CRAB_OUT", "DONE",
    ),
    "roadside_stop": (
        "CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN", "CRAB_OUT", "DONE",
    ),
}
MISSION_INTENTS = ("delivery", "recall")
DEVELOP_PARITY_RUNTIME_SIGNATURE: dict[str, dict[str, Any]] = {
    "/camrod_twist_to_4ws": {
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
        "lanelet_safety_check_reverse": False,
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
        # Match the effective latest-develop node defaults selected by the
        # nested AprilTag detector/controller launch composition.
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
    "/planning/nav2_selector_latch": {
        "reverse_controller_id": "RPP",
    },
    "/ui_backend": {
        "telemetry_raw_lidar_bbox_overlay_enabled": False,
        "telemetry_docking_rear_camera_fallback_enabled": False,
        "telemetry_obstacle_cloud_topic": "/perception/obstacles",
        "charging_departure_delay_s": 7.0,
    },
    "/sensing/lidar/lidar_cost_grid": {
        "raw_lidar_cost_enabled": False,
        "input_topics": ["/perception/obstacles"],
        "max_cost": 95,
    },
}


def _with_site_geometry_runtime_signature(
    base: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    result = {node: dict(parameters) for node, parameters in base.items()}
    result["/carla_charging_contact_emulator"] = {
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
    }
    result["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] = True
    result["/control/cmd_vel_safety_gate"]["speed_scale"] = 1.0
    result["/planning/controller_server"].update({
        "controller_plugins": ["RPP", "RPPReverse", "RotationShim"],
        "RPP.desired_linear_vel": 0.555556,
        "RPP.min_approach_linear_velocity": 0.138889,
        "RPP.regulated_linear_scaling_min_speed": 0.166667,
        "RotationShim.desired_linear_vel": 0.555556,
        "RotationShim.min_approach_linear_velocity": 0.138889,
        "RotationShim.regulated_linear_scaling_min_speed": 0.166667,
        "RPPReverse.allow_reversing": True,
        "RPPReverse.use_rotate_to_heading": False,
        "RPPReverse.desired_linear_vel": 0.20,
        "RPPReverse.lookahead_dist": 0.80,
        "RPPReverse.use_collision_detection": True,
    })
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
        "lanelet_safety_check_reverse"
    ] = True
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
    result["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] = "/planning/auto_reverse_goal_raw"
    result["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] = "RPPReverse"
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
        "roadside_reverse_return_enable": True,
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


DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE = (
    _with_site_geometry_runtime_signature(DEVELOP_PARITY_RUNTIME_SIGNATURE)
)
RUNTIME_PROFILE_SIGNATURES = {
    "develop-parity": DEVELOP_PARITY_RUNTIME_SIGNATURE,
    "develop-plus-carla-site-geometry-v27": (
        DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE
    ),
}


def _load_authoritative_runtime_profile_signatures() -> dict[str, dict[str, Any]]:
    """Load the exact profile contract used to produce the accepted audit.

    The matrix consumes an audit emitted by ``audit_runtime_profile.py``.  It
    must therefore validate every selected parameter from that same source,
    rather than maintaining a weaker hand-copied subset that can drift.
    """
    contract_path = Path(__file__).with_name("audit_runtime_profile.py")
    spec = importlib.util.spec_from_file_location(
        "_camrod_virtual_carla_runtime_profile_contract", contract_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(
            f"cannot load runtime-profile contract: {contract_path}"
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profiles = getattr(module, "AUDITED_PROFILE_PARAMETERS", None)
    required = {
        "develop-parity",
        "develop-plus-carla-site-geometry-v27",
    }
    if not isinstance(profiles, Mapping) or set(profiles) != required:
        raise RuntimeError(
            "runtime-profile contract has unexpected profile keys: "
            f"{sorted(profiles) if isinstance(profiles, Mapping) else profiles!r}"
        )
    return {
        str(profile): {
            str(node): dict(parameters)
            for node, parameters in nodes.items()
        }
        for profile, nodes in profiles.items()
    }


# These public names are retained for offline contract tests, but all three now
# reference the producer's complete authoritative parameter dictionaries.
RUNTIME_PROFILE_SIGNATURES = _load_authoritative_runtime_profile_signatures()
DEVELOP_PARITY_RUNTIME_SIGNATURE = RUNTIME_PROFILE_SIGNATURES["develop-parity"]
DEVELOP_SITE_GEOMETRY_RUNTIME_SIGNATURE = RUNTIME_PROFILE_SIGNATURES[
    "develop-plus-carla-site-geometry-v27"
]


class MatrixError(RuntimeError):
    """A fail-closed matrix validation or runtime error."""


@dataclass(frozen=True)
class Site:
    """One site from the active ``camping_sites.yaml``."""

    key: str
    source_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    service_mode: str


@dataclass(frozen=True)
class DropZone:
    """The active drop-zone reference used for final return error."""

    source_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float


@dataclass
class SiteResult:
    """Durable evidence for one outbound/return attempt."""

    site: str
    source_id: str
    service_mode: str
    target_pose: dict[str, float]
    mission_intent: str = "delivery"
    configured_service_mode: str = ""
    status: str = "NOT_ATTEMPTED"
    started_at_utc: str = ""
    dispatch_started_at_utc: str = ""
    finished_at_utc: str = ""
    elapsed_s: float = 0.0
    outbound_duration_s: float = 0.0
    return_duration_s: float = 0.0
    outbound_distance_m: float = 0.0
    return_distance_m: float = 0.0
    total_odom_distance_m: float = 0.0
    milestones: list[dict[str, Any]] = field(default_factory=list)
    failure_reason: str = ""
    start_localization_pose: dict[str, Any] = field(default_factory=dict)
    arrival_localization_pose: dict[str, Any] = field(default_factory=dict)
    site_phase_sequence: list[str] = field(default_factory=list)
    service_state_sequence: list[str] = field(default_factory=list)
    drop_zone_phase_sequence: list[str] = field(default_factory=list)
    parking_phase_sequence: list[str] = field(default_factory=list)
    ui_operation_request_sequence: list[dict[str, Any]] = field(
        default_factory=list
    )
    controller_operation_request_sequence: list[dict[str, Any]] = field(
        default_factory=list
    )
    destination_request_sequence: list[dict[str, Any]] = field(
        default_factory=list
    )
    recall_request_sequence: list[dict[str, Any]] = field(default_factory=list)
    dispatch_source_verified: bool = False
    motion_metrics: dict[str, Any] = field(default_factory=dict)
    final_service_state: dict[str, Any] = field(default_factory=dict)
    final_parking_status: dict[str, Any] = field(default_factory=dict)
    final_drop_zone_status: dict[str, Any] = field(default_factory=dict)
    final_gate_status: dict[str, Any] = field(default_factory=dict)
    final_physical_four_wheel_status: dict[str, Any] = field(default_factory=dict)
    final_localization_pose: dict[str, Any] = field(default_factory=dict)
    final_carla_odometry: dict[str, Any] = field(default_factory=dict)
    drop_zone_error_m: float | None = None
    parking_confirmed: bool = False
    charging_confirmed: bool = False


def utc_now() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise MatrixError(f"{name} must be numeric, got {value!r}") from None
    if not math.isfinite(number):
        raise MatrixError(f"{name} must be finite, got {value!r}")
    return number


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MatrixError(f"{name} must be a mapping")
    return value


def load_yaml(path: Path, key: str) -> list[Mapping[str, Any]]:
    """Load one required list from an active YAML file."""
    if not path.is_file():
        raise MatrixError(f"active configuration is missing: {path}")
    try:
        import yaml  # deferred so plan helpers remain importable
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise MatrixError(f"cannot read {path}: {error}") from None
    except ImportError:
        raise MatrixError("PyYAML is required to load active CAMROD YAML") from None
    document = _mapping(root, str(path))
    values = document.get(key)
    if not isinstance(values, list) or not values:
        raise MatrixError(f"{path} must contain a non-empty {key}: list")
    return [_mapping(item, f"{path}:{key}[{index}]") for index, item in enumerate(values)]


def load_sites(path: Path) -> dict[str, Site]:
    """Parse B1..B13 from the active CAMROD camping-site configuration."""
    result: dict[str, Site] = {}
    for item in load_yaml(path, "camping_sites"):
        source_id = str(item.get("id", "")).strip()
        match = SITE_TYPE_RE.fullmatch(str(item.get("type", "")).strip())
        if not source_id or match is None:
            raise MatrixError(f"invalid campsite id/type in {path}: {item!r}")
        key = f"B{int(match.group(1))}"
        if key in result:
            raise MatrixError(f"duplicate active campsite {key} in {path}")
        result[key] = Site(
            key=key,
            source_id=source_id,
            x_m=_finite(item.get("x"), f"{key}.x"),
            y_m=_finite(item.get("y"), f"{key}.y"),
            z_m=_finite(item.get("z", 0.0), f"{key}.z"),
            yaw_deg=_finite(item.get("yaw_deg", 0.0), f"{key}.yaw_deg"),
            service_mode=str(item.get("service_mode", "")).strip(),
        )
        if result[key].service_mode not in SITE_PHASES:
            raise MatrixError(
                f"{key}.service_mode must be one of {sorted(SITE_PHASES)}, "
                f"got {result[key].service_mode!r}"
            )
    return result


def load_drop_zone(path: Path) -> DropZone:
    """Load exactly one active drop-zone reference."""
    values = load_yaml(path, "drop_zones")
    if len(values) != 1:
        raise MatrixError(f"expected exactly one active drop zone in {path}, found {len(values)}")
    item = values[0]
    source_id = str(item.get("id", "")).strip()
    if not source_id:
        raise MatrixError(f"drop-zone id is empty in {path}")
    return DropZone(
        source_id=source_id,
        x_m=_finite(item.get("x"), "drop_zone.x"),
        y_m=_finite(item.get("y"), "drop_zone.y"),
        z_m=_finite(item.get("z", 0.0), "drop_zone.z"),
        yaw_deg=_finite(item.get("yaw_deg", 0.0), "drop_zone.yaw_deg"),
    )


def parse_site_selection(value: str | None) -> tuple[str, ...]:
    """Parse and validate ``--sites`` without touching ROS or HTTP."""
    if not value:
        return DEFAULT_SITES
    requested = tuple(part.strip().upper() for part in value.split(",") if part.strip())
    if not requested:
        raise MatrixError("--sites must contain at least one site")
    invalid = [site for site in requested if not re.fullmatch(r"B(?:[1-9]|1[0-3])", site)]
    if invalid:
        raise MatrixError(f"invalid site selection {invalid!r}; expected B1..B13")
    if len(set(requested)) != len(requested):
        raise MatrixError("--sites must not contain duplicates")
    return requested


def pose_error_m(pose: Mapping[str, Any], drop_zone: DropZone) -> float | None:
    """Compute planar localization-to-drop-zone error from a normalized pose."""
    try:
        x = _finite(pose["x_m"], "pose.x_m")
        y = _finite(pose["y_m"], "pose.y_m")
    except (KeyError, MatrixError):
        return None
    return math.hypot(x - drop_zone.x_m, y - drop_zone.y_m)


def contains_ordered_subsequence(
    values: Sequence[Any], expected: Sequence[Any]
) -> bool:
    """Return true when every expected phase appears in order."""
    position = 0
    for value in values:
        if position < len(expected) and value == expected[position]:
            position += 1
    return position == len(expected)


def arrival_ready_for_return(
    snapshot: Mapping[str, Any],
    expected_phases: Sequence[str],
    *,
    expected_service_state: int = WAITING_FOR_RETURN_REQUEST,
) -> bool:
    """Require both lifecycle and controller evidence before UI return.

    ``/service/state`` and the camping-site controller status are independent
    ROS topics.  The lifecycle can therefore publish the intent-specific
    arrival state one callback before the controller's ``WAIT_RETURN`` status
    is observed.  Treating the lifecycle sample alone as arrival would race
    the evidence observer and could issue Return before the controller is
    observably ready.  Normal delivery expects
    ``WAITING_FOR_RETURN_REQUEST``; roadside recall expects
    ``GUEST_LOADING_WAIT``.  Malformed or incomplete snapshots fail closed.
    """
    service_state = snapshot.get("service_state")
    sequences = snapshot.get("sequences")
    if not isinstance(service_state, Mapping) or not isinstance(sequences, Mapping):
        return False
    if service_state.get("state") != expected_service_state:
        return False
    try:
        wait_return_index = expected_phases.index("WAIT_RETURN") + 1
    except ValueError:
        return False
    site_phases = sequences.get("site_phases")
    if not isinstance(site_phases, Sequence) or isinstance(site_phases, (str, bytes)):
        return False
    return contains_ordered_subsequence(
        site_phases, expected_phases[:wait_return_index]
    )


def effective_service_mode(site: Site, mission_intent: str) -> str:
    """Return the controller policy that must be observed for this mission."""
    if mission_intent == "recall":
        # Latest develop defines every B1-B13 recall as a bounded roadside
        # pickup, independent of the site's normal delivery service policy.
        return "roadside_stop"
    if mission_intent == "delivery":
        return site.service_mode
    raise MatrixError(
        f"mission_intent must be one of {MISSION_INTENTS}, got {mission_intent!r}"
    )


def required_service_state_ids(mission_intent: str) -> tuple[int, ...]:
    """Return ordered-state membership required before a mission may pass."""
    if mission_intent == "recall":
        return (
            RECALL_TO_SITE_ROAD,
            GUEST_LOADING_WAIT,
            RETURN_WITH_CARGO,
            DROP_ZONE_PARKING,
            WAITING_FOR_CHARGING,
            CHARGING,
        )
    if mission_intent == "delivery":
        return (DROP_ZONE_PARKING, WAITING_FOR_CHARGING, CHARGING)
    raise MatrixError(
        f"mission_intent must be one of {MISSION_INTENTS}, got {mission_intent!r}"
    )


def return_source_matches(
    observed: Any,
    expected_source: str,
    expected_site: str = "",
) -> bool:
    """Match a frontend RETURN source, including its current mission nonce."""
    if observed == expected_source:
        return True
    if not isinstance(observed, str) or not expected_source:
        return False
    if expected_source == "guest:usage_complete":
        match = GUEST_RETURN_SOURCE_RE.fullmatch(observed)
        return match is not None and (
            not expected_site or match.group(1) == expected_site
        )
    site_exit_suffix = ":site_exit_first"
    if not expected_source.endswith(site_exit_suffix):
        return False
    authority = expected_source[: -len(site_exit_suffix)]
    token_prefix = f"{authority}:ui_return_token="
    if not observed.startswith(token_prefix) or not observed.endswith(
        site_exit_suffix
    ):
        return False
    token = observed[len(token_prefix) : -len(site_exit_suffix)]
    return UI_RETURN_TOKEN_RE.fullmatch(token) is not None


def return_source_observed(
    snapshot: Mapping[str, Any],
    expected_source: str,
    expected_site: str = "",
) -> bool:
    """Require the authenticated frontend RETURN source at a ROS boundary."""
    if not expected_source:
        return True
    sequences = snapshot.get("sequences")
    if not isinstance(sequences, Mapping):
        return False
    for key in ("ui_operation_requests", "controller_operation_requests"):
        requests = sequences.get(key)
        if not isinstance(requests, Sequence) or isinstance(
            requests, (str, bytes)
        ):
            continue
        for request in requests:
            if not isinstance(request, Mapping):
                continue
            try:
                operation = int(request.get("operation", -1))
            except (TypeError, ValueError):
                continue
            if (
                operation == RETURN_OPERATION
                and return_source_matches(
                    request.get("source"), expected_source, expected_site
                )
            ):
                return True
    return False


def dispatch_source_observed(
    snapshot: Mapping[str, Any],
    *,
    site: str,
    mission_intent: str,
    expected_source: str,
) -> bool:
    """Bind a browser dispatch to the matching source-bearing ROS message."""
    sequences = snapshot.get("sequences")
    if not isinstance(sequences, Mapping):
        return False
    def source_matches(observed: Any) -> bool:
        if observed == expected_source:
            return True
        # Direct Robot-UI destination admission publishes one observable ROS
        # echo after the backend has already applied the command.  Its source
        # retains the exact browser authority before an unguessable per-process
        # marker used to suppress its own subscription callback.  Accept only
        # that canonical suffix; a loose prefix match would let another source
        # impersonate the browser in the evidence report.
        if not isinstance(observed, str):
            return False
        marker_prefix = f"{expected_source}|ui_backend_echo="
        if not observed.startswith(marker_prefix):
            return False
        marker = observed[len(marker_prefix):]
        return bool(marker) and marker == marker.strip() and "|" not in marker

    def request_observed(
        sequence_name: str, site_key: str, *, requires_run: bool
    ) -> bool:
        requests = sequences.get(sequence_name)
        if not isinstance(requests, Sequence) or isinstance(
            requests, (str, bytes)
        ):
            return False
        return any(
            isinstance(request, Mapping)
            and request.get(site_key) == site
            and source_matches(request.get("source"))
            and (not requires_run or request.get("run") is True)
            for request in requests
        )

    if mission_intent == "delivery":
        return request_observed(
            "destination_requests", "site", requires_run=True
        )
    if mission_intent == "recall":
        # A recall admitted while CHARGING is intentionally deferred until the
        # charging dwell and Drop-Zone exit finish.  The backend publishes its
        # authenticated direct-destination echo immediately, while the typed
        # PlanningRecallRequest is emitted only after that physical departure.
        # Bind dispatch to either exact ROS boundary without weakening source
        # validation; otherwise a valid recall is falsely rejected by the
        # short pre-motion dispatch timeout.
        return request_observed(
            "destination_requests", "site", requires_run=True
        ) or request_observed(
            "recall_requests", "site_name", requires_run=False
        )
    return False


def mission_segment_metrics(
    *,
    dispatch_distance_m: float,
    arrival_distance_m: float,
    return_request_distance_m: float,
    final_distance_m: float,
    dispatch_monotonic: float,
    arrival_monotonic: float,
    return_request_monotonic: float,
    final_monotonic: float,
) -> dict[str, float]:
    """Calculate explicit outbound/return timing and odometry evidence.

    Distances are cumulative values from the same per-site ROS observer.  Any
    clock or odometry regression is rejected rather than hidden with an
    absolute value or clamp.
    """
    distances = tuple(
        _finite(value, name)
        for value, name in (
            (dispatch_distance_m, "dispatch_distance_m"),
            (arrival_distance_m, "arrival_distance_m"),
            (return_request_distance_m, "return_request_distance_m"),
            (final_distance_m, "final_distance_m"),
        )
    )
    times = tuple(
        _finite(value, name)
        for value, name in (
            (dispatch_monotonic, "dispatch_monotonic"),
            (arrival_monotonic, "arrival_monotonic"),
            (return_request_monotonic, "return_request_monotonic"),
            (final_monotonic, "final_monotonic"),
        )
    )
    if any(next_value + 1.0e-6 < value for value, next_value in zip(distances, distances[1:])):
        raise MatrixError(
            "per-site cumulative odometry regressed across dispatch/arrival/return/final"
        )
    if any(next_value < value for value, next_value in zip(times, times[1:])):
        raise MatrixError(
            "per-site monotonic clock regressed across dispatch/arrival/return/final"
        )
    return {
        "outbound_duration_s": round(times[2] - times[0], 3),
        "return_duration_s": round(times[3] - times[2], 3),
        "outbound_distance_m": round(distances[1] - distances[0], 6),
        "return_distance_m": round(distances[3] - distances[2], 6),
        "total_odom_distance_m": round(distances[3] - distances[0], 6),
    }


def load_sensor_audit(path: Path, *, maximum_age_s: float = 120.0) -> dict[str, Any]:
    """Validate and bind one fresh fail-closed CARLA sensor-source audit."""
    if not path.is_file():
        raise MatrixError(f"sensor-source audit is missing: {path}")
    age_s = time.time() - path.stat().st_mtime
    if not math.isfinite(age_s) or age_s < -5.0 or age_s > maximum_age_s:
        raise MatrixError(
            f"sensor-source audit is not fresh: age={age_s:.3f}s, "
            f"maximum={maximum_age_s:.3f}s"
        )
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MatrixError(
            f"sensor-source audit is unreadable: {path}: {error}"
        ) from None
    if not isinstance(document, dict):
        raise MatrixError(f"sensor-source audit root must be an object: {path}")
    summary = document.get("summary")
    if not isinstance(summary, dict):
        raise MatrixError(f"sensor-source audit summary is missing: {path}")
    exact = {
        "streams_checked": 36,
        "stream_failures": 0,
        "actors_checked": 13,
        "actor_failures": 0,
    }
    mismatches = [
        f"{key}={summary.get(key)!r} expected {expected!r}"
        for key, expected in exact.items()
        if summary.get(key) != expected
    ]
    if document.get("status") != "PASS" or document.get("passed") is not True:
        mismatches.append(
            f"status/passed={document.get('status')!r}/"
            f"{document.get('passed')!r}"
        )
    if mismatches:
        raise MatrixError("sensor-source audit rejected: " + ", ".join(mismatches))
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "status": "PASS",
        "summary": exact,
        "age_s_at_matrix_start": round(max(0.0, age_s), 3),
    }


def normalize_carla_map(value: str) -> str:
    """Normalize `/Game/...` and Python API CARLA map-name spellings."""
    result = str(value).strip()
    if result.startswith("/Game/"):
        result = result[len("/Game/") :]
    return result.strip("/")


def load_runtime_profile_audit(
    path: Path,
    *,
    expected_carla_map: str,
    expected_carla_town: str,
    expected_lanelet_map: Path,
    expected_actor_id: int,
    expected_role_name: str,
    maximum_age_s: float = 120.0,
) -> dict[str, Any]:
    """Validate and bind one fresh motion-authorized live-profile audit."""
    if not path.is_file():
        raise MatrixError(f"runtime-profile audit is missing: {path}")
    age_s = time.time() - path.stat().st_mtime
    if not math.isfinite(age_s) or age_s < -5.0 or age_s > maximum_age_s:
        raise MatrixError(
            f"runtime-profile audit is not fresh: age={age_s:.3f}s, "
            f"maximum={maximum_age_s:.3f}s"
        )
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MatrixError(
            f"runtime-profile audit is unreadable: {path}: {error}"
        ) from None
    if not isinstance(document, Mapping):
        raise MatrixError(f"runtime-profile audit root must be an object: {path}")
    mismatches: list[str] = []
    if document.get("schema") != "camrod.virtual_carla.runtime_profile_audit.v1":
        mismatches.append(f"schema={document.get('schema')!r}")
    if document.get("status") != "PASS" or document.get("accepted") is not True:
        mismatches.append(
            f"status/accepted={document.get('status')!r}/"
            f"{document.get('accepted')!r}"
        )
    if document.get("errors") != []:
        mismatches.append(f"errors={document.get('errors')!r}")
    profile = document.get("profile")
    profile_signature = (
        RUNTIME_PROFILE_SIGNATURES.get(profile)
        if isinstance(profile, str)
        else None
    )
    if profile_signature is None:
        mismatches.append(f"profile={profile!r}")

    expected_map = normalize_carla_map(expected_carla_map)
    expected_town = normalize_carla_map(expected_carla_town)
    if not expected_map or expected_map != expected_town:
        mismatches.append(
            f"expected map/town={expected_map!r}/{expected_town!r}"
        )
    carla_record = document.get("carla")
    if not isinstance(carla_record, Mapping):
        mismatches.append("carla record missing")
        live_map = ""
        live_actor_id = 0
    else:
        live_map = normalize_carla_map(
            str(carla_record.get("normalized_map_name", ""))
        )
        if live_map != expected_map:
            mismatches.append(
                f"live CARLA map={live_map!r} expected {expected_map!r}"
            )
        ego = carla_record.get("ego_actor")
        if not isinstance(ego, Mapping):
            mismatches.append("CARLA ego actor record missing")
            live_actor_id = 0
        else:
            live_actor_id = ego.get("actor_id", 0)
            if type(live_actor_id) is not int or live_actor_id != expected_actor_id:
                mismatches.append(
                    f"CARLA actor_id={live_actor_id!r} expected {expected_actor_id}"
                )
            if ego.get("type_id") != "vehicle.ranger.default":
                mismatches.append(f"CARLA ego type={ego.get('type_id')!r}")
            if ego.get("role_name") != expected_role_name:
                mismatches.append(f"CARLA ego role={ego.get('role_name')!r}")
        settings = carla_record.get("world_settings")
        if not isinstance(settings, Mapping):
            mismatches.append("CARLA world settings missing")
        else:
            if settings.get("synchronous_mode") is not True:
                mismatches.append("CARLA synchronous_mode is not true")
            fixed_delta = settings.get("fixed_delta_seconds")
            if type(fixed_delta) is not float or not math.isclose(
                fixed_delta, 0.05, rel_tol=0.0, abs_tol=1.0e-9
            ):
                mismatches.append(f"CARLA fixed_delta_seconds={fixed_delta!r}")
            if settings.get("no_rendering_mode") is not False:
                mismatches.append("CARLA no_rendering_mode is not false")
        umap = carla_record.get("ue_map_asset")
        if not isinstance(umap, Mapping):
            mismatches.append("CARLA UE map asset record missing")
        else:
            try:
                umap_path = Path(str(umap["path"])).expanduser().resolve()
                umap_sha = str(umap["sha256"])
            except (KeyError, TypeError, ValueError):
                mismatches.append("CARLA UE map asset path/sha256 malformed")
            else:
                if not umap_path.is_file():
                    mismatches.append(f"CARLA UE map asset missing: {umap_path}")
                elif sha256_file(umap_path) != umap_sha:
                    mismatches.append("CARLA UE map asset changed after audit")

    physical = document.get("physical_four_wheel_bridge")
    if not isinstance(physical, Mapping):
        mismatches.append("physical 4WS bridge record missing")
    else:
        if physical.get("status") != "READY":
            mismatches.append(f"physical 4WS status={physical.get('status')!r}")
        if physical.get("actor_id") != expected_actor_id:
            mismatches.append(
                f"physical 4WS actor_id={physical.get('actor_id')!r} "
                f"expected {expected_actor_id}"
            )
        if physical.get("motion_backend") != "PHYSX_FOUR_WHEEL_STEERING":
            mismatches.append(
                f"physical 4WS backend={physical.get('motion_backend')!r}"
            )

    lanelet_record = document.get("lanelet_map")
    if not isinstance(lanelet_record, Mapping):
        mismatches.append("lanelet_map record missing")
        lanelet_path = None
        lanelet_sha = ""
    else:
        try:
            lanelet_path = Path(str(lanelet_record["path"])).expanduser().resolve()
            lanelet_sha = str(lanelet_record["sha256"])
        except (KeyError, TypeError, ValueError):
            lanelet_path = None
            lanelet_sha = ""
            mismatches.append("lanelet_map path/sha256 malformed")
        if lanelet_path is not None:
            expected_lanelet = expected_lanelet_map.expanduser().resolve()
            if lanelet_path != expected_lanelet:
                mismatches.append(
                    f"lanelet map={lanelet_path} expected {expected_lanelet}"
                )
            if not lanelet_path.is_file():
                mismatches.append(f"lanelet map missing: {lanelet_path}")
            elif sha256_file(lanelet_path) != lanelet_sha:
                mismatches.append("lanelet map changed after runtime-profile audit")

    selected = document.get("selected_live_parameters")
    if not isinstance(selected, Mapping) or not selected:
        mismatches.append("selected_live_parameters missing")
    else:
        for node, parameters in (profile_signature or {}).items():
            live_parameters = selected.get(node)
            if not isinstance(live_parameters, Mapping):
                mismatches.append(f"selected parameters missing node {node}")
                continue
            for name, expected in parameters.items():
                actual = live_parameters.get(name, object())
                if type(actual) is not type(expected) or actual != expected:
                    mismatches.append(
                        f"{node}.{name}={actual!r} expected {expected!r}"
                    )
    if mismatches:
        raise MatrixError(
            "runtime-profile audit rejected: " + ", ".join(mismatches)
        )
    assert lanelet_path is not None
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "status": "PASS",
        "profile": profile,
        "actor_id": expected_actor_id,
        "age_s_at_matrix_start": round(max(0.0, age_s), 3),
        "carla_map": live_map,
        "lanelet_map": {
            "path": str(lanelet_path),
            "sha256": lanelet_sha,
        },
        "selected_live_parameters": selected,
    }


def build_plan(sites: Mapping[str, Site], selected: Iterable[str], drop_zone: DropZone,
               *, output: Path, config_paths: Mapping[str, Path],
               role_name: str = "ego_vehicle",
               mission_intent: str = "delivery") -> dict[str, Any]:
    """Build a report that contains no runtime claim and no motion side effect."""
    selected_tuple = tuple(selected)
    selected_set = set(selected_tuple)
    results = []
    for key in selected_tuple:
        site = sites[key]
        service_mode = effective_service_mode(site, mission_intent)
        results.append(asdict(SiteResult(
            site=site.key,
            source_id=site.source_id,
            service_mode=service_mode,
            target_pose={"x_m": site.x_m, "y_m": site.y_m, "z_m": site.z_m, "yaw_deg": site.yaw_deg},
            mission_intent=mission_intent,
            configured_service_mode=site.service_mode,
        )))
    return {
        "schema": "camrod.virtual_carla.camping_site_matrix.v1",
        "status": "PLAN_ONLY",
        "created_at_utc": utc_now(),
        "scope": {
            "selected_sites": list(selected_tuple),
            "unattempted_sites": [key for key in sites if key not in selected_set],
            "mission_intent": mission_intent,
            "motion_commands_sent": False,
            "pose_teleport_used": False,
            "fake_sensor_data_used": False,
            "ui_endpoints": {"dispatch": "POST /ui/destination?site=Bx&run=true", "return": "POST /ui/manual_return", "stop": "POST /ui/stop"},
        },
        "active_configuration": {
            "camping_sites_yaml": str(config_paths["camping_sites"]),
            "drop_zones_yaml": str(config_paths["drop_zones"]),
            "sha256": {name: sha256_file(path) for name, path in config_paths.items()},
            "drop_zone": asdict(drop_zone),
        },
        "sites": results,
        "runtime_observation_contract": observation_contract(role_name),
        "report_path": str(output),
    }


def sha256_file(path: Path) -> str:
    """Return the provenance digest for an active configuration file."""
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise MatrixError(f"cannot hash active configuration {path}: {error}") from None
    return digest.hexdigest()


def observation_contract(role_name: str = "ego_vehicle") -> dict[str, Any]:
    """Document every live source the runner observes."""
    return {
        "required_topics": [
            "/service/state", "/localization/pose", "/control/cmd_vel",
            "/carla/ego_vehicle/odometry",
            "/carla/ego_vehicle/physical_four_wheel_status",
            "/control/camping_site_maneuver_controller/status",
            "/ui/camping_site_operation_request",
            "/control/camping_site_maneuver_controller/operation",
            "/ui/selected_destination",
            "/planning/state_machine/camping_site_recall",
        ],
        "status_topics": [
            "/parking/reverse_parking_controller/status",
            "/parking/apriltag_parking_controller/status",
            "/control/drop_zone_maneuver_controller/status",
            "/control/cmd_vel_safety_gate/status",
        ],
        "optional_evidence_topics": [f"/carla/{role_name}/collision"],
        "collision_policy": (
            "passive bounded evidence only; absence of the topic or events "
            "never blocks preflight or changes control/planning"
        ),
        "sensor_policy": "a fresh 36-stream/13-actor CARLA source audit is required; this runner neither publishes nor synthesizes sensor data",
        "readiness_policy": "physical actor id and strict PHYSX_FOUR_WHEEL_STEERING readiness must remain unchanged for the whole mission",
    }


def update_motion_command_scope(
    report: dict[str, Any], snapshot: Mapping[str, Any]
) -> None:
    """Latch live nonzero-command evidence into the report-level scope."""
    motion = snapshot.get("motion_metrics") or {}
    observed = bool(motion.get("motion_command_observed"))
    scope = report.setdefault("scope", {})
    scope["motion_commands_sent"] = bool(
        scope.get("motion_commands_sent", False) or observed
    )


def write_json_atomic(
    path: Path,
    document: Mapping[str, Any],
    *,
    create_only: bool = False,
) -> None:
    """Atomically replace a JSON report, preserving partial progress."""
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if create_only:
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            raise MatrixError(f"refusing to overwrite existing report: {path}") from None
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    document,
                    stream,
                    indent=2,
                    ensure_ascii=False,
                    sort_keys=False,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            raise
        return
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


class UIClient:
    """Small production UI client with strict success handling."""

    def __init__(self, base_url: str, timeout_s: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        parsed = urlparse(self.base_url)
        if (
            parsed.scheme != "http"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(
                "--ui-url must be a local http://127.0.0.1 or "
                "http://localhost endpoint without credentials/query/fragment"
            )

    def post(self, path: str, query: Mapping[str, str] | None = None) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        if query:
            url += "?" + urlencode(query)
        request = Request(url, method="POST", headers={"Accept": "application/json"})
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # nosec B310 - operator-selected local UI URL
                body = response.read().decode("utf-8")
                status = int(response.status)
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise MatrixError(f"UI request failed {url}: {error}") from None
        if status < 200 or status >= 300:
            raise MatrixError(f"UI request returned HTTP {status}: {url}")
        try:
            decoded = json.loads(body) if body else {}
        except json.JSONDecodeError:
            raise MatrixError(f"UI response was not JSON: {url}") from None
        if not isinstance(decoded, dict):
            raise MatrixError(f"UI response was not an object: {url}")
        if decoded.get("success") is False:
            raise MatrixError(f"UI rejected {url}: {decoded}")
        return decoded

    def dispatch(
        self, site: str, mission_intent: str = "delivery"
    ) -> dict[str, Any]:
        if mission_intent == "recall":
            return self.post(
                "/ui/camping_site_recall",
                {"site": site, "intent": "recall"},
            )
        if mission_intent == "delivery":
            return self.post("/ui/destination", {"site": site, "run": "true"})
        raise MatrixError(
            f"mission_intent must be one of {MISSION_INTENTS}, "
            f"got {mission_intent!r}"
        )

    def request_return(self) -> dict[str, Any]:
        return self.post("/ui/manual_return")

    def stop(self) -> dict[str, Any]:
        return self.post("/ui/stop")

    @property
    def expected_return_source(self) -> str:
        return ""

    def close(self) -> None:
        return None


class GuestBrowserClient:
    """Drive the already-visible production Guest UI through local Chrome CDP.

    The browser remains the sole Guest WebSocket client.  CDP invokes the same
    page functions as a user click, so the tested boundary is
    ``navigate -> usage_complete`` rather than a synthetic ROS publication.
    """

    def __init__(
        self,
        debugging_url: str,
        guest_ui_url: str,
        operator_ui_url: str,
        timeout_s: float = 10.0,
    ) -> None:
        self.debugging_url = self._local_http_url(
            debugging_url, "--guest-cdp-url"
        ).rstrip("/")
        self.guest_ui_url = self._local_http_url(
            guest_ui_url, "--guest-ui-url"
        ).rstrip("/")
        self.timeout_s = timeout_s
        self.stop_client = UIClient(operator_ui_url, timeout_s=timeout_s)
        self._command_id = 0
        self._connection: Any = None
        self._target: dict[str, Any] = {}
        try:
            self._connect()
        except BaseException:
            # A CDP socket can already exist when page readiness evaluation
            # fails.  Never leave that debugger resource attached.
            self.close()
            raise

    @staticmethod
    def _local_http_url(value: str, option: str) -> str:
        parsed = urlparse(str(value))
        if (
            parsed.scheme != "http"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(
                f"{option} must be a local http://127.0.0.1 or "
                "http://localhost endpoint without credentials/query/fragment"
            )
        return str(value)

    def _connect(self) -> None:
        request = Request(
            f"{self.debugging_url}/json",
            method="GET",
            headers={"Accept": "application/json"},
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # nosec B310 - validated localhost CDP
                targets = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as error:
            raise MatrixError(
                f"Guest browser CDP target discovery failed: {error}"
            ) from None
        if not isinstance(targets, list):
            raise MatrixError("Guest browser CDP /json response must be a list")
        matches = [
            item
            for item in targets
            if isinstance(item, dict)
            and item.get("type") == "page"
            and str(item.get("url", "")).rstrip("/") == self.guest_ui_url
            and str(item.get("webSocketDebuggerUrl", "")).strip()
        ]
        if len(matches) != 1:
            raise MatrixError(
                "expected exactly one visible Guest UI CDP page at "
                f"{self.guest_ui_url}, found {len(matches)}"
            )
        self._target = dict(matches[0])
        websocket_url = str(self._target["webSocketDebuggerUrl"])
        parsed = urlparse(websocket_url)
        if (
            parsed.scheme != "ws"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(
                f"Guest browser returned a non-local debugger URL: {websocket_url}"
            )
        try:
            import websocket

            self._connection = websocket.create_connection(
                websocket_url,
                timeout=self.timeout_s,
                origin=self.guest_ui_url,
            )
        except Exception as error:
            raise MatrixError(
                f"Guest browser CDP WebSocket connection failed: {error}"
            ) from None
        ready = self._evaluate(
            "(() => ({"
            "title: document.title, href: document.location.href, "
            "ready: document.readyState, visibility: document.visibilityState, "
            "guestSocketOpen: Boolean(typeof ws !== 'undefined' && ws && "
            "ws.readyState === WebSocket.OPEN), "
            "guestFunctionsReady: Boolean("
            "typeof selectSite === 'function' && "
            "typeof openConfirm === 'function' && "
            "typeof confirmNavigate === 'function' && "
            "typeof sendUsageComplete === 'function' && "
            "typeof isDispatchReady === 'function')"
            "}))()"
        )
        if (
            not isinstance(ready, dict)
            or ready.get("ready") != "complete"
            or str(ready.get("href", "")).rstrip("/") != self.guest_ui_url
            or str(ready.get("title", "")).strip() != GUEST_UI_TITLE
            or ready.get("visibility") != "visible"
            or ready.get("guestSocketOpen") is not True
            or ready.get("guestFunctionsReady") is not True
        ):
            raise MatrixError(f"visible Guest UI page is not ready: {ready!r}")

    def _call(self, method: str, params: Mapping[str, Any]) -> dict[str, Any]:
        if self._connection is None:
            raise MatrixError("Guest browser CDP connection is closed")
        self._command_id += 1
        command_id = self._command_id
        try:
            self._connection.send(json.dumps({
                "id": command_id,
                "method": method,
                "params": dict(params),
            }))
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                remaining = max(0.01, deadline - time.monotonic())
                self._connection.settimeout(remaining)
                payload = json.loads(self._connection.recv())
                if not isinstance(payload, dict):
                    raise MatrixError(
                        "Guest browser CDP frame must be a JSON object"
                    )
                if "id" not in payload:
                    # Runtime events are valid between a command and response.
                    continue
                if payload.get("id") != command_id:
                    raise MatrixError(
                        "Guest browser CDP returned an unexpected command id: "
                        f"{payload.get('id')!r} expected {command_id}"
                    )
                if payload.get("error"):
                    raise MatrixError(
                        f"Guest browser CDP command failed: {payload['error']}"
                    )
                if not isinstance(payload.get("result"), Mapping):
                    raise MatrixError(
                        "Guest browser CDP command response has no result object"
                    )
                return payload
        except MatrixError:
            self.close()
            raise
        except Exception as error:
            self.close()
            raise MatrixError(
                f"Guest browser CDP command transport failed: {error}"
            ) from None
        self.close()
        raise MatrixError(f"Guest browser CDP command timed out: {method}")

    def _evaluate(self, expression: str) -> Any:
        payload = self._call(
            "Runtime.evaluate",
            {
                "expression": expression,
                "returnByValue": True,
                "awaitPromise": True,
            },
        )
        try:
            result = payload.get("result")
            if not isinstance(result, Mapping):
                raise MatrixError("Guest browser CDP evaluate result is malformed")
            if result.get("exceptionDetails"):
                raise MatrixError(
                    f"Guest UI JavaScript failed: {result['exceptionDetails']}"
                )
            remote = result.get("result")
            if not isinstance(remote, Mapping):
                raise MatrixError("Guest browser CDP remote object is missing")
            if "value" not in remote:
                raise MatrixError(
                    "Guest browser CDP evaluation did not return a by-value result: "
                    f"type={remote.get('type')!r}"
                )
            return remote["value"]
        except MatrixError:
            self.close()
            raise

    @staticmethod
    def _accepted(value: Any, action: str) -> dict[str, Any]:
        if not isinstance(value, dict) or value.get("accepted") is not True:
            raise MatrixError(f"Guest UI rejected {action}: {value!r}")
        return dict(value)

    def dispatch(
        self, site: str, mission_intent: str = "recall"
    ) -> dict[str, Any]:
        if mission_intent != "recall":
            raise MatrixError(
                "visible Guest UI dispatch is a roadside recall; "
                "mission_intent must be 'recall'"
            )
        literal = json.dumps(site)
        expression = (
            "(() => {"
            f"const requestedSite = {literal};"
            "if (document.readyState !== 'complete' || typeof ws === 'undefined' || !ws || "
            "ws.readyState !== WebSocket.OPEN || typeof isDispatchReady !== 'function' || "
            "typeof selectSite !== 'function' || typeof openConfirm !== 'function' || "
            "typeof confirmNavigate !== 'function') "
            "return {accepted:false, reason:'guest_ui_not_ready'};"
            "if (!isDispatchReady()) return {accepted:false, reason:'dispatch_not_ready', "
            "phase:currentPhase, state:currentState};"
            "const socket = ws; const originalSend = socket.send; let sentFrame = null;"
            "const captureSend = function(payload) { sentFrame = String(payload); "
            "return originalSend.call(socket, payload); };"
            "try {"
            "socket.send = captureSend;"
            "if (socket.send !== captureSend) return {accepted:false, reason:'send_capture_failed'};"
            "selectSite(requestedSite);"
            "if (selectedSite !== requestedSite) "
            "return {accepted:false, reason:'site_not_selected'};"
            "openConfirm(); confirmNavigate();"
            "} finally { socket.send = originalSend; }"
            "let frame = null; try { frame = JSON.parse(sentFrame); } catch (_) {}"
            "if (!frame || frame.action !== 'navigate' || frame.site !== requestedSite) "
            "return {accepted:false, reason:'navigate_frame_not_sent', frame:sentFrame};"
            "return {accepted:true, action:'navigate', site:requestedSite, "
            "frame:frame, transport:'visible_guest_page_websocket_via_cdp'};"
            "})()"
        )
        try:
            value = self._evaluate(expression)
            return self._accepted(value, "navigate")
        except BaseException:
            self.close()
            raise

    def request_return(self) -> dict[str, Any]:
        expression = (
            "(() => {"
            "if (document.readyState !== 'complete' || typeof ws === 'undefined' || !ws || "
            "ws.readyState !== WebSocket.OPEN || typeof sendUsageComplete !== 'function') "
            "return {accepted:false, reason:'guest_ui_not_ready'};"
            "if (typeof currentPhase === 'undefined' || currentPhase !== 'arrived' || "
            "typeof currentState === 'undefined' || currentState !== 8) "
            "return {accepted:false, reason:'guest_ui_not_waiting_for_return', "
            "phase:currentPhase, state:currentState};"
            "const socket = ws; const originalSend = socket.send; let sentFrame = null;"
            "const captureSend = function(payload) { sentFrame = String(payload); "
            "return originalSend.call(socket, payload); };"
            "try { socket.send = captureSend;"
            "if (socket.send !== captureSend) return {accepted:false, reason:'send_capture_failed'};"
            "sendUsageComplete(); } finally { socket.send = originalSend; }"
            "let frame = null; try { frame = JSON.parse(sentFrame); } catch (_) {}"
            "if (!frame || frame.action !== 'usage_complete') "
            "return {accepted:false, reason:'usage_complete_frame_not_sent', frame:sentFrame};"
            "return {accepted:true, action:'usage_complete', frame:frame, "
            "transport:'visible_guest_page_websocket_via_cdp', state:currentState};"
            "})()"
        )
        try:
            value = self._evaluate(expression)
            return self._accepted(value, "usage_complete")
        except BaseException:
            self.close()
            raise

    def stop(self) -> dict[str, Any]:
        return self.stop_client.stop()

    @property
    def expected_return_source(self) -> str:
        return "guest:usage_complete"

    def close(self) -> None:
        connection = self._connection
        self._connection = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


class OperatorBrowserClient(GuestBrowserClient):
    """Drive the visible production Robot UI with real CDP pointer/input events.

    Runtime evaluation is restricted to locating stable ``data-ui`` hooks and
    passively recording the page's own HTTP/WebSocket traffic.  Every action
    itself uses ``Input.dispatchMouseEvent`` or ``Input.insertText``; no React
    callback or backend endpoint is invoked directly by this client.
    """

    def __init__(
        self,
        debugging_url: str,
        operator_ui_url: str,
        timeout_s: float = 10.0,
    ) -> None:
        self.debugging_url = self._local_http_url(
            debugging_url, "--operator-cdp-url"
        ).rstrip("/")
        self.operator_ui_url = self._local_http_url(
            operator_ui_url, "--ui-url"
        ).rstrip("/")
        self.timeout_s = timeout_s
        self.stop_client = UIClient(operator_ui_url, timeout_s=timeout_s)
        self._command_id = 0
        self._connection: Any = None
        self._target: dict[str, Any] = {}
        self._interactions: list[dict[str, Any]] = []
        try:
            self._connect_operator()
            self._install_transport_probe()
        except BaseException:
            self.close()
            raise

    def _connect_operator(self) -> None:
        request = Request(
            f"{self.debugging_url}/json",
            method="GET",
            headers={"Accept": "application/json"},
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # nosec B310 - validated localhost CDP
                targets = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as error:
            raise MatrixError(
                f"Robot UI browser CDP target discovery failed: {error}"
            ) from None
        if not isinstance(targets, list):
            raise MatrixError("Robot UI browser CDP /json response must be a list")
        matches = [
            item
            for item in targets
            if isinstance(item, dict)
            and item.get("type") == "page"
            and str(item.get("url", "")).rstrip("/") == self.operator_ui_url
            and str(item.get("webSocketDebuggerUrl", "")).strip()
        ]
        if len(matches) != 1:
            raise MatrixError(
                "expected exactly one visible Robot UI CDP page at "
                f"{self.operator_ui_url}, found {len(matches)}"
            )
        self._target = dict(matches[0])
        websocket_url = str(self._target["webSocketDebuggerUrl"])
        parsed = urlparse(websocket_url)
        if (
            parsed.scheme != "ws"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(
                "Robot UI browser returned a non-local debugger URL: "
                f"{websocket_url}"
            )
        try:
            import websocket

            self._connection = websocket.create_connection(
                websocket_url,
                timeout=self.timeout_s,
                origin=self.operator_ui_url,
            )
        except Exception as error:
            raise MatrixError(
                f"Robot UI browser CDP WebSocket connection failed: {error}"
            ) from None
        ready = self._evaluate(
            "(() => ({"
            "title: document.title, href: document.location.href, "
            "ready: document.readyState, visibility: document.visibilityState, "
            "rootReady: Boolean(document.getElementById('root')), "
            "screenReady: Boolean(document.querySelector("
            "'[data-ui=\"operator-waiting-screen\"], "
            "[data-ui=\"operator-control-screen\"]'))"
            "}))()"
        )
        if (
            not isinstance(ready, dict)
            or ready.get("ready") != "complete"
            or str(ready.get("href", "")).rstrip("/")
            != self.operator_ui_url
            or str(ready.get("title", "")).strip() != OPERATOR_UI_TITLE
            or ready.get("visibility") != "visible"
            or ready.get("rootReady") is not True
            or ready.get("screenReady") is not True
        ):
            raise MatrixError(f"visible Robot UI page is not ready: {ready!r}")

    def _install_transport_probe(self) -> None:
        value = self._evaluate(
            "(() => {"
            "if (!window.__camrodOperatorEvidenceInstalled) {"
            "const originalSend = WebSocket.prototype.send;"
            "WebSocket.prototype.send = function(payload) {"
            "const store = window.__camrodOperatorEvidence;"
            "if (store && Array.isArray(store.websocket)) {"
            "let frame = null; try { frame = JSON.parse(String(payload)); } catch (_) {}"
            "store.websocket.push({payload:String(payload), frame:frame});"
            "}"
            "return originalSend.call(this, payload);"
            "};"
            "const originalFetch = window.fetch.bind(window);"
            "window.fetch = async function(input, init) {"
            "const url = String((input && input.url) || input);"
            "const method = String((init && init.method) || "
            "(input && input.method) || 'GET').toUpperCase();"
            "try {"
            "const response = await originalFetch(input, init);"
            "let body = null; try { body = await response.clone().json(); } catch (_) {}"
            "const store = window.__camrodOperatorEvidence;"
            "if (store && Array.isArray(store.http)) "
            "store.http.push({url:url, method:method, status:response.status, "
            "ok:response.ok, body:body});"
            "return response;"
            "} catch (error) {"
            "const store = window.__camrodOperatorEvidence;"
            "if (store && Array.isArray(store.http)) "
            "store.http.push({url:url, method:method, error:String(error)});"
            "throw error;"
            "}"
            "};"
            "window.__camrodOperatorEvidenceInstalled = true;"
            "}"
            "window.__camrodOperatorEvidence = {websocket:[], http:[]};"
            "return {accepted:true, installed:true};"
            "})()"
        )
        self._accepted(value, "transport probe installation")

    def _element(self, selector: str) -> dict[str, Any]:
        literal = json.dumps(selector)
        value = self._evaluate(
            "(() => {"
            f"const selector = {literal};"
            "const matches = Array.from(document.querySelectorAll(selector));"
            "const visible = matches.filter(element => {"
            "const rect = element.getBoundingClientRect();"
            "const style = window.getComputedStyle(element);"
            "return rect.width > 0 && rect.height > 0 && "
            "style.display !== 'none' && style.visibility !== 'hidden' && "
            "Number(style.opacity) !== 0;"
            "});"
            "if (visible.length !== 1) return {count:matches.length, "
            "visibleCount:visible.length};"
            "const element = visible[0]; const rect = element.getBoundingClientRect();"
            "return {count:matches.length, visibleCount:1, disabled:Boolean(element.disabled), "
            "x:rect.left + rect.width / 2, y:rect.top + rect.height / 2, "
            "pressed:element.getAttribute('aria-pressed'), "
            "value:typeof element.value === 'string' ? element.value : null};"
            "})()"
        )
        return dict(value) if isinstance(value, Mapping) else {}

    def _wait_element(
        self, selector: str, description: str, timeout_s: float | None = None
    ) -> dict[str, Any]:
        deadline = time.monotonic() + (
            self.timeout_s if timeout_s is None else timeout_s
        )
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            last = self._element(selector)
            if (
                last.get("visibleCount") == 1
                and last.get("disabled") is not True
            ):
                return last
            time.sleep(0.05)
        raise MatrixError(
            f"Robot UI did not expose one enabled {description} element "
            f"({selector}): {last!r}"
        )

    def _wait_pressed(self, selector: str, description: str) -> None:
        deadline = time.monotonic() + self.timeout_s
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            last = self._element(selector)
            if last.get("visibleCount") == 1 and last.get("pressed") == "true":
                return
            time.sleep(0.05)
        raise MatrixError(
            f"Robot UI did not latch {description}: {last!r}"
        )

    def _click(self, selector: str, description: str) -> dict[str, Any]:
        element = self._wait_element(selector, description)
        x = float(element["x"])
        y = float(element["y"])
        self._call("Input.dispatchMouseEvent", {
            "type": "mouseMoved", "x": x, "y": y,
        })
        self._call("Input.dispatchMouseEvent", {
            "type": "mousePressed", "x": x, "y": y,
            "button": "left", "buttons": 1, "clickCount": 1,
        })
        self._call("Input.dispatchMouseEvent", {
            "type": "mouseReleased", "x": x, "y": y,
            "button": "left", "buttons": 0, "clickCount": 1,
        })
        record = {
            "stage": description,
            "selector": selector,
            "x": round(x, 3),
            "y": round(y, 3),
            "transport": "CDP.Input.dispatchMouseEvent",
        }
        self._interactions.append(record)
        return record

    def _insert_text(self, selector: str, text: str) -> dict[str, Any]:
        element = self._wait_element(selector, "site verification input")
        if element.get("value") not in (None, ""):
            raise MatrixError(
                "Robot UI site verification input was not empty before input: "
                f"{element.get('value')!r}"
            )
        self._click(selector, "site verification input focus")
        self._call("Input.insertText", {"text": text})
        deadline = time.monotonic() + self.timeout_s
        last = None
        while time.monotonic() < deadline:
            last = self._element(selector).get("value")
            if last == text:
                break
            time.sleep(0.05)
        if last != text:
            raise MatrixError(
                f"Robot UI site verification input did not receive {text!r}: "
                f"{last!r}"
            )
        record = {
            "stage": "site verification text",
            "selector": selector,
            "text": text,
            "transport": "CDP.Input.insertText",
        }
        self._interactions.append(record)
        return record

    def _probe(self) -> dict[str, Any]:
        value = self._evaluate(
            "(() => { const value = window.__camrodOperatorEvidence; "
            "return value ? JSON.parse(JSON.stringify(value)) : null; })()"
        )
        return dict(value) if isinstance(value, Mapping) else {}

    def _wait_probe(
        self,
        predicate: Callable[[Mapping[str, Any]], dict[str, Any] | None],
        description: str,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            last = self._probe()
            accepted = predicate(last)
            if accepted is not None:
                return accepted
            time.sleep(0.05)
        raise MatrixError(
            f"Robot UI did not emit the expected {description}: {last!r}"
        )

    @staticmethod
    def _matching_ws_frame(
        probe: Mapping[str, Any], predicate: Callable[[Mapping[str, Any]], bool]
    ) -> dict[str, Any] | None:
        records = probe.get("websocket")
        if not isinstance(records, list):
            return None
        for record in reversed(records):
            if not isinstance(record, Mapping):
                continue
            frame = record.get("frame")
            if isinstance(frame, Mapping) and predicate(frame):
                return dict(frame)
        return None

    def dispatch(
        self, site: str, mission_intent: str = "delivery"
    ) -> dict[str, Any]:
        if mission_intent not in MISSION_INTENTS:
            raise MatrixError(
                f"mission_intent must be one of {MISSION_INTENTS}, "
                f"got {mission_intent!r}"
            )
        self._interactions = []
        self._install_transport_probe()
        waiting = self._element('[data-ui="operator-open-destination"]')
        if waiting.get("visibleCount") == 1:
            self._click(
                '[data-ui="operator-open-destination"]',
                "open destination selection",
            )
        self._wait_element(
            '[data-ui="operator-control-screen"]', "operator control screen"
        )
        self._click(
            f'[data-ui="operator-intent-{mission_intent}"]',
            f"select {mission_intent} intent",
        )
        self._wait_pressed(
            f'[data-ui="operator-intent-{mission_intent}"]',
            f"{mission_intent} intent",
        )
        page = (int(site[1:]) - 1) // 6
        self._click(
            f'[data-ui="operator-site-page-{page}"]',
            f"select site page {page + 1}",
        )
        self._click(f'[data-ui="operator-site-{site}"]', f"select site {site}")
        self._click(
            '[data-ui="operator-site-preview-confirm"]',
            "confirm site preview",
        )
        self._click(
            '[data-ui="operator-move-confirm-yes"]',
            "confirm mission intent",
        )
        self._insert_text('[data-ui="operator-site-code-input"]', site)
        self._click(
            '[data-ui="operator-site-code-confirm"]',
            "confirm typed site code",
        )

        if mission_intent == "delivery":
            frame = self._wait_probe(
                lambda probe: (
                    {"frame": matched}
                    if (matched := self._matching_ws_frame(
                        probe,
                        lambda item: item.get("site") == site
                        and item.get("state") is True,
                    )) is not None
                    else None
                ),
                f"delivery WebSocket frame for {site}",
            )["frame"]
            return {
                "accepted": True,
                "action": "delivery",
                "site": site,
                "frame": frame,
                "source": "ws",
                "transport": "visible_operator_page_websocket_via_cdp_input",
                "interactions": list(self._interactions),
            }

        def recall_request(
            probe: Mapping[str, Any]
        ) -> dict[str, Any] | None:
            records = probe.get("http")
            if not isinstance(records, list):
                return None
            expected_query = f"site={site}&intent=recall"
            for record in reversed(records):
                if not isinstance(record, Mapping):
                    continue
                body = record.get("body")
                if (
                    "/ui/camping_site_recall?" in str(record.get("url", ""))
                    and expected_query in str(record.get("url", ""))
                    and record.get("method") == "POST"
                    and record.get("status") == 200
                    and record.get("ok") is True
                    and isinstance(body, Mapping)
                    and body.get("success") is True
                    and body.get("intent") == "recall"
                ):
                    return dict(record)
            return None

        request_record = self._wait_probe(
            recall_request, f"typed recall HTTP response for {site}"
        )
        return {
            "accepted": True,
            "action": "recall",
            "site": site,
            "intent": "recall",
            "request": request_record,
            "source": "robot_ui:recall",
            "transport": "visible_operator_page_http_via_cdp_input",
            "interactions": list(self._interactions),
        }

    def request_return(self) -> dict[str, Any]:
        self._interactions = []
        # The modal is the explicit arrival acknowledgement shown by the
        # production Robot UI; never substitute the diagnostics REST button.
        self._click(
            '[data-ui="operator-arrival-return-confirm"]',
            "arrival complete and Return",
        )
        frame = self._wait_probe(
            lambda probe: (
                {"frame": matched}
                if (matched := self._matching_ws_frame(
                    probe,
                    lambda item: item.get("usage_complete") is True,
                )) is not None
                else None
            ),
            "arrival usage_complete WebSocket frame",
        )["frame"]
        return {
            "accepted": True,
            "action": "usage_complete",
            "frame": frame,
            "source": "ws:usage_complete",
            "expected_ros_source": "ws:usage_complete:site_exit_first",
            "transport": "visible_operator_page_websocket_via_cdp_input",
            "interactions": list(self._interactions),
        }

    def expected_dispatch_source(self, mission_intent: str) -> str:
        return "robot_ui:recall" if mission_intent == "recall" else "ws"

    @property
    def expected_return_source(self) -> str:
        return "ws:usage_complete:site_exit_first"


def _attr(message: Any, name: str, default: Any = None) -> Any:
    if isinstance(message, Mapping):
        return message.get(name, default)
    return getattr(message, name, default)


def _to_jsonable(message: Any, names: Sequence[str]) -> dict[str, Any]:
    """Copy only stable scalar fields from a ROS message for evidence."""
    result: dict[str, Any] = {}
    for name in names:
        value = _attr(message, name)
        if value is None:
            continue
        if isinstance(value, (str, int, float, bool)):
            result[name] = value
    return result


def collision_event_record(message: Any, *, received_at_utc: str | None = None) -> dict[str, Any]:
    """Convert one CARLA collision event without affecting mission state."""
    header = _attr(message, "header", None)
    stamp = _attr(header, "stamp", None)
    impulse = _attr(message, "normal_impulse", None)

    def finite_or_none(value: Any) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    try:
        other_actor_id = int(_attr(message, "other_actor_id", 0))
    except (TypeError, ValueError):
        other_actor_id = None

    record = {
        "received_at_utc": received_at_utc or utc_now(),
        "other_actor_id": other_actor_id,
        "normal_impulse": {
            axis: finite_or_none(_attr(impulse, axis, None))
            for axis in ("x", "y", "z")
        },
    }
    sec = _attr(stamp, "sec", None)
    nanosec = _attr(stamp, "nanosec", None)
    if isinstance(sec, int) and isinstance(nanosec, int):
        record["timestamp"] = {"sec": sec, "nanosec": nanosec}
    frame = _attr(message, "frame", _attr(header, "frame_id", None))
    if frame not in (None, ""):
        record["frame"] = frame
    return record


def ros_header_stamp_ns(message: Any) -> int | None:
    """Return a validated ROS header timestamp for queue-boundary filtering."""
    header = _attr(message, "header", None)
    stamp = _attr(header, "stamp", None)
    sec = _attr(stamp, "sec", None)
    nanosec = _attr(stamp, "nanosec", None)
    if not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def validate_physical_status(status: Any, expected_actor_id: int | None = None) -> int:
    """Reject every status except the bound, ready physical CARLA backend."""
    required = {
        "ready": True,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": True,
        "independent_wheel_drive_available": True,
        "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
    }
    mismatches = []
    for name, expected in required.items():
        actual = _attr(status, name)
        if actual != expected:
            mismatches.append(f"{name}={actual!r} expected {expected!r}")
    try:
        actor_id = int(_attr(status, "actor_id", 0))
    except (TypeError, ValueError):
        actor_id = 0
    if actor_id <= 0:
        mismatches.append(f"actor_id={actor_id!r} expected >0")
    if expected_actor_id is not None and actor_id != expected_actor_id:
        mismatches.append(f"actor_id changed {expected_actor_id}->{actor_id}")
    for name in (
        "physical_manifest_sha256",
        "production_authorization_sha256",
        "ros_integration_sha256",
        "imported_libcarla_sha256",
    ):
        value = str(_attr(status, name, "")).strip().lower()
        if re.fullmatch(r"[0-9a-f]{64}", value) is None:
            mismatches.append(f"{name} is not a SHA-256")
    try:
        torque_cap = float(_attr(status, "wheel_torque_safety_cap_nm", math.nan))
    except (TypeError, ValueError):
        torque_cap = math.nan
    if not math.isfinite(torque_cap) or abs(torque_cap - 20.0) > 1.0e-6:
        mismatches.append(
            f"wheel_torque_safety_cap_nm={torque_cap!r} expected 20.0"
        )
    if mismatches:
        raise MatrixError("physical CARLA readiness lost: " + ", ".join(mismatches))
    return actor_id


class RosObservation:
    """ROS 2 observer; imports happen only when ``--run`` is selected."""

    def __init__(self, role_name: str, *, expected_actor_id: int | None = None):
        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
            from avg_msgs.msg import (
                AvgPoseStamped,
                AvgServiceState,
                AvgTwist,
                ModuleState,
                MotionOperation,
                PlanningRecallRequest,
                UiDestinationCommand,
            )
            from nav_msgs.msg import Odometry
            from carla_extended_ackermann_msgs.msg import PhysicalFourWheelStatus
        except (ImportError, ModuleNotFoundError) as error:
            raise MatrixError(f"live ROS interfaces unavailable: {error}") from None
        self.rclpy = rclpy
        if int(MotionOperation.RETURN) != RETURN_OPERATION:
            raise MatrixError(
                "avg_msgs/MotionOperation.RETURN no longer matches the "
                f"matrix contract: {int(MotionOperation.RETURN)} != "
                f"{RETURN_OPERATION}"
            )
        rclpy.init(args=[])
        self.node = rclpy.create_node("camrod_carla_camping_site_matrix")
        self.latest: dict[str, Any] = {
            "service_state": None,
            "pose": None,
            "cmd_vel": None,
            "carla_odometry": None,
            "physical": None,
            "site": None,
            "parking": {},
            "drop_zone": None,
            "gate": None,
            "ui_operation_request": None,
            "controller_operation_request": None,
            "destination_request": None,
            "recall_request": None,
        }
        self.expected_actor_id = expected_actor_id
        self._physical_received_monotonic: float | None = None
        self._physical_identity: dict[str, Any] | None = None
        self._fatal_error = ""
        self._service_state_ids: list[int] = []
        self._service_state_names: list[str] = []
        self._site_phases: list[str] = []
        self._parking_phases: list[str] = []
        self._drop_zone_phases: list[str] = []
        self._gate_states: list[str] = []
        self._ui_operation_requests: list[dict[str, Any]] = []
        self._controller_operation_requests: list[dict[str, Any]] = []
        self._destination_requests: list[dict[str, Any]] = []
        self._recall_requests: list[dict[str, Any]] = []
        self._cmd_vel_samples = 0
        self._cmd_vel_max_abs = 0.0
        self._motion_command_observed = False
        self._carla_odom_samples = 0
        self._carla_odom_distance_m = 0.0
        self._last_carla_odom_xy: tuple[float, float] | None = None
        self._latest_odom_stamp_ns: int | None = None
        self._latest_carla_stamp_ns: int | None = None
        self._physical_samples = 0
        self._collision_events: list[dict[str, Any]] = []
        self._collision_total_event_count = 0
        self._collision_stale_queued_event_count = 0
        self._collision_max_normal_impulse_norm = 0.0
        self._collision_cutoff_stamp_ns: int | None = None
        self._collision_topic = f"/carla/{role_name}/collision"
        self._collision_subscriber_created = False
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(
            depth=20, reliability=QoSReliabilityPolicy.RELIABLE
        )
        collision_qos = QoSProfile(
            depth=100, reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.node.create_subscription(AvgServiceState, "/service/state", lambda msg: self._service(msg), qos)
        self.node.create_subscription(AvgPoseStamped, "/localization/pose", lambda msg: self._pose(msg), qos)
        self.node.create_subscription(AvgTwist, "/control/cmd_vel", lambda msg: self._cmd(msg), qos)
        self.node.create_subscription(Odometry, "/carla/ego_vehicle/odometry", lambda msg: self._odom(msg), qos)
        self.node.create_subscription(PhysicalFourWheelStatus, f"/carla/{role_name}/physical_four_wheel_status", lambda msg: self._physical(msg), qos)
        self.node.create_subscription(ModuleState, "/control/camping_site_maneuver_controller/status", lambda msg: self._site(msg), qos)
        self.node.create_subscription(ModuleState, "/parking/reverse_parking_controller/status", lambda msg: self._parking("reverse", msg), qos)
        self.node.create_subscription(ModuleState, "/parking/apriltag_parking_controller/status", lambda msg: self._parking("apriltag", msg), qos)
        self.node.create_subscription(ModuleState, "/control/drop_zone_maneuver_controller/status", lambda msg: self._drop_zone(msg), qos)
        self.node.create_subscription(ModuleState, "/control/cmd_vel_safety_gate/status", lambda msg: self._gate(msg), qos)
        self.node.create_subscription(MotionOperation, "/ui/camping_site_operation_request", lambda msg: self._ui_operation(msg), command_qos)
        self.node.create_subscription(MotionOperation, "/control/camping_site_maneuver_controller/operation", lambda msg: self._controller_operation(msg), command_qos)
        self.node.create_subscription(UiDestinationCommand, "/ui/selected_destination", lambda msg: self._destination_request(msg), command_qos)
        self.node.create_subscription(PlanningRecallRequest, "/planning/state_machine/camping_site_recall", lambda msg: self._recall_request(msg), command_qos)
        try:
            from carla_msgs.msg import CarlaCollisionEvent
        except (ImportError, ModuleNotFoundError):
            CarlaCollisionEvent = None
        if CarlaCollisionEvent is not None:
            self.node.create_subscription(
                CarlaCollisionEvent, self._collision_topic,
                lambda msg: self._collision(msg), collision_qos,
            )
            self._collision_subscriber_created = True

    def _service(self, msg: Any) -> None:
        state = int(_attr(msg, "state", -1))
        state_name = str(_attr(msg, "state_name", "")).strip()
        self.latest["service_state"] = {
            "state": state,
            "state_name": state_name,
            "description": str(_attr(msg, "description", "")),
        }
        self._append_changed(self._service_state_ids, state)
        self._append_changed(self._service_state_names, state_name or str(state))
        if state in FAILURE_STATES and self._site_phases:
            self._fatal_error = (
                f"service lifecycle entered fail-closed state {state_name or state}"
            )

    def _pose(self, msg: Any) -> None:
        pose = _attr(msg, "pose", msg)
        position = _attr(pose, "position", _attr(_attr(pose, "pose", None), "position", None))
        if position is not None:
            values = tuple(
                float(_attr(position, name, math.nan)) for name in ("x", "y", "z")
            )
            if not all(math.isfinite(value) for value in values):
                self._fatal_error = "localization pose contains non-finite values"
                return
            self.latest["pose"] = {
                "x_m": values[0], "y_m": values[1], "z_m": values[2]
            }

    def _cmd(self, msg: Any) -> None:
        linear = _attr(msg, "linear", None); angular = _attr(msg, "angular", None)
        values = (
            float(_attr(linear, "x", math.nan)),
            float(_attr(linear, "y", math.nan)),
            float(_attr(angular, "z", math.nan)),
        )
        if not all(math.isfinite(value) for value in values):
            self._fatal_error = "final cmd_vel contains non-finite values"
            return
        self.latest["cmd_vel"] = {
            "linear_x_mps": values[0],
            "linear_y_mps": values[1],
            "angular_z_radps": values[2],
        }
        magnitude = sum(abs(value) for value in values)
        self._cmd_vel_samples += 1
        self._cmd_vel_max_abs = max(self._cmd_vel_max_abs, magnitude)
        self._motion_command_observed = (
            self._motion_command_observed or magnitude > 0.03
        )

    def _odom(self, msg: Any) -> None:
        stamp_ns = ros_header_stamp_ns(msg)
        if stamp_ns is not None:
            self._latest_odom_stamp_ns = stamp_ns
            self._latest_carla_stamp_ns = max(
                stamp_ns, self._latest_carla_stamp_ns or stamp_ns
            )
        pose = _attr(_attr(msg, "pose", None), "pose", None); twist = _attr(_attr(msg, "twist", None), "twist", None)
        position = _attr(pose, "position", None); linear = _attr(twist, "linear", None)
        if position is not None:
            x = float(_attr(position, "x", math.nan))
            y = float(_attr(position, "y", math.nan))
            z = float(_attr(position, "z", math.nan))
            vx = float(_attr(linear, "x", math.nan))
            vy = float(_attr(linear, "y", math.nan))
            if not all(math.isfinite(value) for value in (x, y, z, vx, vy)):
                self._fatal_error = "CARLA odometry contains non-finite values"
                return
            self.latest["carla_odometry"] = {
                "x_m": x, "y_m": y, "z_m": z,
                "speed_mps": math.hypot(vx, vy),
            }
            if self._last_carla_odom_xy is not None:
                step = math.hypot(
                    x - self._last_carla_odom_xy[0],
                    y - self._last_carla_odom_xy[1],
                )
                if step > 2.0:
                    self._fatal_error = (
                        f"CARLA odometry discontinuity {step:.3f} m; "
                        "teleport/reset is not accepted"
                    )
                    return
                self._carla_odom_distance_m += step
            self._last_carla_odom_xy = (x, y)
            self._carla_odom_samples += 1

    def _physical(self, msg: Any) -> None:
        validate_physical_status(msg, self.expected_actor_id)
        self.latest["physical"] = _to_jsonable(
            msg,
            (
                "actor_id", "ready", "api_version", "motion_backend",
                "physical_gate_accepted", "physical_manifest_sha256",
                "physx_substep_control_verified",
                "physx_substep_threshold_mps", "physx_substep_low_count",
                "physx_substep_high_count", "production_authorization_sha256",
                "ros_integration_sha256", "last_applied_sequence",
                "independent_wheel_drive_available",
                "wheel_torque_safety_cap_nm", "carla_python_origin",
                "imported_libcarla_path", "imported_libcarla_sha256", "reason",
            ),
        )
        self._physical_received_monotonic = time.monotonic()
        self._physical_samples += 1
        if self.expected_actor_id is None:
            self.expected_actor_id = int(self.latest["physical"]["actor_id"])

        identity_keys = (
            "actor_id", "api_version", "motion_backend",
            "physical_manifest_sha256", "production_authorization_sha256",
            "ros_integration_sha256", "wheel_torque_safety_cap_nm",
            "carla_python_origin", "imported_libcarla_path",
            "imported_libcarla_sha256",
        )
        identity = {key: self.latest["physical"].get(key) for key in identity_keys}
        if self._physical_identity is None:
            self._physical_identity = identity
        elif identity != self._physical_identity:
            self._fatal_error = "physical CARLA actor/gate/runtime identity changed"

    def _site(self, msg: Any) -> None:
        self.latest["site"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._site_phases, phase)
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"camping-site controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _parking(self, name: str, msg: Any) -> None:
        self.latest["parking"][name] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._parking_phases, f"{name}:{phase}")
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"{name} parking controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _drop_zone(self, msg: Any) -> None:
        self.latest["drop_zone"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._drop_zone_phases, phase)
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"drop-zone controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _gate(self, msg: Any) -> None:
        self.latest["gate"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        self._append_changed(
            self._gate_states, str(_attr(msg, "operating_state", "")).strip()
        )
        if int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"control safety gate ERROR: {_attr(msg, 'message', '')}"
            )

    def _ui_operation(self, msg: Any) -> None:
        operation = {
            "operation": int(_attr(msg, "operation", -1)),
            "source": str(_attr(msg, "source", "")).strip(),
        }
        self.latest["ui_operation_request"] = operation
        if not self._ui_operation_requests or self._ui_operation_requests[-1] != operation:
            self._ui_operation_requests.append(operation)

    def _controller_operation(self, msg: Any) -> None:
        operation = {
            "operation": int(_attr(msg, "operation", -1)),
            "source": str(_attr(msg, "source", "")).strip(),
        }
        self.latest["controller_operation_request"] = operation
        if (
            not self._controller_operation_requests
            or self._controller_operation_requests[-1] != operation
        ):
            self._controller_operation_requests.append(operation)

    def _destination_request(self, msg: Any) -> None:
        request = {
            "site": str(_attr(msg, "site", "")).strip(),
            "run": bool(_attr(msg, "run", False)),
            "source": str(_attr(msg, "source", "")).strip(),
        }
        self.latest["destination_request"] = request
        if not self._destination_requests or self._destination_requests[-1] != request:
            self._destination_requests.append(request)

    def _recall_request(self, msg: Any) -> None:
        request = {
            "site_name": str(_attr(msg, "site_name", "")).strip(),
            "source": str(_attr(msg, "source", "")).strip(),
        }
        self.latest["recall_request"] = request
        if not self._recall_requests or self._recall_requests[-1] != request:
            self._recall_requests.append(request)

    def _collision(self, msg: Any) -> None:
        """Passively retain an event; this callback has no control side effect."""
        stamp_ns = ros_header_stamp_ns(msg)
        if stamp_ns is not None:
            latest_stamp = getattr(self, "_latest_carla_stamp_ns", None)
            self._latest_carla_stamp_ns = max(
                stamp_ns, latest_stamp or stamp_ns
            )
        if (
            stamp_ns is not None
            and self._collision_cutoff_stamp_ns is not None
            and stamp_ns <= self._collision_cutoff_stamp_ns
        ):
            self._collision_stale_queued_event_count += 1
            return
        self._collision_total_event_count += 1
        record = collision_event_record(msg)
        record["event_index"] = self._collision_total_event_count
        impulse = record["normal_impulse"]
        if all(impulse[axis] is not None for axis in ("x", "y", "z")):
            norm = math.sqrt(sum(impulse[axis] ** 2 for axis in ("x", "y", "z")))
            self._collision_max_normal_impulse_norm = max(
                self._collision_max_normal_impulse_norm, norm
            )
        if len(self._collision_events) >= COLLISION_SAMPLE_LIMIT:
            self._collision_events.pop(COLLISION_FIRST_SAMPLE_LIMIT)
        self._collision_events.append(record)

    @staticmethod
    def _append_changed(values: list[Any], value: Any) -> None:
        if value in (None, ""):
            return
        if not values or values[-1] != value:
            values.append(value)

    def begin_site(self) -> None:
        """Reset per-site accumulators without changing any ROS/CARLA state."""
        self._service_state_ids = []
        self._service_state_names = []
        self._site_phases = []
        self._parking_phases = []
        self._drop_zone_phases = []
        self._gate_states = []
        self._ui_operation_requests = []
        self._controller_operation_requests = []
        self._destination_requests = []
        self._recall_requests = []
        self._cmd_vel_samples = 0
        self._cmd_vel_max_abs = 0.0
        self._motion_command_observed = False
        self._carla_odom_samples = 0
        self._carla_odom_distance_m = 0.0
        self._last_carla_odom_xy = None
        self._physical_samples = 0
        self._reset_collision_evidence()
        self.latest["site"] = None
        self.latest["parking"] = {}
        self.latest["drop_zone"] = None
        self.latest["ui_operation_request"] = None
        self.latest["controller_operation_request"] = None
        self.latest["destination_request"] = None
        self.latest["recall_request"] = None

    def begin_motion_measurement(self) -> None:
        """Start per-site command/odometry metrics at frontend dispatch."""
        self._cmd_vel_samples = 0
        self._cmd_vel_max_abs = 0.0
        self._motion_command_observed = False
        self._carla_odom_samples = 0
        self._carla_odom_distance_m = 0.0
        odometry = self.latest.get("carla_odometry") or {}
        try:
            x = float(odometry["x_m"])
            y = float(odometry["y_m"])
        except (KeyError, TypeError, ValueError):
            self._last_carla_odom_xy = None
        else:
            self._last_carla_odom_xy = (x, y)
        # Evidence is scoped to frontend dispatch through mission completion.
        # Resetting here also discards callbacks drained during live preflight.
        self._reset_collision_evidence()

    def _reset_collision_evidence(self) -> None:
        """Start a bounded collision window at the latest observed CARLA time."""
        self._collision_events = []
        self._collision_total_event_count = 0
        self._collision_stale_queued_event_count = 0
        self._collision_max_normal_impulse_norm = 0.0
        stamps = (
            getattr(self, "_latest_odom_stamp_ns", None),
            getattr(self, "_latest_carla_stamp_ns", None),
        )
        valid_stamps = [stamp for stamp in stamps if stamp is not None]
        self._collision_cutoff_stamp_ns = (
            max(valid_stamps) if valid_stamps else None
        )

    def spin_once(self, timeout_s: float = 0.2) -> None:
        try:
            self.rclpy.spin_once(self.node, timeout_sec=timeout_s)
        except Exception as error:
            raise MatrixError(f"ROS observation failed closed: {error}") from None
        if self._fatal_error:
            raise MatrixError(self._fatal_error)
        physical = self.latest.get("physical")
        if physical is not None:
            validate_physical_status(physical, self.expected_actor_id)
            if (self._physical_received_monotonic is None or
                    time.monotonic() - self._physical_received_monotonic > 1.5):
                raise MatrixError("physical CARLA readiness heartbeat is stale")

    def drain_final_callbacks(self) -> None:
        """Drain callbacks queued with the terminal state before snapshotting."""
        deadline = time.monotonic() + FINAL_OBSERVATION_DRAIN_SECONDS
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            self.spin_once(min(0.01, remaining))

    def snapshot(self) -> dict[str, Any]:
        document = dict(self.latest)
        document["sequences"] = {
            "service_state_ids": list(self._service_state_ids),
            "service_state_names": list(self._service_state_names),
            "site_phases": list(self._site_phases),
            "parking_phases": list(self._parking_phases),
            "drop_zone_phases": list(self._drop_zone_phases),
            "gate_states": list(self._gate_states),
            "ui_operation_requests": list(self._ui_operation_requests),
            "controller_operation_requests": list(
                self._controller_operation_requests
            ),
            "destination_requests": list(self._destination_requests),
            "recall_requests": list(self._recall_requests),
        }
        document["motion_metrics"] = {
            "cmd_vel_samples": self._cmd_vel_samples,
            "cmd_vel_max_component_sum": round(self._cmd_vel_max_abs, 6),
            "motion_command_observed": self._motion_command_observed,
            "carla_odometry_samples": self._carla_odom_samples,
            "carla_odometry_distance_m": round(
                self._carla_odom_distance_m, 6
            ),
            "physical_status_samples": self._physical_samples,
        }
        document["physical_identity"] = dict(self._physical_identity or {})
        try:
            publisher_count = int(self.node.count_publishers(self._collision_topic))
        except (AttributeError, RuntimeError):
            publisher_count = None
        document["collision_evidence"] = {
            "topic": self._collision_topic,
            "subscriber_created": self._collision_subscriber_created,
            "publisher_count": publisher_count,
            "publisher_discovered": (
                publisher_count > 0 if publisher_count is not None else None
            ),
            "subscriber_and_publisher_discovered": (
                self._collision_subscriber_created and publisher_count > 0
                if publisher_count is not None else None
            ),
            "event_count": self._collision_total_event_count,
            "sampled_event_count": len(self._collision_events),
            "omitted_event_count": max(
                0, self._collision_total_event_count - len(self._collision_events)
            ),
            "stale_queued_event_count": self._collision_stale_queued_event_count,
            "maximum_normal_impulse_norm": round(
                self._collision_max_normal_impulse_norm, 6
            ),
            "sample_policy": {
                "maximum_samples": COLLISION_SAMPLE_LIMIT,
                "preserved_first_samples": COLLISION_FIRST_SAMPLE_LIMIT,
                "preserved_recent_samples": (
                    COLLISION_SAMPLE_LIMIT - COLLISION_FIRST_SAMPLE_LIMIT
                ),
            },
            "events": list(self._collision_events),
        }
        return json.loads(json.dumps(document, allow_nan=False))

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def _default_paths() -> tuple[Path, Path, Path]:
    src = Path(
        os.environ.get("CAMROD_SRC_ROOT") or Path(__file__).resolve().parents[2]
    )
    work = Path(os.environ.get("RANGER_WORK_ROOT") or "/tmp/camrod-work")
    output = work / "evidence" / "camping_site_matrix" / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    return (
        src / "camrod_planning/config/camping_sites.yaml",
        src / "camrod_bringup/config/map/drop_zones.yaml",
        output,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--run", action="store_true", help="enable live UI/ROS execution; without this flag only a plan is written")
    parser.add_argument("--sites", help="comma-separated subset such as B1,B12 (default: all B1..B13)")
    parser.add_argument("--camping-sites-yaml", type=Path)
    parser.add_argument("--drop-zones-yaml", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--sensor-audit-report",
        type=Path,
        help="fresh PASS JSON from carla_sensor_source_audit --json (required with --run)",
    )
    parser.add_argument(
        "--runtime-profile-report",
        type=Path,
        help=(
            "fresh PASS JSON from audit_runtime_profile.py "
            "(required with --run)"
        ),
    )
    parser.add_argument("--ui-url", default=os.environ.get("CAMROD_UI_URL", "http://127.0.0.1:8010"))
    parser.add_argument(
        "--return-authority",
        choices=("operator_rest", "operator_browser", "guest_browser"),
        default="operator_rest",
        help="frontend authority used for dispatch and the Return request",
    )
    parser.add_argument(
        "--mission-intent",
        choices=MISSION_INTENTS,
        default="delivery",
        help=(
            "delivery enters the authored campsite; recall uses the latest-"
            "develop roadside pickup contract"
        ),
    )
    parser.add_argument(
        "--guest-cdp-url",
        default=os.environ.get("CAMROD_GUEST_CDP_URL", "http://127.0.0.1:9223"),
        help="local Chrome DevTools endpoint for a visible Guest UI browser",
    )
    parser.add_argument(
        "--guest-ui-url",
        default=os.environ.get("CAMROD_GUEST_UI_URL", "http://127.0.0.1:8012"),
        help="Guest UI page URL that must own the browser WebSocket",
    )
    parser.add_argument(
        "--operator-cdp-url",
        default=os.environ.get(
            "CAMROD_OPERATOR_CDP_URL", "http://127.0.0.1:9224"
        ),
        help="local Chrome DevTools endpoint for the visible Robot UI browser",
    )
    parser.add_argument("--role-name", default=os.environ.get("CARLA_ROLE_NAME", "ego_vehicle"))
    parser.add_argument(
        "--expected-actor-id",
        type=int,
        help="preflight- and physical-bridge-bound live Ranger actor id",
    )
    parser.add_argument("--phase-timeout-s", type=float, default=900.0)
    parser.add_argument(
        "--start-drop-zone-tolerance-m",
        type=float,
        default=5.0,
        help=(
            "initial lane-connected spawn distance from the Drop Zone center; "
            "kept separate from the stricter final parking tolerance"
        ),
    )
    parser.add_argument("--drop-zone-tolerance-m", type=float, default=3.0)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.phase_timeout_s) or args.phase_timeout_s <= 0.0 or args.phase_timeout_s > 3600.0:
        raise MatrixError("--phase-timeout-s must be in (0, 3600]")
    if not math.isfinite(args.drop_zone_tolerance_m) or args.drop_zone_tolerance_m <= 0.0:
        raise MatrixError("--drop-zone-tolerance-m must be positive")
    if (
        not math.isfinite(args.start_drop_zone_tolerance_m)
        or args.start_drop_zone_tolerance_m <= 0.0
    ):
        raise MatrixError("--start-drop-zone-tolerance-m must be positive")
    if not args.role_name.strip():
        raise MatrixError("--role-name must not be empty")
    if args.run and args.sensor_audit_report is None:
        raise MatrixError("--run requires --sensor-audit-report")
    if args.run and args.runtime_profile_report is None:
        raise MatrixError("--run requires --runtime-profile-report")
    if args.run and (
        type(args.expected_actor_id) is not int or args.expected_actor_id <= 0
    ):
        raise MatrixError("--run requires a positive --expected-actor-id")
    if (
        args.return_authority == "guest_browser"
        and args.mission_intent != "recall"
    ):
        raise MatrixError(
            "--return-authority guest_browser requires --mission-intent recall"
        )


def ui_authority_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return the exact frontend authority and motion path for the report."""
    if args.return_authority == "guest_browser":
        if args.mission_intent != "recall":
            raise MatrixError(
                "--return-authority guest_browser requires "
                "--mission-intent recall"
            )
        return {
            "mission_intent": "recall",
            "return_authority": "guest_browser",
            "expected_arrival_state": GUEST_LOADING_WAIT,
            "required_service_state_ids": list(
                required_service_state_ids("recall")
            ),
            "expected_return_operation": RETURN_OPERATION,
            "expected_return_source": "guest:usage_complete",
            "ui_endpoints": {
                "dispatch": (
                    "visible Guest page selectSite/openConfirm/confirmNavigate "
                    "-> page-owned WebSocket action=navigate"
                ),
                "return": (
                    "visible Guest page sendUsageComplete -> page-owned "
                    "WebSocket action=usage_complete"
                ),
                "stop": "POST /ui/stop (operator safety endpoint only)",
            },
            "motion_path": (
                "visible Guest browser page -> page-owned Guest WebSocket "
                "navigate/usage_complete -> camrod_ui_guest ROS commands "
                "-> typed PlanningRecallRequest / roadside wait "
                "(RETURN source guest:usage_complete) -> CAMROD planning -> "
                "CAMROD control -> CARLA physical 4WS bridge"
            ),
        }
    if args.return_authority == "operator_browser":
        recall = args.mission_intent == "recall"
        return {
            "mission_intent": args.mission_intent,
            "return_authority": "operator_browser",
            "expected_arrival_state": (
                GUEST_LOADING_WAIT if recall else WAITING_FOR_RETURN_REQUEST
            ),
            "required_service_state_ids": list(
                required_service_state_ids(args.mission_intent)
            ),
            "expected_dispatch_source": (
                "robot_ui:recall" if recall else "ws"
            ),
            "expected_return_operation": RETURN_OPERATION,
            "expected_return_source": "ws:usage_complete:site_exit_first",
            "ui_endpoints": {
                "dispatch": (
                    "visible Robot UI data-ui controls -> CDP pointer/text "
                    "input -> page-owned "
                    + (
                        "POST /ui/camping_site_recall"
                        if recall
                        else "WebSocket {site,state:true}"
                    )
                ),
                "return": (
                    "visible arrival acknowledgement -> CDP pointer input -> "
                    "page-owned WebSocket {usage_complete:true}"
                ),
                "stop": "POST /ui/stop (operator safety endpoint only)",
            },
            "motion_path": (
                "visible Robot UI browser pointer/text events -> production "
                "Robot UI transport -> CAMROD planning -> CAMROD control -> "
                "CARLA physical 4WS bridge"
            ),
        }
    if args.mission_intent == "recall":
        return {
            "mission_intent": "recall",
            "return_authority": "operator_rest",
            "expected_arrival_state": GUEST_LOADING_WAIT,
            "required_service_state_ids": list(
                required_service_state_ids("recall")
            ),
            "expected_return_operation": None,
            "expected_return_source": "",
            "ui_endpoints": {
                "dispatch": (
                    "POST /ui/camping_site_recall?site=Bx&intent=recall"
                ),
                "return": "POST /ui/manual_return",
                "stop": "POST /ui/stop",
            },
            "motion_path": (
                "Robot UI recall REST -> typed PlanningRecallRequest -> "
                "roadside GUEST_LOADING_WAIT -> operator Return REST -> "
                "CAMROD planning -> CAMROD control -> CARLA physical 4WS bridge"
            ),
        }
    return {
        "mission_intent": "delivery",
        "return_authority": "operator_rest",
        "expected_arrival_state": WAITING_FOR_RETURN_REQUEST,
        "required_service_state_ids": list(
            required_service_state_ids("delivery")
        ),
        "expected_return_operation": None,
        "expected_return_source": "",
        "ui_endpoints": {
            "dispatch": "POST /ui/destination?site=Bx&run=true",
            "return": "POST /ui/manual_return",
            "stop": "POST /ui/stop",
        },
        "motion_path": (
            "UI REST -> CAMROD planning -> CAMROD control -> "
            "CARLA physical 4WS bridge"
        ),
    }


def run_matrix(args: argparse.Namespace, sites: Mapping[str, Site], drop_zone: DropZone, selected: Sequence[str], report: dict[str, Any]) -> int:
    """Execute UI-driven site missions and preserve every milestone."""
    if args.return_authority == "guest_browser":
        client: Any = GuestBrowserClient(
            args.guest_cdp_url,
            args.guest_ui_url,
            args.ui_url,
        )
    elif args.return_authority == "operator_browser":
        client = OperatorBrowserClient(
            args.operator_cdp_url,
            args.ui_url,
        )
    else:
        client = UIClient(args.ui_url)
    observer: RosObservation | None = None
    authority = ui_authority_contract(args)
    try:
        observer = RosObservation(
            args.role_name, expected_actor_id=args.expected_actor_id
        )
        report["scope"].update({
            "mission_intent": authority["mission_intent"],
            "return_authority": authority["return_authority"],
            "expected_arrival_state": authority["expected_arrival_state"],
            "required_service_state_ids": authority[
                "required_service_state_ids"
            ],
            "expected_return_operation": authority["expected_return_operation"],
            "expected_return_source": authority["expected_return_source"],
            "expected_dispatch_source": authority.get(
                "expected_dispatch_source", ""
            ),
            "ui_endpoints": authority["ui_endpoints"],
        })
        report["status"] = "RUNNING"
        report["runtime"] = {
            "started_at_utc": utc_now(),
            "profile": report["runtime_profile_audit"]["profile"],
            "ui_url": args.ui_url,
            "carla_town": os.environ.get("CARLA_TOWN", ""),
            "carla_ue_map": os.environ.get("CARLA_UE_MAP", ""),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "phase_timeout_s": args.phase_timeout_s,
            "start_drop_zone_tolerance_m": args.start_drop_zone_tolerance_m,
            "drop_zone_tolerance_m": args.drop_zone_tolerance_m,
            "motion_path": authority["motion_path"],
            "mission_intent": authority["mission_intent"],
            "return_authority": authority["return_authority"],
            "expected_arrival_state": authority["expected_arrival_state"],
            "required_service_state_ids": authority[
                "required_service_state_ids"
            ],
            "expected_return_operation": authority["expected_return_operation"],
            "expected_return_source": authority["expected_return_source"],
            "expected_dispatch_source": authority.get(
                "expected_dispatch_source", ""
            ),
            "guest_ui_url": (
                args.guest_ui_url
                if args.return_authority == "guest_browser"
                else ""
            ),
            "guest_cdp_url": (
                args.guest_cdp_url
                if args.return_authority == "guest_browser"
                else ""
            ),
            "operator_cdp_url": (
                args.operator_cdp_url
                if args.return_authority == "operator_browser"
                else ""
            ),
            "per_site_metric_contract": {
                "elapsed_s": "frontend dispatch attempt through final PASS/FAIL",
                "milestones[].elapsed_s": (
                    "live per-site preflight start through milestone"
                ),
                "outbound_duration_s": (
                    "frontend dispatch attempt through frontend return request"
                ),
                "return_duration_s": (
                    "frontend return request attempt through PARKED+CHARGING"
                ),
                "outbound_distance_m": (
                    "CARLA odometry accumulated from dispatch to WAIT_RETURN"
                ),
                "return_distance_m": (
                    "CARLA odometry accumulated from return request to final"
                ),
                "total_odom_distance_m": (
                    "CARLA odometry accumulated from dispatch to final"
                ),
            },
        }
        write_json_atomic(args.output, report)
    except BaseException:
        if observer is not None:
            try:
                observer.close()
            except Exception:
                pass
        client.close()
        raise

    # The guarded construction above guarantees this for type checkers and
    # for the runtime code below.
    assert observer is not None

    def progress(message: str) -> None:
        print(f"[camping-matrix] {message}", flush=True)

    started_monotonic: dict[str, float] = {}
    dispatch_monotonic: dict[str, float] = {}
    arrival_monotonic: dict[str, float] = {}
    return_request_monotonic: dict[str, float] = {}
    dispatch_distance_m: dict[str, float] = {}
    arrival_distance_m: dict[str, float] = {}
    return_request_distance_m: dict[str, float] = {}

    def checkpoint(result: dict[str, Any], event: str) -> None:
        elapsed = round(time.monotonic() - started_monotonic.get(result["site"], time.monotonic()), 3)
        result.setdefault("milestones", []).append({"at_utc": utc_now(), "elapsed_s": elapsed, "event": event, "observation": observer.snapshot()})
        write_json_atomic(args.output, report)
        progress(f"{result['site']}: {event}")

    def wait_until(predicate: Callable[[dict[str, Any]], bool], timeout_s: float, label: str, result: dict[str, Any]) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            observer.spin_once(min(0.2, max(0.01, deadline - time.monotonic())))
            snapshot = observer.snapshot()
            if predicate(snapshot):
                checkpoint(result, label)
                return
        raise MatrixError(f"{result['site']}: timeout waiting for {label}")

    def copy_final_observation(result: dict[str, Any], snapshot: Mapping[str, Any]) -> None:
        result["final_service_state"] = snapshot.get("service_state") or {}
        result["final_parking_status"] = snapshot.get("parking") or {}
        result["final_drop_zone_status"] = snapshot.get("drop_zone") or {}
        result["final_gate_status"] = snapshot.get("gate") or {}
        result["final_physical_four_wheel_status"] = snapshot.get("physical") or {}
        result["final_localization_pose"] = snapshot.get("pose") or {}
        result["final_carla_odometry"] = snapshot.get("carla_odometry") or {}
        sequences = snapshot.get("sequences") or {}
        result["site_phase_sequence"] = list(sequences.get("site_phases") or [])
        result["service_state_sequence"] = list(
            sequences.get("service_state_names") or []
        )
        result["drop_zone_phase_sequence"] = list(
            sequences.get("drop_zone_phases") or []
        )
        result["parking_phase_sequence"] = list(
            sequences.get("parking_phases") or []
        )
        result["ui_operation_request_sequence"] = list(
            sequences.get("ui_operation_requests") or []
        )
        result["controller_operation_request_sequence"] = list(
            sequences.get("controller_operation_requests") or []
        )
        result["destination_request_sequence"] = list(
            sequences.get("destination_requests") or []
        )
        result["recall_request_sequence"] = list(
            sequences.get("recall_requests") or []
        )
        result["motion_metrics"] = dict(snapshot.get("motion_metrics") or {})
        result["collision_evidence"] = dict(
            snapshot.get("collision_evidence") or {}
        )
        key = str(result.get("site", ""))
        current_distance = float(
            result["motion_metrics"].get("carla_odometry_distance_m", 0.0)
        )
        if key in dispatch_distance_m:
            total_distance = max(
                0.0, current_distance - dispatch_distance_m[key]
            )
            result["total_odom_distance_m"] = round(total_distance, 6)
            outbound_end_distance = return_request_distance_m.get(
                key, current_distance
            )
            result["outbound_distance_m"] = round(
                max(
                    0.0,
                    outbound_end_distance - dispatch_distance_m[key],
                ),
                6,
            )
            result["return_distance_m"] = round(
                max(
                    0.0,
                    current_distance - outbound_end_distance,
                ),
                6,
            )
        now = time.monotonic()
        if key in dispatch_monotonic:
            outbound_end = return_request_monotonic.get(
                key, now
            )
            result["outbound_duration_s"] = round(
                outbound_end - dispatch_monotonic[key], 3
            )
            result["return_duration_s"] = round(
                now - outbound_end, 3
            )
        update_motion_command_scope(report, snapshot)

    try:
        for index, key in enumerate(selected):
            result = next(item for item in report["sites"] if item["site"] == key)
            site = sites[key]
            result["status"] = "RUNNING"
            result["started_at_utc"] = utc_now()
            started_monotonic[key] = time.monotonic()
            observer.begin_site()
            write_json_atomic(args.output, report)
            progress(
                f"{key} ({index + 1}/{len(selected)}): waiting for live preflight"
            )
            wait_until(
                lambda snap: all(
                    snap.get(name) is not None
                    for name in (
                        "physical", "pose", "carla_odometry",
                        "site", "drop_zone", "gate",
                    )
                ),
                20.0,
                "live dispatch preflight",
                result,
            )
            preflight = observer.snapshot()
            actor_id = observer.expected_actor_id
            if actor_id is None:
                raise MatrixError(f"{key}: no bound CARLA actor readiness")
            service_state = preflight.get("service_state") or {}
            if service_state:
                start_state = int(service_state.get("state", -1))
                if start_state not in STATIONARY_DROP_ZONE_STATES:
                    raise MatrixError(
                        f"{key}: dispatch must start at a stationary Drop Zone "
                        f"state; got {start_state}"
                    )
                result["start_service_state_source"] = "observed_ros_topic"
            elif index == 0:
                # /service/state is volatile and event-driven.  A runner that
                # attaches after bringup can legitimately miss the initial
                # DROP_ZONE_WAIT sample.  Do not invent a state: bind the cold
                # start to independent live evidence instead (STANDBY gate,
                # stationary CARLA odometry, accepted physical actor, and the
                # checked-in lane-connected Drop Zone-front spawn).  Once the
                # UI dispatches, every service transition is observed normally.
                gate_state = str(
                    (preflight.get("gate") or {}).get("operating_state", "")
                ).strip()
                speed_mps = float(
                    (preflight.get("carla_odometry") or {}).get(
                        "speed_mps", math.inf
                    )
                )
                if gate_state not in {"STANDBY", "CHARGING"}:
                    raise MatrixError(
                        f"{key}: unpublished initial service state requires a "
                        f"stationary control gate; got {gate_state!r}"
                    )
                if not math.isfinite(speed_mps) or speed_mps > 0.05:
                    raise MatrixError(
                        f"{key}: unpublished initial service state requires a "
                        f"stationary actor; speed={speed_mps!r} m/s"
                    )
                result["start_service_state_source"] = (
                    "cold_start_volatile_topic_unpublished"
                )
            else:
                raise MatrixError(
                    f"{key}: service state disappeared after an earlier site"
                )
            start_error = pose_error_m(preflight.get("pose") or {}, drop_zone)
            if (
                start_error is None
                or start_error > args.start_drop_zone_tolerance_m
            ):
                raise MatrixError(
                    f"{key}: start pose is not at the accepted Drop Zone-front "
                    f"spawn: error={start_error!r} m"
                )
            result["start_localization_pose"] = preflight.get("pose") or {}
            result["start_drop_zone_error_m"] = start_error
            result["actor_id"] = actor_id
            progress(
                f"{key} ({index + 1}/{len(selected)}): dispatching via "
                f"{authority['return_authority']} "
                f"intent={authority['mission_intent']}"
            )
            observer.begin_motion_measurement()
            dispatch_snapshot = observer.snapshot()
            dispatch_distance_m[key] = float(
                (dispatch_snapshot.get("motion_metrics") or {}).get(
                    "carla_odometry_distance_m", 0.0
                )
            )
            dispatch_monotonic[key] = time.monotonic()
            result["dispatch_started_at_utc"] = utc_now()
            response = client.dispatch(key, args.mission_intent)
            result["dispatch_response"] = response
            checkpoint(result, "dispatch accepted")
            expected_dispatch_source = str(
                authority.get("expected_dispatch_source", "")
            )
            if expected_dispatch_source:
                wait_until(
                    lambda snap: dispatch_source_observed(
                        snap,
                        site=key,
                        mission_intent=args.mission_intent,
                        expected_source=expected_dispatch_source,
                    ),
                    10.0,
                    (
                        f"dispatch source {expected_dispatch_source} "
                        "observed at ROS boundary"
                    ),
                    result,
                )
                result["dispatch_source_verified"] = True
            service_mode = effective_service_mode(site, args.mission_intent)
            expected_phases = SITE_PHASES[service_mode]
            expected_arrival_state = int(authority["expected_arrival_state"])
            expected_arrival_name = (
                "GUEST_LOADING_WAIT"
                if expected_arrival_state == GUEST_LOADING_WAIT
                else "WAITING_FOR_RETURN_REQUEST"
            )
            wait_until(
                lambda snap: arrival_ready_for_return(
                    snap,
                    expected_phases,
                    expected_service_state=expected_arrival_state,
                ),
                args.phase_timeout_s,
                f"{expected_arrival_name} with ordered WAIT_RETURN phase",
                result,
            )
            arrival = observer.snapshot()
            arrival_monotonic[key] = time.monotonic()
            arrival_distance_m[key] = float(
                (arrival.get("motion_metrics") or {}).get(
                    "carla_odometry_distance_m", 0.0
                )
            )
            result["arrival_localization_pose"] = arrival.get("pose") or {}
            arrival_phases = list(
                (arrival.get("sequences") or {}).get("site_phases") or []
            )
            wait_return_index = expected_phases.index("WAIT_RETURN") + 1
            if not contains_ordered_subsequence(
                arrival_phases, expected_phases[:wait_return_index]
            ):
                raise MatrixError(
                    f"{key}: arrival phase sequence is incomplete for "
                    f"{service_mode}: {arrival_phases}"
                )
            return_request_monotonic[key] = time.monotonic()
            return_request_distance_m[key] = arrival_distance_m[key]
            result["return_response"] = client.request_return()
            checkpoint(result, "return request accepted")
            if client.expected_return_source:
                wait_until(
                    lambda snap: return_source_observed(
                        snap, client.expected_return_source, key
                    ),
                    10.0,
                    f"RETURN source {client.expected_return_source} observed",
                    result,
                )
            wait_until(
                lambda snap: WAITING_FOR_CHARGING
                in ((snap.get("sequences") or {}).get("service_state_ids") or []),
                args.phase_timeout_s,
                "WAITING_FOR_CHARGING at Drop Zone",
                result,
            )
            wait_until(
                lambda snap: (
                    (snap.get("service_state") or {}).get("state") == CHARGING
                    and any(
                        phase.endswith(":PARKED")
                        for phase in (
                            (snap.get("sequences") or {}).get(
                                "parking_phases"
                            ) or []
                        )
                    )
                ),
                args.phase_timeout_s,
                "PARKED and CHARGING",
                result,
            )
            # PARKED/CHARGING and a same-frame collision can be ready in the
            # executor together.  wait_until() returns after the terminal
            # state callback, so drain the remaining ready callbacks before
            # freezing per-site evidence.
            observer.drain_final_callbacks()
            final_snapshot = observer.snapshot()
            copy_final_observation(result, final_snapshot)
            final_monotonic = time.monotonic()
            final_distance = float(
                (final_snapshot.get("motion_metrics") or {}).get(
                    "carla_odometry_distance_m", 0.0
                )
            )
            result.update(mission_segment_metrics(
                dispatch_distance_m=dispatch_distance_m[key],
                arrival_distance_m=arrival_distance_m[key],
                return_request_distance_m=return_request_distance_m[key],
                final_distance_m=final_distance,
                dispatch_monotonic=dispatch_monotonic[key],
                arrival_monotonic=arrival_monotonic[key],
                return_request_monotonic=return_request_monotonic[key],
                final_monotonic=final_monotonic,
            ))
            missing_observations = [
                name for name in (
                    "pose", "carla_odometry", "cmd_vel", "site", "parking",
                    "drop_zone", "gate", "physical",
                )
                if not final_snapshot.get(name)
            ]
            if missing_observations:
                raise MatrixError(
                    f"{key}: required live observations missing at charging: "
                    f"{', '.join(missing_observations)}"
                )
            error = pose_error_m(result["final_localization_pose"], drop_zone)
            result["drop_zone_error_m"] = error
            sequences = final_snapshot.get("sequences") or {}
            site_phases = list(sequences.get("site_phases") or [])
            parking_phases = list(sequences.get("parking_phases") or [])
            drop_phases = list(sequences.get("drop_zone_phases") or [])
            service_ids = list(sequences.get("service_state_ids") or [])
            motion = final_snapshot.get("motion_metrics") or {}
            if not contains_ordered_subsequence(site_phases, expected_phases):
                raise MatrixError(
                    f"{key}: complete site phase sequence is invalid for "
                    f"{service_mode}: {site_phases}"
                )
            if "ALIGN_PARKING_YAW" not in drop_phases:
                raise MatrixError(
                    f"{key}: Drop Zone ALIGN_PARKING_YAW was not observed: "
                    f"{drop_phases}"
                )
            required_states = authority["required_service_state_ids"]
            if not contains_ordered_subsequence(service_ids, required_states):
                raise MatrixError(
                    f"{key}: {authority['mission_intent']} service sequence "
                    f"is incomplete; required={required_states}, "
                    f"observed={service_ids}"
                )
            parked = any(phase.endswith(":PARKED") for phase in parking_phases)
            result["parking_confirmed"] = parked
            result["charging_confirmed"] = bool(
                (final_snapshot.get("service_state") or {}).get("state")
                == CHARGING
            )
            if not bool(motion.get("motion_command_observed")):
                raise MatrixError(f"{key}: no nonzero final cmd_vel was observed")
            if int(motion.get("cmd_vel_samples", 0)) < 2:
                raise MatrixError(f"{key}: insufficient final cmd_vel samples")
            if int(motion.get("carla_odometry_samples", 0)) < 2:
                raise MatrixError(f"{key}: insufficient CARLA odometry samples")
            if float(motion.get("carla_odometry_distance_m", 0.0)) < 1.0:
                raise MatrixError(f"{key}: CARLA actor movement was below 1.0 m")
            final_speed = float(
                (final_snapshot.get("carla_odometry") or {}).get(
                    "speed_mps", math.nan
                )
            )
            if not math.isfinite(final_speed) or final_speed > 0.05:
                raise MatrixError(
                    f"{key}: final CARLA speed {final_speed!r} m/s is not stopped"
                )
            if error is None or error > args.drop_zone_tolerance_m:
                raise MatrixError(f"{key}: drop-zone final error {error!r} m exceeds {args.drop_zone_tolerance_m:g} m")
            result["status"] = "PASS"
            result["finished_at_utc"] = utc_now()
            result["elapsed_s"] = round(
                final_monotonic - dispatch_monotonic[key], 3
            )
            checkpoint(result, f"PASS; return error={error:.3f} m")
        report["status"] = "PASS"
        report["runtime"]["finished_at_utc"] = utc_now()
        write_json_atomic(args.output, report)
        progress(f"matrix PASS: {len(selected)} site(s)")
        return 0
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["runtime"]["finished_at_utc"] = utc_now()
        for item in report["sites"]:
            if item["status"] == "RUNNING":
                item["status"] = "INTERRUPTED"
                item["finished_at_utc"] = utc_now()
                item["elapsed_s"] = (
                    round(
                        time.monotonic()
                        - dispatch_monotonic[item["site"]],
                        3,
                    )
                    if item["site"] in dispatch_monotonic
                    else 0.0
                )
                copy_final_observation(item, observer.snapshot())
        write_json_atomic(args.output, report)
        try:
            client.stop()
        except MatrixError:
            pass
        raise
    except Exception as error:
        report["status"] = "FAIL"
        report["failure_reason"] = str(error)
        report["runtime"]["finished_at_utc"] = utc_now()
        for item in report["sites"]:
            if item["status"] == "RUNNING":
                item["status"] = "FAIL"
                item["failure_reason"] = str(error)
                item["finished_at_utc"] = utc_now()
                item["elapsed_s"] = (
                    round(
                        time.monotonic()
                        - dispatch_monotonic[item["site"]],
                        3,
                    )
                    if item["site"] in dispatch_monotonic
                    else 0.0
                )
                copy_final_observation(item, observer.snapshot())
            elif item["status"] == "NOT_ATTEMPTED":
                item["failure_reason"] = "not attempted after earlier site failure"
        write_json_atomic(args.output, report)
        progress(f"FAIL CLOSED: {error}")
        try:
            client.stop()
        except MatrixError as stop_error:
            report["stop_error"] = str(stop_error)
            write_json_atomic(args.output, report)
        return 1
    finally:
        try:
            observer.close()
        finally:
            client.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        _validate_args(args)
        default_sites, default_drop_zones, default_output = _default_paths()
        camping_path = (args.camping_sites_yaml or default_sites).expanduser().resolve()
        drop_path = (args.drop_zones_yaml or default_drop_zones).expanduser().resolve()
        output_explicit = args.output is not None
        args.output = (args.output or default_output).expanduser().resolve()
        all_sites = load_sites(camping_path)
        selected = parse_site_selection(args.sites)
        missing = [key for key in selected if key not in all_sites]
        if missing:
            raise MatrixError(f"selected sites are missing from active YAML: {missing}")
        drop_zone = load_drop_zone(drop_path)
        report = build_plan(
            all_sites,
            selected,
            drop_zone,
            output=args.output,
            config_paths={
                "camping_sites": camping_path,
                "drop_zones": drop_path,
            },
            role_name=args.role_name,
            mission_intent=args.mission_intent,
        )
        authority = ui_authority_contract(args)
        report["scope"].update({
            "mission_intent": authority["mission_intent"],
            "return_authority": authority["return_authority"],
            "expected_arrival_state": authority["expected_arrival_state"],
            "required_service_state_ids": authority[
                "required_service_state_ids"
            ],
            "expected_return_operation": authority[
                "expected_return_operation"
            ],
            "expected_return_source": authority["expected_return_source"],
            "expected_dispatch_source": authority.get(
                "expected_dispatch_source", ""
            ),
            "ui_endpoints": authority["ui_endpoints"],
        })
        if args.run:
            assert args.sensor_audit_report is not None
            assert args.runtime_profile_report is not None
            sensor_audit_path = args.sensor_audit_report.expanduser().resolve()
            report["sensor_source_audit"] = load_sensor_audit(sensor_audit_path)
            runtime_profile_path = args.runtime_profile_report.expanduser().resolve()
            report["runtime_profile_audit"] = load_runtime_profile_audit(
                runtime_profile_path,
                expected_carla_map=os.environ.get("CARLA_UE_MAP", ""),
                expected_carla_town=os.environ.get("CARLA_TOWN", ""),
                expected_lanelet_map=Path(
                    os.environ.get("CAMROD_DEVELOP_LANELET_MAP", "")
                ),
                expected_actor_id=args.expected_actor_id,
                expected_role_name=args.role_name,
            )
        if args.run or output_explicit:
            write_json_atomic(args.output, report, create_only=True)
        print(f"[camping-matrix] {'RUN' if args.run else 'PLAN_ONLY'}: {', '.join(selected)}")
        print(f"[camping-matrix] active drop zone={drop_zone.source_id} ({drop_zone.x_m:.4f}, {drop_zone.y_m:.4f}); report={args.output}")
        if not args.run:
            if output_explicit:
                print(f"[camping-matrix] explicit plan report written: {args.output}")
            else:
                print("[camping-matrix] no file or directory was created")
            print("[camping-matrix] no ROS/UI/motion side effects; add --run only after server -> bridge -> pacer -> spawn -> camrod is healthy")
            return 0
        return run_matrix(args, all_sites, drop_zone, selected, report)
    except KeyboardInterrupt:
        return 130
    except MatrixError as error:
        print(f"[camping-matrix] ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
