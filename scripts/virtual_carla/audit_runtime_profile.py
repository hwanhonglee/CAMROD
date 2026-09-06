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
import re
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Sequence


SCHEMA = "camrod.virtual_carla.runtime_profile_audit.v1"
EXACT_RANGER_BLUEPRINT = "vehicle.ranger.default"
PROFILE_LAUNCHES = {
    "camrod_carla_full.launch.py": "develop-parity",
    "camrod_carla_develop_site_geometry.launch.py": (
        "develop-plus-carla-site-geometry-v27"
    ),
    "camrod_carla_woraksan_tuned.launch.py": "woraksan-tuned",
}
SITE_GEOMETRY_CURRENT_CARLA_MAP = (
    "map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15/"
    "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15"
)
# Current v27 evidence is accepted only on the exact v15 world. Older maps
# remain usable for replaying their already-labelled historical artifacts, but
# they must never authorize a new current-profile motion run.
SITE_GEOMETRY_ALLOWED_CARLA_MAPS = (SITE_GEOMETRY_CURRENT_CARLA_MAP,)
CARLA_CHARGING_CONTACT_PARAMETERS = {
    "drop_zone_id": "drop_zone",
    "pose_topic": "/localization/pose",
    "odometry_topic": "/odom",
    "parking_status_topic": "/parking/apriltag_parking_controller/status",
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
EXPECTED_SOURCE_BRANCH = "virtual/carla"
ORIGIN_DEVELOP_REF = "refs/remotes/origin/develop"
PROTECTED_SOURCE_RELATIVE_PATH = (
    "camrod_sensing/external/vanjee_lidar/vanjee_lidar_sdk/src/"
    "vanjee_driver/cmake/vanjee_driverConfig.cmake"
)
RUNTIME_SOURCE_PATHS = (
    "avg_msgs",
    "camrod_bringup",
    "camrod_carla_adapter",
    "camrod_common",
    "camrod_control",
    "camrod_localization",
    "camrod_map",
    "camrod_perception",
    "camrod_planning",
    "camrod_platform",
    "camrod_runtime",
    "camrod_sensing",
    "camrod_sensor_kit",
    "camrod_system",
    "camrod_ui",
    "scripts/virtual_carla",
    "colcon_build.sh",
    "cyclonedds.xml",
    "lanelet2_maps.osm",
)
RUNTIME_INSTALL_PREFIXES = (
    "camrod_ui",
    "camrod_carla_adapter",
    "camrod_control",
    "camrod_planning",
    "camrod_bringup",
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
UI_MAIN_BUNDLE_RE = re.compile(r"(?:^|[\"'])/?(static/js/main\.[a-z0-9]+\.js)(?:[\"']|$)")
CANONICAL_UI_BUILD_ENV = {
    "BUILD_PATH": "build",
    "CI": "false",
    "DISABLE_ESLINT_PLUGIN": "false",
    "GENERATE_SOURCEMAP": "false",
    "IMAGE_INLINE_SIZE_LIMIT": "10000",
    "INLINE_RUNTIME_CHUNK": "true",
    "NODE_ENV": "production",
    "NODE_PATH": "",
    "PUBLIC_URL": "",
    "REACT_APP_OPERATING_HOURS_END": "23",
    "REACT_APP_OPERATING_HOURS_GATE_ENABLED": "false",
    "REACT_APP_OPERATING_HOURS_START": "3",
    "SKIP_PREFLIGHT_CHECK": "false",
    "TSC_COMPILE_ON_ERROR": "false",
}
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
        "camping_site_maneuver_controller_static_bypass_phases": (
            "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
            "ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT"
        ),
        "camping_site_maneuver_controller_lanelet_bypass_phases": (
            "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
            "ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT"
        ),
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
    # Plain run.sh camrod keeps the latest develop controller parameters. The
    # CARLA camera detector remains a sensor-boundary adapter, while simulated
    # charging contact is isolated to the explicit site-geometry test profile.
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
        "controller_id": "RPP",
        "reverse_controller_id": "RPP",
    },
    "/planning/planning_state_machine": {
        "return_goal_reached_distance_m": 0.30,
    },
    "/localization/pose_selector": {
        "primary_pose_cov_topic": "/localization/primary/pose_with_covariance",
        "fallback_pose_cov_topic": "",
        "primary_odom_topic": "/localization/primary/odometry",
        "fallback_odom_topic": "",
    },
    "/ui_backend": {
        "enable_operator_telemetry": True,
        "return_site_exit_rearm_enabled": False,
        "telemetry_raw_lidar_bbox_overlay_enabled": False,
        "telemetry_docking_rear_camera_fallback_enabled": False,
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
    # Full/develop-parity must retain the current develop detector threshold.
    # Only the site-geometry wrapper selects the measured Woraksan overlay.
    "/perception/yolov9mit": {
        "min_confidence": 0.5,
    },
}


def _with_site_geometry(
    base: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    """Apply the proven site geometry and scoped CARLA recovery lease."""
    result = {node: dict(parameters) for node, parameters in base.items()}
    result["/carla_charging_contact_emulator"] = dict(
        CARLA_CHARGING_CONTACT_PARAMETERS
    )
    result["/perception/yolov9mit"]["min_confidence"] = 0.95
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
    result["/ui_backend"]["return_site_exit_rearm_enabled"] = True
    result["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] = True
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
    result["/control/cmd_vel_safety_gate"][
        "camping_site_maneuver_controller_static_bypass_phases"
    ] = (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,"
        "ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
    )
    result["/control/cmd_vel_safety_gate"][
        "camping_site_maneuver_controller_lanelet_bypass_phases"
    ] = (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,"
        "ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
    )
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
    result["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] = "/planning/auto_reverse_goal_raw"
    result["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] = "RPPReverse"
    result["/planning/planning_state_machine"][
        "return_goal_reached_distance_m"
    ] = 0.35
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


DEVELOP_SITE_GEOMETRY_PARAMETERS = _with_site_geometry(
    DEVELOP_PARITY_PARAMETERS
)
AUDITED_PROFILE_PARAMETERS = {
    "develop-parity": DEVELOP_PARITY_PARAMETERS,
    "develop-plus-carla-site-geometry-v27": DEVELOP_SITE_GEOMETRY_PARAMETERS,
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


def _git_output(source_root: Path, *arguments: str) -> str:
    try:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=source_root,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="surrogateescape",
            timeout=20.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise AuditError(f"cannot inspect source Git identity: {error}") from None
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise AuditError(
            f"Git identity command failed ({' '.join(arguments)}): {detail}"
        )
    return completed.stdout


def _runtime_git_pathspecs() -> list[str]:
    pathspecs = [f":(top,literal){path}" for path in RUNTIME_SOURCE_PATHS]
    pathspecs.append(
        f":(top,exclude,literal){PROTECTED_SOURCE_RELATIVE_PATH}"
    )
    return pathspecs


def _porcelain_dirty_paths(raw: str) -> list[str]:
    """Decode status -z paths without ever opening the named files."""
    tokens = [token for token in raw.split("\0") if token]
    paths: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        status = token[:2] if len(token) >= 3 and token[2] == " " else ""
        path = token[3:] if status else token
        if path != PROTECTED_SOURCE_RELATIVE_PATH:
            paths.append(path)
        # In porcelain v1 -z, rename/copy records carry a second NUL path.
        if status and any(code in status for code in ("R", "C")):
            index += 1
            if index < len(tokens):
                original = tokens[index]
                if original != PROTECTED_SOURCE_RELATIVE_PATH:
                    paths.append(original)
        index += 1
    return sorted(set(paths))


def _canonical_tree_digest(entries: Sequence[Mapping[str, Any]]) -> str:
    digest = hashlib.sha256()
    for entry in entries:
        digest.update(
            json.dumps(
                entry,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8", "surrogateescape")
        )
        digest.update(b"\n")
    return digest.hexdigest()


def _source_tree_fingerprint(source_root: Path) -> dict[str, Any]:
    tracked_raw = _git_output(
        source_root,
        "ls-files",
        "-z",
        "--",
        *_runtime_git_pathspecs(),
    )
    relative_paths = sorted(path for path in tracked_raw.split("\0") if path)
    if not relative_paths:
        raise AuditError("runtime source fingerprint contains no tracked files")
    if PROTECTED_SOURCE_RELATIVE_PATH in relative_paths:
        raise AuditError("protected Vanjee config entered the source fingerprint")

    entries: list[dict[str, Any]] = []
    for relative in relative_paths:
        path = source_root / relative
        try:
            metadata = path.lstat()
        except OSError as error:
            raise AuditError(
                f"tracked runtime source is unavailable: {relative}: {error}"
            ) from None
        if stat.S_ISLNK(metadata.st_mode):
            try:
                link_target = os.readlink(path)
            except OSError as error:
                raise AuditError(
                    f"cannot read runtime source symlink {relative}: {error}"
                ) from None
            entries.append({
                "kind": "symlink",
                "link_target": link_target,
                "mode": "120000",
                "path": relative,
            })
        elif stat.S_ISREG(metadata.st_mode):
            entries.append({
                "content_sha256": sha256_file(path),
                "kind": "file",
                "mode": (
                    "100755"
                    if metadata.st_mode & stat.S_IXUSR
                    else "100644"
                ),
                "path": relative,
            })
        else:
            raise AuditError(
                f"unsupported tracked runtime source type: {relative}"
            )
    return {
        "algorithm": "sha256-canonical-path-kind-mode-content-v1",
        "sha256": _canonical_tree_digest(entries),
        "tracked_entry_count": len(entries),
        "roots": list(RUNTIME_SOURCE_PATHS),
    }


def audit_source_identity(source_root: Path) -> dict[str, Any]:
    source_root = source_root.expanduser().resolve()
    if not source_root.is_dir():
        raise AuditError(f"CAMROD source root is missing: {source_root}")
    git_root = Path(
        _git_output(source_root, "rev-parse", "--show-toplevel").strip()
    ).resolve()
    if git_root != source_root:
        raise AuditError(
            f"source root is not the Git top level: {source_root} != {git_root}"
        )

    branch = _git_output(
        source_root, "symbolic-ref", "--quiet", "--short", "HEAD"
    ).strip()
    if branch != EXPECTED_SOURCE_BRANCH:
        raise AuditError(
            f"current v27 source branch is {branch!r}, expected "
            f"{EXPECTED_SOURCE_BRANCH!r}"
        )
    head_commit = _git_output(source_root, "rev-parse", "HEAD").strip()
    origin_develop_commit = _git_output(
        source_root, "rev-parse", ORIGIN_DEVELOP_REF
    ).strip()
    merge_base = _git_output(
        source_root, "merge-base", "HEAD", ORIGIN_DEVELOP_REF
    ).strip()
    counts = _git_output(
        source_root,
        "rev-list",
        "--left-right",
        "--count",
        f"HEAD...{ORIGIN_DEVELOP_REF}",
    ).split()
    if len(counts) != 2:
        raise AuditError(f"malformed Git ahead/behind result: {counts!r}")
    try:
        ahead, behind = (int(value) for value in counts)
    except ValueError:
        raise AuditError(f"malformed Git ahead/behind result: {counts!r}") from None
    if behind != 0 or merge_base != origin_develop_commit:
        raise AuditError(
            "virtual/carla does not contain the exact fetched origin/develop "
            f"tip: ahead={ahead} behind={behind} merge_base={merge_base} "
            f"origin_develop={origin_develop_commit}"
        )

    dirty_raw = _git_output(
        source_root,
        "status",
        "--porcelain=v1",
        "-z",
        "--untracked-files=all",
        "--",
        *_runtime_git_pathspecs(),
    )
    dirty_paths = _porcelain_dirty_paths(dirty_raw)
    if dirty_paths:
        preview = ", ".join(dirty_paths[:20])
        suffix = " ..." if len(dirty_paths) > 20 else ""
        raise AuditError(
            "runtime source tree is dirty outside the explicitly excluded "
            f"Vanjee user config: {preview}{suffix}"
        )

    return {
        "path": str(source_root),
        "branch": branch,
        "head_commit": head_commit,
        "origin_develop_commit": origin_develop_commit,
        "merge_base": merge_base,
        "ahead_of_origin_develop": ahead,
        "behind_origin_develop": behind,
        "runtime_worktree_clean": True,
        "runtime_tree": _source_tree_fingerprint(source_root),
        "explicit_exclusions": [{
            "path": PROTECTED_SOURCE_RELATIVE_PATH,
            "content_hashed": False,
            "dirty_state_checked": False,
            "reason": "protected user-owned generated Vanjee CMake config",
        }],
    }


def _install_cache_path(relative: Path) -> bool:
    return (
        "__pycache__" in relative.parts
        or relative.suffix in (".pyc", ".pyo")
    )


def _installed_entry(path: Path, relative: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise AuditError(f"cannot inspect installed entry {path}: {error}") from None
    if stat.S_ISREG(metadata.st_mode):
        return {
            "content_sha256": sha256_file(path),
            "executable": bool(metadata.st_mode & stat.S_IXUSR),
            "kind": "file",
            "path": relative.as_posix(),
        }
    if not stat.S_ISLNK(metadata.st_mode):
        raise AuditError(f"unsupported installed entry type: {path}")
    try:
        link_target = os.readlink(path)
        resolved_target = path.resolve(strict=True)
        target_metadata = resolved_target.stat()
    except (OSError, RuntimeError) as error:
        raise AuditError(f"broken installed symlink {path}: {error}") from None
    entry: dict[str, Any] = {
        "kind": "symlink",
        "link_target": link_target,
        "path": relative.as_posix(),
        "resolved_target": str(resolved_target),
    }
    if stat.S_ISREG(target_metadata.st_mode):
        entry.update({
            "target_content_sha256": sha256_file(resolved_target),
            "target_executable": bool(target_metadata.st_mode & stat.S_IXUSR),
            "target_kind": "file",
        })
    elif stat.S_ISDIR(target_metadata.st_mode):
        entry["target_kind"] = "directory"
    else:
        raise AuditError(f"unsupported installed symlink target: {path}")
    return entry


def fingerprint_install_prefix(prefix: Path) -> dict[str, Any]:
    prefix = prefix.expanduser().absolute()
    if not prefix.is_dir():
        raise AuditError(f"required install prefix is missing: {prefix}")
    entries: list[dict[str, Any]] = []

    def walk_error(error: OSError) -> None:
        raise AuditError(f"cannot enumerate install prefix {prefix}: {error}")

    for directory, dirnames, filenames in os.walk(
        prefix, topdown=True, followlinks=False, onerror=walk_error
    ):
        directory_path = Path(directory)
        kept_directories: list[str] = []
        for name in sorted(dirnames):
            path = directory_path / name
            relative = path.relative_to(prefix)
            if _install_cache_path(relative):
                continue
            if path.is_symlink():
                entries.append(_installed_entry(path, relative))
            else:
                kept_directories.append(name)
        dirnames[:] = kept_directories
        for name in sorted(filenames):
            path = directory_path / name
            relative = path.relative_to(prefix)
            if _install_cache_path(relative):
                continue
            entries.append(_installed_entry(path, relative))
    entries.sort(key=lambda entry: entry["path"])
    if not entries:
        raise AuditError(f"install prefix contains no runtime entries: {prefix}")
    return {
        "path": str(prefix),
        "algorithm": "sha256-canonical-install-entry-v1",
        "sha256": _canonical_tree_digest(entries),
        "entry_count": len(entries),
        "regular_file_count": sum(
            entry["kind"] == "file" for entry in entries
        ),
        "symlink_count": sum(
            entry["kind"] == "symlink" for entry in entries
        ),
        "cache_policy": "exclude __pycache__ directories and .pyc/.pyo files",
    }


def fingerprint_runtime_install(install_root: Path) -> dict[str, Any]:
    install_root = install_root.expanduser().absolute()
    if not install_root.is_dir():
        raise AuditError(f"CAMROD install root is missing: {install_root}")
    prefixes: dict[str, dict[str, Any]] = {}
    combined_entries: list[dict[str, Any]] = []
    for package in RUNTIME_INSTALL_PREFIXES:
        fingerprint = fingerprint_install_prefix(install_root / package)
        prefixes[package] = fingerprint
        combined_entries.append({
            "package": package,
            "sha256": fingerprint["sha256"],
        })
    return {
        "path": str(install_root),
        "algorithm": "sha256-canonical-package-fingerprint-v1",
        "sha256": _canonical_tree_digest(combined_entries),
        "prefixes": prefixes,
    }


def frontend_source_input_fingerprint(frontend_root: Path) -> str:
    frontend_root = frontend_root.expanduser().absolute()
    dotenv_files = sorted(path.name for path in frontend_root.glob(".env*"))
    if dotenv_files:
        raise AuditError(
            "CRA .env files are forbidden in the canonical UI build: "
            + ", ".join(dotenv_files)
        )
    build_env_path = frontend_root / "camrod-build-env.json"
    try:
        build_env = json.loads(build_env_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise AuditError(
            f"cannot read canonical UI build environment {build_env_path}: {error}"
        ) from None
    if build_env != CANONICAL_UI_BUILD_ENV:
        raise AuditError(
            "canonical UI build environment manifest is not the exact "
            "audited PUBLIC_URL/REACT_APP/CRA environment"
        )
    required_files = [
        Path("package.json"),
        Path("package-lock.json"),
        Path("camrod-build-env.json"),
    ]
    relative_files = list(required_files)
    for directory_name in ("src", "public"):
        directory = frontend_root / directory_name
        if not directory.is_dir():
            raise AuditError(f"UI source input directory is missing: {directory}")
        relative_files.extend(
            path.relative_to(frontend_root)
            for path in directory.rglob("*")
            if path.is_file() and not path.is_symlink()
        )
    relative_files = sorted(set(relative_files), key=lambda path: path.as_posix())
    digest = hashlib.sha256()
    for relative in relative_files:
        path = frontend_root / relative
        if not path.is_file() or path.is_symlink():
            raise AuditError(f"UI source input is missing: {path}")
        digest.update(os.fsencode(relative.as_posix()))
        digest.update(b"\0")
        digest.update(sha256_file(path).encode("ascii"))
        # Match colcon_build.sh's existing awk fingerprint byte contract.
        digest.update(b"\\0")
    return digest.hexdigest()


def _read_sha256_stamp(path: Path) -> str:
    if not path.is_file() or path.is_symlink():
        raise AuditError(f"UI input fingerprint stamp is missing: {path}")
    try:
        value = path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as error:
        raise AuditError(f"cannot read UI input fingerprint {path}: {error}") from None
    if not SHA256_RE.fullmatch(value):
        raise AuditError(f"UI input fingerprint is malformed: {path}")
    return value


def _read_ui_build_environment(path: Path) -> dict[str, str]:
    if not path.is_file() or path.is_symlink():
        raise AuditError(f"UI build environment evidence is missing: {path}")
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise AuditError(f"cannot read UI build environment {path}: {error}") from None
    if document != CANONICAL_UI_BUILD_ENV:
        raise AuditError(
            "UI build environment is not the exact canonical "
            "PUBLIC_URL/REACT_APP/CRA environment: "
            f"{path}"
        )
    return document


def _ui_main_bundle(build_root: Path) -> tuple[Path, Path]:
    index = build_root / "index.html"
    if not index.is_file() or index.is_symlink():
        raise AuditError(f"UI build index is missing: {index}")
    try:
        index_text = index.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise AuditError(f"cannot read UI build index {index}: {error}") from None
    matches = sorted(set(UI_MAIN_BUNDLE_RE.findall(index_text)))
    if len(matches) != 1:
        raise AuditError(
            f"UI index must reference exactly one hashed main bundle: {index}"
        )
    bundle = build_root / matches[0]
    if not bundle.is_file() or bundle.is_symlink():
        raise AuditError(f"UI main bundle is missing: {bundle}")
    return index, bundle


def audit_installed_ui_identity(
    source_root: Path, install_root: Path
) -> dict[str, Any]:
    frontend_root = (
        source_root
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
    )
    source_build = frontend_root / "build"
    installed_build = (
        install_root
        / "camrod_ui"
        / "share"
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
        / "build"
    )
    current_inputs = frontend_source_input_fingerprint(frontend_root)
    source_stamp_path = source_build / ".camrod-inputs.sha256"
    installed_stamp_path = installed_build / ".camrod-inputs.sha256"
    source_build_env_path = source_build / ".camrod-build-env.json"
    installed_build_env_path = installed_build / ".camrod-build-env.json"
    source_build_env = _read_ui_build_environment(source_build_env_path)
    installed_build_env = _read_ui_build_environment(installed_build_env_path)
    if source_build_env != installed_build_env:
        raise AuditError("UI install/source mismatch: build environments differ")
    source_stamp = _read_sha256_stamp(source_stamp_path)
    installed_stamp = _read_sha256_stamp(installed_stamp_path)
    if source_stamp != current_inputs:
        raise AuditError(
            "UI source/build mismatch: .camrod-inputs.sha256 does not match "
            "the current src/public/package inputs; rebuild camrod_ui"
        )
    if installed_stamp != source_stamp:
        raise AuditError(
            "UI install/source mismatch: installed .camrod-inputs.sha256 "
            "differs from the current source build"
        )
    source_index, source_bundle = _ui_main_bundle(source_build)
    installed_index, installed_bundle = _ui_main_bundle(installed_build)
    if source_bundle.relative_to(source_build) != installed_bundle.relative_to(
        installed_build
    ):
        raise AuditError("UI install/source mismatch: main bundle names differ")
    source_index_sha = sha256_file(source_index)
    installed_index_sha = sha256_file(installed_index)
    source_bundle_sha = sha256_file(source_bundle)
    installed_bundle_sha = sha256_file(installed_bundle)
    if source_index_sha != installed_index_sha:
        raise AuditError("UI install/source mismatch: index.html content differs")
    if source_bundle_sha != installed_bundle_sha:
        raise AuditError("UI install/source mismatch: main bundle content differs")

    return {
        "source_inputs_sha256": current_inputs,
        "canonical_build_environment": installed_build_env,
        "installed_build_environment": {
            "path": str(installed_build_env_path),
            "sha256": sha256_file(installed_build_env_path),
        },
        "installed_inputs_stamp": {
            "path": str(installed_stamp_path),
            "file_sha256": sha256_file(installed_stamp_path),
            "value": installed_stamp,
        },
        "installed_index": {
            "path": str(installed_index),
            "sha256": installed_index_sha,
        },
        "installed_main_bundle": {
            "path": str(installed_bundle),
            "sha256": installed_bundle_sha,
        },
        "source_install_match": True,
    }


def audit_software_identity(
    source_root: Path, install_root: Path
) -> dict[str, Any]:
    source_root = source_root.expanduser().resolve()
    install_root = install_root.expanduser().absolute()
    source = audit_source_identity(source_root)
    install = fingerprint_runtime_install(install_root)
    ui_build = audit_installed_ui_identity(source_root, install_root)
    binding = [{
        "head_commit": source["head_commit"],
        "install_sha256": install["sha256"],
        "runtime_source_sha256": source["runtime_tree"]["sha256"],
        "ui_build_environment_sha256": ui_build[
            "installed_build_environment"
        ]["sha256"],
        "ui_inputs_sha256": ui_build["source_inputs_sha256"],
        "ui_main_bundle_sha256": ui_build["installed_main_bundle"][
            "sha256"
        ],
    }]
    return {
        "schema": "camrod.virtual_carla.software_identity.v1",
        "binding_sha256": _canonical_tree_digest(binding),
        "source": source,
        "install": install,
        "ui_build": ui_build,
    }


def normalize_carla_map(value: str) -> str:
    """Normalize CARLA's `/Game/...` and Python API map-name spellings."""
    result = str(value).strip()
    if result.startswith("/Game/"):
        result = result[len("/Game/") :]
    return result.strip("/")


def validate_audited_map(profile: str, normalized_map: str) -> None:
    """Restrict the CARLA-only site profile to exact built map identities."""
    if (
        profile == "develop-plus-carla-site-geometry-v27"
        and normalized_map != SITE_GEOMETRY_CURRENT_CARLA_MAP
    ):
        raise AuditError(
            "current site-geometry v27 profile requires the exact v15 map "
            f"{SITE_GEOMETRY_CURRENT_CARLA_MAP!r}; got {normalized_map!r}"
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
    result.add_argument(
        "--source-root",
        type=Path,
        required=True,
        help="clean virtual/carla Git checkout used by the live install",
    )
    result.add_argument(
        "--install-root",
        type=Path,
        required=True,
        help="isolated CAMROD install root containing the audited prefixes",
    )
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

    # Bind source and installed artifacts before reading the live graph. This
    # prevents an uncommitted or stale build from authorizing vehicle motion.
    software_identity = audit_software_identity(
        args.source_root, args.install_root
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
        "software_identity": software_identity,
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
                "v27-current-pose-return-roadside-exact-reverse-wide-b2-"
                "access-apriltag-two-retries-"
                "charging-site-yaw-tolerance-bounded-rotation-torque-"
                "saturation-crab-out-yaw-recovery-docking-rear-fallback"
                if audited_profile ==
                "develop-plus-carla-site-geometry-v27" else "disabled"
            ),
            "carla_recovery_breakaway_lease": (
                "bounded_controller_authenticated"
                if audited_profile ==
                "develop-plus-carla-site-geometry-v27"
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
