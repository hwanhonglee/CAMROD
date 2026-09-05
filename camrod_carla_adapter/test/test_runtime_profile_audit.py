"""Pure tests for the fail-closed live CAMROD profile audit."""

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "audit_runtime_profile.py"
)
SPEC = importlib.util.spec_from_file_location("audit_runtime_profile", SCRIPT)
assert SPEC and SPEC.loader
audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit
SPEC.loader.exec_module(audit)


def _write_cmdline(root: Path, pid: int, argv: list[str]) -> None:
    process = root / str(pid)
    process.mkdir()
    (process / "cmdline").write_bytes(
        b"\0".join(item.encode("utf-8") for item in argv) + b"\0"
    )


def test_map_name_normalization_is_exact():
    expected = "map_package/Maps/Woraksan/Map"
    assert audit.normalize_carla_map(f"/Game/{expected}") == expected
    assert audit.normalize_carla_map(expected) == expected
    assert audit.normalize_carla_map(" /Game/map_package/Maps/A ") == (
        "map_package/Maps/A"
    )


def test_site_geometry_map_allowlist_defaults_to_v13_and_keeps_v12_v11():
    v13, v12, v11 = audit.SITE_GEOMETRY_ALLOWED_CARLA_MAPS
    assert v13.endswith(
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13/"
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13"
    )
    assert v12.endswith(
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12/"
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12"
    )
    assert v11.endswith(
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11/"
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11"
    )
    for value in (v13, v12, v11):
        audit.validate_audited_map(
            "develop-plus-carla-site-geometry-v26", value
        )
    with pytest.raises(audit.AuditError, match="v13/v12/v11 allowlist"):
        audit.validate_audited_map(
            "develop-plus-carla-site-geometry-v26",
            "map_package/Maps/unreviewed/unreviewed",
        )
    # The production/develop-parity profile keeps its existing map policy.
    audit.validate_audited_map(
        "develop-parity", "map_package/Maps/unreviewed/unreviewed"
    )


def test_launch_discovery_requires_one_profile_and_lanelet_argument(tmp_path):
    lanelet = tmp_path / "lanelet.osm"
    lanelet.write_text("map", encoding="utf-8")
    _write_cmdline(
        tmp_path,
        410,
        [
            "/opt/ros/humble/bin/ros2",
            "launch",
            "camrod_carla_adapter",
            "camrod_carla_full.launch.py",
            f"camrod_map_path:={lanelet}",
        ],
    )
    discovered = audit.discover_camrod_launch(tmp_path)
    assert discovered["pid"] == 410
    assert discovered["profile"] == "develop-parity"
    assert discovered["lanelet_map_argument"] == str(lanelet.resolve())
    assert len(discovered["cmdline_sha256"]) == 64

    _write_cmdline(
        tmp_path,
        411,
        [
            "/opt/ros/humble/bin/ros2",
            "launch",
            "camrod_carla_adapter",
            "camrod_carla_woraksan_tuned.launch.py",
            f"camrod_map_path:={lanelet}",
        ],
    )
    with pytest.raises(audit.AuditError, match="exactly one"):
        audit.discover_camrod_launch(tmp_path)


def test_site_geometry_launch_is_auto_detected_but_tuned_is_not_authorized(
    tmp_path,
):
    lanelet = tmp_path / "lanelet.osm"
    lanelet.write_text("map", encoding="utf-8")
    _write_cmdline(
        tmp_path,
        420,
        [
            "/opt/ros/humble/bin/ros2",
            "launch",
            "camrod_carla_adapter",
            "camrod_carla_develop_site_geometry.launch.py",
            f"camrod_map_path:={lanelet}",
        ],
    )
    discovered = audit.discover_camrod_launch(tmp_path)
    assert discovered["profile"] == "develop-plus-carla-site-geometry-v26"
    assert (
        audit.resolve_audited_profile("auto", discovered["profile"])
        == "develop-plus-carla-site-geometry-v26"
    )
    with pytest.raises(audit.AuditError, match="expected 'develop-parity'"):
        audit.resolve_audited_profile(
            "develop-parity", discovered["profile"]
        )
    with pytest.raises(audit.AuditError, match="no motion-authorized"):
        audit.resolve_audited_profile("auto", "woraksan-tuned")
    with pytest.raises(audit.AuditError, match="no motion-authorized"):
        audit.resolve_audited_profile(
            "auto", "develop-plus-carla-site-geometry-v16"
        )


def test_site_geometry_profile_changes_only_the_proven_carla_parameters():
    parity = audit.DEVELOP_PARITY_PARAMETERS
    site = audit.DEVELOP_SITE_GEOMETRY_PARAMETERS
    controller = "/control/camping_site_maneuver_controller"

    charger = "/carla_charging_contact_emulator"
    detector = "/perception/apriltag_parking_detector"
    assert set(site) == set(parity)
    command_adapter = "/camrod_twist_to_4ws"
    recovery_nodes = {
        "/control/cmd_vel_safety_gate",
        "/control/route_safety_recovery_controller",
        "/planning/goal_snapper",
        "/planning/controller_server",
        "/parking/apriltag_parking_controller",
        "/ui_backend",
    }
    for node in parity:
        if node not in ({controller, command_adapter} | recovery_nodes):
            assert site[node] == parity[node]

    assert site[charger] == parity[charger] == {
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
    assert site[detector] == parity[detector] == {
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
    }

    assert parity[command_adapter]["recovery_breakaway_enable"] is False
    assert site[command_adapter]["recovery_breakaway_enable"] is True
    assert parity[command_adapter][
        "rotation_recovery_breakaway_enable"
    ] is False
    assert site[command_adapter][
        "rotation_recovery_breakaway_enable"
    ] is True
    assert parity[command_adapter][
        "rotation_recovery_breakaway_status_timeout_sec"
    ] == 0.30
    assert site[command_adapter][
        "rotation_recovery_breakaway_status_timeout_sec"
    ] == 1.25
    assert parity["/control/cmd_vel_safety_gate"][
        "route_safety_path_relative_recovery_enable"
    ] is False
    assert site["/control/cmd_vel_safety_gate"][
        "route_safety_path_relative_recovery_enable"
    ] is True
    assert parity["/control/cmd_vel_safety_gate"][
        "route_safety_path_center_reentry_m"
    ] == 0.08
    assert site["/control/cmd_vel_safety_gate"][
        "route_safety_path_center_reentry_m"
    ] == 0.15
    assert parity["/control/cmd_vel_safety_gate"][
        "lanelet_safety_footprint_enable"
    ] is True
    assert site["/control/cmd_vel_safety_gate"][
        "lanelet_safety_footprint_enable"
    ] is False
    for profile in (parity, site):
        gate = profile["/control/cmd_vel_safety_gate"]
        assert gate["lanelet_safety_enable"] is True
        assert gate["lanelet_safety_body_hard_stop_enable"] is True
        assert gate["lanelet_safety_body_hard_stop_threshold"] == 100
    assert parity["/control/cmd_vel_safety_gate"][
        "cost_stop_latch_use_trigger_source_for_merged_clear"
    ] is False
    assert site["/control/cmd_vel_safety_gate"][
        "cost_stop_latch_use_trigger_source_for_merged_clear"
    ] is True
    assert parity["/control/cmd_vel_safety_gate"][
        "cost_stop_merged_dynamic_source_labels"
    ] == ""
    assert site["/control/cmd_vel_safety_gate"][
        "cost_stop_merged_dynamic_source_labels"
    ] == "radar"
    assert parity["/perception/yolov9mit"]["min_confidence"] == 0.95
    assert site["/perception/yolov9mit"]["min_confidence"] == 0.95
    assert "return_site_exit_rearm_enabled" not in parity["/ui_backend"]
    assert site["/ui_backend"]["return_site_exit_rearm_enabled"] is True
    assert site["/control/route_safety_recovery_controller"] == {
        "zero_hold_pauses_limits": True,
        "allow_corrective_yaw_beyond_limit": True,
    }
    assert parity["/control/route_safety_recovery_controller"] == {
        "zero_hold_pauses_limits": False,
        "allow_corrective_yaw_beyond_limit": False,
    }
    assert site["/planning/goal_snapper"][
        "reissue_active_goal_after_route_recovery_when_nav_active"
    ] is True
    assert parity["/planning/goal_snapper"]["pose_jump_check_topic"] == ""
    assert (
        site["/planning/goal_snapper"]["pose_jump_check_topic"]
        == "/localization/pose"
    )
    parking = "/parking/apriltag_parking_controller"
    assert parity[parking]["heading_gain"] == 1.5
    assert parity[parking]["lateral_to_heading_gain"] == 2.5
    assert parity[parking]["reverse_approach_speed_mps"] == 0.2
    assert parity[parking]["final_insertion_speed_mps"] == 0.05
    assert site[parking]["heading_gain"] == 1.5
    assert site[parking]["lateral_to_heading_gain"] == 2.7
    assert site[parking]["reverse_approach_speed_mps"] == 0.2
    assert site[parking]["final_insertion_speed_mps"] == 0.05
    assert site[parking]["translation_stop_tag_distance_m"] == 0.40
    assert site[parking]["final_lateral_tolerance_m"] == 0.03
    assert site[parking]["minimum_approach_turn_radius_m"] == 0.85
    assert parity[parking]["enable_bounded_lateral_retry"] is False
    assert parity[parking]["retry_forward_distance_m"] == 1.0
    assert parity[parking]["retry_forward_speed_mps"] == 0.10
    assert parity[parking]["retry_forward_timeout_s"] == 25.0
    assert parity[parking]["maximum_retries"] == 5
    assert site[parking]["enable_bounded_lateral_retry"] is True
    assert site[parking]["retry_forward_distance_m"] == 0.8
    assert site[parking]["retry_forward_speed_mps"] == 0.20
    assert site[parking]["retry_forward_timeout_s"] == 30.0
    assert site[parking]["retry_yaw_alignment_timeout_s"] == 8.0
    assert site[parking]["retry_maximum_lateral_error_m"] == 0.15
    assert site[parking]["retry_maximum_heading_error_rad"] == 0.35
    assert site[parking]["retry_maximum_forward_exit_lateral_drift_m"] == 0.15
    assert site[parking]["retry_maximum_odometry_step_m"] == 0.10
    assert site[parking]["retry_minimum_tag_distance_m"] == 0.35
    assert site[parking]["retry_maximum_tag_distance_m"] == 0.45
    assert site[parking]["maximum_retries"] == 2
    gated_timeout_budget_m = (
        site[parking]["retry_forward_speed_mps"]
        * site["/control/cmd_vel_safety_gate"]["speed_scale"]
        * site[parking]["retry_forward_timeout_s"]
    )
    assert gated_timeout_budget_m == 6.0
    assert site[parking]["retry_forward_distance_m"] <= gated_timeout_budget_m
    assert (
        site[parking]["retry_forward_distance_m"]
        + site[parking]["retry_maximum_odometry_step_m"]
    ) == 0.9
    assert parity["/planning/goal_snapper"][
        "reissue_active_goal_after_route_recovery_when_nav_active"
    ] is False
    assert parity[controller]["crab_entry_max_heading_drift_deg"] == 0.0
    assert site[controller]["crab_entry_max_heading_drift_deg"] == 0.0

    changed = {
        name: value
        for name, value in site[controller].items()
        if parity[controller][name] != value
    }
    assert changed == {
        "max_angular_speed_radps": 0.45,
        "crab_approach_slowdown_distance_m": 1.0,
        "crab_approach_min_speed_mps": 0.12,
        "rotate_180_timeout_s": 90.0,
        "entry_position_tolerance_m": 0.05,
        "rotate_entry_max_position_error_m": 0.05,
        "rotate_entry_centering_max_initial_error_m": 0.65,
        "crab_entry_body_yaw_compensation_deg": 2.0,
        "crab_entry_body_yaw_alignment_tolerance_deg": 1.5,
        "crab_out_yaw_recovery_enable": True,
        "crab_out_yaw_recovery_max_attempts": 8,
        "crab_out_yaw_recovery_global_timeout_s": 90.0,
    }
    assert parity[controller][
        "crab_entry_body_yaw_alignment_tolerance_deg"
    ] == 0.5
    assert parity[controller]["max_angular_speed_radps"] == 0.35
    assert site[controller]["max_angular_speed_radps"] == 0.45
    assert site[controller][
        "crab_entry_body_yaw_alignment_tolerance_deg"
    ] == 1.5
    assert parity[controller]["return_lateral_transition_tolerance_m"] == 0.02
    assert site[controller]["return_lateral_transition_tolerance_m"] == 0.02
    assert parity[controller]["return_lateral_hysteresis_m"] == 0.10
    assert site[controller]["return_lateral_hysteresis_m"] == 0.10
    assert tuple(audit.AUDITED_PROFILE_PARAMETERS) == (
        "develop-parity",
        "develop-plus-carla-site-geometry-v26",
    )


def test_ros_parameter_dump_requires_exact_node_root():
    def runner(*_args, **_kwargs):
        return subprocess.CompletedProcess(
            args=[],
            returncode=0,
            stdout=(
                "/control/cmd_vel_safety_gate:\n"
                "  ros__parameters:\n"
                "    speed_scale: 0.5\n"
                "    RPP:\n"
                "      desired_linear_vel: 0.555556\n"
            ),
            stderr="",
        )

    result = audit.dump_ros_parameters(
        "/control/cmd_vel_safety_gate", runner=runner
    )
    assert result == {
        "speed_scale": 0.5,
        "RPP.desired_linear_vel": 0.555556,
    }

    with pytest.raises(audit.AuditError, match="missing exact node"):
        audit.dump_ros_parameters("/wrong/node", runner=runner)


def test_selected_parameter_contract_rejects_tuned_values():
    expected = {
        "/control/cmd_vel_safety_gate": {
            "speed_scale": 0.5,
            "navigation_minimum_ackermann_turn_radius_m": 0.0,
        }
    }
    actual = {
        "/control/cmd_vel_safety_gate": {
            "speed_scale": 0.5,
            "navigation_minimum_ackermann_turn_radius_m": 0.0,
            "unrelated": 12,
        }
    }
    selected = audit.select_and_validate_parameters(actual, expected)
    assert selected == {
        "/control/cmd_vel_safety_gate": {
            "speed_scale": 0.5,
            "navigation_minimum_ackermann_turn_radius_m": 0.0,
        }
    }

    actual["/control/cmd_vel_safety_gate"]["speed_scale"] = 1.0
    with pytest.raises(audit.AuditError, match="expected 0.5"):
        audit.select_and_validate_parameters(actual, expected)


@pytest.mark.parametrize(
    ("node", "parameter", "stale_value"),
    (
        (
            "/perception/apriltag_parking_detector",
            "quad_decimate",
            2.0,
        ),
        (
            "/carla_charging_contact_emulator",
            "parking_status_topic",
            "/parking/reverse_parking_controller/status",
        ),
    ),
)
def test_carla_plant_boundary_parameters_fail_closed(
    node, parameter, stale_value
):
    expected = {node: audit.DEVELOP_PARITY_PARAMETERS[node]}
    actual = {node: dict(expected[node])}
    actual[node][parameter] = stale_value

    with pytest.raises(audit.AuditError, match="runtime parameter profile mismatch"):
        audit.select_and_validate_parameters(actual, expected)

    with pytest.raises(audit.AuditError, match="missing node parameter dump"):
        audit.select_and_validate_parameters({}, expected)
