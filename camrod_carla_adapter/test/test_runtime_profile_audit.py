"""Pure tests for the fail-closed live CAMROD profile audit."""

import importlib.util
import json
from pathlib import Path
import shutil
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


def _git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _source_identity_fixture(tmp_path: Path) -> Path:
    root = tmp_path / "src"
    root.mkdir()
    _git(root, "init", "--initial-branch=virtual/carla")
    _git(root, "config", "user.email", "test@example.com")
    _git(root, "config", "user.name", "Test")
    runtime_file = root / "scripts" / "virtual_carla" / "run.sh"
    runtime_file.parent.mkdir(parents=True)
    runtime_file.write_text("runtime-v1\n", encoding="utf-8")
    protected = root / audit.PROTECTED_SOURCE_RELATIVE_PATH
    protected.parent.mkdir(parents=True)
    protected.write_text("user-generated-v1\n", encoding="utf-8")
    _git(root, "add", "scripts/virtual_carla/run.sh", str(protected.relative_to(root)))
    _git(root, "commit", "-m", "baseline")
    _git(root, "update-ref", audit.ORIGIN_DEVELOP_REF, "HEAD")
    return root


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


def test_source_identity_is_clean_current_develop_and_excludes_protected_file(
    tmp_path,
):
    root = _source_identity_fixture(tmp_path)
    first = audit.audit_source_identity(root)
    assert first["branch"] == "virtual/carla"
    assert first["ahead_of_origin_develop"] == 0
    assert first["behind_origin_develop"] == 0
    assert first["merge_base"] == first["origin_develop_commit"]
    assert first["runtime_worktree_clean"] is True
    exclusion = first["explicit_exclusions"]
    assert exclusion == [{
        "path": audit.PROTECTED_SOURCE_RELATIVE_PATH,
        "content_hashed": False,
        "dirty_state_checked": False,
        "reason": "protected user-owned generated Vanjee CMake config",
    }]

    # This exact user-owned path is excluded from both dirty admission and the
    # fingerprint. Changing it must not perturb the accepted source identity.
    protected = root / audit.PROTECTED_SOURCE_RELATIVE_PATH
    protected.write_text("user-generated-v2\n", encoding="utf-8")
    second = audit.audit_source_identity(root)
    assert second["runtime_tree"] == first["runtime_tree"]

    runtime_file = root / "scripts" / "virtual_carla" / "run.sh"
    runtime_file.write_text("dirty-runtime\n", encoding="utf-8")
    with pytest.raises(audit.AuditError, match="runtime source tree is dirty"):
        audit.audit_source_identity(root)


def test_source_identity_accepts_ahead_only_and_rejects_wrong_branch(tmp_path):
    root = _source_identity_fixture(tmp_path)
    runtime_file = root / "scripts" / "virtual_carla" / "run.sh"
    runtime_file.write_text("runtime-v2\n", encoding="utf-8")
    _git(root, "add", "scripts/virtual_carla/run.sh")
    _git(root, "commit", "-m", "virtual overlay")

    identity = audit.audit_source_identity(root)
    assert identity["ahead_of_origin_develop"] == 1
    assert identity["behind_origin_develop"] == 0
    assert identity["merge_base"] == identity["origin_develop_commit"]

    _git(root, "branch", "-m", "wrong-branch")
    with pytest.raises(audit.AuditError, match="expected 'virtual/carla'"):
        audit.audit_source_identity(root)


def test_source_identity_rejects_when_origin_develop_is_not_fully_merged(
    tmp_path,
):
    root = _source_identity_fixture(tmp_path)
    _git(root, "switch", "--create", "upstream-change")
    runtime_file = root / "scripts" / "virtual_carla" / "run.sh"
    runtime_file.write_text("upstream-v2\n", encoding="utf-8")
    _git(root, "add", "scripts/virtual_carla/run.sh")
    _git(root, "commit", "-m", "new develop")
    _git(root, "update-ref", audit.ORIGIN_DEVELOP_REF, "HEAD")
    _git(root, "switch", "virtual/carla")

    with pytest.raises(audit.AuditError, match="does not contain the exact"):
        audit.audit_source_identity(root)


def test_install_fingerprint_covers_files_and_symlink_targets_but_not_caches(
    tmp_path,
):
    install = tmp_path / "install"
    target = tmp_path / "built-node"
    target.write_bytes(b"binary-v1")
    for package in audit.RUNTIME_INSTALL_PREFIXES:
        prefix = install / package
        prefix.mkdir(parents=True)
        (prefix / "runtime.txt").write_text(
            f"{package}-v1\n", encoding="utf-8"
        )
        cache = prefix / "__pycache__"
        cache.mkdir()
        (cache / "ignored.pyc").write_bytes(b"cache-v1")
    (install / "camrod_control" / "node").symlink_to(target)

    first = audit.fingerprint_runtime_install(install)
    second = audit.fingerprint_runtime_install(install)
    assert second == first
    assert set(first["prefixes"]) == set(audit.RUNTIME_INSTALL_PREFIXES)
    assert first["prefixes"]["camrod_control"]["symlink_count"] == 1

    (install / "camrod_ui" / "__pycache__" / "ignored.pyc").write_bytes(
        b"cache-v2"
    )
    assert audit.fingerprint_runtime_install(install)["sha256"] == first["sha256"]

    target.write_bytes(b"binary-v2")
    assert audit.fingerprint_runtime_install(install)["sha256"] != first["sha256"]


def _ui_identity_fixture(tmp_path: Path) -> tuple[Path, Path]:
    source = tmp_path / "src"
    install = tmp_path / "install"
    frontend = (
        source
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
    )
    (frontend / "src").mkdir(parents=True)
    (frontend / "public").mkdir()
    (frontend / "package.json").write_text("{}\n", encoding="utf-8")
    (frontend / "package-lock.json").write_text("{}\n", encoding="utf-8")
    (frontend / "camrod-build-env.json").write_text(
        json.dumps(audit.CANONICAL_UI_BUILD_ENV, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    (frontend / "src" / "App.js").write_text("app-v1\n", encoding="utf-8")
    (frontend / "public" / "base.html").write_text("base\n", encoding="utf-8")
    build = frontend / "build"
    bundle = build / "static" / "js" / "main.abc123.js"
    bundle.parent.mkdir(parents=True)
    bundle.write_text("bundle-v1\n", encoding="utf-8")
    (build / "index.html").write_text(
        '<script defer src="/static/js/main.abc123.js"></script>\n',
        encoding="utf-8",
    )
    (build / ".camrod-inputs.sha256").write_text(
        audit.frontend_source_input_fingerprint(frontend) + "\n",
        encoding="ascii",
    )
    shutil.copy2(
        frontend / "camrod-build-env.json",
        build / ".camrod-build-env.json",
    )
    installed_build = (
        install
        / "camrod_ui"
        / "share"
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
        / "build"
    )
    shutil.copytree(build, installed_build)
    return source, install


def test_ui_identity_records_installed_artifacts_and_rejects_source_mismatch(
    tmp_path,
):
    source, install = _ui_identity_fixture(tmp_path)
    identity = audit.audit_installed_ui_identity(source, install)
    assert identity["source_install_match"] is True
    assert identity["installed_inputs_stamp"]["path"].endswith(
        "/.camrod-inputs.sha256"
    )
    assert identity["canonical_build_environment"] == (
        audit.CANONICAL_UI_BUILD_ENV
    )
    assert identity["installed_build_environment"]["path"].endswith(
        "/.camrod-build-env.json"
    )
    assert identity["installed_index"]["path"].endswith("/index.html")
    assert identity["installed_main_bundle"]["path"].endswith(
        "/static/js/main.abc123.js"
    )
    for artifact in (
        identity["installed_inputs_stamp"],
        identity["installed_index"],
        identity["installed_main_bundle"],
    ):
        assert len(artifact["file_sha256"] if "file_sha256" in artifact else artifact["sha256"]) == 64

    frontend = (
        source
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
    )
    (frontend / "src" / "App.js").write_text("app-v2\n", encoding="utf-8")
    with pytest.raises(audit.AuditError, match="UI source/build mismatch"):
        audit.audit_installed_ui_identity(source, install)


def test_ui_identity_rejects_dotenv_and_noncanonical_build_environment(tmp_path):
    source, install = _ui_identity_fixture(tmp_path)
    frontend = (
        source
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
    )
    (frontend / ".env.production").write_text(
        "PUBLIC_URL=/host-dependent\n", encoding="utf-8"
    )
    with pytest.raises(audit.AuditError, match="CRA .env files are forbidden"):
        audit.audit_installed_ui_identity(source, install)
    (frontend / ".env.production").unlink()

    installed_env = (
        install
        / "camrod_ui"
        / "share"
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
        / "build"
        / ".camrod-build-env.json"
    )
    altered = dict(audit.CANONICAL_UI_BUILD_ENV)
    altered["REACT_APP_OPERATING_HOURS_START"] = "8"
    installed_env.write_text(json.dumps(altered), encoding="utf-8")
    with pytest.raises(audit.AuditError, match="not the exact canonical"):
        audit.audit_installed_ui_identity(source, install)


def test_ui_identity_rejects_installed_bundle_content_mismatch(tmp_path):
    source, install = _ui_identity_fixture(tmp_path)
    installed_bundle = (
        install
        / "camrod_ui"
        / "share"
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
        / "build"
        / "static"
        / "js"
        / "main.abc123.js"
    )
    installed_bundle.write_text("stale-bundle\n", encoding="utf-8")
    with pytest.raises(audit.AuditError, match="main bundle content differs"):
        audit.audit_installed_ui_identity(source, install)


def test_current_site_geometry_map_is_exactly_v15():
    assert audit.SITE_GEOMETRY_ALLOWED_CARLA_MAPS == (
        audit.SITE_GEOMETRY_CURRENT_CARLA_MAP,
    )
    v15 = audit.SITE_GEOMETRY_CURRENT_CARLA_MAP
    assert v15.endswith(
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15/"
        "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15"
    )
    audit.validate_audited_map(
        "develop-plus-carla-site-geometry-v27", v15
    )
    for legacy_version in ("v13", "v12", "v11"):
        legacy = v15.replace("v15", legacy_version)
        with pytest.raises(audit.AuditError, match="exact v15 map"):
            audit.validate_audited_map(
                "develop-plus-carla-site-geometry-v27", legacy
            )
    with pytest.raises(audit.AuditError, match="exact v15 map"):
        audit.validate_audited_map(
            "develop-plus-carla-site-geometry-v27",
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
    assert discovered["profile"] == "develop-plus-carla-site-geometry-v27"
    assert (
        audit.resolve_audited_profile("auto", discovered["profile"])
        == "develop-plus-carla-site-geometry-v27"
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
    assert set(site) == set(parity) | {charger}
    command_adapter = "/camrod_twist_to_4ws"
    recovery_nodes = {
        "/control/cmd_vel_safety_gate",
        "/control/route_safety_recovery_controller",
        "/planning/goal_snapper",
        "/planning/controller_server",
        "/planning/nav2_selector_latch",
        "/planning/planning_state_machine",
        "/parking/apriltag_parking_controller",
        "/perception/yolov9mit",
        "/ui_backend",
    }
    for node in parity:
        if node not in ({controller, command_adapter} | recovery_nodes):
            assert site[node] == parity[node]

    assert charger not in parity
    assert site[charger] == audit.CARLA_CHARGING_CONTACT_PARAMETERS == {
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
        "allow_manual_departure_while_charging"
    ] is False
    assert site["/control/cmd_vel_safety_gate"][
        "allow_manual_departure_while_charging"
    ] is True
    for profile in (parity, site):
        assert profile["/control/cmd_vel_safety_gate"][
            "manual_charging_departure_command_timeout_s"
        ] == 0.35
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
    assert parity["/control/cmd_vel_safety_gate"][
        "lanelet_safety_check_reverse"
    ] is False
    assert site["/control/cmd_vel_safety_gate"][
        "lanelet_safety_check_reverse"
    ] is True
    base_bypass = (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
        "ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT"
    )
    site_bypass = (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,"
        "ALIGN_RETRACE_YAW,ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
    )
    for parameter in (
        "camping_site_maneuver_controller_static_bypass_phases",
        "camping_site_maneuver_controller_lanelet_bypass_phases",
    ):
        assert parity["/control/cmd_vel_safety_gate"][parameter] == base_bypass
        assert "ALIGN_OUTBOUND_LANE_YAW" not in (
            parity["/control/cmd_vel_safety_gate"][parameter].split(",")
        )
        assert site["/control/cmd_vel_safety_gate"][parameter] == site_bypass
        assert site["/control/cmd_vel_safety_gate"][parameter].split(",").count(
            "ALIGN_OUTBOUND_LANE_YAW"
        ) == 1
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
    assert parity["/perception/yolov9mit"]["min_confidence"] == 0.5
    assert site["/perception/yolov9mit"]["min_confidence"] == 0.95
    assert parity["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] is False
    assert site["/ui_backend"][
        "telemetry_docking_rear_camera_fallback_enabled"
    ] is True
    assert parity["/ui_backend"]["return_site_exit_rearm_enabled"] is False
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
    assert parity["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] == ""
    assert site["/planning/goal_snapper"][
        "reverse_auxiliary_input_goal_topic"
    ] == "/planning/auto_reverse_goal_raw"
    assert parity["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] == "RPP"
    assert site["/planning/nav2_selector_latch"][
        "reverse_controller_id"
    ] == "RPPReverse"
    assert parity["/planning/planning_state_machine"] == {
        "return_goal_reached_distance_m": 0.30,
    }
    assert site["/planning/planning_state_machine"] == {
        "return_goal_reached_distance_m": 0.35,
    }
    assert site["/planning/controller_server"]["controller_plugins"] == [
        "RPP", "RPPReverse", "RotationShim"
    ]
    assert site["/planning/controller_server"][
        "RPPReverse.allow_reversing"
    ] is True
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
        "roadside_reverse_return_enable": True,
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
        "develop-plus-carla-site-geometry-v27",
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
    profile = (
        audit.DEVELOP_SITE_GEOMETRY_PARAMETERS
        if node == "/carla_charging_contact_emulator"
        else audit.DEVELOP_PARITY_PARAMETERS
    )
    expected = {node: profile[node]}
    actual = {node: dict(expected[node])}
    actual[node][parameter] = stale_value

    with pytest.raises(audit.AuditError, match="runtime parameter profile mismatch"):
        audit.select_and_validate_parameters(actual, expected)

    with pytest.raises(audit.AuditError, match="missing node parameter dump"):
        audit.select_and_validate_parameters({}, expected)
