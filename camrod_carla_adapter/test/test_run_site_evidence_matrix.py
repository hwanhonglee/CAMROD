"""Offline contracts for the site-by-site CARLA evidence orchestrator."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = SRC_ROOT / "scripts/virtual_carla/run_site_evidence_matrix.sh"
DEFAULT_RUNNER = SRC_ROOT / "scripts/virtual_carla/site_access.sh"
DELEGATED_RUNNER = SRC_ROOT / "scripts/virtual_carla/run.sh"
MATRIX_DOC = SRC_ROOT / "docs/evidence/virtual_carla/CAMPING_SITE_MATRIX.md"
VIRTUAL_DOC = SRC_ROOT / "docs/virtual_carla.md"


def _environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in (
        "RANGER_CARLA_ROOT",
        "RANGER_ENV_FILE",
        "RANGER_EVIDENCE_ROOT",
        "CAMROD_VIRTUAL_CARLA_ENTRYPOINT",
        "DISPLAY",
        "XAUTHORITY",
    ):
        environment.pop(name, None)
    return environment


def _run(*arguments: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(SCRIPT), *arguments],
        cwd="/tmp",
        env=_environment(),
        check=check,
        capture_output=True,
        text=True,
    )


def test_script_is_executable_and_help_is_offline() -> None:
    assert SCRIPT.is_file()
    assert SCRIPT.stat().st_mode & 0o111
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)

    result = _run("--help")
    assert "Usage:" in result.stdout
    assert "--sites" in result.stdout
    assert "--authority operator|operator-browser|guest" in result.stdout
    assert "--mission-intent delivery|recall" in result.stdout
    assert "--output-root" in result.stdout
    assert "--phase-timeout-s" in result.stdout
    assert "--retain-source-video" in result.stdout
    assert "--capture-fps" in result.stdout
    assert "--wheel-rate-hz" in result.stdout
    assert "wheel_summary" in result.stdout
    assert "<output>.strict_validation" in result.stdout


def test_default_lifecycle_chain_is_parseable_and_checked_before_live_evidence() -> None:
    """The site wrapper and its delegate must fail before evidence or motion."""

    subprocess.run(["bash", "-n", str(DEFAULT_RUNNER)], check=True)
    subprocess.run(["bash", "-n", str(DELEGATED_RUNNER)], check=True)
    source = SCRIPT.read_text(encoding="utf-8")
    default_selection = 'DEFAULT_RUNNER="${SCRIPT_DIR}/site_access.sh"'
    runner_selection = 'RUNNER="${DEFAULT_RUNNER_REAL}"'
    syntax_check = 'if ! bash -n "${lifecycle_shell}"; then'
    capture_preflight = '[[ -x "${CAPTURE_SCRIPT}" ]]'

    assert default_selection in source
    assert runner_selection in source
    assert "refuses noncanonical CAMROD_VIRTUAL_CARLA_ENTRYPOINT" in source
    assert "pin_default_site_access_map" in source
    assert 'lifecycle_shells+=("${SCRIPT_DIR}/run.sh")' in source
    assert syntax_check in source
    assert "lifecycle runner has invalid shell syntax" in source
    assert source.index(syntax_check) < source.index(capture_preflight)


def test_live_evidence_rejects_noncanonical_runner_before_output_creation(
    tmp_path: Path,
) -> None:
    malformed_runner = tmp_path / "run.sh"
    malformed_runner.write_text(
        "#!/usr/bin/env bash\nprintf 'unterminated\n",
        encoding="utf-8",
    )
    malformed_runner.chmod(0o755)
    output_root = tmp_path / "must-not-be-created"
    environment = _environment()
    environment["CAMROD_VIRTUAL_CARLA_ENTRYPOINT"] = str(malformed_runner)

    result = subprocess.run(
        [
            str(SCRIPT),
            "run",
            "--sites",
            "B1",
            "--output-root",
            str(output_root),
        ],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "refuses noncanonical CAMROD_VIRTUAL_CARLA_ENTRYPOINT" in result.stderr
    assert not output_root.exists()


def test_dry_plan_preserves_site_order_and_writes_nothing(tmp_path: Path) -> None:
    output_root = tmp_path / "must-not-exist"
    result = _run(
        "run",
        "--sites",
        "B13,B2,B7",
        "--output-root",
        str(output_root),
        "--dry-run",
    )

    assert not output_root.exists()
    assert "PLAN ONLY" in result.stdout
    assert "Selected sites (strict order): B13,B2,B7" in result.stdout
    commands = [
        line.strip()
        for line in result.stdout.splitlines()
        if "CAMROD_CARLA_CAMPING_SITES=" in line
    ]
    assert [line.split("=", 1)[1].split()[0] for line in commands] == [
        "B13",
        "B2",
        "B7",
    ]
    assert all(line.endswith("site_access.sh camping-sites") for line in commands)
    assert "CARLA map profile: woraksan-camrod-site-geometry-v15" in result.stdout
    assert "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15" in result.stdout


def test_default_entrypoint_self_pins_v15_over_stale_ranger_environment(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    config_root = ranger_root / "config"
    config_root.mkdir(parents=True)
    environment_file = config_root / "environment.env"
    environment_file.write_text(
        "CAMROD_CARLA_MAP_PROFILE=stale-profile\n"
        "CARLA_UE_MAP=/Game/map_package/Maps/stale/stale\n"
        "CARLA_TOWN=map_package/Maps/stale/stale\n",
        encoding="utf-8",
    )
    environment = _environment()
    environment.update(
        {
            "RANGER_CARLA_ROOT": str(ranger_root),
            "RANGER_ENV_FILE": str(environment_file),
        }
    )

    result = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B2"],
        cwd="/tmp",
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

    expected_name = "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v15"
    expected_town = f"map_package/Maps/{expected_name}/{expected_name}"
    assert "Lifecycle entrypoint:" in result.stdout
    assert result.stdout.count("site_access.sh") >= 2
    assert "CARLA map profile: woraksan-camrod-site-geometry-v15" in result.stdout
    assert f"CARLA map: /Game/{expected_town}" in result.stdout
    assert f"CARLA town: {expected_town}" in result.stdout
    assert "stale-profile" not in result.stdout
    assert "/Maps/stale/stale" not in result.stdout


def test_default_entrypoint_supports_explicit_legacy_profile_over_stale_env(
    tmp_path: Path,
) -> None:
    environment_file = tmp_path / "environment.env"
    environment_file.write_text(
        "CARLA_UE_MAP=/Game/map_package/Maps/stale/stale\n"
        "CARLA_TOWN=map_package/Maps/stale/stale\n",
        encoding="utf-8",
    )
    environment = _environment()
    environment.update(
        {
            "RANGER_ENV_FILE": str(environment_file),
            "CAMROD_CARLA_MAP_PROFILE": "woraksan-camrod-site-geometry-v13",
        }
    )

    result = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B1"],
        cwd="/tmp",
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "CARLA map profile: woraksan-camrod-site-geometry-v13" in result.stdout
    assert "Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13" in result.stdout
    assert "/Maps/stale/stale" not in result.stdout


def test_default_entrypoint_rejects_explicit_conflicting_map_or_profile() -> None:
    conflicting_map_environment = _environment()
    conflicting_map_environment["CARLA_UE_MAP"] = "/Game/map_package/Maps/other/other"
    conflicting_map = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B2"],
        cwd="/tmp",
        env=conflicting_map_environment,
        capture_output=True,
        text=True,
    )
    assert conflicting_map.returncode != 0
    assert "refuses conflicting CARLA_UE_MAP" in conflicting_map.stderr

    unsupported_profile_environment = _environment()
    unsupported_profile_environment["CAMROD_CARLA_MAP_PROFILE"] = "unsupported"
    unsupported_profile = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B2"],
        cwd="/tmp",
        env=unsupported_profile_environment,
        capture_output=True,
        text=True,
    )
    assert unsupported_profile.returncode != 0
    assert "unsupported site-access map profile" in unsupported_profile.stderr


def test_explicit_noncanonical_lifecycle_entrypoint_is_rejected(tmp_path: Path) -> None:
    custom_runner = tmp_path / "custom-entrypoint.sh"
    environment = _environment()
    environment["CAMROD_VIRTUAL_CARLA_ENTRYPOINT"] = str(custom_runner)

    result = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B2"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "refuses noncanonical CAMROD_VIRTUAL_CARLA_ENTRYPOINT" in result.stderr
    assert "PLAN ONLY" not in result.stdout


def test_explicit_canonical_lifecycle_entrypoint_is_accepted() -> None:
    environment = _environment()
    environment["CAMROD_VIRTUAL_CARLA_ENTRYPOINT"] = str(DEFAULT_RUNNER)

    result = subprocess.run(
        [str(SCRIPT), "plan", "--sites", "B2"],
        cwd="/tmp",
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "site_access.sh camping-sites" in result.stdout
    assert "CARLA map profile: woraksan-camrod-site-geometry-v15" in result.stdout


def test_guest_plan_selects_visible_usage_complete_authority_and_writes_nothing(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "guest-must-not-exist"
    result = _run(
        "run",
        "--authority",
        "guest",
        "--mission-intent",
        "recall",
        "--sites",
        "B3",
        "--output-root",
        str(output_root),
        "--dry-run",
    )

    assert not output_root.exists()
    assert "Frontend authority: guest (guest_browser)" in result.stdout
    assert "Mission intent: recall" in result.stdout
    assert "camrod_camping_site_matrix_guest_usage_complete" in result.stdout
    assert "국립공원 로봇 서비스" in result.stdout
    assert "site_access.sh camping-sites-guest" in result.stdout
    assert "guest:usage_complete" in result.stdout


def test_plan_rejects_duplicate_or_out_of_range_sites() -> None:
    duplicate = _run("plan", "--sites", "B2,B2", check=False)
    assert duplicate.returncode != 0
    assert "duplicate site: B2" in duplicate.stderr

    invalid = _run("plan", "--sites", "B0,B14", check=False)
    assert invalid.returncode != 0
    assert "expected an uppercase site in B1..B13" in invalid.stderr

    authority = _run("plan", "--authority", "synthetic", check=False)
    assert authority.returncode != 0
    assert "--authority must be operator, operator-browser, or guest" in authority.stderr

    intent = _run("plan", "--mission-intent", "tour", check=False)
    assert intent.returncode != 0
    assert "--mission-intent must be delivery or recall" in intent.stderr

    guest_delivery = _run("plan", "--authority", "guest", check=False)
    assert guest_delivery.returncode != 0
    assert "--authority guest requires --mission-intent recall" in guest_delivery.stderr


def test_operator_recall_plan_selects_typed_recall_matrix(tmp_path: Path) -> None:
    output_root = tmp_path / "operator-recall-must-not-exist"
    result = _run(
        "plan",
        "--authority",
        "operator",
        "--mission-intent",
        "recall",
        "--sites",
        "B11",
        "--output-root",
        str(output_root),
    )

    assert not output_root.exists()
    assert "Mission intent: recall" in result.stdout
    assert "camrod_camping_site_matrix_operator_recall" in result.stdout
    assert "site_access.sh camping-sites-recall" in result.stdout


def test_operator_browser_plan_selects_visible_robot_ui_and_cdp_input_matrix(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "operator-browser-must-not-exist"
    result = _run(
        "plan",
        "--authority",
        "operator-browser",
        "--mission-intent",
        "recall",
        "--sites",
        "B13",
        "--output-root",
        str(output_root),
    )

    assert not output_root.exists()
    assert "Frontend authority: operator-browser (operator_browser)" in result.stdout
    assert "Robot UI" in result.stdout
    assert "local-only CDP 9224" in result.stdout
    assert "site_access.sh camping-sites-browser-recall" in result.stdout
    assert "ws:usage_complete:site_exit_first" in result.stdout


def test_plan_renders_explicit_timeout_capture_and_mp4_policy(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "explicit-options-must-not-exist"
    result = _run(
        "plan",
        "--sites",
        "B11",
        "--output-root",
        str(output_root),
        "--phase-timeout-s",
        "1200",
        "--capture-fps",
        "2",
        "--gif-fps",
        "10",
        "--derived-width",
        "960",
        "--wheel-rate-hz",
        "20",
        "--retain-source-video",
        "true",
    )

    assert not output_root.exists()
    assert "Phase timeout: 1200 s per matrix wait" in result.stdout
    assert "Capture: 2 FPS, GIF 10 FPS, width 960px" in result.stdout
    assert "Physical-wheel observer: 20 Hz" in result.stdout
    assert "Retain source MP4: true" in result.stdout
    assert "--phase-timeout-s '1200'" in result.stdout
    assert "--retain-source-video 'true'" in result.stdout
    assert "<output>/summary/" in result.stdout
    assert "<output>/<site>/wheel_summary/" in result.stdout
    assert "wheel_measurements.csv" in result.stdout


def test_plan_rejects_invalid_timeout_or_mp4_policy() -> None:
    timeout = _run("plan", "--phase-timeout-s", "3600.1", check=False)
    assert timeout.returncode != 0
    assert "phase timeout must be in (0, 3600]" in timeout.stderr

    retention = _run(
        "plan", "--retain-source-video", "sometimes", check=False
    )
    assert retention.returncode != 0
    assert "--retain-source-video must be true or false" in retention.stderr


def test_static_control_and_retention_contract() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert '"${RUNNER}" "${MATRIX_SUBCOMMAND}" 2>&1 | tee "${matrix_log}"' in source
    assert 'CAMROD_CARLA_CAMPING_SITES="${site}"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites-recall"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites-guest"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites-browser"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites-browser-recall"' in source
    assert 'EXPECTED_RETURN_SOURCE="guest:usage_complete"' in source
    assert 'EXPECTED_RETURN_SOURCE="ws:usage_complete:site_exit_first"' in source
    assert '--ui-window-title "${CAPTURE_UI_TITLE}"' in source
    assert '--ui-kind "${CAPTURE_UI_KIND}"' in source
    assert 'response.get("action") != "usage_complete"' in source
    assert '"visible_guest_page_websocket_via_cdp"' in source
    assert '"visible_operator_page_websocket_via_cdp_input"' in source
    assert '--retain-source-video "${RETAIN_SOURCE_VIDEO}"' in source
    assert "--allow-short-capture true" in source
    assert 'CAMROD_CARLA_MATRIX_PHASE_TIMEOUT_S="${PHASE_TIMEOUT_S}"' in source
    assert 'CAMROD_CARLA_MATRIX_MISSION_INTENT="${MISSION_INTENT}"' in source
    assert 'matrix_scope.get("mission_intent") != mission_intent' in source
    assert '"entrypoint": artifact(entrypoint)' in source
    assert '"only_command": f"site_access.sh {matrix_subcommand}"' in source
    assert '"argv": ["site_access.sh", matrix_subcommand]' in source
    assert 'matrix_item.get("service_mode") != "roadside_stop"' in source
    assert "printf 'q'" in source
    assert 'kill -INT "${CURRENT_RECORDER_PID}"' in source
    assert '"INTERRUPTED" "${CURRENT_SITE}"' in source
    assert "camping_site_matrix.json" in source
    assert "summarize_camping_site_metrics.py" in source
    assert "summarize_physical_wheel_telemetry.py" in source
    assert "camping_site_metrics.json" in source
    assert "--require-all-sites" in source
    assert '"metrics_summary": metrics_summary' in source
    assert '"physical_wheel_summaries": physical_wheel_summaries' in source
    assert "sha256" in source

    # The wrapper may observe and invoke the existing matrix, but must not add
    # a second control path of its own.
    for forbidden in (
        "ros2 topic pub",
        "ros2 service call",
        "carla.VehicleControl",
        "apply_control(",
        "/ui/stop",
        "/ui/engage",
    ):
        assert forbidden not in source


def test_current_v27_documented_command_has_no_presource_and_exact_retention_policy() -> None:
    matrix_doc = MATRIX_DOC.read_text(encoding="utf-8")
    virtual_doc = VIRTUAL_DOC.read_text(encoding="utf-8")

    assert "source ./scripts/virtual_carla/env.sh" not in matrix_doc
    assert 'export RANGER_EVIDENCE_ROOT="${RANGER_CARLA_ROOT}/.work/evidence"' in matrix_doc
    assert "runner 기본값은 `false`" in matrix_doc
    assert (
        "`capture_ui_evidence.sh`를 직접 호출할 때의 "
        "`--retain-source-video` 기본값은 `true`"
    ) in matrix_doc
    assert "noncanonical `CAMROD_VIRTUAL_CARLA_ENTRYPOINT`" in matrix_doc
    assert "noncanonical `CAMROD_VIRTUAL_CARLA_ENTRYPOINT` override" in virtual_doc


def test_wheel_summary_runs_after_recorder_shutdown_and_is_manifest_bound() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    recorder_wait = 'wait "${CURRENT_RECORDER_PID}"\n  recorder_status=$?'
    summary_invocation = (
        'python3 "${WHEEL_SUMMARY_SCRIPT}" \\\n'
        '    --input "${wheel_output}"'
    )
    site_manifest_invocation = 'if ! finalize_site_manifest "${site}"'
    assert recorder_wait in source
    assert summary_invocation in source
    assert site_manifest_invocation in source
    assert source.index(recorder_wait) < source.index(summary_invocation)
    assert source.index(summary_invocation) < source.index(site_manifest_invocation)

    assert '--output-dir "${wheel_summary_dir}"' in source
    assert 'wheel_summary_dir="${site_dir}/wheel_summary"' in source
    assert 'wheel_summary_status="${wheel_summary_pipeline_status[0]}"' in source
    assert 'wheel_summary_tee_status="${wheel_summary_pipeline_status[1]}"' in source
    assert 'physical-wheel summarizer exited {wheel_summary_exit}' in source
    assert 'physical-wheel summary log tee exited {wheel_summary_tee_exit}' in source
    assert '"wheel_summary.json"' in source
    assert '"wheel_measurements.csv"' in source
    assert '"wheel_summary.png"' in source
    assert '"SHA256SUMS"' in source
    assert 'wheel_summary_record["artifacts"][filename] = artifact(path)' in source
    assert 'wheel_record["summary"] = wheel_summary_record' in source
    assert 'physical-wheel summary source SHA-256 mismatch' in source
    assert 'physical-wheel summary sample count mismatch' in source
    assert 'physical-wheel summary actor identity mismatch' in source
    assert 'physical-wheel SHA256SUMS mismatch' in source


def test_embedded_manifest_validators_parse_as_python() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    marker = "<<'PY'"
    offset = 0
    bodies = []
    while True:
        marker_offset = source.find(marker, offset)
        if marker_offset < 0:
            break
        body_start = source.index("\n", marker_offset) + 1
        body_end = source.index("\nPY\n", body_start)
        bodies.append(source[body_start:body_end])
        offset = body_end + len("\nPY\n")

    assert len(bodies) == 4
    for index, body in enumerate(bodies, start=1):
        compile(body, f"<run-site-evidence-heredoc-{index}>", "exec")


def test_runner_requires_current_v27_strict_collection_validation() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert (
        'STRICT_VALIDATOR="${SCRIPT_DIR}/validate_site_evidence_collection.py"'
        in source
    )
    assert '--output-dir "${STRICT_VALIDATION_DIR}"' in source
    assert '--sites "${SITES_CSV}"' in source
    assert '--authority "${AUTHORITY}"' in source
    assert '--mission-intent "${MISSION_INTENT}"' in source
    validator_offset = source.index('python3 "${STRICT_VALIDATOR}"')
    assert source.index('write_run_manifest "VALIDATING" "" "" 0') < validator_offset
    assert validator_offset < source.index('write_run_manifest "PASS" "" "" 0')
    assert 'write_run_manifest "FAIL" "" "${FAILURE_REASON}"' in source
