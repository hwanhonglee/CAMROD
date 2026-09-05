"""Offline contracts for the site-by-site CARLA evidence orchestrator."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = SRC_ROOT / "scripts/virtual_carla/run_site_evidence_matrix.sh"


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
    assert "--authority operator|guest" in result.stdout
    assert "--output-root" in result.stdout


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
    assert all(line.endswith("run.sh camping-sites") for line in commands)


def test_guest_plan_selects_visible_usage_complete_authority_and_writes_nothing(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "guest-must-not-exist"
    result = _run(
        "run",
        "--authority",
        "guest",
        "--sites",
        "B3",
        "--output-root",
        str(output_root),
        "--dry-run",
    )

    assert not output_root.exists()
    assert "Frontend authority: guest (guest_browser)" in result.stdout
    assert "camrod_camping_site_matrix_guest_usage_complete" in result.stdout
    assert "국립공원 로봇 서비스" in result.stdout
    assert "run.sh camping-sites-guest" in result.stdout
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
    assert "--authority must be operator or guest" in authority.stderr


def test_static_control_and_retention_contract() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert '"${RUNNER}" "${MATRIX_SUBCOMMAND}" 2>&1 | tee "${matrix_log}"' in source
    assert 'CAMROD_CARLA_CAMPING_SITES="${site}"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites"' in source
    assert 'MATRIX_SUBCOMMAND="camping-sites-guest"' in source
    assert 'EXPECTED_RETURN_SOURCE="guest:usage_complete"' in source
    assert '--ui-window-title "${CAPTURE_UI_TITLE}"' in source
    assert '--ui-kind "${CAPTURE_UI_KIND}"' in source
    assert 'response.get("action") != "usage_complete"' in source
    assert '"visible_guest_page_websocket_via_cdp"' in source
    assert "--retain-source-video false" in source
    assert "--allow-short-capture true" in source
    assert "printf 'q'" in source
    assert 'kill -INT "${CURRENT_RECORDER_PID}"' in source
    assert "camping_site_matrix.json" in source
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
