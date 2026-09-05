"""Validate maintained workspace shell entrypoints without changing the host."""

# HH_260810 - Exercise syntax/help contracts only; setup, CAN, Docker, and
# hardware scripts must never mutate a test workstation during CTest.

import os
from pathlib import Path
import re
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
OWNED_SHELL_TOOLS = (
    "setup_camrod.sh",
    "colcon_build.sh",
    "scripts/virtual_carla/env.sh",
    "scripts/virtual_carla/setup.sh",
    "scripts/virtual_carla/build.sh",
    "scripts/virtual_carla/test.sh",
    "scripts/virtual_carla/run.sh",
    "camrod_bringup/scripts/field_test_tool.sh",
    "camrod_bringup/scripts/run_bringup.sh",
    "camrod_platform/scripts/install_can0_service.sh",
    "camrod_platform/scripts/setup_can0.sh",
    "camrod_ui/scripts/sync_frontend_build.sh",
    "camrod_voice/setup_bt_audio.sh",
    "docker/build_module.sh",
    "docker/buildx_camrod.sh",
    "docker/entrypoint.camrod.sh",
    "docker/run_module.sh",
)


def test_maintained_shell_entrypoints_parse() -> None:
    """Every maintained non-vendor shell entrypoint must pass Bash parsing."""
    for relative_path in OWNED_SHELL_TOOLS:
        path = SRC_ROOT / relative_path
        assert path.is_file(), relative_path
        assert path.stat().st_mode & 0o111, relative_path
        subprocess.run(
            ["bash", "-n", str(path)],
            cwd=SRC_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )


def test_setup_help_is_non_mutating_and_resolves_workspace() -> None:
    """The dependency installer must explain usage before any host action."""
    result = subprocess.run(
        [str(SRC_ROOT / "setup_camrod.sh"), "--help"],
        cwd="/tmp",
        check=True,
        capture_output=True,
        text=True,
    )
    assert "One-time workspace setup" in result.stdout
    assert "./setup_camrod.sh --no-rosdep" in result.stdout


def test_setup_declares_renderer_runtime_dependencies() -> None:
    """No-rosdep setup must still install dependencies of installed renderers."""
    source = (SRC_ROOT / "setup_camrod.sh").read_text(encoding="utf-8")
    package = (SRC_ROOT / "camrod_bringup/package.xml").read_text(encoding="utf-8")
    for dependency in ("python3-numpy", "python3-matplotlib", "python3-pil", "python3-yaml"):
        assert dependency in source
        assert f"<exec_depend>{dependency}</exec_depend>" in package


def test_setup_scopes_optional_rosdep_skip_keys_to_callers() -> None:
    """The generic setup must not silently hide CARLA-only dependencies."""
    source = (SRC_ROOT / "setup_camrod.sh").read_text(encoding="utf-8")
    base_skip_match = re.search(
        r"_ROSDEP_SKIP_KEYS=\(\n(?P<body>.*?)\n  \)", source, re.DOTALL
    )
    assert base_skip_match is not None
    base_skip_keys = base_skip_match.group("body")
    assert "carla_extended_ackermann_control" not in base_skip_keys
    assert "carla_extended_ackermann_msgs" not in base_skip_keys
    assert '_ROSDEP_SKIP_KEYS+=("${CAMROD_PARSED_EXTRA_ROSDEP_SKIP_KEYS[@]}")' in source

    virtual_setup = (
        SRC_ROOT / "scripts/virtual_carla/setup.sh"
    ).read_text(encoding="utf-8")
    assert (
        'CAMROD_EXTRA_ROSDEP_SKIP_KEYS="carla_extended_ackermann_control '
        'carla_extended_ackermann_msgs"'
    ) in virtual_setup


def test_setup_rejects_unsafe_extra_rosdep_skip_keys_before_host_actions() -> None:
    """Caller-provided rosdep keys are parsed as package names, never shell text."""
    environment = os.environ.copy()
    environment["CAMROD_EXTRA_ROSDEP_SKIP_KEYS"] = "valid_key --bad-option"
    result = subprocess.run(
        [str(SRC_ROOT / "setup_camrod.sh"), "--help"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "invalid rosdep skip key" in result.stdout

    environment["CAMROD_EXTRA_ROSDEP_SKIP_KEYS"] = "valid_key;touch_/tmp/never"
    result = subprocess.run(
        [str(SRC_ROOT / "setup_camrod.sh"), "--help"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "invalid rosdep skip key" in result.stdout

    environment["CAMROD_EXTRA_ROSDEP_SKIP_KEYS"] = (
        "carla_extended_ackermann_control carla_extended_ackermann_msgs"
    )
    result = subprocess.run(
        [str(SRC_ROOT / "setup_camrod.sh"), "--help"],
        cwd="/tmp",
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    assert "One-time workspace setup" in result.stdout


def test_frontend_sync_publishes_complete_assets_before_atomic_index() -> None:
    """A running kiosk must never observe an index before its assets exist."""
    sync_source = (
        SRC_ROOT / "camrod_ui/scripts/sync_frontend_build.sh"
    ).read_text(encoding="utf-8")
    sync_body = sync_source[
        sync_source.index("sync_build_tree() {"):sync_source.index("\nsynced=0")
    ]

    assert "set -euo pipefail" in sync_source
    assert 'find "$SRC" -type f' in sync_body
    assert 'find "$SRC/static" -type f' in sync_body
    assert sync_body.index('find "$SRC" -type f') < sync_body.index(
        'publish_index "$dst"'
    )
    assert sync_body.index('find "$SRC/static" -type f') < sync_body.index(
        'publish_index "$dst"'
    )
    assert 'mv -f "$temporary_index" "$dst/index.html"' in sync_source
