"""Validate maintained workspace shell entrypoints without changing the host."""

# HH_260810 - Exercise syntax/help contracts only; setup, CAN, Docker, and
# hardware scripts must never mutate a test workstation during CTest.

from pathlib import Path
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
OWNED_SHELL_TOOLS = (
    "setup_camrod.sh",
    "colcon_build.sh",
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
