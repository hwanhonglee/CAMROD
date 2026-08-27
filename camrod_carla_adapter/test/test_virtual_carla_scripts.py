"""Portable, non-mutating contracts for the virtual CARLA entrypoints."""

import os
from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parent
SCRIPT_ROOT = SRC_ROOT / "scripts" / "virtual_carla"
SCRIPTS = ("env.sh", "setup.sh", "build.sh", "test.sh", "run.sh")
VIRTUAL_ENV_KEYS = (
    "RANGER_CARLA_ROOT",
    "RANGER_WORK_ROOT",
    "RANGER_ROS_WS",
    "RANGER_ENV_FILE",
    "RANGER_EVIDENCE_ROOT",
    "RANGER_BASELINE_MANIFEST",
    "RANGER_PHYSICAL_MANIFEST",
    "RANGER_CARLA_PYTHON_EGG",
    "RANGER_PYTHON_EGG_CACHE",
    "CARLA_ROOT",
    "CARLA_ROS_BRIDGE_WS",
    "RANGER_ROS_BRIDGE_WS",
    "CARLA_PYTHON_EGG",
    "CARLA_PYTHON_EGG_CACHE",
    "UE_ROOT",
    "RANGER_UE_ROOT",
)


def _bash(script: str, *, environment=None) -> subprocess.CompletedProcess:
    merged_environment = os.environ.copy()
    for name in VIRTUAL_ENV_KEYS:
        merged_environment.pop(name, None)
    if environment:
        merged_environment.update(environment)
    return subprocess.run(
        ["bash", "-c", script],
        cwd=SRC_ROOT,
        env=merged_environment,
        check=True,
        capture_output=True,
        text=True,
    )


def test_entrypoints_are_executable_and_parse() -> None:
    for filename in SCRIPTS:
        path = SCRIPT_ROOT / filename
        assert path.is_file(), filename
        assert path.stat().st_mode & 0o111, filename
        subprocess.run(
            ["bash", "-n", str(path)],
            check=True,
            capture_output=True,
            text=True,
        )


def test_help_is_available_without_external_workspaces() -> None:
    for filename in ("setup.sh", "build.sh", "test.sh", "run.sh"):
        result = subprocess.run(
            [str(SCRIPT_ROOT / filename), "--help"],
            cwd="/tmp",
            check=True,
            capture_output=True,
            text=True,
        )
        assert "Usage:" in result.stdout


def test_scripts_and_composition_launches_embed_no_developer_home() -> None:
    paths = [SCRIPT_ROOT / filename for filename in SCRIPTS]
    paths += [
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py",
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py",
    ]
    for path in paths:
        source = path.read_text(encoding="utf-8")
        assert "/home/hong" not in source, path
        assert "/home/nvidia" not in source, path


def test_runtime_runner_contains_no_automatic_motion_client() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    forbidden = (
        "ros2 topic pub",
        "ros2 action send_goal",
        "ros2 service call",
    )
    assert not any(token in source for token in forbidden)
    assert "prepare_ros_carla_python bridge" in source
    assert "prepare_ros_carla_python spawn" in source


def test_env_uses_ranger_work_root_and_portable_gate_contract(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    (ranger_root / "ros_ws").mkdir(parents=True)
    script = f"""
set -euo pipefail
source {str(SCRIPT_ROOT / 'env.sh')!r}
printf '%s\\0' \
  "$RANGER_WORK_ROOT" \
  "$RANGER_ROS_WS" \
  "$CARLA_ROOT" \
  "$CARLA_ROS_BRIDGE_WS" \
  "$RANGER_BASELINE_MANIFEST" \
  "$RANGER_PHYSICAL_MANIFEST" \
  "$RANGER_CARLA_PYTHON_EGG" \
  "$CARLA_PYTHON_EGG"
"""
    result = _bash(
        script,
        environment={"RANGER_CARLA_ROOT": str(ranger_root)},
    )
    values = result.stdout.split("\0")
    assert values[-1] == ""
    work_root = ranger_root / ".work"
    assert values[:-1] == [
        str(work_root),
        str(ranger_root / "ros_ws"),
        str(work_root / "src" / "carla"),
        str(work_root / "ros-bridge-ws"),
        str(work_root / "evidence" / "ranger_ros_backend_gate.json"),
        str(
            work_root
            / "evidence"
            / "ranger_physical_4ws_acceptance_gate.json"
        ),
        str(
            work_root
            / "src"
            / "carla"
            / "PythonAPI"
            / "carla"
            / "dist"
            / "carla-0.9.15-py3.10-linux-x86_64.egg"
        ),
        str(
            work_root
            / "src"
            / "carla"
            / "PythonAPI"
            / "carla"
            / "dist"
            / "carla-0.9.15-py3.10-linux-x86_64.egg"
        ),
    ]


def test_ranger_environment_file_loads_but_shell_override_wins(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    config = ranger_root / "config"
    config.mkdir(parents=True)
    configured_carla = ranger_root / ".work" / "configured-carla"
    configured_ue = ranger_root / ".work" / "licensed-unreal-engine"
    configured_bridge = ranger_root / ".work" / "configured-ros-bridge"
    (config / "environment.env").write_text(
        "export RANGER_WORK_ROOT=\"$RANGER_CARLA_ROOT/.work\"\n"
        f"export CARLA_ROOT={configured_carla}\n"
        f"export RANGER_UE_ROOT={configured_ue}\n"
        f"export RANGER_ROS_BRIDGE_WS={configured_bridge}\n",
        encoding="utf-8",
    )
    explicit_carla = tmp_path / "caller-carla"
    script = f"""
set -euo pipefail
source {str(SCRIPT_ROOT / 'env.sh')!r}
printf '%s\\n%s\\n%s\\n' \
  "$CARLA_ROOT" "$UE_ROOT" "$CARLA_ROS_BRIDGE_WS"
"""
    result = _bash(
        script,
        environment={
            "RANGER_CARLA_ROOT": str(ranger_root),
            "CARLA_ROOT": str(explicit_carla),
        },
    )
    assert result.stdout.splitlines() == [
        str(explicit_carla),
        str(configured_ue),
        str(configured_bridge),
    ]


def test_commands_prints_all_explicit_lifecycle_stages(tmp_path: Path) -> None:
    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "commands"],
        cwd="/tmp",
        env={**os.environ, "RANGER_CARLA_ROOT": str(tmp_path)},
        check=True,
        capture_output=True,
        text=True,
    )
    for command in ("server", "bridge", "spawn", "camrod"):
        assert f"run.sh {command}" in result.stdout
    assert "carla_ros_bridge.launch.py" in result.stdout
    assert "carla_spawn_objects.launch.py" in result.stdout
    assert "camrod_carla_full.launch.py" in result.stdout
    assert "No motion command" in result.stdout


def test_doctor_rejects_template_and_packaged_paths_without_traceback(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    config = ranger_root / "config"
    packaged_carla = tmp_path / "CARLA_0.9.15"
    config.mkdir(parents=True)
    packaged_carla.mkdir()
    (packaged_carla / "CarlaUE4.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (packaged_carla / "CarlaUE4.sh").chmod(0o755)
    (config / "environment.env").write_text(
        "RANGER_UE_ROOT=/absolute/path/to/UnrealEngine-4.26.2\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    for name in VIRTUAL_ENV_KEYS:
        environment.pop(name, None)
    environment.update(
        {
            "RANGER_CARLA_ROOT": str(ranger_root),
            "CARLA_ROOT": str(packaged_carla),
        }
    )
    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "doctor"],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )
    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert "CONFIG_PRECEDENCE=" in combined
    assert "RANGER_UE_ROOT is still a template placeholder" in combined
    assert "CARLA_ROOT points to a packaged runtime" in combined
    assert "dependent file, gate, ROS and renderer checks were skipped" in combined
    assert "Traceback" not in combined
    assert "FileNotFoundError" not in combined


def test_doctor_missing_gate_skips_dependent_semantic_checks(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    work_root = ranger_root / ".work"
    ue_root = tmp_path / "UnrealEngine_4.26"
    carla_root = work_root / "src" / "carla"
    bridge_ws = work_root / "ros-bridge-ws"
    ranger_ws = ranger_root / "ros_ws"
    editor = ue_root / "Engine" / "Binaries" / "Linux" / "UE4Editor"
    project = carla_root / "Unreal" / "CarlaUE4" / "CarlaUE4.uproject"
    map_file = (
        carla_root
        / "Unreal"
        / "CarlaUE4"
        / "Content"
        / "map_package"
        / "Maps"
        / "Woraksan_v1_0_3_parking_lot_hegiht_fit"
        / "Woraksan_v1_0_3_parking_lot_hegiht_fit.umap"
    )
    for path in (
        editor,
        project,
        map_file,
        bridge_ws / "install" / "local_setup.bash",
        ranger_ws / "install" / "local_setup.bash",
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n", encoding="utf-8")
    editor.chmod(0o755)
    (bridge_ws / "src" / "ros-bridge").mkdir(parents=True)
    (work_root / "evidence").mkdir(parents=True)
    (ranger_root / "config").mkdir(parents=True)
    (ranger_root / "config" / "environment.env").write_text(
        f"RANGER_WORK_ROOT={work_root}\n"
        f"RANGER_UE_ROOT={ue_root}\n"
        f"CARLA_ROOT={carla_root}\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    for name in VIRTUAL_ENV_KEYS:
        environment.pop(name, None)
    environment["RANGER_CARLA_ROOT"] = str(ranger_root)
    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "doctor"],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )
    combined = result.stdout + result.stderr
    assert result.returncode != 0
    assert f"missing Ranger baseline gate: {work_root}/evidence/" in combined
    assert "dependent JSON, ROS, Python API and renderer checks were skipped" in combined
    assert "CARLA Python API lacks" not in combined
    assert "NVIDIA driver/GPU" not in combined
    assert "Traceback" not in combined
    assert "FileNotFoundError" not in combined


def test_adapter_metadata_has_real_maintainer() -> None:
    package = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert ">hwanhonglee</maintainer>" in package
    assert 'maintainer="hwanhonglee"' in setup
    assert "TODO" not in package
    assert "TODO" not in setup
