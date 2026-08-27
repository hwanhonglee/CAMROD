"""Portable, non-mutating contracts for the virtual CARLA entrypoints."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_ROOT.parent
SCRIPT_ROOT = SRC_ROOT / "scripts" / "virtual_carla"
GATE_VALIDATOR = SCRIPT_ROOT / "validate_runtime_gates.py"
ACTOR_PREFLIGHT = SCRIPT_ROOT / "check_ranger_actor.py"
PHYSICAL_PREFLIGHT = SCRIPT_ROOT / "check_physical_bridge_status.py"
UI_MANUAL_PREFLIGHT = SCRIPT_ROOT / "check_camrod_ui_manual_ready.py"
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
    # env.sh derives these from CARLA_ROOT/UE_ROOT.  Clear parent-shell
    # values in fixture subprocesses so a real local project cannot mask the
    # deliberately incomplete temporary CARLA/UE trees under test.
    "CARLA_UPROJECT",
    "UE_EDITOR",
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

    for path in (
        GATE_VALIDATOR,
        ACTOR_PREFLIGHT,
        PHYSICAL_PREFLIGHT,
        UI_MANUAL_PREFLIGHT,
    ):
        source = path.read_text(encoding="utf-8")
        compile(source, str(path), "exec")


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
        GATE_VALIDATOR,
        ACTOR_PREFLIGHT,
        PHYSICAL_PREFLIGHT,
        UI_MANUAL_PREFLIGHT,
    ]
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


def test_manual_keyboard_path_is_fail_closed_and_uses_camrod_safety_gate() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    physical_source = PHYSICAL_PREFLIGHT.read_text(encoding="utf-8")
    ui_source = UI_MANUAL_PREFLIGHT.read_text(encoding="utf-8")
    manual_case = source.split("  manual)\n", 1)[1].split("    ;;\n", 1)[0]

    assert "teleop_twist_keyboard" in source
    assert "cmd_vel:=/control/nav2_cmd_vel_ros" in source
    assert "-p speed:=0.20" in source
    assert "-p turn:=0.20" in source
    assert "radius = speed / turn = 1.0 m" in source
    assert "validate_runtime_gates" in manual_case
    assert "validate_spawn_file" in manual_case
    assert "validate_ranger_actor_ready" in manual_case
    assert "validate_camrod_ui_manual_ready" in manual_case
    assert "validate_physical_bridge_ready" in manual_case
    assert manual_case.index("validate_ranger_actor_ready") < manual_case.index(
        "validate_camrod_ui_manual_ready"
    )
    assert manual_case.index(
        "validate_camrod_ui_manual_ready"
    ) < manual_case.index("validate_physical_bridge_ready")
    assert manual_case.index("validate_physical_bridge_ready") < manual_case.index(
        "manual_command"
    )
    assert "physical_four_wheel_status" in physical_source
    for field in (
        '"ready": True',
        '"physical_gate_accepted": True',
        '"physx_substep_control_verified": True',
        '"independent_wheel_drive_available": True',
        '"motion_backend": EXPECTED_BACKEND',
    ):
        assert field in physical_source
    assert 'EXPECTED_BACKEND = "PHYSX_FOUR_WHEEL_STEERING"' in physical_source
    assert "actor_id <= 0" in physical_source
    assert "--kill-after=2s 9s" in source
    assert "--timeout-seconds 5" in source
    assert "--kill-after=2s 7s" in source
    assert 'f"{root}/ui/health"' in ui_source
    assert 'f"{root}/ui/state"' in ui_source
    assert 'document.get("ready") is not True' in ui_source
    assert 'document.get("engaged") is not False' in ui_source
    assert 'mission_phase != "READY"' in ui_source
    assert 'mission_source != "none"' in ui_source
    assert "ros2 topic echo" not in source
    assert "this command never engages automatically" in manual_case
    assert "ros2 topic pub" not in manual_case
    assert "ros2 action send_goal" not in manual_case
    assert "/planning/engage" not in manual_case


def _load_physical_preflight_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "physical_bridge_preflight_test", PHYSICAL_PREFLIGHT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_physical_status_validator_accepts_only_fully_ready_bound_actor() -> None:
    module = _load_physical_preflight_module()
    status = {
        "ready": True,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": True,
        "independent_wheel_drive_available": True,
        "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
        "actor_id": 42,
        "reason": "physical steering API ready",
    }

    assert module.validate_status(status, "ego_vehicle") == 42


def test_physical_status_validator_rejects_interlock_with_lifecycle_hint() -> None:
    module = _load_physical_preflight_module()
    status = {
        "ready": False,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": False,
        "independent_wheel_drive_available": False,
        "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
        "actor_id": 0,
        "reason": "exact Ranger actor not found",
    }

    try:
        module.validate_status(status, "ego_vehicle")
    except module.BridgeStatusError as error:
        message = str(error)
    else:
        raise AssertionError("interlocked physical status was accepted")

    assert "ready=False" in message
    assert "actor_id=0" in message
    assert "exact Ranger actor not found" in message
    assert "server -> bridge -> spawn -> camrod" in message


def _load_ui_manual_preflight_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "ui_manual_preflight_test", UI_MANUAL_PREFLIGHT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ui_manual_validator_requires_healthy_ready_idle_state() -> None:
    module = _load_ui_manual_preflight_module()

    module.validate_health({"ok": True, "node": "ui_backend"})
    module.validate_state(
        {
            "ready": True,
            "engaged": False,
            "mission_phase": "READY",
            "mission_source": "none",
        }
    )


def test_ui_manual_validator_rejects_not_ready_or_active_mission() -> None:
    module = _load_ui_manual_preflight_module()
    state = {
        "ready": False,
        "engaged": True,
        "mission_phase": "DRIVING",
        "mission_source": "manual",
        "ready_message": "localization unhealthy",
    }

    try:
        module.validate_state(state)
    except module.UiManualReadyError as error:
        message = str(error)
    else:
        raise AssertionError("active, not-ready UI state was accepted")

    assert "ready=False" in message
    assert "engaged=True" in message
    assert "mission_phase='DRIVING'" in message
    assert "mission_source='manual'" in message
    assert "localization unhealthy" in message
    assert "Press UI STOP" in message


def test_camrod_requires_spawn_contract_and_live_actor_before_main_cache() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    assert "spawn JSON must have exactly one actor id" in source
    assert 'matches[0].get("type") != expected_type' in source
    assert "vehicle.ranger.default <<'PY'" in source
    camrod_case = source.split("  camrod)\n", 1)[1].split("    ;;\n", 1)[0]
    gate_index = camrod_case.index("validate_runtime_gates")
    spawn_index = camrod_case.index("validate_spawn_file")
    actor_index = camrod_case.index("validate_ranger_actor_ready")
    main_cache_index = camrod_case.index('if [[ -z "${CARLA_PYTHON_EGG_CACHE}" ]]')
    assert gate_index < spawn_index < actor_index < main_cache_index

    preflight_function = source.split(
        "validate_ranger_actor_ready() {\n", 1
    )[1].split("\n}\n", 1)[0]
    assert "camrod-carla-actor-preflight.XXXXXX" in preflight_function
    assert 'export PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert 'export CARLA_PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert 'export RANGER_PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert "virtual_carla_use_python_egg" in preflight_function
    assert "check_ranger_actor.py" in preflight_function


def _write_fake_carla_egg(path: Path) -> None:
    source = """
import os

_inventory_calls = 0
_world_calls = 0

class Actor:
    def __init__(self, actor_id, type_id, role_name):
        self.id = actor_id
        self.type_id = type_id
        self.attributes = {"role_name": role_name}

class World:
    def get_actors(self):
        global _inventory_calls
        _inventory_calls += 1
        mode = os.environ.get("FAKE_CARLA_MODE", "one")
        role = os.environ.get("FAKE_CARLA_ROLE", "ego_vehicle")
        if mode == "transient" and _inventory_calls == 1:
            return []
        if mode == "zero":
            return [
                Actor(20, "vehicle.ranger.default", "another_role"),
                Actor(21, "vehicle.other.default", role),
            ]
        if mode == "many":
            return [
                Actor(31, "vehicle.ranger.default", role),
                Actor(32, "vehicle.ranger.default", role),
            ]
        return [Actor(41, "vehicle.ranger.default", role)]

class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port
    def set_timeout(self, timeout):
        self.timeout = timeout
    def get_world(self):
        global _world_calls
        _world_calls += 1
        mode = os.environ.get("FAKE_CARLA_MODE")
        if mode == "endpoint":
            raise RuntimeError("fixture endpoint unavailable")
        if mode == "endpoint_transient" and _world_calls == 1:
            raise RuntimeError("fixture transient endpoint error")
        return World()
""".lstrip()
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("carla/__init__.py", source)


def _run_actor_preflight(
    tmp_path: Path,
    mode: str,
    *,
    accepted_egg: Path | None = None,
    timeout_seconds: str = "0.05",
) -> subprocess.CompletedProcess:
    egg = tmp_path / "fake-carla.egg"
    if not egg.exists():
        _write_fake_carla_egg(egg)
    cache = tmp_path / f"cache-{mode}"
    cache.mkdir()
    environment = os.environ.copy()
    environment.update(
        {
            "PYTHONPATH": str(egg),
            "PYTHON_EGG_CACHE": str(cache),
            "PYTHONDONTWRITEBYTECODE": "1",
            "FAKE_CARLA_MODE": mode,
            "FAKE_CARLA_ROLE": "ego_vehicle",
        }
    )
    return subprocess.run(
        [
            sys.executable,
            str(ACTOR_PREFLIGHT),
            "--host",
            "127.0.0.1",
            "--port",
            "2000",
            "--role-name",
            "ego_vehicle",
            "--accepted-python-egg",
            str(accepted_egg or egg),
            "--private-egg-cache",
            str(cache),
            "--timeout-seconds",
            timeout_seconds,
        ],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_actor_preflight_accepts_exactly_one_bound_ranger(tmp_path: Path) -> None:
    result = _run_actor_preflight(tmp_path, "one")

    assert result.returncode == 0, result.stderr
    assert "Ranger actor preflight ready" in result.stdout
    assert "actor_id=41" in result.stdout
    assert "type=vehicle.ranger.default" in result.stdout
    assert "role_name=ego_vehicle" in result.stdout


def test_actor_preflight_retries_transient_empty_inventory(
    tmp_path: Path,
) -> None:
    result = _run_actor_preflight(
        tmp_path,
        "transient",
        timeout_seconds="0.5",
    )

    assert result.returncode == 0, result.stderr
    assert "Ranger actor preflight ready" in result.stdout
    assert "actor_id=41" in result.stdout


def test_actor_preflight_retries_transient_endpoint_error(
    tmp_path: Path,
) -> None:
    result = _run_actor_preflight(
        tmp_path,
        "endpoint_transient",
        timeout_seconds="0.5",
    )

    assert result.returncode == 0, result.stderr
    assert "Ranger actor preflight ready" in result.stdout
    assert "actor_id=41" in result.stdout


def test_actor_preflight_polls_persistent_zero_until_deadline(
    tmp_path: Path,
) -> None:
    result = _run_actor_preflight(
        tmp_path,
        "zero",
        timeout_seconds="0.45",
    )

    assert result.returncode != 0
    assert "found 0 exact Ranger actors" in result.stderr
    assert "after 2 actor inventory poll" in result.stderr or (
        "after 3 actor inventory poll" in result.stderr
    )
    assert "within 0.45s" in result.stderr
    assert "server -> bridge -> spawn -> camrod" in result.stderr


def test_actor_preflight_rejects_endpoint_zero_and_duplicate_actor(
    tmp_path: Path,
) -> None:
    expected_messages = {
        "endpoint": "cannot read CARLA endpoint 127.0.0.1:2000",
        "zero": "found 0 exact Ranger actors",
        "many": "found 2 exact Ranger actors",
    }
    for mode, expected in expected_messages.items():
        result = _run_actor_preflight(tmp_path, mode)
        assert result.returncode != 0
        assert expected in result.stderr
        assert "server -> bridge -> spawn -> camrod" in result.stderr
        assert "Traceback" not in result.stderr
        if mode == "endpoint":
            assert "last error: RuntimeError: fixture endpoint unavailable" in (
                result.stderr
            )


def test_actor_preflight_rejects_python_api_outside_bound_egg(
    tmp_path: Path,
) -> None:
    other_egg = tmp_path / "other-carla.egg"
    _write_fake_carla_egg(other_egg)

    result = _run_actor_preflight(tmp_path, "one", accepted_egg=other_egg)

    assert result.returncode != 0
    assert "resolved outside the gate-bound egg" in result.stderr
    assert "Traceback" not in result.stderr


def _write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _renderer_server_fixture(tmp_path: Path) -> tuple[dict, Path]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    for command in ("bash", "dirname", "readlink", "python3"):
        target = shutil.which(command)
        assert target is not None, command
        (fake_bin / command).symlink_to(target)
    _write_executable(fake_bin / "nvidia-smi", "#!/bin/bash\nexit 0\n")

    ue_root = tmp_path / "UnrealEngine_4.26"
    editor = ue_root / "Engine" / "Binaries" / "Linux" / "UE4Editor"
    editor.parent.mkdir(parents=True)
    _write_executable(
        editor,
        "#!/bin/bash\nprintf '[fake-ue] server command reached\\n'\n",
    )

    carla_root = tmp_path / "carla"
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
    for path in (project, map_file):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n", encoding="utf-8")

    environment = os.environ.copy()
    for name in VIRTUAL_ENV_KEYS:
        environment.pop(name, None)
    environment.update(
        {
            "PATH": str(fake_bin),
            "RANGER_CARLA_ROOT": str(tmp_path / "ranger"),
            "CARLA_ROOT": str(carla_root),
            "UE_ROOT": str(ue_root),
            "RANGER_UE_ROOT": str(ue_root),
            "UE_EDITOR": str(editor),
            "CARLA_UPROJECT": str(project),
            "CARLA_UE_MAP": (
                "/Game/map_package/Maps/"
                "Woraksan_v1_0_3_parking_lot_hegiht_fit/"
                "Woraksan_v1_0_3_parking_lot_hegiht_fit"
            ),
            "CARLA_RENDER_MODE": "onscreen",
            "CARLA_PORT": "65534",
        }
    )
    return environment, fake_bin


def test_renderer_warns_and_continues_when_vulkaninfo_is_not_installed(
    tmp_path: Path,
) -> None:
    environment, _ = _renderer_server_fixture(tmp_path)

    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "server"],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "vulkaninfo is not installed" in result.stderr
    assert "skipping the optional Vulkan summary preflight" in result.stderr
    assert "[fake-ue] server command reached" in result.stdout


def test_renderer_rejects_an_installed_but_broken_vulkan_runtime(
    tmp_path: Path,
) -> None:
    environment, fake_bin = _renderer_server_fixture(tmp_path)
    _write_executable(fake_bin / "vulkaninfo", "#!/bin/bash\nexit 1\n")

    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "server"],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "Vulkan is not ready for rendered CARLA" in result.stderr
    assert "[fake-ue] server command reached" not in result.stdout


def test_renderer_still_requires_nvidia_driver_probe(tmp_path: Path) -> None:
    environment, fake_bin = _renderer_server_fixture(tmp_path)
    (fake_bin / "nvidia-smi").unlink()

    result = subprocess.run(
        [str(SCRIPT_ROOT / "run.sh"), "server"],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "nvidia-smi is required for rendered CARLA" in result.stderr
    assert "[fake-ue] server command reached" not in result.stdout


def test_runtime_runner_deeply_validates_all_authorized_gate_paths() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    assert "validate_gate_status" not in source
    assert source.count("validate_runtime_gates ||") == 2
    assert "\n    validate_runtime_gates\n" in source
    validator_source = GATE_VALIDATOR.read_text(encoding="utf-8")
    assert "validate_baseline_manifest" in validator_source
    assert "validate_physical_four_wheel_manifest" in validator_source
    assert "document.get(\"status\")" not in validator_source


def _write_fake_ranger_validators(root: Path) -> Path:
    source = (
        root
        / "src"
        / "carla_extended_ackermann_control"
        / "src"
    )
    package = source / "carla_extended_ackermann_control"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "backend_contract.py").write_text(
        """
import hashlib
import json
from types import SimpleNamespace

def validate_baseline_manifest(path, expected_blueprint):
    payload = open(path, encoding="utf-8").read()
    document = json.loads(payload)
    accepted = (
        expected_blueprint == "vehicle.ranger.default"
        and document.get("deep_binding") == "baseline-ok"
    )
    return SimpleNamespace(
        accepted=accepted,
        reason_text=(
            "accepted" if accepted else "fixture baseline deep binding mismatch"
        ),
        manifest_sha256=hashlib.sha256(payload.encode()).hexdigest(),
    )
""".lstrip(),
        encoding="utf-8",
    )
    (package / "physical_gate_contract.py").write_text(
        """
import hashlib
import json
from types import SimpleNamespace

def validate_physical_four_wheel_manifest(path, expected_blueprint, baseline_path):
    payload = open(path, encoding="utf-8").read()
    document = json.loads(payload)
    baseline = json.load(open(baseline_path, encoding="utf-8"))
    accepted = (
        expected_blueprint == "vehicle.ranger.default"
        and baseline.get("deep_binding") == "baseline-ok"
        and document.get("deep_binding") == "physical-ok"
    )
    return SimpleNamespace(
        accepted=accepted,
        reason_text=(
            "accepted" if accepted else "fixture physical deep binding mismatch"
        ),
        manifest_sha256=hashlib.sha256(payload.encode()).hexdigest(),
    )
""".lstrip(),
        encoding="utf-8",
    )
    return source


def _run_gate_validator(
    validator_source: Path,
    baseline: Path,
    physical: Path,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            str(GATE_VALIDATOR),
            "--validator-source",
            str(validator_source),
            "--baseline-manifest",
            str(baseline),
            "--physical-manifest",
            str(physical),
        ],
        cwd=SRC_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_deep_gate_validator_accepts_both_portable_validators(
    tmp_path: Path,
) -> None:
    validator_source = _write_fake_ranger_validators(tmp_path / "ranger")
    baseline = tmp_path / "baseline.json"
    physical = tmp_path / "physical.json"
    baseline.write_text(
        '{"status":"VERIFIED","deep_binding":"baseline-ok"}',
        encoding="utf-8",
    )
    physical.write_text(
        '{"status":"VERIFIED","deep_binding":"physical-ok"}',
        encoding="utf-8",
    )

    result = _run_gate_validator(validator_source, baseline, physical)

    assert result.returncode == 0, result.stderr
    assert "baseline gate deep VERIFIED sha256=" in result.stdout
    assert "physical 4WS gate deep VERIFIED sha256=" in result.stdout


def test_deep_gate_validator_rejects_status_only_manifest_without_traceback(
    tmp_path: Path,
) -> None:
    validator_source = _write_fake_ranger_validators(tmp_path / "ranger")
    baseline = tmp_path / "baseline.json"
    physical = tmp_path / "physical.json"
    baseline.write_text('{"status":"VERIFIED"}', encoding="utf-8")
    physical.write_text(
        '{"status":"VERIFIED","deep_binding":"physical-ok"}',
        encoding="utf-8",
    )

    result = _run_gate_validator(validator_source, baseline, physical)

    assert result.returncode != 0
    assert "fixture baseline deep binding mismatch" in result.stderr
    assert "Traceback" not in result.stderr


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
    assert "# REQUIRED lifecycle order (four terminals)" in result.stdout
    assert "Wait for each preceding stage to report success" in result.stdout
    assert "run.sh manual" in result.stdout
    assert "teleop_twist_keyboard" in result.stdout
    assert "cmd_vel:=/control/nav2_cmd_vel_ros" in result.stdout
    assert "export RANGER_UE_ROOT=" in result.stdout
    assert "export ROS_DOMAIN_ID=" in result.stdout
    assert "export RMW_IMPLEMENTATION=" in result.stdout
    assert "export CARLA_RENDER_MODE=" in result.stdout
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
