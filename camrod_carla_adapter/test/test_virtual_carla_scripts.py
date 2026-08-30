"""Portable, non-mutating contracts for the virtual CARLA entrypoints."""

import json
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
SENSOR_PREFLIGHT = SCRIPT_ROOT / "check_carla_sensor_streams.py"
SCRIPTS = ("env.sh", "setup.sh", "build.sh", "test.sh", "run.sh")
VIRTUAL_ENV_KEYS = (
    "CAMROD_CYCLONEDDS_CONFIG",
    "CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES",
    "CAMROD_MANUAL_LINEAR_LIMIT_MPS",
    "CAMROD_MANUAL_LATERAL_LIMIT_MPS",
    "CAMROD_MANUAL_ANGULAR_LIMIT_RADPS",
    "CAMROD_MANUAL_DEADMAN_TIMEOUT_S",
    "CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE",
    "CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ",
    "CAMROD_CARLA_SENSOR_MIN_RATE_HZ",
    "CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS",
    "CAMROD_LANELET_MAP",
    "CAMROD_MAP_ALIGNMENT_FILE",
    "CYCLONEDDS_URI",
    "RANGER_CARLA_ROOT",
    "RANGER_WORK_ROOT",
    "RANGER_ROS_WS",
    "RANGER_ENV_FILE",
    "RANGER_EVIDENCE_ROOT",
    "RANGER_BASELINE_MANIFEST",
    "RANGER_PHYSICAL_MANIFEST",
    "RANGER_SPAWN_FILE",
    "RANGER_CARLA_PYTHON_EGG",
    "RANGER_PYTHON_EGG_CACHE",
    "CARLA_ROOT",
    "CARLA_ROS_BRIDGE_WS",
    "RANGER_ROS_BRIDGE_WS",
    "CARLA_PYTHON_EGG",
    "CARLA_PYTHON_EGG_CACHE",
    "CARLA_RENDER_MODE",
    "CARLA_RENDER_MAX_FPS",
    "CARLA_SYNCHRONOUS_MODE",
    "CARLA_WAIT_FOR_CONTROL_COMMAND",
    "CAMROD_CARLA_STEP_PACING",
    "CAMROD_CARLA_STEP_PERIOD_SECONDS",
    "CARLA_FIXED_DELTA_SECONDS",
    "CAMROD_LAUNCH_SENSOR_RELAY",
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
        SENSOR_PREFLIGHT,
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
        SENSOR_PREFLIGHT,
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
    )
    assert not any(token in source for token in forbidden)
    # The only service CLI use is a read-only Trigger health probe.  Motion,
    # engage, goal and bridge PLAY/PAUSE are owned by their dedicated nodes.
    assert source.count("ros2 service call") == 1
    assert "/virtual_carla/step_pacer/health std_srvs/srv/Trigger" in source
    assert "prepare_ros_carla_python bridge" in source
    assert "prepare_ros_carla_python spawn" in source


def test_manual_keyboard_path_is_fail_closed_and_uses_camrod_safety_gate() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    physical_source = PHYSICAL_PREFLIGHT.read_text(encoding="utf-8")
    ui_source = UI_MANUAL_PREFLIGHT.read_text(encoding="utf-8")
    manual_case = source.split("  manual)\n", 1)[1].split("    ;;\n", 1)[0]

    assert "teleop_twist_keyboard" in source
    assert "cmd_vel:=/control/manual_cmd_vel_ros" in source
    assert "standalone ENGAGE on the last page" in source
    assert "not a B1-B13 destination button" in source
    assert "-p speed:=0.20" in source
    assert "-p turn:=0.20" in source
    assert "radius = speed / turn = 1.0 m" in source
    assert "validate_runtime_gates" in manual_case
    assert "validate_spawn_file" in manual_case
    assert "validate_ranger_actor_ready" in manual_case
    assert "validate_camrod_ui_manual_ready" in manual_case
    assert "validate_physical_bridge_ready" in manual_case
    assert "validate_manual_actor_binding" in manual_case
    assert manual_case.count("validate_ranger_actor_ready") == 2
    assert manual_case.index("validate_ranger_actor_ready") < manual_case.index(
        "validate_camrod_ui_manual_ready"
    )
    assert manual_case.index(
        "validate_camrod_ui_manual_ready"
    ) < manual_case.index("validate_physical_bridge_ready")
    assert manual_case.index("validate_physical_bridge_ready") < manual_case.index(
        "validate_manual_actor_binding"
    )
    assert manual_case.index("validate_manual_actor_binding") < manual_case.index(
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
    assert module.format_success(42, "ego_vehicle", True) == "42"
    assert "actor_id=42" in module.format_success(42, "ego_vehicle", False)


def test_manual_actor_binding_guard_matches_or_fails_closed() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    function = "validate_manual_actor_binding() {\n" + source.split(
        "validate_manual_actor_binding() {\n", 1
    )[1].split("\n}\n", 1)[0] + "\n}\n"
    harness = f"""
set -euo pipefail
virtual_carla_log() {{ printf '[virtual_carla] %s\\n' "$*"; }}
virtual_carla_die() {{ printf '[virtual_carla] ERROR: %s\\n' "$*" >&2; return 1; }}
{function}
validate_manual_actor_binding
"""

    matching_environment = os.environ.copy()
    matching_environment.update(
        {"RANGER_LIVE_ACTOR_ID": "41", "PHYSICAL_BRIDGE_ACTOR_ID": "41"}
    )
    matching = subprocess.run(
        ["bash", "-c", harness],
        cwd=SRC_ROOT,
        env=matching_environment,
        check=False,
        capture_output=True,
        text=True,
    )
    assert matching.returncode == 0, matching.stderr
    assert "manual actor identity matched: actor_id=41" in matching.stdout

    mismatched_environment = os.environ.copy()
    mismatched_environment.update(
        {"RANGER_LIVE_ACTOR_ID": "42", "PHYSICAL_BRIDGE_ACTOR_ID": "41"}
    )
    mismatched = subprocess.run(
        ["bash", "-c", harness],
        cwd=SRC_ROOT,
        env=mismatched_environment,
        check=False,
        capture_output=True,
        text=True,
    )
    assert mismatched.returncode != 0
    assert "live CARLA actor_id=42" in mismatched.stderr
    assert "physical 4WS bridge actor_id=41" in mismatched.stderr
    assert "manual remains disabled" in mismatched.stderr
    assert "rerun ./scripts/virtual_carla/run.sh camrod" in mismatched.stderr


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
    assert 'vehicle.get("type") != expected_type' in source
    assert '"${accepted_control}" "${accepted_visual}" <<\'PY\'' in source
    assert "breaks the accepted spawn/alignment cohort" in source
    camrod_case = source.split("  camrod)\n", 1)[1].split("    ;;\n", 1)[0]
    gate_index = camrod_case.index("validate_runtime_gates")
    spawn_index = camrod_case.index("validate_spawn_file")
    actor_index = camrod_case.index("validate_ranger_actor_ready")
    sensor_index = camrod_case.index("validate_carla_sensor_streams")
    main_cache_index = camrod_case.index('if [[ -z "${CARLA_PYTHON_EGG_CACHE}" ]]')
    assert gate_index < spawn_index < actor_index < sensor_index < main_cache_index

    preflight_function = source.split(
        "validate_ranger_actor_ready() {\n", 1
    )[1].split("\n}\n", 1)[0]
    assert "camrod-carla-actor-preflight.XXXXXX" in preflight_function
    assert 'export PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert 'export CARLA_PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert 'export RANGER_PYTHON_EGG_CACHE="${cache_dir}"' in preflight_function
    assert "virtual_carla_use_python_egg" in preflight_function
    assert "check_ranger_actor.py" in preflight_function


def _run_spawn_contract_validator(
    spawn_file: Path, render_mode: str
) -> subprocess.CompletedProcess:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    function = "validate_spawn_file() {\n" + source.split(
        "validate_spawn_file() {\n", 1
    )[1].split("\n}\n", 1)[0] + "\n}\n"
    harness = f"""
set -euo pipefail
virtual_carla_require_file() {{ [[ -f "$1" ]]; }}
{function}
validate_spawn_file
"""
    environment = os.environ.copy()
    environment.update(
        {
            "RANGER_SPAWN_FILE": str(spawn_file),
            "CARLA_ROLE_NAME": "ego_vehicle",
            "CARLA_RENDER_MODE": render_mode,
            "CAMROD_SRC_ROOT": str(SRC_ROOT),
        }
    )
    return subprocess.run(
        ["bash", "-c", harness],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_spawn_contract_requires_visual_payload_actors_only_when_rendered(
    tmp_path: Path,
) -> None:
    control_only = (
        PACKAGE_ROOT / "config" / "ranger_spawn_camrod_control_only.json"
    )
    full_sensors = (
        PACKAGE_ROOT / "config" / "ranger_spawn_camrod_full_sensors.json"
    )

    rendered_control = _run_spawn_contract_validator(
        control_only, "offscreen"
    )
    assert rendered_control.returncode != 0
    assert "rendered spawn JSON must contain exactly one 'rgb_view'" in (
        rendered_control.stderr
    )

    nullrhi_control = _run_spawn_contract_validator(control_only, "nullrhi")
    assert nullrhi_control.returncode == 0, nullrhi_control.stderr

    rendered_full = _run_spawn_contract_validator(full_sensors, "onscreen")
    assert rendered_full.returncode == 0, rendered_full.stderr

    wrong_pose = tmp_path / "wrong-pose.json"
    wrong_pose_document = json.loads(full_sensors.read_text(encoding="utf-8"))
    ranger = next(
        item
        for item in wrong_pose_document["objects"]
        if item.get("id") == "ego_vehicle"
    )
    ranger["spawn_point"]["x"] = 0.0
    wrong_pose.write_text(json.dumps(wrong_pose_document), encoding="utf-8")
    rejected_pose = _run_spawn_contract_validator(wrong_pose, "offscreen")
    assert rejected_pose.returncode != 0
    assert "breaks the accepted spawn/alignment cohort" in rejected_pose.stderr


def _run_hot_respawn_guard(
    node_list: str, *, list_exit: int = 0
) -> subprocess.CompletedProcess:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    function = "refuse_hot_respawn() {\n" + source.split(
        "refuse_hot_respawn() {\n", 1
    )[1].split("\n}\n", 1)[0] + "\n}\n"
    harness = f"""
set -uo pipefail
virtual_carla_die() {{ printf '[virtual_carla] ERROR: %s\\n' "$*" >&2; return 1; }}
timeout() {{ printf '%s\\n' "${{FAKE_NODE_LIST}}"; return "${{FAKE_LIST_EXIT}}"; }}
{function}
refuse_hot_respawn
"""
    environment = os.environ.copy()
    environment.update(
        {
            "FAKE_NODE_LIST": node_list,
            "FAKE_LIST_EXIT": str(list_exit),
        }
    )
    return subprocess.run(
        ["bash", "-c", harness],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_spawn_refuses_hot_respawn_and_unknown_ros_graph_state() -> None:
    idle = _run_hot_respawn_guard("/carla_ros_bridge")
    assert idle.returncode == 0, idle.stderr

    active = _run_hot_respawn_guard(
        "/carla_ros_bridge\n/carla/ego_vehicle/physical_four_wheel_bridge"
    )
    assert active.returncode != 0
    assert "hot respawn would change the actor ID" in active.stderr

    duplicate_spawn = _run_hot_respawn_guard("/carla_spawn_objects")
    assert duplicate_spawn.returncode != 0
    assert "duplicate/hot respawn" in duplicate_spawn.stderr

    unknown = _run_hot_respawn_guard("graph unavailable", list_exit=124)
    assert unknown.returncode != 0
    assert "cannot verify whether CAMROD is active" in unknown.stderr
    assert "exit=124" in unknown.stderr


def _run_render_timing_contract(
    *,
    render_mode: str = "offscreen",
    synchronous: str = "True",
    wait_for_control: str = "False",
    step_pacing: str = "True",
    step_period: str = "0.05",
    fixed_delta: str = "0.05",
    render_max_fps: str = "30",
) -> subprocess.CompletedProcess:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    function = "validate_render_timing_contract() {\n" + source.split(
        "validate_render_timing_contract() {\n", 1
    )[1].split("\n}\n", 1)[0] + "\n}\n"
    harness = f"""
set -uo pipefail
virtual_carla_die() {{ printf '[virtual_carla] ERROR: %s\\n' "$*" >&2; return 1; }}
{function}
validate_render_timing_contract
"""
    environment = os.environ.copy()
    environment.update(
        {
            "CARLA_RENDER_MODE": render_mode,
            "CARLA_SYNCHRONOUS_MODE": synchronous,
            "CARLA_WAIT_FOR_CONTROL_COMMAND": wait_for_control,
            "CAMROD_CARLA_STEP_PACING": step_pacing,
            "CAMROD_CARLA_STEP_PERIOD_SECONDS": step_period,
            "CARLA_FIXED_DELTA_SECONDS": fixed_delta,
            "CARLA_RENDER_MAX_FPS": render_max_fps,
        }
    )
    return subprocess.run(
        ["bash", "-c", harness],
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_rendered_timing_contract_prevents_sensor_startup_deadlocks():
    accepted = _run_render_timing_contract()
    assert accepted.returncode == 0, accepted.stderr

    waiting = _run_render_timing_contract(wait_for_control="True")
    assert waiting.returncode != 0
    assert "wait_for_vehicle_control_command=False" in waiting.stderr

    asynchronous = _run_render_timing_contract(synchronous="False")
    assert asynchronous.returncode != 0
    assert "requires synchronous_mode=True" in asynchronous.stderr

    unpaced = _run_render_timing_contract(step_pacing="False")
    assert unpaced.returncode != 0
    assert "CAMROD_CARLA_STEP_PACING=True" in unpaced.stderr

    wrong_period = _run_render_timing_contract(step_period="0.10")
    assert wrong_period.returncode != 0
    assert "CAMROD_CARLA_STEP_PERIOD_SECONDS" in wrong_period.stderr

    slow_tick = _run_render_timing_contract(fixed_delta="0.2")
    assert slow_tick.returncode != 0
    assert "CARLA_FIXED_DELTA_SECONDS" in slow_tick.stderr

    uncapped_renderer = _run_render_timing_contract(render_max_fps="0")
    assert uncapped_renderer.returncode != 0
    assert "CARLA_RENDER_MAX_FPS" in uncapped_renderer.stderr

    nullrhi = _run_render_timing_contract(
        render_mode="nullrhi",
        synchronous="False",
        wait_for_control="True",
        step_pacing="False",
        step_period="1.0",
        fixed_delta="1.0",
    )
    assert nullrhi.returncode == 0, nullrhi.stderr


def _write_fake_carla_egg(path: Path) -> None:
    source = """
import os

_inventory_calls = 0
_snapshot_calls = 0
_world_calls = 0

class Actor:
    def __init__(self, actor_id, type_id, role_name):
        self.id = actor_id
        self.type_id = type_id
        self.attributes = {"role_name": role_name}

class Snapshot:
    def __init__(self, frame):
        self.frame = frame

class World:
    def get_snapshot(self):
        global _snapshot_calls
        _snapshot_calls += 1
        mode = os.environ.get("FAKE_CARLA_MODE", "one")
        if mode == "frame_zero_transient" and _snapshot_calls == 1:
            return Snapshot(0)
        return Snapshot(42 + _snapshot_calls)

    def get_actors(self):
        global _inventory_calls
        _inventory_calls += 1
        mode = os.environ.get("FAKE_CARLA_MODE", "one")
        role = os.environ.get("FAKE_CARLA_ROLE", "ego_vehicle")
        if mode in ("transient", "frame_zero_transient") and _inventory_calls == 1:
            return []
        if mode == "blank":
            return []
        if mode == "zero":
            return [
                Actor(20, "vehicle.ranger.default", "another_role"),
                Actor(21, "vehicle.other.default", role),
            ]
        if mode == "empty":
            return [
                Actor(22, "vehicle.ranger.default", "another_role"),
                Actor(23, "vehicle.other.default", "another_role"),
            ]
        if mode == "many":
            return [
                Actor(31, "vehicle.ranger.default", role),
                Actor(32, "vehicle.ranger.default", role),
            ]
        if mode == "mixed":
            return [
                Actor(33, "vehicle.ranger.default", role),
                Actor(34, "vehicle.other.default", role),
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
    actor_id_only: bool = False,
    expected_count: int = 1,
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
    arguments = [
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
        "--expected-count",
        str(expected_count),
    ]
    if actor_id_only:
        arguments.append("--actor-id-only")
    return subprocess.run(
        arguments,
        cwd=SRC_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def _run_spawn_inventory_guard(
    tmp_path: Path,
    mode: str,
    *,
    timeout_seconds: str = "0.5",
) -> subprocess.CompletedProcess:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    function = "validate_no_ranger_actor_present() {\n" + source.split(
        "validate_no_ranger_actor_present() {\n", 1
    )[1].split("\n}\n", 1)[0] + "\n}\n"
    egg = tmp_path / "fake-carla.egg"
    if not egg.exists():
        _write_fake_carla_egg(egg)
    cache = tmp_path / f"spawn-cache-{mode}"
    cache.mkdir()
    harness = f"""
set -uo pipefail
script_dir={str(SCRIPT_ROOT)!r}
CARLA_HOST=127.0.0.1
CARLA_PORT=2000
CARLA_ROLE_NAME=ego_vehicle
CARLA_PYTHON_EGG={str(egg)!r}
PYTHON_EGG_CACHE={str(cache)!r}
export CARLA_HOST CARLA_PORT CARLA_ROLE_NAME CARLA_PYTHON_EGG
export PYTHON_EGG_CACHE
virtual_carla_die() {{ printf '[virtual_carla] ERROR: %s\\n' "$*" >&2; return 1; }}
{function}
validate_no_ranger_actor_present {timeout_seconds!r}
"""
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
        ["bash", "-c", harness],
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


def test_actor_preflight_actor_id_only_is_machine_readable(tmp_path: Path) -> None:
    result = _run_actor_preflight(tmp_path, "one", actor_id_only=True)

    assert result.returncode == 0, result.stderr
    assert result.stdout == "41\n"
    assert result.stderr == ""


def test_actor_preflight_requires_empty_inventory_before_spawn(
    tmp_path: Path,
) -> None:
    empty = _run_actor_preflight(tmp_path, "empty", expected_count=0)
    assert empty.returncode == 0, empty.stderr
    assert "Ranger actor inventory ready for spawn" in empty.stdout
    assert "count=0" in empty.stdout

    existing = _run_actor_preflight(tmp_path, "one", expected_count=0)
    assert existing.returncode != 0
    assert "pre-existing actor" in existing.stderr
    assert "refusing duplicate spawn" in existing.stderr

    wrong_type = _run_actor_preflight(tmp_path, "zero", expected_count=0)
    assert wrong_type.returncode != 0
    assert "vehicle.other.default" in wrong_type.stderr
    assert "refusing duplicate spawn" in wrong_type.stderr


def test_actor_preflight_empty_check_waits_past_transient_inventory(
    tmp_path: Path,
) -> None:
    for mode in ("frame_zero_transient", "transient"):
        result = _run_actor_preflight(
            tmp_path,
            mode,
            expected_count=0,
            timeout_seconds="0.5",
        )

        assert result.returncode != 0
        assert "pre-existing actor" in result.stderr
        assert "refusing duplicate spawn" in result.stderr
        assert "inventory ready for spawn" not in result.stdout


def test_actor_preflight_empty_check_rejects_persistently_blank_inventory(
    tmp_path: Path,
) -> None:
    result = _run_actor_preflight(
        tmp_path,
        "blank",
        expected_count=0,
        timeout_seconds="0.45",
    )

    assert result.returncode != 0
    assert "actor inventory did not synchronize" in result.stderr
    assert "empty actor inventory" in result.stderr
    assert "never accepted as proof that no Ranger exists" in result.stderr
    assert "inventory ready for spawn" not in result.stdout


def test_spawn_inventory_guard_waits_past_frame_zero_before_accepting_zero(
    tmp_path: Path,
) -> None:
    result = _run_spawn_inventory_guard(
        tmp_path,
        "frame_zero_transient",
    )

    assert result.returncode != 0
    assert "pre-existing actor" in result.stderr
    assert "refusing duplicate spawn" in result.stderr
    assert "inventory ready for spawn" not in result.stdout


def test_spawn_inventory_guard_waits_past_empty_first_inventory(
    tmp_path: Path,
) -> None:
    result = _run_spawn_inventory_guard(tmp_path, "transient")

    assert result.returncode != 0
    assert "pre-existing actor" in result.stderr
    assert "refusing duplicate spawn" in result.stderr
    assert "inventory ready for spawn" not in result.stdout


def test_spawn_inventory_guard_accepts_synchronized_non_target_inventory(
    tmp_path: Path,
) -> None:
    result = _run_spawn_inventory_guard(tmp_path, "empty")

    assert result.returncode == 0, result.stderr
    assert "Ranger actor inventory ready for spawn" in result.stdout
    assert "count=0" in result.stdout


def test_spawn_inventory_guard_fails_closed_on_persistently_empty_inventory(
    tmp_path: Path,
) -> None:
    result = _run_spawn_inventory_guard(
        tmp_path,
        "blank",
        timeout_seconds="0.45",
    )

    assert result.returncode != 0
    assert "actor inventory did not synchronize" in result.stderr
    assert "empty actor inventory" in result.stderr
    assert "never accepted as proof that no Ranger exists" in result.stderr
    assert "inventory ready for spawn" not in result.stdout


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
        "empty",
        timeout_seconds="0.45",
    )

    assert result.returncode != 0
    assert "found 0 actors using the required Ranger role" in result.stderr
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
        "empty": "found 0 actors using the required Ranger role",
        "zero": "has type='vehicle.other.default'",
        "many": "found 2 actors using required role_name",
        "mixed": "found 2 actors using required role_name",
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


def test_env_selects_visual_relay_only_for_rendered_modes(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    ranger_root.mkdir()
    script = f"""
set -euo pipefail
source {str(SCRIPT_ROOT / 'env.sh')!r}
printf '%s\n%s\n' "$RANGER_SPAWN_FILE" "$CAMROD_LAUNCH_SENSOR_RELAY"
"""

    rendered = _bash(
        script,
        environment={
            "RANGER_CARLA_ROOT": str(ranger_root),
            "CARLA_RENDER_MODE": "offscreen",
        },
    )
    assert rendered.stdout.splitlines() == [
        str(PACKAGE_ROOT / "config" / "ranger_spawn_camrod_full_sensors.json"),
        "true",
    ]

    nullrhi = _bash(
        script,
        environment={
            "RANGER_CARLA_ROOT": str(ranger_root),
            "CARLA_RENDER_MODE": "nullrhi",
        },
    )
    assert nullrhi.stdout.splitlines() == [
        str(PACKAGE_ROOT / "config" / "ranger_spawn_camrod_control_only.json"),
        "false",
    ]


def test_env_selects_virtual_map_by_default_and_preserves_override(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    ranger_root.mkdir()
    script = f"""
set -euo pipefail
source {str(SCRIPT_ROOT / 'env.sh')!r}
printf '%s\n' "$CAMROD_LANELET_MAP"
"""
    default_result = _bash(
        script,
        environment={"RANGER_CARLA_ROOT": str(ranger_root)},
    )
    assert default_result.stdout.strip() == str(
        PACKAGE_ROOT / "config" / "woraksan_carla_lanelet2.osm"
    )

    explicit_map = tmp_path / "caller-map.osm"
    override_result = _bash(
        script,
        environment={
            "RANGER_CARLA_ROOT": str(ranger_root),
            "CAMROD_LANELET_MAP": str(explicit_map),
        },
    )
    assert override_result.stdout.strip() == str(explicit_map)


def test_env_applies_one_checked_in_cyclonedds_contract_to_every_ros_shell(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    ranger_root.mkdir()
    script = f"""
set -euo pipefail
source {str(SCRIPT_ROOT / 'env.sh')!r}
printf '%s\n%s\n%s\n' \
  "$CAMROD_CYCLONEDDS_CONFIG" \
  "$CYCLONEDDS_URI" \
  "$CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES"
"""
    result = _bash(
        script,
        environment={"RANGER_CARLA_ROOT": str(ranger_root)},
    )
    config = SRC_ROOT / "cyclonedds.xml"
    assert result.stdout.splitlines() == [
        str(config),
        f"file://{config}",
        "20971520",
    ]
    env_source = (SCRIPT_ROOT / "env.sh").read_text(encoding="utf-8")
    source_ros = env_source.split("virtual_carla_source_ros() {", 1)[1].split(
        "\n}", 1
    )[0]
    assert "export CYCLONEDDS_URI" in source_ros
    assert "export CAMROD_CYCLONEDDS_CONFIG" in source_ros


def test_dds_transport_preflight_accepts_twenty_mib_socket_buffers(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    ranger_root.mkdir()
    script = f"""
set -euo pipefail
sysctl() {{
  case "$2" in
    net.core.rmem_max) printf '%s\n' 20971520 ;;
    net.core.wmem_max) printf '%s\n' 33554432 ;;
    *) return 1 ;;
  esac
}}
source {str(SCRIPT_ROOT / 'env.sh')!r}
virtual_carla_require_dds_transport
"""
    result = _bash(
        script,
        environment={"RANGER_CARLA_ROOT": str(ranger_root)},
    )
    assert "DDS camera transport ready" in result.stdout


def test_dds_transport_preflight_rejects_low_buffers_with_exact_remedy(
    tmp_path: Path,
) -> None:
    ranger_root = tmp_path / "ranger"
    ranger_root.mkdir()
    script = f"""
set -euo pipefail
sysctl() {{
  case "$2" in
    net.core.rmem_max) printf '%s\n' 212992 ;;
    net.core.wmem_max) printf '%s\n' 425984 ;;
    *) return 1 ;;
  esac
}}
source {str(SCRIPT_ROOT / 'env.sh')!r}
if virtual_carla_require_dds_transport; then
  exit 90
fi
"""
    result = _bash(
        script,
        environment={"RANGER_CARLA_ROOT": str(ranger_root)},
    )
    assert "net.core.rmem_max=212992" in result.stderr
    assert "net.core.wmem_max=425984" in result.stderr
    assert (
        "sudo sysctl -w net.core.rmem_max=20971520 "
        "net.core.wmem_max=20971520"
    ) in result.stderr
    assert "/etc/sysctl.d/99-camrod-carla-dds.conf" in result.stderr


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
    for command in ("server", "bridge", "pacer", "spawn", "camrod"):
        assert f"run.sh {command}" in result.stdout
    assert "ros2 run carla_ros_bridge bridge" in result.stdout
    assert "--log-level warn" in result.stdout
    assert "carla_spawn_objects.launch.py" in result.stdout
    assert "camrod_carla_full.launch.py" in result.stdout
    assert "launch_sensor_relay:=true" in result.stdout
    assert "# REQUIRED lifecycle order (five terminals)" in result.stdout
    assert "Wait for each preceding stage to report success" in result.stdout
    assert "run.sh manual" in result.stdout
    assert "run.sh audit-sensors" in result.stdout
    assert "teleop_twist_keyboard" in result.stdout
    assert "cmd_vel:=/control/manual_cmd_vel_ros" in result.stdout
    assert "export RANGER_UE_ROOT=" in result.stdout
    assert "export ROS_DOMAIN_ID=" in result.stdout
    assert "export RMW_IMPLEMENTATION=" in result.stdout
    assert "export CAMROD_CYCLONEDDS_CONFIG=" in result.stdout
    assert "export CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES=20971520" in result.stdout
    assert "export CYCLONEDDS_URI=" in result.stdout
    assert "export CAMROD_MANUAL_LINEAR_LIMIT_MPS=1.40" in result.stdout
    assert "export CAMROD_MANUAL_LATERAL_LIMIT_MPS=1.00" in result.stdout
    assert "export CAMROD_MANUAL_ANGULAR_LIMIT_RADPS=0.7853" in result.stdout
    assert "export CAMROD_MANUAL_DEADMAN_TIMEOUT_S=0.75" in result.stdout
    assert "export CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE=1.0" in result.stdout
    assert (
        "export CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ=5.0"
        in result.stdout
    )
    assert "export CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ=10.0" in result.stdout
    assert "export CAMROD_CARLA_SENSOR_MIN_RATE_HZ=2.0" in result.stdout
    assert (
        "export CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS=3.0"
        in result.stdout
    )
    assert "manual_drive_linear_limit_mps:=1.40" in result.stdout
    assert "manual_drive_lateral_limit_mps:=1.00" in result.stdout
    assert "manual_drive_angular_limit_radps:=0.7853" in result.stdout
    assert "manual_drive_deadman_timeout_s:=0.75" in result.stdout
    assert "compressed_image_max_rate_hz:=5.0" in result.stdout
    assert "raw_image_max_rate_hz:=10.0" in result.stdout
    assert (
        "sudo sysctl -w net.core.rmem_max=20971520 "
        "net.core.wmem_max=20971520"
    ) in result.stdout
    assert "export CARLA_RENDER_MODE=" in result.stdout
    assert "export CARLA_RENDER_MAX_FPS=30" in result.stdout
    assert "export CAMROD_CARLA_STEP_PACING=True" in result.stdout
    assert "export CAMROD_CARLA_STEP_PERIOD_SECONDS=0.05" in result.stdout
    assert "carla_step_pacer" in result.stdout
    assert "synchronous_mode_wall_time_pacing" not in result.stdout
    assert "No motion command" in result.stdout


def test_sensor_audit_is_fail_closed_on_actual_carla_actor_inventory() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    case_body = source.split("  audit-sensors)\n", 1)[1].split(
        "    ;;\n", 1
    )[0]
    function_body = source.split("run_sensor_source_audit() {\n", 1)[1].split(
        "\n}\n", 1
    )[0]
    assert "validate_ranger_actor_ready" in case_body
    assert "carla_sensor_source_audit" in function_body
    assert "--actor-policy require" in function_body


def test_every_runtime_ros_terminal_and_doctor_fail_closed_on_dds_buffers() -> None:
    source = (SCRIPT_ROOT / "run.sh").read_text(encoding="utf-8")
    for stage in ("bridge", "spawn", "camrod", "manual", "audit-sensors"):
        case_body = source.split(f"  {stage})\n", 1)[1].split("    ;;\n", 1)[0]
        assert case_body.count("virtual_carla_require_dds_transport") == 1
    doctor_body = source.split("run_doctor() {", 1)[1].split(
        "\n}\n\ncase", 1
    )[0]
    assert doctor_body.count("virtual_carla_require_dds_transport") == 1


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
