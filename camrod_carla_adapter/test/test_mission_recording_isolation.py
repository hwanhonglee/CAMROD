"""HH_260921 - CARLA journals stay separate from real robot operation data."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration


ROOT = Path(__file__).resolve().parents[2]
FULL = ROOT / "camrod_carla_adapter/launch/camrod_carla_full.launch.py"
UI = ROOT / "camrod_ui/camrod_ui_robot/launch/ui.launch.py"


def _module(path):
    spec = importlib.util.spec_from_file_location(path.name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def isolated_environment(monkeypatch):
    for name in (
        "CAMROD_CARLA_MISSION_RECORDS_ROOT", "RANGER_WORK_ROOT",
        "RANGER_CARLA_ROOT", "RANGER_SPAWN_FILE", "RANGER_ENV_FILE",
    ):
        monkeypatch.delenv(name, raising=False)
    # Deliberately inherit production settings; the CARLA boundary must win.
    monkeypatch.setenv("CAMROD_RECORDING_ENVIRONMENT", "real")
    monkeypatch.setenv("XDG_STATE_HOME", "/production-state")


@pytest.mark.parametrize("case", ("work", "ranger", "override", "unanchored"))
def test_direct_launch_resolves_dedicated_journal(
        case, monkeypatch, isolated_environment):
    module = _module(FULL)
    if case == "work":
        monkeypatch.setenv("RANGER_WORK_ROOT", "/virtual-work")
        expected = "/virtual-work/camrod/mission_records"
    elif case == "ranger":
        monkeypatch.setenv("RANGER_CARLA_ROOT", "/virtual-ranger")
        expected = "/virtual-ranger/.work/camrod/mission_records"
    elif case == "override":
        monkeypatch.setenv("RANGER_WORK_ROOT", "/virtual-work")
        monkeypatch.setenv("CAMROD_CARLA_MISSION_RECORDS_ROOT", "/separate/run-02")
        expected = "/separate/run-02"
    else:
        expected = str(Path.home() / ".local/state/camrod_carla/mission_records")
    assert module._mission_records_root() == expected
    assert os.environ["XDG_STATE_HOME"] == "/production-state"


def test_full_launch_pins_simulation_and_both_ui_nodes_share_root(
        monkeypatch, isolated_environment):
    full = _module(FULL)
    monkeypatch.setenv("RANGER_WORK_ROOT", "/virtual-work")
    monkeypatch.setattr(
        full, "get_package_share_directory", lambda name: str(ROOT / name))
    context = LaunchContext()
    context.launch_configurations["mission_recorder_environment"] = "real"
    for action in full.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(context)
    expected_root = "/virtual-work/camrod/mission_records"
    assert (
        context.launch_configurations["mission_recorder_environment"]
        == "simulation")
    assert context.launch_configurations["mission_records_root"] == expected_root

    ui = _module(UI)
    monkeypatch.setattr(
        ui, "get_package_share_directory", lambda name: str(ROOT / name))
    captured = []

    def capture_node(**kwargs):
        captured.append(kwargs)
        # Inert launch action; no node, database or browser is started.
        return SetLaunchConfiguration("test_node_capture", "true")

    monkeypatch.setattr(ui, "Node", capture_node)
    for action in ui.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = {node["executable"]: node for node in captured}
    recorder = nodes["mission_recorder_node"]["parameters"][0]
    backend = nodes["ui_backend_node"]["parameters"][0]
    assert recorder["environment"].perform(context) == "simulation"
    assert recorder["storage_root"].perform(context) == expected_root
    assert backend["mission_records_root"].perform(context) == expected_root
    assert os.environ["XDG_STATE_HOME"] == "/production-state"


@pytest.mark.parametrize("filename", (
    "camrod_carla_woraksan_tuned.launch.py",
    "camrod_carla_develop_site_geometry.launch.py",
))
def test_profile_wrappers_inherit_shared_full_launch(filename, monkeypatch):
    module = _module(FULL.parent / filename)
    monkeypatch.setattr(
        module, "get_package_share_directory", lambda name: str(ROOT / name))
    actions = module.generate_launch_description().entities
    assert len(actions) == 1
    include = actions[0]
    context = LaunchContext()
    source = include.launch_description_source
    monkeypatch.setattr(
        source, "_get_launch_description", lambda _path: LaunchDescription([]))
    source.get_launch_description(context)
    assert source.location == str(FULL)
    arguments = dict(include.launch_arguments)
    assert "mission_records_root" not in arguments
    assert "mission_recorder_environment" not in arguments


@pytest.mark.parametrize("override", ("", "/separate/run-03"))
def test_shell_environment_does_not_repoint_legacy_state(
        override, isolated_environment):
    environment = dict(
        os.environ, RANGER_CARLA_ROOT="/virtual-ranger",
        RANGER_WORK_ROOT="/virtual-work", RANGER_ENV_FILE="/nonexistent")
    if override:
        environment["CAMROD_CARLA_MISSION_RECORDS_ROOT"] = override
    probe = (
        'import json, os; print(json.dumps({k: os.environ.get(k) for k in '
        '("CAMROD_CARLA_MISSION_RECORDS_ROOT", "CAMROD_RECORDING_ENVIRONMENT", '
        '"XDG_STATE_HOME")}))'
    )
    result = subprocess.run([
        "bash", "-c", 'source "$1"; python3 -c "$2"',
        "test", str(ROOT / "scripts/virtual_carla/env.sh"), probe,
    ], env=environment, text=True, capture_output=True, check=True)
    settings = json.loads(result.stdout)
    assert settings["CAMROD_CARLA_MISSION_RECORDS_ROOT"] == (
        override or "/virtual-work/camrod/mission_records")
    assert settings["CAMROD_RECORDING_ENVIRONMENT"] == "simulation"
    assert settings["XDG_STATE_HOME"] == "/production-state"


def test_printed_commands_include_isolated_root_for_all_three_profiles(
        isolated_environment):
    environment = dict(
        os.environ, RANGER_CARLA_ROOT="/virtual-ranger",
        RANGER_WORK_ROOT="/virtual-work", RANGER_ENV_FILE="/nonexistent")
    result = subprocess.run([
        "bash", str(ROOT / "scripts/virtual_carla/run.sh"), "commands",
    ], env=environment, text=True, capture_output=True, check=True)
    root = "/virtual-work/camrod/mission_records"
    assert result.stdout.count("mission_records_root:=" + root) == 3
    assert "export CAMROD_RECORDING_ENVIRONMENT=simulation" in result.stdout
    assert "export CAMROD_CARLA_MISSION_RECORDS_ROOT=" + root in result.stdout
