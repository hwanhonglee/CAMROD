"""No ROS graph or audio playback: distinguish parking from charging docking."""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE))
from voice_event_policy import VoiceEventPolicy, parking_status_identity  # noqa: E402


def ready_policy():
    policy = VoiceEventPolicy([])
    policy.announce_startup()
    return policy


def update(policy, phase, method="apriltag", attempt=4, source="dispatcher"):
    return [event.key for event in policy.update_docking(
        phase, parking_method=method, attempt=attempt, source=source)]


@pytest.mark.parametrize("method", ["reverse", "none", "", "auto", "apriltag_typo"])
def test_non_docking_methods_never_publish_docking_or_unmapped_keys(method):
    policy = ready_policy()
    for phase in ("IDLE", "WAITING_FOR_PARKING_OWNER", "REVERSE_APPROACH",
                  "WAIT_FOR_CHARGING", "PARKED", "ERROR", "PARKED"):
        assert update(policy, phase, method) == []


def test_selected_apriltag_announces_one_start_and_one_outcome():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_PARKING_OWNER") == ["docking.started"]
    for phase in ("WAITING_FOR_PARKING_OWNER", "WAITING_FOR_TAG", "TAG_GUIDED_REVERSE",
                  "RETRY_FORWARD_EXIT", "FINAL_REVERSE_INSERTION", "FINAL_YAW_ALIGNMENT"):
        assert update(policy, phase) == []
    assert update(policy, "PARKED") == ["docking.succeeded"]
    assert update(policy, "PARKED") == []
    assert update(policy, "ERROR") == []


def test_reverse_to_dock_switch_and_same_phase_new_attempt_are_not_lost():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_PARKING_OWNER", "reverse", 4) == []
    assert update(policy, "WAITING_FOR_PARKING_OWNER", "apriltag", 5) == ["docking.started"]
    assert update(policy, "PARKED", "reverse", 4) == []
    assert update(policy, "PARKED", "apriltag", 5) == ["docking.succeeded"]
    assert update(policy, "WAITING_FOR_PARKING_OWNER", "apriltag", 6) == ["docking.started"]
    assert update(policy, "WAITING_FOR_PARKING_OWNER", "apriltag", 7) == ["docking.started"]


def test_error_then_verified_parked_recovers_once_without_a_second_start():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_TAG") == ["docking.started"]
    assert update(policy, "ERROR") == ["docking.failed"]
    assert update(policy, "ERROR") == []
    assert update(policy, "PARKED") == ["docking.succeeded"]
    assert update(policy, "PARKED") == []


def test_dispatcher_abort_generation_preserves_current_failure_announcement():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_PARKING_OWNER", attempt=4) == ["docking.started"]
    assert update(policy, "ERROR", attempt=5) == ["docking.failed"]
    assert update(policy, "ERROR", attempt=5) == []
    assert update(policy, "PARKED", attempt=4) == []


def test_cancel_and_late_other_controller_status_cannot_announce_success():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_TAG") == ["docking.started"]
    assert update(policy, "IDLE", "none", 5) == []
    assert update(policy, "PARKED", attempt=4) == []
    assert update(policy, "WAITING_FOR_TAG", attempt=4) == []
    assert update(policy, "PARKED", attempt=5) == []
    assert update(policy, "WAITING_FOR_TAG", source="legacy:apriltag", attempt=None) == []


def test_same_generation_cannot_change_reverse_into_docking():
    policy = ready_policy()
    assert update(policy, "REVERSE_APPROACH", "reverse", 4) == []
    assert update(policy, "WAITING_FOR_TAG", "apriltag", 4) == []
    assert update(policy, "PARKED", "apriltag", 4) == []
    assert update(policy, "PARKED", "reverse", 4) == []


def test_cancelled_generation_cannot_restart_from_late_active_status():
    policy = ready_policy()
    assert update(policy, "WAITING_FOR_TAG", attempt=4) == ["docking.started"]
    assert update(policy, "IDLE", "none", 5) == []
    assert update(policy, "WAITING_FOR_TAG", "apriltag", 5) == []
    assert update(policy, "PARKED", "apriltag", 5) == []


@pytest.mark.parametrize("phase", ["PARKED", "ERROR"])
def test_terminal_status_without_current_attempt_start_is_silent(phase):
    assert update(ready_policy(), phase) == []


@pytest.mark.parametrize("message", [
    "parking_method=auto attempt=4", "parking_method=apriltag",
    "parking_method=apriltag attempt=-1", "parking_method=apriltag attempt=4.0",
    "parking_method=apriltag attempt=4 attempt=5",
    "parking_method=apriltag parking_method=reverse attempt=4",
    "not_parking_method=apriltag attempt=4",
])
def test_malformed_shared_selection_does_not_guess_a_docking_method(message):
    assert parking_status_identity(message, "parking", "/parking/status") == (
        "", None, "dispatcher")


def test_shared_status_preserves_exact_method_and_attempt():
    assert parking_status_identity(
        "parking_method=reverse battery_percent=89.0 charging_required=false forced=false attempt=4 phase=PARKED",
        "parking", "/parking/status") == ("reverse", 4, "dispatcher")


def test_legacy_method_comes_from_controller_identity_not_generic_phase():
    assert parking_status_identity("phase=PARKED", "parking", "/parking/reverse_parking_controller/status") == (
        "reverse", None, "legacy:reverse")
    assert parking_status_identity("phase=PARKED", "apriltag_parking_controller", "/custom/status") == (
        "apriltag", None, "legacy:apriltag")
    assert parking_status_identity("phase=WAITING_FOR_TAG", "parking", "/custom/status") == (
        "", None, "unknown")


def test_adapter_forwards_method_generation_and_owner_without_starting_ros():
    # Compile only the actual callback body so this contract remains independent
    # of ROS Python availability and cannot initialize a node or audio device.
    tree = ast.parse((SOURCE / "voice_event_adapter_node.py").read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    callback = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                    and node.name == "_on_parking_status")
    module = ast.Module(body=[callback], type_ignores=[])
    namespace = {"ModuleState": object, "parking_status_identity": parking_status_identity}
    exec(compile(ast.fix_missing_locations(module), "voice_callback", "exec"), namespace)
    backend = SimpleNamespace(_en_docking=True, _policy=Mock(), _emit_policy_events=Mock())
    message = SimpleNamespace(operating_state="PARKED", module_name="parking",
                              message="parking_method=reverse attempt=4 phase=PARKED")
    namespace["_on_parking_status"](backend, message, "/parking/status")
    backend._policy.update_docking.assert_called_once_with(
        "PARKED", parking_method="reverse", attempt=4, source="dispatcher")
