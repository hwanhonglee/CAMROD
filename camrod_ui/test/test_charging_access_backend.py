"""HH_261102 - Cargo access reuses station control without claiming a campsite mission."""

import threading
import time
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))

from avg_msgs.msg import AvgBool, AvgPlatformStatus, AvgServiceState, ModuleState, MotionOperation
from camrod_ui.ui_backend_node import UiBackendNode
from test_drop_zone_departure_origin import backend  # noqa: F401 - reuse the existing ROS-free station fixture


@pytest.fixture
def access_backend(backend):
    backend._destination_dispatch_lock = threading.RLock()
    backend._charging_access_phase = "inactive"
    backend._charging_access_message = ""
    backend._latest_platform_control_mode = 1
    backend._latest_platform_is_charging = True
    backend._latest_service_state = int(AvgServiceState.CHARGING)
    backend.parking_method = "auto"
    backend.minimum_mission_dispatch_battery_percent = 35.0
    backend._state.battery_percentage = 35
    backend._state.service_state = int(AvgServiceState.CHARGING)
    backend._state.service_state_name = "CHARGING"
    backend._state.service_state_description = "charging"
    backend._state.ready = True
    backend._safety_reset_motion_observation = {
        "received_monotonic_s": time.monotonic(), "source_stamp_s": 100.0,
        "speed_mps": 0.0, "yaw_rate_radps": 0.0,
        "estop": False, "error_code": 0, "vehicle_state": 0,
    }
    backend._on_drop_zone_exit_complete = lambda msg: UiBackendNode._on_drop_zone_exit_complete(backend, msg)
    backend._on_service_state = lambda msg: UiBackendNode._on_service_state(backend, msg)
    backend._mark_drop_zone_exit_failed = lambda source: UiBackendNode._mark_drop_zone_exit_failed(backend, source)
    backend._mark_low_battery_return_started_if_needed = lambda: None
    backend._update_low_battery_return_policy = lambda *args, **kwargs: None
    backend._request_return_to_drop_zone = lambda source: UiBackendNode._request_return_to_drop_zone(backend, source)
    backend._request_nav2_cancel = lambda source: backend.events.append(("nav_cancel", source))
    backend._start_drop_zone_parking_alignment = lambda source: UiBackendNode._start_drop_zone_parking_alignment(backend, source)
    backend.destroy_timer = lambda timer: None

    def timer(period, callback):
        return SimpleNamespace(period=period, callback=callback, cancel=Mock())

    backend.create_timer = timer
    return backend


def request(node, action="leave", confirm=True):
    return UiBackendNode.request_charging_access(node, action, confirm)


def exit_complete(node):
    status = ModuleState()
    status.operating_state = "EXIT_STRAIGHT"
    UiBackendNode._on_drop_zone_maneuver_status(node, status)
    complete = AvgBool()
    complete.data = True
    UiBackendNode._on_drop_zone_exit_complete(node, complete)


@pytest.mark.parametrize("confirm", [False, None, "true", 1])
def test_confirmation_must_be_explicit_boolean(access_backend, confirm):
    result = request(access_backend, confirm=confirm)
    assert not result["success"]
    assert result["error"] == "confirmation_required"
    assert access_backend.events == []


@pytest.mark.parametrize("bad_input", ["low_soc", "unknown_soc", "stale_platform", "moving", "estop", "fault", "rc", "outside", "stale_pose", "mission", "not_ready", "startup"])
def test_rejection_never_releases_a_motion_owner(access_backend, bad_input):
    node = access_backend
    if bad_input == "low_soc":
        node._state.battery_percentage = 34
    elif bad_input == "unknown_soc":
        node._state.battery_percentage = -1
    elif bad_input == "stale_platform":
        node._safety_reset_motion_observation["source_stamp_s"] = 99.0
    elif bad_input == "moving":
        node._safety_reset_motion_observation["speed_mps"] = 0.03
    elif bad_input == "estop":
        node._safety_reset_motion_observation["estop"] = True
    elif bad_input == "fault":
        node._safety_reset_motion_observation["error_code"] = 1
    elif bad_input == "rc":
        node._latest_platform_control_mode = 0
    elif bad_input == "outside":
        node._latest_arrival_pose.pose.position.x = 5.0
    elif bad_input == "stale_pose":
        node._latest_arrival_pose_time_s = 95.0
    elif bad_input == "mission":
        node._active_mission_site = "B1"
    elif bad_input == "not_ready":
        node._state.ready = False
    else:
        node._startup_recovery_pending = True
    assert not request(node)["success"]
    assert node.events == []


def test_read_only_snapshot_never_moves_or_claims_electrical_disconnect(access_backend):
    snapshot = UiBackendNode.charging_access_snapshot(access_backend)
    assert snapshot["can_leave"]
    assert "can_redock" not in snapshot
    assert snapshot["target_label"] == "drop_zone 내 대기 위치"
    assert not snapshot["electrical_stop_available"]
    assert not snapshot["physical_disconnect_verified"]
    assert snapshot["platform_feedback_fresh"]
    assert access_backend.events == []
    access_backend._safety_reset_motion_observation["source_stamp_s"] = 99.0
    snapshot = UiBackendNode.charging_access_snapshot(access_backend)
    assert not snapshot["platform_feedback_fresh"]
    assert not snapshot["can_leave"]


def test_leave_stops_at_existing_exit_without_any_campsite_dispatch(access_backend):
    node = access_backend
    assert request(node)["success"]
    assert request(node)["success"]  # a double press must not emit a second EXIT
    assert node._pending_site_after_drop_zone_exit is None
    assert sum(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events) == 1
    exit_complete(node)
    assert node._charging_access_phase == "ready"
    assert not any(kind in {"goal", "mission_key", "recall"} for kind, _ in node.events)
    assert ("platform_drive_enable", False) in node.events
    assert ("mission_engage", False) in node.events
    exit_index = node.events.index(("drop_zone_operation", MotionOperation.EXIT))
    for kind in ("engage", "mission_engage", "platform_drive_enable"):
        assert max(index for index, event in enumerate(node.events) if event == (kind, False)) > exit_index
    assert not UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]
    node._latest_platform_is_charging = False
    assert UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]
    node._safety_reset_motion_observation["speed_mps"] = 0.1
    assert not UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]


def test_stale_completion_cannot_skip_native_exit_acceptance(access_backend):
    node = access_backend
    request(node)
    complete = AvgBool()
    complete.data = True
    UiBackendNode._on_drop_zone_exit_complete(node, complete)
    assert node._charging_access_phase == "departing"
    assert not UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]


# HJ_261106 - Cargo access no longer reuses the 7 s charging departure dwell;
# the move_front announcement is the only wait, so the same guards apply to it.
def test_move_front_cue_holds_and_revalidates_soc_before_exit(access_backend):
    node = access_backend
    node.charging_departure_delay_s = 7.0
    callbacks = deferred_voice(node)
    assert request(node)["success"]
    assert ("platform_drive_enable", False) in node.events
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)
    node._state.battery_percentage = 34
    callbacks[0]()
    assert node._charging_access_phase == "error"
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


def test_stop_invalidates_pending_cue_and_late_terminal(access_backend):
    node = access_backend
    node.charging_departure_delay_s = 7.0
    callbacks = deferred_voice(node)
    request(node)
    UiBackendNode._stop_active_service(node, "test_stop")
    callbacks[0]()
    exit_complete(node)
    assert node._charging_access_phase == "inactive"
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


@pytest.mark.parametrize("phase", ["departing"])
def test_cargo_access_blocks_other_motion_admission(access_backend, phase):
    node = access_backend
    node._charging_access_phase = phase
    assert UiBackendNode._apply_destination_command(node, "B1", True, "http_ui_destination")["error"] == "charging_access_confirmation_required"
    assert UiBackendNode._manual_goal_dispatch_block(node)["error"] == "charging_access_confirmation_required"
    assert not UiBackendNode.set_engage(node, True)["success"]
    assert not UiBackendNode.set_auto(node)["success"]
    assert not UiBackendNode.set_operation_mode(node, True)["success"]
    assert not UiBackendNode.request_manual_dock(node)["success"]
    assert not UiBackendNode.request_manual_return(node)["success"]
    assert not any(kind.endswith("operation") or kind == "goal" for kind, _ in node.events)


def test_ready_uses_normal_mission_flow_without_a_new_redock_step(access_backend):
    # HH_261102 - Normal Return already selects reverse parking and low-SOC docking.
    node = access_backend
    request(node)
    exit_complete(node)
    node._latest_platform_is_charging = False
    before = len(node.events)
    assert not request(node, "redock")["success"]
    assert len(node.events) == before
    assert UiBackendNode._charging_access_motion_block(node) is None
    assert not node._drop_zone_exit_handoff_ready
    dispatch = UiBackendNode._apply_destination_command(node, "B1", True, "http_ui_destination")
    assert not dispatch.get("blocked", False)
    assert node._charging_access_phase == "inactive"


def test_reverse_only_profile_does_not_need_an_added_redock_capability(access_backend):
    access_backend.parking_method = "reverse"
    assert request(access_backend)["success"]


def test_a_stale_charging_screen_cannot_move_a_noncharging_robot(access_backend):
    # HH_261102 - Preserve the charging-only scope even if a client keeps an old charging page open.
    access_backend._latest_platform_is_charging = False
    assert not request(access_backend)["success"]
    assert access_backend.events == []


def test_in_zone_terminal_is_not_a_road_handoff(access_backend):
    node = access_backend
    assert request(node)["success"]
    progress = ModuleState()
    progress.operating_state = "EXIT_STRAIGHT"
    UiBackendNode._on_drop_zone_maneuver_status(node, progress)
    ready = ModuleState()
    ready.operating_state = "CARGO_ACCESS_READY"
    UiBackendNode._on_drop_zone_maneuver_status(node, ready)
    assert node._charging_access_phase == "ready"
    assert not node._drop_zone_exit_handoff_ready
    assert node._latest_service_state == int(AvgServiceState.DROP_ZONE_WAIT)
    UiBackendNode._on_drop_zone_maneuver_status(node, progress)
    assert node._latest_service_state == int(AvgServiceState.DROP_ZONE_WAIT)


def test_native_error_clears_access_intent_and_holds_motion(access_backend):
    node = access_backend
    request(node)
    status = ModuleState()
    status.operating_state = "ERROR"
    UiBackendNode._on_drop_zone_maneuver_status(node, status)
    assert node._charging_access_phase == "error"
    assert node._pending_site_after_drop_zone_exit is None
    assert not node._drop_zone_exit_active
    assert ("mission_engage", False) in node.events


@pytest.mark.parametrize("alias", ["set_engage", "set_operation_mode"])
def test_public_disengage_retires_access_before_a_late_cue(alias, access_backend):
    node = access_backend
    node.charging_departure_delay_s = 7.0
    callbacks = deferred_voice(node)
    request(node)
    assert getattr(UiBackendNode, alias)(node, False)["success"]
    callbacks[0]()
    assert node._charging_access_phase == "inactive"
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


def test_stop_after_ready_ignores_late_completion_and_phase_heartbeats(access_backend):
    node = access_backend
    request(node)
    exit_complete(node)
    terminal = ModuleState()
    terminal.operating_state = "ROAD_HANDOFF_READY"
    UiBackendNode._on_drop_zone_maneuver_status(node, terminal)
    assert node._latest_service_state == int(AvgServiceState.DROP_ZONE_WAIT)
    UiBackendNode._stop_active_service(node, "test_stop_after_ready")
    exit_complete(node)
    assert node._charging_access_phase == "inactive"
    assert not UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]


def test_websocket_engage_source_uses_the_checked_public_method():
    # HH_261102 - Keep the legacy wire handler from silently bypassing tested HTTP admission.
    source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
    handler = source.split('if "engage" in payload:', 1)[1].split('if payload.get("usage_complete"):', 1)[0]
    assert "result = node.set_engage(new_engage)" in handler
    assert "node._publish_engage" not in handler
    assert 'if result.get("success", False):' in handler
    assert "await ws.send_json(result)" in handler


def deferred_voice(node):
    # HH_261103 - Exercise the real delayed release after its own parking CANCEL, not an immediate fixture callback.
    callbacks = []
    node._voice_gate = Mock()
    node._voice_gate.start.side_effect = lambda keys, callback, **kwargs: callbacks.append(callback) or []
    return callbacks


def contact_released(node):
    # HH_261103 - Deliver healthy, fresh stopped BMS feedback through the real platform callback.
    node._runtime_policy.update_platform = Mock()
    message = AvgPlatformStatus(control_mode=1, battery_state_available=True,
                                battery_percentage=0.35, is_charging=False, battery_power_supply_status=3)
    message.velocity.header.stamp.sec = 100
    UiBackendNode._on_platform_status(node, message)


def test_own_parking_handoff_contact_release_still_releases_native_exit(access_backend):
    node = access_backend
    callbacks = deferred_voice(node)
    assert request(node)["success"]
    assert ("parking_operation", MotionOperation.CANCEL) in node.events
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)
    contact_released(node)
    snapshot = UiBackendNode.charging_access_snapshot(node)
    assert snapshot["phase"] == "departing"
    assert not snapshot["can_leave"] and not snapshot["cargo_access_ready"]
    assert node._state.battery_percentage == 35
    callbacks[0]()
    assert node._charging_access_phase == "departing"
    assert sum(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events) == 1
    assert not any(kind in {"goal", "mission_key", "recall"} for kind, _ in node.events)
    exit_complete(node)
    assert UiBackendNode.charging_access_snapshot(node)["cargo_access_ready"]


@pytest.mark.parametrize("invalid", ["soc34", "stale", "fault", "foreign_mission", "moving", "estop", "rc", "outside", "not_ready"])
def test_contact_release_does_not_skip_other_delayed_admission_guards(access_backend, invalid):
    node = access_backend
    callbacks = deferred_voice(node)
    request(node)
    contact_released(node)
    if invalid == "soc34":
        node._state.battery_percentage = 34
    elif invalid == "stale":
        node._safety_reset_motion_observation["source_stamp_s"] = 99.0
    elif invalid == "fault":
        node._safety_reset_motion_observation["error_code"] = 1
    elif invalid == "foreign_mission":
        node._active_mission_site = "B1"
    elif invalid == "moving":
        node._safety_reset_motion_observation["speed_mps"] = .03
    elif invalid == "estop":
        node._safety_reset_motion_observation["estop"] = True
    elif invalid == "rc":
        node._latest_platform_control_mode = 0
    elif invalid == "outside":
        node._latest_arrival_pose.pose.position.x = 5.0
    else:
        node._state.ready = False
    assert UiBackendNode._charging_access_leave_block(node, require_charging=False)
    callbacks[0]()
    # HH_261103 - A foreign identity is rejected even earlier by the existing voice ownership guard.
    assert node._charging_access_phase == ("departing" if invalid == "foreign_mission" else "error")
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


def test_stop_after_own_handoff_invalidates_late_voice_release(access_backend):
    node = access_backend
    callbacks = deferred_voice(node)
    request(node)
    contact_released(node)
    UiBackendNode._stop_active_service(node, "test_stop_during_announcement")
    callbacks[0]()
    assert node._charging_access_phase == "inactive"
    assert not any(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


# HJ_261106 - "앞으로 이동" speaks system.move_front before any motion and
# system.ready once the forward move has stopped.
def recording_voice(node):
    calls = []
    node._voice_gate = Mock()
    node._voice_gate.busy = True

    def start(keys, callback, **kwargs):
        calls.append((tuple(keys), callback))
        if not keys:
            callback()  # Same as VoiceDepartureGate: nothing to say.
        return tuple(keys)

    node._voice_gate.start.side_effect = start
    node.pub_voice_say = Mock()
    node._publish_voice_say = lambda key: UiBackendNode._publish_voice_say(node, key)
    node._now_s = lambda: 100.0
    return calls


def said(node):
    return [call.args[0].key for call in node.pub_voice_say.publish.call_args_list]


def exits(node):
    return sum(event == ("drop_zone_operation", MotionOperation.EXIT) for event in node.events)


def test_move_front_cue_precedes_exit_and_ready_follows_stop(access_backend):
    node = access_backend
    calls = recording_voice(node)
    assert request(node)["success"]
    assert calls[0][0] == ("system.move_front",)
    assert exits(node) == 0
    contact_released(node)
    calls[0][1]()
    assert exits(node) == 1
    exit_complete(node)
    assert said(node)[-1] == "system.ready"


def test_charging_access_skips_departure_dwell_and_moves_after_cue(access_backend):
    # HJ_261106 - The 7 s charging departure dwell belongs to campsite
    # departures; pulling forward off the charger only waits for move_front.
    node = access_backend
    node.charging_departure_delay_s = 7.0
    calls = recording_voice(node)
    assert request(node)["success"]
    assert getattr(node, "_charging_departure_transition_timer", None) is None
    assert [keys for keys, _ in calls] == [("system.move_front",)]
    assert exits(node) == 0
    contact_released(node)
    calls[0][1]()
    assert exits(node) == 1
