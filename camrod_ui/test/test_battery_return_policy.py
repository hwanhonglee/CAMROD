"""SOC, urgent Return, and explicit Dock regressions without ROS/HTTP I/O."""

from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace
from unittest import mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from avg_msgs.msg import AvgBool, AvgPlatformStatus, AvgServiceState, ModuleState, MotionOperation  # noqa: E402
from camrod_ui.battery_policy import battery_policy_snapshot, urgent_return_required  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


def backend(*, soc=24.9, state=AvgServiceState.GUEST_LOADING_WAIT, phase="UNLOAD_WAIT"):
    node = SimpleNamespace(
        _lock=threading.RLock(),
        _destination_dispatch_lock=threading.RLock(),
        _state=SimpleNamespace(battery_percentage=soc),
        _active_mission_site="B4",
        _active_mission_source="guest:dispatch:r=current",
        _active_mission_generation=17,
        _return_requested_generation=0,
        _active_mission_retryable=False,
        _startup_recovery_pending=False,
        _mission_execution_error="",
        _latest_service_state=int(state),
        _latest_campsite_phase=phase,
        _latest_campsite_site="camping_site_4",
        _latest_campsite_status_time_s=100.0,
        _latest_platform_motion_ready=True,
        _latest_platform_status_time_s=100.0,
        _latest_platform_is_charging=False,
        _drop_zone_exit_active=False,
        _pending_site_after_drop_zone_exit=None,
        _urgent_return_after_departure=False,
        _urgent_return_generation=0,
        _battery_return_urgent=False,
        _low_battery_return_pending=False,
        _low_battery_return_started=False,
        _low_battery_return_wait_notified=False,
        _manual_return_transition_pending=False,
        low_battery_return_after_current_mission=True,
        low_battery_return_threshold_percent=35.0,
        urgent_battery_return_threshold_percent=25.0,
        minimum_mission_dispatch_battery_percent=35.0,
        require_battery_for_mission_dispatch=True,
        publish_mission_engage_from_destination=True,
        parking_method="auto",
        _parking_selected_method="",
        _parking_controller_operating_states={},
        _now_s=lambda: 100.0,
        _resolve_mission_key_for_site=lambda site: f"camping_site_{site[1:]}",
        _site_arrival_match=lambda _: (False, "camping_site_4", 10.0, "outside"),
        get_logger=lambda: mock.Mock(),
        _schedule_broadcast=mock.Mock(),
        _publish_mission_engage=mock.Mock(),
        _publish_platform_drive_enable=mock.Mock(),
        _publish_engage=mock.Mock(),
        _publish_camping_site_maneuver_controller_return=mock.Mock(),
        _request_nav2_cancel=mock.Mock(),
        _schedule_manual_return_transition=mock.Mock(),
        _publish_parking_operation=mock.Mock(),
        _publish_site_goal_pose=mock.Mock(),
    )
    for name in (
        "_low_battery_mission_state", "_low_battery_station_state",
        "_clear_low_battery_return_if_stationary",
        "_low_battery_return_payload", "_maybe_notify_low_battery_waiting_for_user",
    ):
        setattr(node, name, lambda *args, _name=name, **kwargs:
                getattr(UiBackendNode, _name)(node, *args, **kwargs))
    return node


@pytest.mark.parametrize("soc, urgent, charging, dispatch", [
    (24.9, True, True, False),
    (25.0, False, True, False),
    (34.9, False, True, False),
    (35.0, False, False, True),
])
def test_soc_boundaries_match_backend_dispatch_and_return(soc, urgent, charging, dispatch):
    snapshot = battery_policy_snapshot(soc)
    assert urgent_return_required(soc) is urgent
    assert snapshot["charging_required"] is charging
    node = backend(soc=soc)
    assert (UiBackendNode._mission_dispatch_battery_block(node, "B4") is None) is dispatch
    UiBackendNode._update_low_battery_return_policy(node, soc, "platform_status")
    assert node._battery_return_urgent is urgent
    assert node._publish_camping_site_maneuver_controller_return.call_count == int(urgent)
    assert node._low_battery_return_pending is (soc < 35.0)


@pytest.mark.parametrize("fraction, expected", [
    (0.35, 35),
    (struct.unpack("<f", struct.pack("<I", 0x3EB33332))[0], 34),
    (0.25, 25),
    (0.0, 0),
    (1.0, 100),
    (-0.01, -1),
    (1.01, -1),
    (float("nan"), -1),
    (float("inf"), -1),
    (-float("inf"), -1),
])
def test_platform_soc_callback_uses_float32_percentage_boundary(fraction, expected):
    node = backend(soc=80, state=AvgServiceState.DROP_ZONE_WAIT, phase="IDLE")
    node._runtime_policy = mock.Mock()
    node._update_runtime_state = lambda update: update()
    node._update_low_battery_return_policy = mock.Mock()
    # Reproduce the actual float32 ROS wire value, including the immediately
    # lower representable value: widening before multiplication floors 35 to 34.
    wire_fraction = struct.unpack("<f", struct.pack("<f", fraction))[0]
    message = AvgPlatformStatus(
        control_mode=1, battery_state_available=True,
        battery_percentage=wire_fraction,
    )
    with mock.patch.object(UiBackendNode, "_publish_destination_dispatch_status"):
        UiBackendNode._on_platform_status(node, message)
    assert node._state.battery_percentage == expected
    node._update_low_battery_return_policy.assert_called_once_with(
        expected, source="platform_status")
    payload = node._schedule_broadcast.call_args.args[0]
    assert payload["battery"] == expected
    assert payload["charging_required"] is (expected < 35)
    assert (UiBackendNode._mission_dispatch_battery_block(node, "B4") is None) is (
        expected >= 35)


def test_unavailable_platform_soc_revokes_previous_admission():
    node = backend(soc=80, state=AvgServiceState.DROP_ZONE_WAIT, phase="IDLE")
    node._runtime_policy = mock.Mock()
    node._update_runtime_state = lambda update: update()
    node._update_low_battery_return_policy = mock.Mock()
    with mock.patch.object(UiBackendNode, "_publish_destination_dispatch_status"):
        UiBackendNode._on_platform_status(
            node, AvgPlatformStatus(control_mode=1, battery_state_available=False))
    assert node._state.battery_percentage == -1
    node._update_low_battery_return_policy.assert_called_once_with(-1, source="platform_status")
    assert node._schedule_broadcast.call_args.args[0]["battery"] == -1
    assert UiBackendNode._mission_dispatch_battery_block(node, "B4") is not None


@pytest.mark.parametrize("soc", [None, -1, 101, float("nan"), float("inf"), "invalid"])
def test_unknown_soc_requires_charging_but_never_synthesizes_urgent_motion(soc):
    assert not urgent_return_required(soc)
    assert battery_policy_snapshot(soc)["charging_required"]


def test_policy_snapshot_keeps_operator_mode_and_selected_method_distinct():
    snapshot = battery_policy_snapshot(
        24.9, urgent_latched=True, parking_method="auto", selected_method="apriltag")
    assert snapshot["battery_return_urgent"]
    assert snapshot["parking_policy_mode"] == "auto"
    assert snapshot["parking_selected_method"] == "apriltag"
    assert snapshot["urgent_return_battery_percentage"] == 25.0
    assert snapshot["minimum_battery_percentage"] == 35.0


@pytest.mark.parametrize("phase", [
    "ALIGN_ENTRY_YAW", "CRAB_IN", "ROTATE_180", "UNLOAD_WAIT", "WAIT_RETURN",
    "RECALL_CLEARANCE_WAIT", "CRAB_OUT",
])
def test_active_site_phase_uses_controller_only_and_once_per_generation(phase):
    node = backend(state=AvgServiceState.RECALL_TO_SITE_ROAD, phase=phase)
    for _ in range(3):
        UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    node._publish_camping_site_maneuver_controller_return.assert_called_once()
    source = node._publish_camping_site_maneuver_controller_return.call_args.kwargs["source"]
    assert "battery_urgent_return" in source.split(":")
    assert "site=B4" in source.split(":")
    assert "g=17" in source.split(":")
    assert node._return_requested_generation == 17
    assert node._low_battery_return_started
    node._request_nav2_cancel.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()
    node._publish_site_goal_pose.assert_not_called()


def test_road_idle_uses_existing_cancel_before_return_transition():
    node = backend(state=AvgServiceState.MOVING_TO_SITE, phase="IDLE")
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    node._request_nav2_cancel.assert_called_once()
    node._schedule_manual_return_transition.assert_called_once()
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
    assert node._publish_mission_engage.call_args.args[0] is False


@pytest.mark.parametrize("changes", [
    {"_latest_platform_motion_ready": False},  # RC, EStop, or actual BMS fault
    {"_latest_platform_status_time_s": 97.0},
    {"_latest_campsite_phase": ""},
    {"_latest_campsite_status_time_s": 97.0},
    {"_latest_campsite_site": "camping_site_9"},
])
def test_urgent_return_is_deferred_without_fresh_safe_matching_owner(changes):
    node = backend()
    vars(node).update(changes)
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    assert node._battery_return_urgent
    assert node._return_requested_generation == 0
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()
    node._request_nav2_cancel.assert_not_called()
    assert not any(call.args and call.args[0] is True
                   for call in node._publish_platform_drive_enable.call_args_list)


def test_dropzone_exit_clears_old_goal_and_requests_return_only_at_road_handoff():
    node = backend(state=AvgServiceState.DEPARTING_DROP_ZONE, phase="IDLE")
    node._drop_zone_exit_active = True
    node._pending_site_after_drop_zone_exit = ("B4", "camping_site_4", "guest")
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    assert node._pending_site_after_drop_zone_exit is None
    assert node._urgent_return_after_departure
    node._schedule_manual_return_transition.assert_not_called()
    UiBackendNode._on_drop_zone_exit_complete(node, AvgBool(data=True))
    node._request_nav2_cancel.assert_called_once()
    node._schedule_manual_return_transition.assert_called_once()
    node._publish_site_goal_pose.assert_not_called()
    assert not node._urgent_return_after_departure
    assert node._return_requested_generation == 17


def test_road_handoff_during_rc_or_estop_defers_urgent_transition():
    node = backend(state=AvgServiceState.DEPARTING_DROP_ZONE, phase="IDLE")
    node._drop_zone_exit_active = True
    node._pending_site_after_drop_zone_exit = ("B4", "camping_site_4", "guest")
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    node._latest_platform_motion_ready = False
    UiBackendNode._on_drop_zone_exit_complete(node, AvgBool(data=True))
    node._request_nav2_cancel.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()
    assert node._return_requested_generation == 0


def test_old_departure_handoff_cannot_recall_replacement_generation():
    node = backend(state=AvgServiceState.DEPARTING_DROP_ZONE, phase="IDLE")
    node._drop_zone_exit_active = True
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    node._active_mission_generation = 18
    UiBackendNode._on_drop_zone_exit_complete(node, AvgBool(data=True))
    node._schedule_manual_return_transition.assert_not_called()
    node._request_nav2_cancel.assert_not_called()


def test_stop_cleared_identity_prevents_late_low_soc_heartbeat_restart():
    node = backend()
    UiBackendNode._clear_active_mission_identity(node)
    # Even a delayed old site/phase heartbeat cannot reconstruct ownership.
    message = ModuleState(operating_state="UNLOAD_WAIT", message="site=camping_site_4")
    UiBackendNode._on_campsite_policy_status(node, message)
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()
    assert not node._battery_return_urgent


def test_urgent_battery_does_not_move_post_turn_robot_while_user_is_loading():
    node = backend(state=AvgServiceState.GUEST_LOADING_WAIT, phase="RECALL_RETURN_WAIT")
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    assert node._battery_return_urgent
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()
    node._request_nav2_cancel.assert_not_called()
    node._publish_mission_engage.assert_not_called()


@pytest.mark.parametrize("mode", ["auto", "apriltag"])
@pytest.mark.parametrize("observed_method", ["", "reverse", "apriltag"])
def test_explicit_dock_ack_requests_final_method_without_claiming_controller(mode, observed_method):
    node = backend(soc=80, state=AvgServiceState.DROP_ZONE_WAIT, phase="IDLE")
    node.parking_method = mode
    node._parking_selected_method = observed_method
    result = UiBackendNode.request_manual_dock(node)
    assert result["success"]
    assert result["action"] == "docking_requested"
    assert result["message"] == "Explicit charging docking requested"
    assert result["parking_requested_final_method"] == "apriltag"
    assert "parking_selected_method" not in result
    assert node._parking_selected_method == observed_method
    node._publish_parking_operation.assert_called_once_with(
        MotionOperation.START, source="http:manual_dock:force_docking")
    node._request_nav2_cancel.assert_not_called()
    node._schedule_manual_return_transition.assert_not_called()


@pytest.mark.parametrize("state", [
    AvgServiceState.SITE_ENTRY, AvgServiceState.GUEST_LOADING_WAIT,
    AvgServiceState.MOVING_TO_SITE, AvgServiceState.RETURN_WITH_CARGO,
])
def test_explicit_dock_cannot_start_away_from_dropzone(state):
    node = backend(state=state)
    assert UiBackendNode.request_manual_dock(node)["error"] == "docking_requires_drop_zone"
    node._publish_parking_operation.assert_not_called()


def test_explicit_dock_is_idempotent_while_charging_and_protects_active_parking():
    node = backend(state=AvgServiceState.CHARGING)
    node._latest_platform_is_charging = True
    assert UiBackendNode.request_manual_dock(node)["action"] == "already_charging"
    node._publish_parking_operation.assert_not_called()
    node._latest_platform_is_charging = False
    node._latest_service_state = int(AvgServiceState.DROP_ZONE_PARKING)
    node._parking_controller_operating_states = {"auto": "RUNNING"}
    assert UiBackendNode.request_manual_dock(node)["error"] == "parking_in_progress"
    node._publish_parking_operation.assert_not_called()


def final_wait_backend():
    node = backend(phase="RECALL_RETURN_WAIT")
    node._return_requested_generation = 17
    node._return_progress_generation = 17
    node._recall_final_return_generation = 0
    return node


def test_final_loading_wait_blocks_urgent_and_generic_manual_return():
    node = final_wait_backend()
    UiBackendNode._maybe_start_urgent_battery_return(node, "platform_status")
    assert UiBackendNode._request_return_to_drop_zone_serialized(node, "http:manual_return") == "recall_final_confirmation_required"
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
    node._request_nav2_cancel.assert_not_called()


def test_first_button_replay_cannot_authorize_final_loading_departure():
    node = final_wait_backend()
    result = UiBackendNode.request_owned_return_to_drop_zone(
        node, "B4", 17, source="robot_ui:usage_complete", allowed_owners={"robot"},
        recall_final_return=False,
    )
    assert not result["success"]
    assert result["error"] == "recall_final_confirmation_required"
    node._publish_camping_site_maneuver_controller_return.assert_not_called()


def test_final_button_requires_stage_and_generation_and_publishes_only_once():
    node = final_wait_backend()
    assert UiBackendNode._recall_final_return_ready(node)
    stale = UiBackendNode.request_owned_return_to_drop_zone(
        node, "B4", 16, source="robot_ui:usage_complete", allowed_owners={"robot"},
        recall_final_return=True,
    )
    assert not stale["success"]
    for _ in range(2):
        result = UiBackendNode.request_owned_return_to_drop_zone(
            node, "B4", 17, source="robot_ui:usage_complete", allowed_owners={"robot"},
            recall_final_return=True,
        )
        assert result["success"]
    node._publish_camping_site_maneuver_controller_return.assert_called_once()
    source = node._publish_camping_site_maneuver_controller_return.call_args.kwargs["source"]
    assert "recall_final_return" in source.split(":")
    assert not UiBackendNode._recall_final_return_ready(node)
    node._request_nav2_cancel.assert_not_called()


def test_early_final_button_is_rejected_at_initial_roadside_wait():
    node = backend(phase="WAIT_RETURN")
    result = UiBackendNode.request_owned_return_to_drop_zone(
        node, "B4", 17, source="robot_ui:usage_complete", allowed_owners={"robot"},
        recall_final_return=True,
    )
    assert result["error"] == "recall_final_return_not_ready"
    node._publish_camping_site_maneuver_controller_return.assert_not_called()
