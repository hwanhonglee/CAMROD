"""HH_261002 - Explicit resume never follows disarm, a stale token or stale data."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))
from camrod_ui.manual_mission_resume import ROAD_STAGES, resume_payload, resume_reason
from camrod_ui.mission_recording_bridge import MissionRecordingEmitter
from camrod_ui.mission_journal import MissionJournal
from camrod_ui.ui_backend_node import UiBackendNode
from avg_msgs.msg import AvgServiceState


def conditions():
    return dict(enabled=True, manual={"armed": False, "holding": False},
                active_generation=0, now_s=100., pose_received_s=99.9,
                pose_stamp_s=99.9, platform_received_s=99.9, platform_stamp_s=99.9,
                speed_mps=0., yaw_rate_rps=0., pose_valid=True, platform_ready=True,
                ready=True, startup_pending=False, cancellation_ready=True,
                settled=True, outside_station=True)


@pytest.mark.parametrize("changes,expected", [
    ({}, "ready"), ({"enabled": False}, "disabled"),
    ({"manual": {"armed": True}}, "manual_drive_active"),
    ({"manual": {"holding": True}}, "manual_drive_active"),
    ({"active_generation": 9}, "another_mission_active"),
    ({"startup_pending": True}, "cancellation_pending"),
    ({"cancellation_ready": False}, "cancellation_pending"),
    ({"settled": False}, "cancellation_pending"),
    ({"pose_received_s": 97.}, "stale_feedback"),
    ({"pose_stamp_s": 0.}, "stale_feedback"),
    ({"pose_stamp_s": 101.}, "stale_feedback"),
    ({"platform_stamp_s": float("nan")}, "stale_feedback"),
    ({"platform_received_s": 97.}, "stale_feedback"),
    ({"pose_valid": False}, "invalid_pose"),
    ({"outside_station": False}, "station_maneuver_required"),
    ({"speed_mps": .04}, "robot_not_stationary"),
    ({"yaw_rate_rps": -.04}, "robot_not_stationary"),
    ({"speed_mps": float("nan")}, "invalid_motion_feedback"),
    ({"platform_ready": False}, "platform_not_ready"),
    ({"ready": False}, "system_not_ready"),
])
def test_fail_closed_resume_policy(changes, expected):
    args = {**conditions(), **changes}
    assert resume_reason({"stage": "outbound"}, **args) == expected


def test_only_normal_road_stages_supported():
    assert set(ROAD_STAGES) == {"MOVING_TO_SITE", "RECALL_TO_SITE_ROAD", "RETURNING_TO_DROP_ZONE"}
    assert resume_reason({"stage": "unsupported"}, **conditions()) == "unsupported_stage"
    assert resume_reason(None, **conditions()) == "no_suspended_mission"
    assert not resume_payload(None, "no_suspended_mission")["pending"]


def backend(stage="outbound", intent="delivery"):
    context = dict(token="exact-token", site="B9", intent=intent, owner="robot",
                   source="robot_ui:recall" if intent == "recall" else "robot_ui:destination",
                   generation=77, stage=stage, return_requested_generation=77 if stage == "return" else 0,
                   recall_final_return_generation=77 if stage == "return" else 0,
                   service_metrics_return_generation=77 if stage == "return" else -1,
                   suspended_monotonic=1.)
    node = SimpleNamespace(
        _manual_resume_context=context,
        _destination_dispatch_lock=threading.RLock(),
        _manual_drive_transition_lock=threading.RLock(), _lock=threading.RLock(),
        _state=SimpleNamespace(destination={}), _active_mission_generation=0,
        _resolve_mission_key_for_site=lambda site: "B9-key",
        _keypoints_by_mission_key={"B9-key": object()},
        _publish_planning_return_request=Mock(),
        _publish_planning_camping_site_recall=Mock(return_value=True),
        _publish_goal_for_site=Mock(return_value={"goal_pose_published": True}),
        _publish_engage=Mock(), _publish_mission_engage=Mock(),
        _publish_service_state=Mock(), _schedule_broadcast=Mock(),
        _stop_active_service=Mock(), _mission_recording=Mock(),
    )
    return node


@pytest.mark.parametrize("stage,intent", [("outbound", "delivery"), ("outbound", "recall"), ("return", "recall")])
def test_explicit_token_replans_correct_leg_and_preserves_identity(stage, intent):
    node = backend(stage, intent)
    original = dict(node._manual_resume_context)
    with patch.object(UiBackendNode, "_manual_resume_snapshot", return_value=resume_payload(original, "ready")):
        result = UiBackendNode.request_manual_mission_resume(node, "exact-token")
        duplicate = UiBackendNode.request_manual_mission_resume(node, "exact-token")
    assert result["accepted"]
    assert not duplicate["accepted"]
    assert node._manual_resume_context is None
    assert node._active_mission_generation == 77
    assert node._active_mission_intent == intent
    assert node._active_mission_owner == "robot"
    if stage == "return":
        node._publish_planning_return_request.assert_called_once()
        assert node._recall_final_return_generation == 77
        node._publish_goal_for_site.assert_not_called()
    elif intent == "recall":
        node._publish_planning_camping_site_recall.assert_called_once()
        node._publish_goal_for_site.assert_not_called()
    else:
        node._publish_goal_for_site.assert_called_once()
    node._mission_recording.resume.assert_called_once()


@pytest.mark.parametrize("token,reason", [(None, "ready"), ("", "ready"), ("old-token", "ready"),
    ("exact-token", "manual_drive_active"), ("exact-token", "system_not_ready")])
def test_rejected_resume_never_reopens_control(token, reason):
    node = backend()
    with patch.object(UiBackendNode, "_manual_resume_snapshot", return_value=resume_payload(node._manual_resume_context, reason)):
        assert not UiBackendNode.request_manual_mission_resume(node, token)["accepted"]
    assert node._manual_resume_context
    node._publish_engage.assert_not_called()
    node._publish_goal_for_site.assert_not_called()
    node._mission_recording.resume.assert_not_called()


def test_failed_replan_consumes_token_and_full_stops():
    node = backend()
    node._publish_goal_for_site.return_value = {"goal_pose_published": False}
    with patch.object(UiBackendNode, "_manual_resume_snapshot", return_value=resume_payload(node._manual_resume_context, "ready")):
        result = UiBackendNode.request_manual_mission_resume(node, "exact-token")
    assert not result["accepted"]
    assert node._manual_resume_context is None
    node._stop_active_service.assert_called_once_with(source="manual_resume_failed")


def test_same_journal_survives_manual_stop_resume_and_return(tmp_path):
    journal = MissionJournal(tmp_path, environment="test", now_fn=lambda: 1000.)
    emitter = MissionRecordingEmitter(
        lambda raw: journal.observe_event(json.loads(raw), received_unix=1000.),
        now_fn=lambda: 1000., session="same-session")
    emitter.start("B9", "recall", 77, "first")
    emitter.request_return("guest-final", final_return=True)
    original = emitter.mission_id
    emitter.stop("ws_manual_drive_arm")
    emitter.resume("manual_resume", "return", "single-token")
    assert emitter.mission_id == original
    assert emitter.return_seen
    emitter.phase(3, "RETURNING_TO_DROP_ZONE")
    emitter.phase(10, "DROP_ZONE_PARKING")
    emitter.phase(0, "DROP_ZONE_WAIT")
    rows = journal.snapshot()["missions"]
    assert len(rows) == 1
    assert rows[0]["result"] == "completed"
    assert rows[0]["attempt_count"] == 2
    journal.close()


def test_capture_preserves_original_identity_and_rearm_does_not_replace_token():
    node = backend("return", "recall")
    original = node._manual_resume_context
    node.manual_mission_resume_enabled = True
    node._active_mission_generation = 77
    node._active_mission_site = "B9"
    node._active_mission_source = "guest:recall"
    node._active_mission_owner = "guest"
    node._active_mission_intent = "recall"
    node._return_requested_generation = 77
    node._recall_final_return_generation = 77
    node._latest_service_state = AvgServiceState.RETURNING_TO_DROP_ZONE
    UiBackendNode._capture_manual_resume(node)
    captured = dict(node._manual_resume_context)
    assert captured["token"] != original["token"]
    assert captured["owner"] == "guest"
    assert captured["stage"] == "return"
    node._active_mission_generation = 0
    UiBackendNode._capture_manual_resume(node)
    assert node._manual_resume_context == captured


def test_new_mission_invalidates_old_resume_context():
    node = backend()
    UiBackendNode._claim_active_mission(node, "B8", "robot_ui:destination")
    assert node._manual_resume_context is None
    assert node._active_mission_site == "B8"


def test_global_stop_invalidates_but_manual_preemption_preserves_context():
    for source in ("http_stop", "service_state:OPERATOR_STOPPED", "ws_manual_drive_arm"):
        node = backend()
        node.site_names = ["B9"]
        node.publish_mission_engage_from_destination = True
        node._cancel_pending_manual_return_transition = Mock()
        node._cancel_active_motion = Mock()
        with patch.object(UiBackendNode, "_publish_destination_dispatch_status"):
            UiBackendNode._stop_active_service_serialized(node, source)
        assert bool(node._manual_resume_context) == (source == "ws_manual_drive_arm")


def test_cancelled_cargo_heartbeat_cannot_restore_visible_return_state():
    node = backend("return", "recall")
    node._state.service_state_name = "OPERATOR_STOPPED"
    message = AvgServiceState()
    message.state = AvgServiceState.RETURN_WITH_CARGO
    message.state_name = "RETURN_WITH_CARGO"
    message.description = "camping_site_maneuver_controller:CRAB_OUT"
    UiBackendNode._on_service_state_serialized(node, message)
    assert node._state.service_state_name == "OPERATOR_STOPPED"
    node._publish_engage.assert_not_called()


def test_restart_without_memory_cannot_reconstruct_a_token():
    node = SimpleNamespace(_manual_resume_context=None)
    assert not UiBackendNode._manual_resume_snapshot(node)["pending"]


def feedback_backend():
    node = backend()
    node.manual_mission_resume_enabled = True
    node._manual_resume_cancel_services_ready = True
    node._manual_resume_cancel_futures = []
    node._latest_arrival_pose = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=SimpleNamespace(sec=100, nanosec=0)),
        pose=SimpleNamespace(position=SimpleNamespace(x=5., y=5.),
            orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    node._latest_arrival_pose_time_s = 100.
    node._latest_platform_status_time_s = 100.
    node._manual_resume_velocity_stamp_s = 100.
    node._manual_resume_speed_mps = 0.
    node._manual_resume_yaw_rate_rps = 0.
    node._latest_platform_motion_ready = True
    node._state.ready = True
    node._state.battery_percentage = 80
    node._now_s = lambda: 100.
    node.site_arrival_pose_timeout_s = 2.
    node._drop_zone_polygons = [[(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]]
    return node


@pytest.mark.parametrize("done,code,goals,expected", [
    (True, 0, [], "ready"), (False, 0, [], "cancellation_pending"),
    (True, 3, [], "ready"), (True, 3, [object()], "cancellation_pending"),
    (True, 1, [], "cancellation_pending"), (True, 2, [], "cancellation_pending"),
])
def test_actual_snapshot_waits_for_successful_cancellation(done, code, goals, expected):
    node = feedback_backend()
    node._manual_resume_cancel_futures = [SimpleNamespace(
        done=lambda: done, result=lambda: SimpleNamespace(return_code=code, goals_canceling=goals))]
    assert UiBackendNode._manual_resume_snapshot(node)["reason"] == expected


def test_actual_snapshot_blocks_station_geometry_and_outbound_low_battery():
    node = feedback_backend()
    node._state.battery_percentage = 20
    assert UiBackendNode._manual_resume_snapshot(node)["reason"] == "battery_below_mission_minimum"
    node._manual_resume_context["stage"] = "return"
    assert UiBackendNode._manual_resume_snapshot(node)["can_resume"]
    node._latest_arrival_pose.pose.position.x = 0.
    node._latest_arrival_pose.pose.position.y = 0.
    assert UiBackendNode._manual_resume_snapshot(node)["reason"] == "station_maneuver_required"


@pytest.mark.parametrize("nav_status", [1, 2, 3])
def test_successful_cancel_ack_still_waits_for_old_nav_action_to_end(nav_status):
    node = feedback_backend()
    node._runtime_policy = SimpleNamespace(nav_status=nav_status)
    assert UiBackendNode._manual_resume_snapshot(node)["reason"] == "cancellation_pending"
