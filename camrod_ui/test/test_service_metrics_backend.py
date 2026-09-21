"""Offline regression of metrics through production backend callbacks.

No ROS node is initialized and all command publishers are in-memory stubs.
B1-B13 parameterization checks event handling, not physical route traversal.
"""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))

import rclpy
from rclpy.node import Node
from avg_msgs.msg import AvgBool, AvgServiceState, ModuleState
from camrod_ui.service_metrics import ServiceMetricsTracker
from camrod_ui.ui_backend_node import UiBackendNode


@pytest.fixture(autouse=True)
def forbid_live_ros(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("metrics regression must not initialize ROS")
    monkeypatch.setattr(rclpy, "init", forbidden)
    monkeypatch.setattr(rclpy, "create_node", forbidden)
    monkeypatch.setattr(Node, "__init__", forbidden)


def make_backend(tracker, site="B1", intent="delivery"):
    source = "guest:kiosk" if intent == "recall" else "robot_ui"
    key = f"camping_site_{int(site[1:])}"
    node = SimpleNamespace(
        _active_mission_site=site, _active_mission_source=source,
        _active_mission_generation=1, _return_requested_generation=0,
        _drop_zone_exit_active=True,
        _pending_site_after_drop_zone_exit=(site, key, source),
        _latest_platform_is_charging=False,
        _latest_service_state=int(AvgServiceState.DEPARTING_DROP_ZONE),
        _drop_zone_exit_handoff_ready=False, _drop_zone_exit_failure_latched=False,
        _drop_zone_exit_waiting_for_fresh_status=False,
        _drop_zone_exit_cancel_suppressed=False,
        _service_metrics=tracker, service_state_topic="/service/state",
        _lock=threading.RLock(),
        _state=SimpleNamespace(
            service_state=15, service_state_name="DEPARTING_DROP_ZONE",
            service_state_description="", battery_percentage=80,
        ),
        publish_mission_engage_from_destination=False,
    )
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)
    node._schedule_broadcast = lambda payload: None
    node.pub_service_state = SimpleNamespace(publish=lambda msg: None)
    node._publish_service_state = lambda state, source: UiBackendNode._publish_service_state(node, state, source)
    node._on_service_state = lambda msg: UiBackendNode._on_service_state(node, msg)
    node._on_drop_zone_exit_complete = lambda msg: UiBackendNode._on_drop_zone_exit_complete(node, msg)
    node._publish_site_mission_key = lambda *args: None
    node._publish_site_goal_pose = lambda *args: True
    node._update_low_battery_return_policy = lambda *args, **kwargs: None
    return node


@pytest.mark.parametrize("site", [f"B{i}" for i in range(1, 14)])
@pytest.mark.parametrize("intent", ["delivery", "recall"])
@pytest.mark.parametrize("order", [("bool", "status"), ("status", "bool")])
def test_all_sites_departure_keeps_record_and_velocity_anchor(site, intent, order):
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker, site, intent)
    UiBackendNode._start_service_metrics(
        node, site, node._pending_site_after_drop_zone_exit[1],
        node._active_mission_source, 1,
    )
    node._publish_service_state(15, "test")
    tracker.observe_velocity(1.0, 0.0, 1.0)
    tracker.observe_velocity(1.0, 0.0, 2.0)
    complete = AvgBool()
    complete.data = True
    status = ModuleState()
    status.operating_state = "ROAD_HANDOFF_READY"
    callbacks = {
        "bool": lambda: UiBackendNode._on_drop_zone_exit_complete(node, complete),
        "status": lambda: UiBackendNode._on_drop_zone_maneuver_status(node, status),
    }
    def release_recall(backend, mission_key, origin):
        backend._publish_service_state(7, origin)
        return True
    with patch.object(UiBackendNode, "_publish_planning_camping_site_recall", release_recall):
        for event in order:
            callbacks[event]()
    tracker.observe_velocity(1.0, 0.0, 3.0)
    tracker.observe_velocity(1.0, 0.0, 4.0)
    result = tracker.snapshot()
    assert node._latest_service_state == (7 if intent == "recall" else 1)
    assert result["current_service"]["site"] == site
    assert result["current_service"]["intent"] == intent
    assert result["current_service"]["request_id"].endswith(":mission:1")
    assert result["lifetime"]["completed_service_count"] == 0
    assert result["lifetime"]["distance_m"] == pytest.approx(3.0)
    assert result["lifetime"]["distance_breakdown_m"][intent] == pytest.approx(3.0)
    assert sum(result["lifetime"]["distance_breakdown_m"].values()) == pytest.approx(3.0)
    tracker.close()


@pytest.mark.parametrize("charging", [False, True])
def test_failed_departure_is_interrupted_before_safe_station_state(charging):
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker, intent="recall")
    node._latest_platform_is_charging = charging
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "guest:kiosk", 1)
    node._publish_service_state(15, "test")
    tracker.observe_velocity(1, 0, 0)
    tracker.observe_velocity(1, 0, 1)
    UiBackendNode._mark_drop_zone_exit_failed(node, "controller_error")
    UiBackendNode._mark_drop_zone_exit_failed(node, "controller_error")
    result = tracker.snapshot()
    assert result["current_service"] is None
    assert result["lifetime"]["completed_service_count"] == 0
    assert result["lifetime"]["interrupted_service_count"] == 1
    assert result["lifetime"]["distance_breakdown_m"]["recall"] == 1
    assert "drop_zone_exit_failed" in result["recent_services"][0]["interruption_reason"]
    tracker.close()


def test_recall_turnaround_before_confirmation_and_final_return_are_separate():
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker, intent="recall")
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "guest:kiosk", 1)
    node._publish_service_state(7, "test")
    tracker.observe_velocity(1, 0, 0)
    tracker.observe_velocity(1, 0, 1)
    UiBackendNode._observe_service_metrics(
        node, 9, "RETURN_WITH_CARGO",
        "camping_site_maneuver_controller:ROTATE_180:clearance",
    )
    tracker.observe_velocity(1, 0, 2)
    assert tracker.summary()["current_service"]["distance_breakdown_m"]["recall"] == 2
    node._recall_final_return_generation = 1
    UiBackendNode._observe_service_metrics(node, 9, "RETURN_WITH_CARGO")
    tracker.observe_velocity(-1, 0, 3)  # crossing interval is unknown, not lost
    tracker.observe_velocity(-1, 0, 4)
    node._publish_service_state(10, "parking")
    tracker.observe_velocity(-1, 0, 5)
    node._publish_service_state(12, "parked")
    result = tracker.summary()
    assert result["current_service"] is None
    assert result["lifetime"]["distance_breakdown_m"] == {
        "delivery": 0.0, "recall": 2.0, "return": 2.0, "unknown": 1.0,
    }
    assert result["lifetime"]["distance_m"] == 5.0
    assert result["lifetime"]["completed_service_count"] == 1
    tracker.close()


@pytest.mark.parametrize("intent", ["delivery", "recall"])
def test_failed_same_site_retry_keeps_motion_generation_but_gets_new_record(intent):
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker, intent=intent)
    source = node._active_mission_source
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", source, 1)
    first_id = tracker.summary()["current_service"]["request_id"]
    node._publish_service_state(15, "first_attempt")
    tracker.observe_velocity(1, 0, 0)
    tracker.observe_velocity(1, 0, 1)
    UiBackendNode._mark_drop_zone_exit_failed(node, "controller_error")
    UiBackendNode._mark_drop_zone_exit_failed(node, "controller_error")
    # The actual control identity intentionally coalesces this same-site retry.
    generation = UiBackendNode._claim_active_mission(node, "B1", source)
    assert generation == 1
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", source, generation)
    assert tracker.has_active_service
    assert tracker.summary()["current_service"]["request_id"] != first_id
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", source, generation)
    node._publish_service_state(15, "retry")
    tracker.observe_velocity(1, 0, 2)
    tracker.observe_velocity(1, 0, 3)
    result = tracker.summary()["lifetime"]
    assert result["service_attempt_count"] == 2
    assert result["interrupted_service_count"] == 1
    assert result["completed_service_count"] == 0
    assert result["distance_breakdown_m"][intent] == 2
    tracker.close()


def test_request_identity_deduplicates_retry_but_not_backend_restart():
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker)
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "robot_ui", 1)
    request = tracker.summary()["current_service"]["request_id"]
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "robot_ui", 1)
    assert tracker.summary()["lifetime"]["service_attempt_count"] == 1
    restarted = make_backend(tracker)
    UiBackendNode._start_service_metrics(restarted, "B1", "camping_site_1", "robot_ui", 1)
    assert tracker.summary()["current_service"]["request_id"] != request
    assert tracker.summary()["lifetime"]["service_attempt_count"] == 2
    tracker.close()


def test_standalone_return_starts_once_without_reopening_an_interrupted_record():
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker)
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "robot_ui", 1)
    tracker.interrupt_service("operator_stop")
    UiBackendNode._ensure_return_service_metrics(node, "robot_ui:return")
    first = tracker.summary()["current_service"]["id"]
    UiBackendNode._ensure_return_service_metrics(node, "robot_ui:return")
    node._publish_service_state(3, "return")
    tracker.observe_velocity(-1, 0, 0)
    tracker.observe_velocity(-1, 0, 1)
    node._publish_service_state(12, "parked")
    result = tracker.snapshot()
    assert result["last_completed_service"]["id"] == first
    assert result["last_completed_service"]["intent"] == "return"
    assert result["lifetime"]["distance_breakdown_m"]["return"] == 1
    assert result["lifetime"]["completed_service_count"] == 1
    assert result["lifetime"]["interrupted_service_count"] == 1
    tracker.close()

@pytest.mark.parametrize("site", [f"B{i}" for i in range(1, 14)])
@pytest.mark.parametrize("intent", ["delivery", "recall"])
def test_all_sites_outbound_wait_return_and_parking_are_one_completed_record(site, intent, tmp_path):
    database = tmp_path / "metrics.sqlite3"
    tracker = ServiceMetricsTracker(database)
    node = make_backend(tracker, site, intent)
    UiBackendNode._start_service_metrics(node, site, f"camping_site_{site[1:]}", node._active_mission_source, 1)
    node._publish_service_state(7 if intent == "recall" else 1, "outbound")
    for stamp in range(4):
        tracker.observe_velocity(1, 0, stamp)
    node._publish_service_state(8 if intent == "recall" else 11, "site_wait")
    tracker.observe_velocity(0, 0, 4)
    tracker.observe_velocity(0, 0, 5)
    assert tracker.summary()["lifetime"]["completed_service_count"] == 0
    node._recall_final_return_generation = 1
    node._publish_service_state(3, "return")
    tracker.observe_velocity(0, 0, 6)
    tracker.observe_velocity(-1, 0, 7)
    tracker.observe_velocity(-1, 0, 8)
    node._publish_service_state(10, "reverse_parking")
    tracker.observe_velocity(-1, 0, 9)
    tracker.observe_velocity(0, 0, 10)
    node._publish_service_state(12, "parked")
    node._publish_service_state(13, "charging")
    result = tracker.snapshot()
    assert result["lifetime"]["distance_m"] == 6.5
    assert result["lifetime"]["completed_service_count"] == 1
    assert result["lifetime"]["service_attempt_count"] == 1
    assert result["lifetime"]["distance_breakdown_m"][intent] == 3.5
    assert result["lifetime"]["distance_breakdown_m"]["return"] == 3.0
    assert result["lifetime"]["distance_breakdown_m"]["unknown"] == 0.0
    assert result["lifetime"]["moving_s"] == 8
    assert result["lifetime"]["waiting_s"] == 2
    tracker.close()
    reopened = ServiceMetricsTracker(database)
    assert reopened.snapshot()["lifetime"] == result["lifetime"]
    reopened.close()


def test_admitted_planning_return_dispatch_starts_standalone_accounting():
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker)
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    node._publish_platform_drive_enable = lambda *args, **kwargs: None
    published = []
    node.pub_planning_return_to_drop_zone = SimpleNamespace(publish=published.append)
    node.planning_return_to_drop_zone_topic = "/planning/return_to_drop_zone"
    UiBackendNode._publish_planning_return_request(node, "robot_ui:return")
    assert len(published) == 1
    assert tracker.summary()["current_service"]["intent"] == "return"
    assert tracker.summary()["current_service"]["state"] == 3
    tracker.close()


def test_phase_status_cannot_finish_newly_accepted_record_from_retained_state_zero():
    tracker = ServiceMetricsTracker(None)
    node = make_backend(tracker)
    UiBackendNode._start_service_metrics(node, "B1", "camping_site_1", "robot_ui", 1)
    node._latest_service_state = 0
    node._now_s = lambda: 100
    node._resolve_mission_key_for_site = lambda site: "camping_site_1"
    status = ModuleState()
    status.operating_state = "WAIT_RETURN"
    status.message = "site=camping_site_1"
    with patch.object(UiBackendNode, "_publish_destination_dispatch_status"):
        UiBackendNode._on_campsite_policy_status(node, status)
    assert tracker.has_active_service
    assert tracker.summary()["lifetime"]["completed_service_count"] == 0
    tracker.close()
