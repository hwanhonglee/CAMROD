"""Station exit admission uses live geometry, not restart UI state or owner."""

from pathlib import Path
import json
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))

from avg_msgs.msg import AvgBool, AvgServiceState, ModuleState, MotionOperation  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


POLYGON = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]


@pytest.fixture
def backend():
    events = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = Time(sec=100)
    pose.pose.position.x = 1.0
    pose.pose.position.y = 1.0
    pose.pose.orientation.w = 1.0
    node = SimpleNamespace(
        events=events,
        _drop_zone_polygons=[POLYGON],
        _latest_arrival_pose=pose,
        _latest_arrival_pose_time_s=100.0,
        site_arrival_pose_timeout_s=2.0,
        _now_s=lambda: 100.0,
        _drop_zone_exit_active=False,
        _drop_zone_exit_handoff_ready=False,
        _drop_zone_exit_failure_latched=False,
        _drop_zone_exit_waiting_for_fresh_status=False,
        _drop_zone_exit_cancel_suppressed=False,
        _pending_site_after_drop_zone_exit=None,
        _charging_departure_delay_pending=False,
        _latest_platform_is_charging=False,
        _latest_service_state=None,
        _active_mission_site="",
        _service_metrics=None,
        _lock=threading.Lock(),
        _pending_site_route_goal_stamps={},
        _site_route_anchors={},
        _state=SimpleNamespace(ws_site_states={}, destination={}, service_state=None),
        site_names=["B1", "B2"],
        _runtime_policy=SimpleNamespace(update_goal_received=lambda mode: events.append(("runtime_goal", mode))),
        publish_engage_from_destination=True,
        publish_mission_engage_from_destination=True,
        charging_departure_delay_s=0.0,
        planning_camping_site_recall_topic="/planning/state_machine/camping_site_recall",
        pub_planning_camping_site_recall=SimpleNamespace(
            publish=lambda message: events.append(("recall", message.site_name))
        ),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=100))),
        get_logger=lambda: Mock(),
        _resolve_mission_key_for_site=lambda site: f"camping_site_{site[1:]}",
        _is_site_occupied=lambda site: False,
        _site_arrival_match=lambda site: (False, f"camping_site_{site[1:]}", 5.0, "outside"),
        _mission_dispatch_battery_block=lambda site: None,
        _update_runtime_state=lambda callback: callback(),
        _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
        _cancel_pending_manual_return_transition=lambda source: None,
    )
    for name in ("engage", "mission_engage", "platform_drive_enable", "parking_operation", "drop_zone_operation", "camping_site_operation"):
        setattr(node, f"_publish_{name}", lambda value, source, name=name, **kwargs: events.append((name, value)))

    def publish_state(state, source):
        node._latest_service_state = int(state)
        events.append(("service_state", int(state)))

    node._publish_service_state = publish_state
    node._publish_site_mission_key = lambda key, source: events.append(("mission_key", key))
    node._publish_site_goal_pose = lambda site, key, source: events.append(("goal", site)) or True
    node._publish_goal_for_site = lambda site, source: {
        "mission_key": f"camping_site_{site[1:]}",
        "goal_pose_published": events.append(("goal", site)) is None,
        "message": "dispatched",
    }
    node._cancel_active_motion = lambda source: UiBackendNode._cancel_service_motion_writers(node, source)
    return node


def dispatch(node, source="http_ui_destination", site="B1"):
    return UiBackendNode._apply_destination_command(node, site, True, source)


@pytest.mark.parametrize("source", ["http_ui_destination", "ws", "guest:kiosk", "robot_ui:recall"])
@pytest.mark.parametrize("state", [None, AvgServiceState.OPERATOR_STOPPED, AvgServiceState.DROP_ZONE_WAIT])
def test_robot_and_guest_inside_station_exit_before_site_goal(backend, source, state):
    backend._latest_service_state = state
    # A prior road handoff cannot override the new physical parked pose.
    backend._drop_zone_exit_handoff_ready = True
    result = dispatch(backend, source)
    assert result["run"] and not result["goal_pose_published"]
    assert not result["recall_request_published"]
    assert backend._pending_site_after_drop_zone_exit == ("B1", "camping_site_1", source)
    operations = [(kind, value) for kind, value in backend.events if kind.endswith("operation")]
    assert operations == [
        ("drop_zone_operation", MotionOperation.CANCEL),
        ("parking_operation", MotionOperation.CANCEL),
        ("drop_zone_operation", MotionOperation.EXIT),
    ]
    assert backend.events.index(("parking_operation", MotionOperation.CANCEL)) < backend.events.index(("engage", True))
    assert backend.events.index(("engage", True)) < backend.events.index(("drop_zone_operation", MotionOperation.EXIT))
    assert not any(kind in {"goal", "recall", "mission_key"} for kind, _ in backend.events)

    complete = AvgBool()
    complete.data = True
    UiBackendNode._on_drop_zone_exit_complete(backend, complete)
    expected = "recall" if UiBackendNode._is_guest_recall_source(source) else "goal"
    assert sum(kind == expected for kind, _ in backend.events) == 1


@pytest.mark.parametrize("source", ["http_ui_destination", "guest:kiosk"])
@pytest.mark.parametrize("state", [None, AvgServiceState.OPERATOR_STOPPED, AvgServiceState.DROP_ZONE_WAIT])
def test_outside_station_never_runs_blind_exit(backend, source, state):
    backend._latest_arrival_pose.pose.position.x = 5.0
    backend._latest_service_state = state
    backend._drop_zone_exit_cancel_suppressed = True
    result = dispatch(backend, source)
    assert result["run"]
    assert result["goal_pose_published"] or result["recall_request_published"]
    assert [(kind, value) for kind, value in backend.events if kind.endswith("operation")] == [
        ("drop_zone_operation", MotionOperation.CANCEL),
        ("parking_operation", MotionOperation.CANCEL),
    ]
    first_motion = next(index for index, event in enumerate(backend.events) if event == ("engage", True))
    assert backend.events.index(("parking_operation", MotionOperation.CANCEL)) < first_motion


@pytest.mark.parametrize("source", ["http_ui_destination", "guest:kiosk"])
@pytest.mark.parametrize("bad_input,expected", [
    ("missing_map", "drop_zone_map_unavailable"),
    ("missing_pose", "drop_zone_pose_unavailable"),
    ("stale_receive", "drop_zone_pose_stale"),
    ("stale_stamp", "drop_zone_pose_stale"),
    ("future_receive", "drop_zone_pose_timestamp_future"),
    ("future_stamp", "drop_zone_pose_timestamp_future"),
    ("zero_stamp", "drop_zone_pose_timestamp_invalid"),
    ("wrong_frame", "drop_zone_pose_frame_mismatch"),
    ("nan_coordinate", "drop_zone_pose_nonfinite"),
    ("boundary", "drop_zone_pose_on_boundary"),
    ("malformed_pose", "drop_zone_pose_invalid"),
])
def test_unknown_origin_rejects_before_claim_or_motion(backend, source, bad_input, expected):
    if bad_input == "missing_map":
        backend._drop_zone_polygons = []
    elif bad_input == "missing_pose":
        backend._latest_arrival_pose = None
    elif bad_input == "stale_receive":
        backend._latest_arrival_pose_time_s = 97.0
    elif bad_input == "stale_stamp":
        backend._latest_arrival_pose.header.stamp.sec = 97
    elif bad_input == "future_receive":
        backend._latest_arrival_pose_time_s = 101.0
    elif bad_input == "future_stamp":
        backend._latest_arrival_pose.header.stamp.sec = 101
    elif bad_input == "zero_stamp":
        backend._latest_arrival_pose.header.stamp.sec = 0
    elif bad_input == "wrong_frame":
        backend._latest_arrival_pose.header.frame_id = "odom"
    elif bad_input == "nan_coordinate":
        backend._latest_arrival_pose.pose.position.x = float("nan")
    elif bad_input == "boundary":
        backend._latest_arrival_pose.pose.position.x = 0.0
    else:
        backend._latest_arrival_pose = SimpleNamespace()
    result = dispatch(backend, source)
    assert result["blocked"] and result["error"] == expected
    assert backend._active_mission_site == ""
    assert backend._pending_site_after_drop_zone_exit is None
    assert not backend._drop_zone_exit_active
    assert not any(kind != "broadcast" for kind, _ in backend.events)


def test_charge_contact_outside_station_refuses_goal_and_exit(backend):
    backend._latest_platform_is_charging = True
    backend._latest_arrival_pose.pose.position.x = 5.0
    result = dispatch(backend)
    assert result["error"] == "drop_zone_charging_pose_mismatch"
    assert not any(kind != "broadcast" for kind, _ in backend.events)


def test_missing_startup_snapshot_still_publishes_safe_rejection(backend):
    messages = []
    backend.pub_destination_dispatch_status = SimpleNamespace(publish=messages.append)
    backend._latest_arrival_pose = None
    result = dispatch(backend)
    assert result["blocked"]
    payload = json.loads(messages[-1].data)
    assert payload["accepted"] is False
    assert payload["service_state"] == -1
    assert payload["error"] == "drop_zone_pose_unavailable"


@pytest.mark.parametrize("inside", [False, True])
def test_battery_rejection_preserves_station_owners_and_queued_redock(backend, inside):
    backend._latest_arrival_pose.pose.position.x = 1.0 if inside else 5.0
    backend._latest_service_state = AvgServiceState.DROP_ZONE_WAIT
    timer = Mock()
    backend._parking_rearm_transition_pending = True
    backend._parking_rearm_transition_timer = timer
    backend._mission_dispatch_battery_block = lambda site: {
        "message": "battery below mission minimum",
        "battery_percentage": 34,
        "minimum_battery_percentage": 35,
    }
    result = dispatch(backend)
    assert result["error"] == "battery_below_mission_minimum"
    assert backend._active_mission_site == ""
    assert not any(kind != "broadcast" for kind, _ in backend.events)
    timer.cancel.assert_not_called()
    assert backend._parking_rearm_transition_pending
    assert backend._parking_rearm_transition_timer is timer


@pytest.mark.parametrize("source", ["http_ui_destination", "guest:kiosk"])
def test_duplicate_and_cancel_do_not_reissue_or_resurrect_exit(backend, source):
    first = dispatch(backend, source)
    backend.events.clear()
    duplicate = dispatch(backend, source)
    assert duplicate["mission_generation"] == first["mission_generation"]
    assert not any(kind != "broadcast" for kind, _ in backend.events)
    assert dispatch(backend, source, "B2")["error"] == "mission_already_active"
    stopped = UiBackendNode._apply_destination_command(
        backend, "B1", False, source, mission_generation=first["mission_generation"]
    )
    assert not stopped["run"]
    assert backend._pending_site_after_drop_zone_exit is None
    assert backend._drop_zone_exit_cancel_suppressed
    assert backend._active_mission_site == ""
    backend.events.clear()
    complete = AvgBool()
    complete.data = True
    UiBackendNode._on_drop_zone_exit_complete(backend, complete)
    heartbeat = ModuleState()
    heartbeat.operating_state = "EXIT_STRAIGHT"
    UiBackendNode._on_drop_zone_maneuver_status(backend, heartbeat)
    assert backend.events == []


@pytest.mark.parametrize("corners", [
    None,
    [{"x": 0, "y": 0}, {"x": 1, "y": 1}],
    [{"x": 0, "y": 0}, {"x": 1, "y": 1}, {"x": 2, "y": 2}],
    [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": float("nan"), "y": 1}],
    [{"x": False, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}],
    [{"x": 0, "y": 0}, {"x": 2, "y": 2}, {"x": 2, "y": 0}, {"x": 0, "y": 1}],
])
def test_malformed_polygon_invalidates_whole_catalog(backend, tmp_path, corners):
    valid = {"corners": [{"x": x, "y": y} for x, y in POLYGON]}
    path = tmp_path / "drop_zones.yaml"
    path.write_text(yaml.safe_dump({"drop_zones": [valid, {"corners": corners}]}))
    backend._drop_zone_polygons = UiBackendNode._load_drop_zone_polygons(backend, str(path))
    assert dispatch(backend)["error"] == "drop_zone_map_unavailable"


def test_missing_or_invalid_yaml_is_fail_closed(backend, tmp_path):
    assert UiBackendNode._load_drop_zone_polygons(backend, "") == []
    assert UiBackendNode._load_drop_zone_polygons(backend, str(tmp_path / "missing")) == []
    path = tmp_path / "invalid.yaml"
    path.write_text("drop_zones: [\n")
    assert UiBackendNode._load_drop_zone_polygons(backend, str(path)) == []


def test_current_bringup_polygon_contains_both_reported_station_poses(backend):
    path = Path(__file__).resolve().parents[2] / "camrod_bringup/config/map/drop_zones.yaml"
    backend._drop_zone_polygons = UiBackendNode._load_drop_zone_polygons(backend, str(path))
    assert len(backend._drop_zone_polygons) == 1
    for x, y in [(-10.690, 41.396), (-11.39, 38.85)]:
        backend._latest_arrival_pose.pose.position.x = x
        backend._latest_arrival_pose.pose.position.y = y
        assert UiBackendNode._station_departure_origin(backend) == (True, "inside_drop_zone")
