"""HJ_261105 - A backend launched parked restores CHARGING / DROP_ZONE_WAIT once after startup recovery."""

from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from builtin_interfaces.msg import Time as RosTime  # noqa: E402

from avg_msgs.msg import AvgServiceState  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402

STOPPED = int(AvgServiceState.OPERATOR_STOPPED)
CHARGING = int(AvgServiceState.CHARGING)
DROP_ZONE_WAIT = int(AvgServiceState.DROP_ZONE_WAIT)


def _fresh_observation():
    return {
        "received_monotonic_s": time.monotonic(), "source_stamp_s": 100.0,
        "speed_mps": 0.0, "yaw_rate_radps": 0.0,
        "estop": False, "error_code": 0, "vehicle_state": 0,
    }


@pytest.fixture
def node():
    backend = SimpleNamespace(
        _destination_dispatch_lock=threading.RLock(),
        _startup_charging_restore_pending=True,
        _startup_recovery_pending=False,
        _admin_safety_reset_pending=False,
        _safety_reset_motion_observation=_fresh_observation(),
        _latest_platform_is_charging=True,
        _latest_service_state=STOPPED,
        _active_mission_site="",
        _charging_access_phase="inactive",
        _now_s=lambda: 100.0,
        published=[],
    )

    def publish(state, source):
        backend.published.append((int(state), source))
        backend._latest_service_state = int(state)

    backend._publish_service_state = publish
    return backend


def restore(backend, source="test"):
    UiBackendNode._restore_startup_charging_state(backend, source)


def test_docked_launch_restores_charging_once(node):
    restore(node, "backend_startup_recovery_complete")
    assert node.published == [
        (CHARGING, "backend_startup_recovery_complete:startup_charging_restored")]
    assert node._startup_charging_restore_pending is False
    # Later stop placeholders are no longer rewritten: this is startup-only (option 1).
    node._latest_service_state = STOPPED
    restore(node, "platform_status")
    assert len(node.published) == 1


def test_waits_while_startup_recovery_is_still_cancelling(node):
    node._startup_recovery_pending = True
    restore(node)
    assert node.published == []
    assert node._startup_charging_restore_pending is True
    node._startup_recovery_pending = False
    restore(node, "backend_startup_recovery_complete")
    assert [state for state, _ in node.published] == [CHARGING]


def test_waits_for_fresh_platform_feedback_then_restores(node):
    node._safety_reset_motion_observation = {}
    restore(node, "backend_startup_recovery_complete")
    assert node.published == []
    assert node._startup_charging_restore_pending is True
    node._safety_reset_motion_observation = _fresh_observation()
    restore(node, "platform_status")
    assert node.published == [(CHARGING, "platform_status:startup_charging_restored")]


def _set_pose(backend, *, x):
    """Fresh map-frame localization against a 4 m x 4 m drop_zone around the origin."""
    backend._drop_zone_polygons = [[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]]
    backend._latest_arrival_pose = SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=RosTime(sec=100)),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0)),
    )
    backend._latest_arrival_pose_time_s = 100.0
    backend.site_arrival_pose_timeout_s = 2.0


def test_uncharged_launch_inside_drop_zone_restores_standby(node):
    node._latest_platform_is_charging = False
    _set_pose(node, x=0.5)
    restore(node, "backend_startup_recovery_complete")
    assert node.published == [
        (DROP_ZONE_WAIT, "backend_startup_recovery_complete:startup_drop_zone_restored")]
    assert node._startup_charging_restore_pending is False


def test_uncharged_launch_waits_for_localization_then_restores(node):
    node._latest_platform_is_charging = False
    restore(node, "backend_startup_recovery_complete")  # no map pose yet
    assert node.published == []
    assert node._startup_charging_restore_pending is True
    _set_pose(node, x=0.5)
    restore(node, "platform_status")
    assert node.published == [(DROP_ZONE_WAIT, "platform_status:startup_drop_zone_restored")]


def test_uncharged_launch_outside_drop_zone_keeps_stop_and_retires(node):
    node._latest_platform_is_charging = False
    _set_pose(node, x=10.0)
    restore(node)
    assert node.published == []
    assert node._startup_charging_restore_pending is False
    assert node._latest_service_state == STOPPED


def test_stale_localization_is_not_treated_as_inside(node):
    node._latest_platform_is_charging = False
    _set_pose(node, x=0.5)
    node._latest_arrival_pose_time_s = 90.0  # older than the 2 s timeout
    restore(node)
    assert node.published == []
    assert node._startup_charging_restore_pending is True


def test_charger_plugged_while_waiting_retires_without_double_publish(node):
    node._latest_platform_is_charging = False
    restore(node)  # pose unknown -> still pending
    node._latest_service_state = CHARGING  # normal charging edge already published
    restore(node, "platform_status")
    assert node.published == []
    assert node._startup_charging_restore_pending is False


@pytest.mark.parametrize("overrides", [
    {"_active_mission_site": "B2"},
    {"_charging_access_phase": "departing"},
    {"_latest_service_state": int(AvgServiceState.DROP_ZONE_PARKING)},
    {"_latest_service_state": CHARGING},
    {"_admin_safety_reset_pending": True},
])
def test_never_overrides_active_or_non_stop_states(node, overrides):
    for name, value in overrides.items():
        setattr(node, name, value)
    restore(node)
    assert node.published == []


def test_backend_without_restore_flag_is_untouched():
    # Existing ROS-free fixtures that never opted in must not change behaviour.
    backend = SimpleNamespace(_destination_dispatch_lock=threading.RLock())
    backend._publish_service_state = lambda *args, **kwargs: pytest.fail("published")
    restore(backend)
