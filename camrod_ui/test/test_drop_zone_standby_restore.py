"""HJ_261120 - A stopped robot parked in drop_zone becomes standby again (Guest can call it)."""

from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from avg_msgs.msg import AvgServiceState  # noqa: E402
from camrod_ui import ui_backend_node as backend_module  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402

STOPPED = int(AvgServiceState.OPERATOR_STOPPED)
CHARGING = int(AvgServiceState.CHARGING)
DROP_ZONE_WAIT = int(AvgServiceState.DROP_ZONE_WAIT)


def _observation(speed=0.0, estop=False):
    return {
        "received_monotonic_s": time.monotonic(), "source_stamp_s": 100.0,
        "speed_mps": speed, "yaw_rate_radps": 0.0,
        "estop": estop, "error_code": 0, "vehicle_state": 0,
    }


@pytest.fixture
def node(monkeypatch):
    backend = SimpleNamespace(
        _destination_dispatch_lock=threading.RLock(),
        _startup_charging_restore_pending=False,
        _startup_recovery_pending=False,
        _admin_safety_reset_pending=False,
        _safety_reset_motion_observation=_observation(),
        _latest_platform_is_charging=False,
        _latest_service_state=STOPPED,
        _active_mission_site="",
        _charging_access_phase="inactive",
        _drop_zone_exit_active=False,
        _now_s=lambda: 100.0,
        inside=True,
        published=[],
    )

    def publish(state, source):
        backend.published.append((int(state), source))
        backend._latest_service_state = int(state)

    backend._publish_service_state = publish
    monkeypatch.setattr(UiBackendNode, "_station_departure_origin",
                        staticmethod(lambda self: (self.inside, "test")))
    monkeypatch.setattr(backend_module, "DROP_ZONE_STANDBY_DWELL_S", 0.0)
    return backend


def tick(backend):
    backend._safety_reset_motion_observation["received_monotonic_s"] = time.monotonic()
    UiBackendNode._restore_drop_zone_standby(backend, "platform_status")


def test_parked_stop_becomes_drop_zone_wait_after_dwell(node):
    tick(node)  # first sample only starts the dwell
    assert node.published == []
    tick(node)
    assert node.published == [(DROP_ZONE_WAIT, "platform_status:drop_zone_standby_restored")]


def test_charger_contact_restores_charging(node):
    node._latest_platform_is_charging = True
    tick(node)
    tick(node)
    assert [state for state, _ in node.published] == [CHARGING]


def test_dwell_must_elapse(node, monkeypatch):
    monkeypatch.setattr(backend_module, "DROP_ZONE_STANDBY_DWELL_S", 60.0)
    tick(node)
    tick(node)
    assert node.published == []


@pytest.mark.parametrize("change", [
    {"inside": False},
    {"inside": None},
    {"_active_mission_site": "B3"},
    {"_charging_access_phase": "departing"},
    {"_drop_zone_exit_active": True},
    {"_startup_charging_restore_pending": True},
    {"_startup_recovery_pending": True},
    {"_admin_safety_reset_pending": True},
    {"_latest_service_state": DROP_ZONE_WAIT},
])
def test_never_restores_outside_idle_parked_stop(node, change):
    for key, value in change.items():
        setattr(node, key, value)
    tick(node)
    tick(node)
    assert node.published == []


@pytest.mark.parametrize("observation", [_observation(speed=0.2), _observation(estop=True), None])
def test_requires_measured_healthy_standstill(node, observation):
    node._safety_reset_motion_observation = observation
    UiBackendNode._restore_drop_zone_standby(node, "platform_status")
    UiBackendNode._restore_drop_zone_standby(node, "platform_status")
    assert node.published == []


def test_any_break_restarts_the_dwell(node):
    tick(node)
    node.inside = False
    tick(node)
    node.inside = True
    tick(node)  # dwell restarts here
    assert node.published == []
    tick(node)
    assert [state for state, _ in node.published] == [DROP_ZONE_WAIT]
