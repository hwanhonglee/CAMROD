"""HH_261002 - Backend mission events remain passive and distinguish Recall legs."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "runtime" / "python")
)

from avg_msgs.msg import AvgServiceState  # noqa: E402
from camrod_ui.mission_recording_bridge import MissionRecordingEmitter  # noqa: E402
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


def _observer(source="robot_ui:destination"):
    events = []
    recorder = MissionRecordingEmitter(
        lambda payload: events.append(json.loads(payload)),
        now_fn=lambda: 1000.0,
        session="backend-test",
    )
    backend = SimpleNamespace(
        _mission_recording=recorder,
        _active_mission_source=source,
        _active_mission_generation=7,
        _recall_final_return_generation=0,
    )
    return backend, events


def test_delivery_records_attempt_stop_and_final_return_without_motion_commands():
    backend, events = _observer()
    UiBackendNode._record_mission_start(backend, "B7", "robot_ui:destination", 7)
    UiBackendNode._record_return_request(
        backend, "robot_ui:return", final_return=True
    )
    UiBackendNode._record_service_phase(
        backend, int(AvgServiceState.RETURN_WITH_CARGO),
        "RETURN_WITH_CARGO", "Leaving campsite",
    )
    backend._mission_recording.stop("robot_ui:stop")
    assert [event["event"] for event in events] == [
        "mission_started", "return_requested", "phase", "stop_requested",
    ]
    assert events[0]["intent"] == "delivery"
    assert events[0]["attempt_id"] == "generation:7:attempt:1"
    assert events[1]["final_return"] is True
    assert events[2]["leg_kind"] == "return"


def test_recall_first_site_departure_is_not_final_road_return():
    backend, events = _observer("guest_ui:recall")
    UiBackendNode._record_mission_start(backend, "B8", "guest_ui:recall", 7)
    UiBackendNode._record_return_request(
        backend, "guest_ui:first_loading_complete", final_return=False
    )
    UiBackendNode._record_service_phase(
        backend, int(AvgServiceState.RETURN_WITH_CARGO),
        "RETURN_WITH_CARGO", "Clearing site",
    )
    assert events[-1]["leg_kind"] == "recall"
    backend._recall_final_return_generation = 7
    UiBackendNode._record_return_request(
        backend, "robot_ui:final_loading_complete", final_return=True
    )
    UiBackendNode._record_service_phase(
        backend, int(AvgServiceState.RETURN_WITH_CARGO),
        "RETURN_WITH_CARGO", "Final road return",
    )
    assert events[-1]["leg_kind"] == "return"
    assert events[-2]["final_return"] is True


def test_recording_hooks_are_optional_and_do_not_grant_motion_authority():
    backend = SimpleNamespace()
    UiBackendNode._record_mission_start(backend, "B9", "robot_ui:destination", 2)
    UiBackendNode._record_return_request(
        backend, "robot_ui:return", final_return=True
    )
    UiBackendNode._record_service_phase(
        backend, int(AvgServiceState.MOVING_TO_SITE), "MOVING_TO_SITE"
    )


def test_service_metrics_backend_marks_recall_then_final_return_separately():
    class Metrics:
        has_active_service = False

        def __init__(self):
            self.starts = []
            self.states = []

        def start_service(self, site, **kwargs):
            self.starts.append((site, kwargs))
            self.has_active_service = True
            return True

        def observe_service_state(self, state, name, **kwargs):
            self.states.append((state, name, kwargs))

    metrics = Metrics()
    backend = SimpleNamespace(
        _service_metrics=metrics,
        _active_mission_source="guest_ui:recall",
        _active_mission_site="B8",
        _active_mission_generation=7,
        _recall_final_return_generation=0,
    )
    UiBackendNode._start_service_metrics(
        backend, "B8", "camping_site_8", "guest_ui:recall", 7
    )
    assert metrics.starts[0][1]["intent"] == "recall"
    stable_id = metrics.starts[0][1]["request_id"]
    UiBackendNode._start_service_metrics(
        backend, "B8", "camping_site_8", "guest_ui:recall", 7
    )
    assert metrics.starts[1][1]["request_id"] == stable_id
    UiBackendNode._observe_service_metrics(
        backend, int(AvgServiceState.RETURN_WITH_CARGO),
        "RETURN_WITH_CARGO", "Clearing site",
    )
    assert metrics.states[-1][2]["leg_kind"] == "recall"
    backend._recall_final_return_generation = 7
    UiBackendNode._observe_service_metrics(
        backend, int(AvgServiceState.RETURN_WITH_CARGO),
        "RETURN_WITH_CARGO", "Final road return",
    )
    assert metrics.states[-1][2]["leg_kind"] == "return"


def test_approved_standalone_return_gets_separate_metrics_record():
    class Metrics:
        has_active_service = False

        def __init__(self):
            self.starts = []

        def start_service(self, site, **kwargs):
            self.starts.append((site, kwargs))
            self.has_active_service = True
            return True

    metrics = Metrics()
    backend = SimpleNamespace(_service_metrics=metrics, _active_mission_site="")
    UiBackendNode._ensure_return_service_metrics(backend, "robot_ui:return")
    UiBackendNode._ensure_return_service_metrics(backend, "robot_ui:return")
    assert len(metrics.starts) == 1
    assert metrics.starts[0][0] == "DROP_ZONE"
    assert metrics.starts[0][1]["intent"] == "return"
