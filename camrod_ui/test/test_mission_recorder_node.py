"""HH_260915 - Isolated recorder normalization, queue, and ROS-contract tests."""
import copy
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
from camrod_ui import mission_recorder_node as recorder  # noqa: E402


def stamp(seconds=100, nanos=0):
    return NS(sec=seconds, nanosec=nanos)


def header(seconds=100):
    return NS(stamp=stamp(seconds), frame_id="robot_center_link")


def gate():
    return NS(stamp=stamp(), module_name="cmd_vel_safety_gate", level=1,
              operating_state="SAFETY_HOLD", message="state=SAFETY_HOLD reasons=estop,cost_stop_latched",
              missing_nodes=[], missing_topics=[], missing_lifecycle_nodes=[])


def platform():
    twist = NS(linear=NS(x=0.2, y=-0.1, z=0.0), angular=NS(x=0.0, y=0.0, z=0.3))
    return NS(header=header(), stamp=stamp(0), velocity=NS(header=header(), twist=twist),
              odometry=NS(header=header()), wheel=NS(header=header()), state=gate(),
              control_mode=1, estop=False, error_code=0, vehicle_state=0, motion_mode=1,
              is_charging=False, battery_state_available=True, battery_percentage=0.35,
              battery_voltage=26.0, battery_current_a=-1.0, battery_temperature_c=20.0,
              battery_power_supply_status=2, motor_rpm=[1.0, 2.0], motor_speed=[0.2], motor_angle=[1.57])


def normalize(message=None, **kwargs):
    return recorder.normalize_platform(message or platform(), received_unix_s=1000.0,
        received_ros_s=100.1, received_monotonic_s=10.0, **kwargs)


def test_platform_preserves_observed_mode_values_and_source_stamps():
    sample = normalize()
    assert (sample["vx"], sample["vy"], sample["control_mode"]) == (0.2, -0.1, 1)
    assert sample["sample_time_s"] == 100
    assert sample["decoded"]["aggregate_stamp"]["seconds"] is None
    assert sample["source_quality"]["timestamp_basis"] == "velocity.header.stamp"
    assert sample["source_quality"]["fresh"] is True
    assert "not_observable" in sample["source_quality"]["can_frame_freshness"]
    assert sample["decoded"]["motor_angle"] == [1.57]
    assert sample["battery_percentage"] == 0.35


def test_zero_velocity_does_not_reclassify_mode_as_manual():
    message = platform()
    message.velocity.twist.linear.x = message.velocity.twist.linear.y = 0
    assert normalize(message)["control_mode"] == 1
    message.control_mode = 0
    assert normalize(message)["control_mode"] == 0
    message.control_mode = 255
    assert normalize(message)["control_mode"] == 255


def test_stamp_fallback_keeps_explicit_quality_not_fabricated_source_stamp():
    message = platform()
    message.velocity.header.stamp = stamp(0)
    assert normalize(message)["source_quality"]["timestamp_basis"] == "header.stamp_fallback"
    message.header.stamp = stamp(0)
    result = normalize(message)
    assert result["sample_time_s"] == 100.1
    assert result["decoded"]["velocity_stamp"]["seconds"] is None
    assert result["source_quality"]["timestamp_basis"] == "receipt_ros_time_fallback"


@pytest.mark.parametrize("seconds", [90, 101])
def test_stale_or_future_source_stamp_cannot_add_distance(seconds):
    message = platform()
    message.velocity.header.stamp = stamp(seconds)
    assert normalize(message)["source_quality"]["fresh"] is False


def test_nonfinite_raw_decoded_values_are_json_safe_and_velocity_invalid():
    message = platform()
    message.velocity.twist.linear.x = float("nan")
    message.motor_angle = [float("inf")]
    message.battery_state_available = False
    result = normalize(message)
    assert result["source_quality"]["valid"] is False
    assert result["vx"] is None and result["decoded"]["motor_angle"] == [None]
    assert result["battery_percentage"] is None
    json.dumps(result, allow_nan=False)


def test_gate_preserves_message_reasons_and_source_timestamp():
    result = recorder.normalize_gate(gate(), received_unix_s=1000, received_ros_s=100, received_monotonic_s=10)
    assert result["message"] == "state=SAFETY_HOLD reasons=estop,cost_stop_latched"
    assert result["stamp"]["seconds"] == 100
    assert result["received_unix_s"] == 1000
    assert "mission_id" not in result


@pytest.mark.parametrize("requested,sim,effective", [
    ("real", False, "real"), ("real", True, "simulation"),
    ("simulation", False, "simulation"), ("test", False, "test"),
    ("test", True, "test"), ("other", False, "unknown"),
])
def test_environment_prevents_sim_clock_silently_labelled_real(requested, sim, effective):
    result = recorder.recording_environment(requested, sim)
    assert result["effective_environment"] == effective
    assert result["requested_environment"] == requested


def test_default_root_uses_xdg_without_touching_it(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "new-state"))
    assert recorder.default_mission_records_root() == tmp_path / "new-state" / "camrod" / "mission_records"
    assert not (tmp_path / "new-state").exists()


class FakeJournal:
    def __init__(self):
        self.calls, self.health, self.current = [], {}, None

    def _call(self, kind, payload=None):
        self.calls.append((kind, payload, threading.get_ident()))

    def observe_event(self, payload, **kwargs):
        self.current = {"id": payload.get("mission_id")}
        self._call("event", payload)

    def observe_sample(self, payload, **kwargs):
        self._call("sample", payload)

    def observe_gate(self, payload, **kwargs):
        self._call("gate", payload)

    def observe_raw_frame(self, payload, **kwargs):
        self._call("raw", payload)

    def set_raw_can_status(self, payload):
        self._call("raw_status", payload)

    def set_recorder_status(self, payload):
        self.health.update(payload)

    def mark_incomplete(self, reason):
        self._call("gap", reason)

    def snapshot(self):
        return {"recorder": dict(self.health), "current_mission": self.current}

    def flush_snapshot(self):
        self._call("flush")

    def close(self):
        self._call("close")


def test_callbacks_do_not_write_and_one_worker_owns_all_journal_calls():
    journal = FakeJournal()
    worker = recorder.RecorderWorker(lambda: journal, queue_capacity=5, monotonic_fn=lambda: 10.0)
    worker.note_platform_receipt(10.0, valid=True)
    worker.submit("sample", normalize())
    worker.submit("gate", {"message": "fixture"})
    assert not journal.calls
    caller = threading.get_ident()
    worker.start()
    assert worker.ready.wait(1)
    assert worker.close()
    assert {thread for _, _, thread in journal.calls} != {caller}
    assert len({thread for _, _, thread in journal.calls}) == 1
    assert [kind for kind, _, _ in journal.calls if kind in {"sample", "gate"}] == ["sample", "gate"]


def test_bounded_queue_overflow_resets_anchor_before_any_sample():
    journal = FakeJournal()
    worker = recorder.RecorderWorker(lambda: journal, queue_capacity=1, monotonic_fn=lambda: 10.0)
    worker.note_platform_receipt(10.0, valid=True)
    assert worker.submit("sample", normalize()) is True
    assert worker.submit("event", {"mission_id": "dropped"}) is False
    worker.start()
    assert worker.ready.wait(1) and worker.close()
    assert journal.calls[0][:2] == ("gap", "recorder_queue_overflow")
    assert worker.cached_snapshot()["recorder"]["queue_dropped_total"] == 1
    assert worker.cached_snapshot()["recorder"]["capture_incomplete"] is True


def test_missing_startup_status_reported_without_poisoning_empty_history():
    journal = FakeJournal()
    worker = recorder.RecorderWorker(lambda: journal)
    worker._journal = journal
    assert worker._health()["platform_stale"] is True
    assert not journal.calls
    worker._process("event", {"mission_id": "fixture"}, 1000)
    worker._health()
    assert ("gap", "platform_status_stale_or_unavailable") in [entry[:2] for entry in journal.calls]


def test_actual_receipt_gap_and_queued_old_samples_pause_integration():
    journal = FakeJournal()
    clock = [10.0]
    worker = recorder.RecorderWorker(lambda: journal, monotonic_fn=lambda: clock[0])
    worker._journal = journal
    worker.note_platform_receipt(10.0, valid=True)
    assert worker._health()["platform_stale"] is False
    clock[0] = 12.0
    assert worker._health()["platform_stale"] is True
    worker._process("sample", normalize(), 1000)
    sample = [data for kind, data, _ in journal.calls if kind == "sample"][0]
    assert sample["source_quality"]["fresh"] is False
    assert sample["source_quality"]["queue_delay_stale"] is True


def test_raw_capture_failure_is_not_silent_success():
    journal = FakeJournal()
    worker = recorder.RecorderWorker(lambda: journal)
    worker._journal = journal
    worker._process("raw_status", {"state": "not_available", "error": "fixture"}, 1000)
    assert ("gap", "raw_can_not_available:fixture") in [entry[:2] for entry in journal.calls]


def test_journal_initialization_failure_visible_without_starting_runtime():
    def fail():
        raise OSError("fixture quota/path failure")
    worker = recorder.RecorderWorker(fail)
    worker.start()
    assert worker.ready.wait(1) and worker.close()
    status = worker.cached_snapshot()["recorder"]
    assert status["status"] == "error" and "fixture" in status["last_worker_error"]


def test_actual_ros_factory_contract_using_stub_node_no_ros_graph(tmp_path, monkeypatch):
    published, subscriptions, workers = [], [], []
    params = {"storage_root": str(tmp_path / "records"), "use_sim_time": True, "environment": "real"}
    class FakeWorker:
        def __init__(self, factory, **kwargs):
            self.jobs = []
            workers.append(self)
        def start(self): pass
        def submit(self, *args, **kwargs): self.jobs.append((args, kwargs)); return True
        def note_platform_receipt(self, *args, **kwargs): pass
        def cached_snapshot(self): return {"recorder": {"status": "fixture"}}
        def close(self): return True
    class FakeNode:
        def __init__(self, name): self.name = name
        def declare_parameter(self, name, default): return NS(value=params.get(name, default))
        def get_parameter(self, name): return NS(value=params[name])
        def has_parameter(self, name): return name in params
        def create_publisher(self, kind, topic, qos):
            published.append((topic, qos)); return NS(publish=lambda message: None)
        def create_subscription(self, kind, topic, callback, qos):
            subscriptions.append((topic, qos, callback)); return callback
        def create_timer(self, period, callback): return callback
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=100100000000))
        def get_logger(self): return NS(info=lambda message: None, warning=lambda message: None, error=lambda message: None)
        def destroy_node(self): return True
    class String:
        def __init__(self, data=""): self.data = data
    monkeypatch.setitem(sys.modules, "rclpy.node", NS(Node=FakeNode))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(QoSProfile=lambda **kwargs: NS(**kwargs),
        ReliabilityPolicy=NS(RELIABLE="reliable", BEST_EFFORT="best_effort"),
        DurabilityPolicy=NS(VOLATILE="volatile", TRANSIENT_LOCAL="transient")))
    monkeypatch.setitem(sys.modules, "avg_msgs.msg", NS(AvgPlatformStatus=object, ModuleState=object))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=String))
    monkeypatch.setattr(recorder, "RecorderWorker", FakeWorker)
    node = recorder.create_ros_node()
    assert [topic for topic, _ in published] == ["/ui/mission_recording/status"]
    qoses = {topic: qos for topic, qos, _ in subscriptions}
    assert qoses["/ui/mission_recording/events"].depth == 100
    assert qoses["/ui/mission_recording/events"].durability == "transient"
    assert qoses["/platform/status"].reliability == "best_effort"
    assert qoses["/control/cmd_vel_safety_gate/status"].durability == "transient"
    node._platform(platform())
    node._gate(gate())
    node._event(String('{"schema":1,"event":"fixture"}'))
    node._event(String('{"schema": NaN}'))
    node._event(String('[]'))
    metadata = [args[1] for args, _ in workers[0].jobs if args[0] == "metadata"][0]
    assert metadata["effective_environment"] == "simulation"
    assert [args[1] for args, _ in workers[0].jobs if args[0] == "sample"][0]["source_topic"] == "/platform/status"
    assert [args[1] for args, _ in workers[0].jobs if args[0] == "gate"][0]["source"] == "/control/cmd_vel_safety_gate/status"
    assert len([args for args, _ in workers[0].jobs if args[0] == "gap"]) == 2
    assert not (tmp_path / "records").exists()  # Stub never opens any data store.
    node.destroy_node()


def test_worker_with_actual_journal_writes_only_fixture_directory(tmp_path):
    from camrod_ui.mission_journal import MissionJournal
    root = tmp_path / "fixture_records"
    worker = recorder.RecorderWorker(
        lambda: MissionJournal(root, robot_id="fixture", environment="test", now_fn=lambda: 1001.0),
        monotonic_fn=lambda: 10.0, now_fn=lambda: 1001.0)
    worker.note_platform_receipt(10.0, valid=True)
    worker.submit("metadata", recorder.recording_environment("test", False))
    worker.submit("raw_status", {"state": "disabled", "configured": False, "interface": ""})
    worker.submit("event", {"schema": 1, "event": "mission_started", "mission_id": "fixture-mission",
        "producer_session": "fixture-producer", "seq": 1, "at_unix": 1000.0,
        "site": "B7", "intent": "delivery", "source": "fixture"})
    first, second = normalize(), normalize()
    first.update(sample_time_s=100.0, vx=0.2, vy=0.0)
    second.update(sample_time_s=100.5, vx=0.2, vy=0.0)
    worker.submit("sample", first)
    worker.submit("sample", second)
    worker.start()
    assert worker.ready.wait(1) and worker.close()
    stored = json.loads((root / "snapshot.json").read_text())
    assert stored["recorder"]["environment"] == "test"
    assert stored["recorder"]["effective_environment"] == "test"
    assert stored["recorder"]["raw_can_status"] == "disabled"
    assert stored["recorder"]["platform_stale"] is False
    mission = stored["current_mission"] or stored["missions"][0]
    assert mission["site"] == "B7"
    assert mission["total_m"] == pytest.approx(0.1)
    assert mission["autonomous_m"] == pytest.approx(0.1)
