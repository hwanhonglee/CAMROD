"""HH_260915 - Temporary independent journal fixtures; no ROS or operational DB."""

from datetime import datetime
import json
import math
from pathlib import Path
import sqlite3
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))
from camrod_ui.mission_journal import MissionJournal, read_snapshot  # noqa: E402


START = datetime.fromisoformat("2026-09-15T09:00:00+09:00").timestamp()


class Clock:
    def __init__(self):
        self.now = START

    def __call__(self):
        return self.now


class Driver:
    def __init__(self, journal, clock):
        self.journal, self.clock, self.seq = journal, clock, 0

    def event(self, event="phase", mission_id="m1", **values):
        self.seq += 1
        return self.journal.observe_event({
            "schema": 1, "event": event, "mission_id": mission_id,
            "attempt_id": "a1", "site": "B1", "intent": "delivery",
            "phase": "", "state": None, "state_name": "", "reason": "",
            "producer_session": "producer", "seq": self.seq,
            "at_unix": self.clock.now, **values,
        })

    def sample(self, stamp, mode=1, vx=1.0, vy=0.0, **values):
        self.clock.now = START + stamp
        return self.journal.observe_sample({
            "sample_time_s": stamp, "received_monotonic_s": stamp,
            "vx": vx, "vy": vy, "control_mode": mode, "estop": False,
            "error_code": 0, "vehicle_state": 0, "battery_percentage": 0.7,
            "raw": {"decoded_field": 12},
            "source_quality": {"fresh": True, "valid": True}, **values,
        })


@pytest.fixture
def fixture(tmp_path):
    clock = Clock()
    journal = MissionJournal(tmp_path / "journal", robot_id="test-robot", environment="test", now_fn=clock)
    driver = Driver(journal, clock)
    yield journal, driver, clock
    journal.close()


@pytest.mark.parametrize("site", [f"B{i}" for i in range(1, 14)])
@pytest.mark.parametrize("intent", ["delivery", "recall"])
def test_each_site_round_trip_is_one_mission(fixture, site, intent):
    journal, driver, _ = fixture
    assert driver.event("mission_started", site=site, intent=intent)
    driver.sample(0)
    driver.sample(1)
    assert driver.event(state=0, state_name="ROAD_HANDOFF_READY")
    driver.sample(2)
    assert journal.snapshot()["current_mission"]["total_m"] == 2
    driver.event(state=8 if intent == "recall" else 11, phase="site")
    driver.event("return_requested", final_return=True)
    driver.sample(3, vx=-1)
    driver.event(state=10)
    driver.sample(4, vx=-1)
    driver.event(state=12, state_name="PARKED")
    snapshot = journal.snapshot()
    assert snapshot["current_mission"] is None
    assert snapshot["lifetime"]["mission_count"] == 1
    assert snapshot["lifetime"]["completed_count"] == 1
    mission = snapshot["missions"][0]
    assert (mission["site"], mission["intent"], mission["total_m"]) == (site, intent, 4)
    assert mission["autonomous_m"] == 4
    assert snapshot["sites"][0]["completed_count"] == 1


@pytest.mark.parametrize("state", [0, 12, 13])
def test_initial_station_state_and_first_recall_confirmation_cannot_complete(fixture, state):
    journal, driver, _ = fixture
    driver.event("mission_started", intent="recall")
    driver.event(state=state)
    assert journal.snapshot()["current_mission"] is not None
    driver.event("return_requested", final_return=False)
    driver.event(state=9, leg_kind="recall")
    driver.event(state=state)
    assert journal.snapshot()["lifetime"]["completed_count"] == 0
    driver.event("return_requested", final_return=True)
    driver.event(state=state)
    assert journal.snapshot()["lifetime"]["completed_count"] == 1


def test_two_mode_axes_and_transition_distance_count_once(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started", intent="recall")
    for stamp, mode in enumerate([1, 1, 0, 0, 1, 1, 77, 77]):
        driver.sample(stamp, mode)
    mission = journal.snapshot()["current_mission"]
    assert mission["autonomous_m"] == 2
    assert mission["manual_m"] == 1
    assert mission["unknown_m"] == 4
    assert mission["total_m"] == 7
    assert mission["manual_interventions"] == 1
    assert mission["intent"] == "recall"
    assert sum(mission[k] for k in ("autonomous_m", "manual_m", "unknown_m")) == mission["total_m"]


def test_pause_and_blank_standalone_return_keep_same_mission(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.sample(0)
    driver.sample(1)
    driver.event("stop_requested", reason="operator button")
    assert journal.snapshot()["current_mission"]["result"] == "paused"
    driver.event(state=0)
    assert journal.snapshot()["lifetime"]["completed_count"] == 0
    driver.event("return_requested", mission_id="", final_return=True)
    driver.sample(2, vx=-1)
    driver.event(state=12)
    result = journal.snapshot()
    assert result["lifetime"]["mission_count"] == 1
    assert result["missions"][0]["id"] == "m1"
    assert result["missions"][0]["total_m"] == 2
    assert result["missions"][0]["result"] == "completed"


def test_blank_phase_only_after_same_producer_authorized_return(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.event("stop_requested")
    assert not driver.event(mission_id="", state=12)
    driver.event("return_requested", mission_id="", final_return=True)
    assert not driver.event(mission_id="", state=12, producer_session="unrelated")
    assert driver.event(mission_id="", state=12)
    assert journal.snapshot()["lifetime"]["completed_count"] == 1
    assert not driver.event(mission_id="", state=12)
    driver.event("mission_started", mission_id="m2", attempt_id="a2")
    assert not driver.event(mission_id="", state=12)
    assert journal.snapshot()["current_mission"]["id"] == "m2"


def test_only_explicit_blank_global_stop_pauses_existing_mission(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    assert not driver.event("stop_requested", mission_id="", producer_session="restart")
    assert journal.snapshot()["current_mission"]["result"] == "active"
    assert not driver.event("stop_requested", mission_id="wrong", global_stop=True)
    assert driver.event("stop_requested", mission_id="", producer_session="restart",
                        global_stop=True, reason="backend_startup_recovery")
    current = journal.snapshot()["current_mission"]
    assert current["id"] == "m1"
    assert current["result"] == "paused"
    assert not driver.event(mission_id="", state=12, producer_session="restart")
    assert journal.snapshot()["lifetime"]["completed_count"] == 0


def test_idle_global_stop_never_creates_zero_trip_or_orphan_error(fixture):
    journal, driver, _ = fixture
    assert driver.event("stop_requested", mission_id="", global_stop=True,
                        reason="backend_startup_recovery")
    result = journal.snapshot()
    assert result["current_mission"] is None
    assert result["lifetime"]["mission_count"] == 0
    assert result["recorder"]["status"] == "READY"


def test_blank_return_producer_binding_survives_recorder_restart(tmp_path):
    clock = Clock()
    root = tmp_path / "journal"
    journal = MissionJournal(root, environment="test", now_fn=clock)
    driver = Driver(journal, clock)
    driver.event("mission_started")
    driver.event("stop_requested")
    driver.event("return_requested", mission_id="", final_return=True)
    journal.close()
    assert read_snapshot(root)["recorder"]["status"] == "CLOSED"
    reopened = MissionJournal(root, environment="test", now_fn=clock)
    second = Driver(reopened, clock)
    assert second.event(mission_id="", state=12, seq=4)
    assert reopened.snapshot()["lifetime"]["completed_count"] == 1
    reopened.close()


def test_retries_same_mission_and_daily_sequence_across_replacement(fixture):
    journal, driver, clock = fixture
    driver.event("mission_started")
    driver.event("stop_requested")
    driver.event("mission_started", attempt_id="a2")
    assert journal.snapshot()["current_mission"]["attempt_count"] == 2
    assert journal.snapshot()["current_mission"]["sequence"] == 1
    driver.event("mission_started", mission_id="m2", attempt_id="a3", site="B2")
    result = journal.snapshot()
    assert result["current_mission"]["sequence"] == 2
    assert next(m for m in result["missions"] if m["id"] == "m1")["result"] == "interrupted"
    driver.event("mission_cancelled", mission_id="m2", reason="explicit cancel")
    clock.now += 86400
    driver.event("mission_started", mission_id="m3", site="B3")
    assert journal.snapshot()["current_mission"]["sequence"] == 1
    assert journal.snapshot()["current_mission"]["date"] == "2026-09-16"


def test_duplicate_stale_mismatched_and_terminal_identity_rejected(fixture):
    journal, driver, clock = fixture
    driver.event("mission_started")
    assert not driver.event("mission_started")  # new sequence, same heartbeat
    count = journal.snapshot()["current_mission"]["event_count"]
    assert not driver.event(state=3, mission_id="another")
    assert not driver.event(state=3, seq=1)
    assert not driver.event(state=3, at_unix=clock.now - 1)
    assert journal.snapshot()["current_mission"]["event_count"] == count
    driver.event("mission_cancelled")
    assert not driver.event("return_requested", final_return=True)
    assert not driver.event("mission_started")
    assert journal.snapshot()["lifetime"]["mission_count"] == 1


def test_restart_retains_mission_attempts_and_producer_watermark_not_downtime(tmp_path):
    clock = Clock()
    root = tmp_path / "journal"
    journal = MissionJournal(root, environment="test", now_fn=clock)
    driver = Driver(journal, clock)
    driver.event("mission_started")
    driver.sample(0)
    driver.sample(1)
    driver.event("stop_requested")
    journal.close()
    reopened = MissionJournal(root, environment="test", now_fn=clock)
    second = Driver(reopened, clock)
    assert not second.event("stop_requested", seq=1)
    assert reopened.snapshot()["current_mission"]["result"] == "paused"
    assert reopened.snapshot()["current_mission"]["current_mode"] == "unknown"
    assert second.sample(100) == 0
    assert second.sample(101) == 1
    assert reopened.snapshot()["current_mission"]["total_m"] == 2
    assert reopened.snapshot()["current_mission"]["sequence"] == 1
    reopened.close()


@pytest.mark.parametrize("bad", [
    {"vx": float("nan")}, {"vx": float("inf")}, {"vx": 3.1},
    {"source_quality": {"fresh": False}}, {"source_quality": {"valid": False}},
])
def test_invalid_sample_never_bridges_missing_distance(fixture, bad):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.sample(0)
    driver.sample(1, **bad)
    assert driver.sample(2) == 0
    assert driver.sample(3) == 1
    assert journal.snapshot()["current_mission"]["total_m"] == 1
    assert journal.snapshot()["current_mission"]["incomplete"] is True


def test_duplicate_mode_and_backwards_or_large_gap_not_integrated(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.sample(1)
    assert driver.sample(1, 0) == 0
    assert journal.snapshot()["current_mission"]["manual_interventions"] == 0
    assert driver.sample(0) == 0
    assert driver.sample(100) == 0
    assert driver.sample(101) == 1
    assert journal.snapshot()["current_mission"]["total_m"] == 1


def test_reverse_and_crab_hypot_and_stationary_no_fake_angular_distance(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.sample(0, vx=-0.6, vy=0.8)
    driver.sample(1, vx=-0.6, vy=0.8)
    driver.sample(2, vx=0, vy=0, raw={"angular_z": 1})
    driver.sample(3, vx=0, vy=0, raw={"angular_z": 1})
    assert journal.snapshot()["current_mission"]["total_m"] == 1.5


def test_observed_stop_resume_reason_changes_are_deduplicated(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    gate = {"level": 2, "operating_state": "SAFETY_STOP", "message": "lanelet cost",
            "source": "safety_gate", "reason_codes": ["lanelet_cost"]}
    assert journal.observe_gate(gate)
    assert not journal.observe_gate(gate)
    driver.sample(0, vx=0)
    driver.sample(0.25, vx=0)
    assert journal.snapshot()["current_mission"]["stop_count"] == 0
    driver.sample(0.5, vx=0)
    driver.sample(1.0, vx=0)
    journal.observe_gate({**gate, "message": "operator stop", "reason_codes": ["operator"]})
    driver.sample(1.5)
    current = journal.snapshot()["current_mission"]
    assert current["stop_count"] == 1
    assert current["stop_duration_s"] == 1
    events = current["events"]
    stopped = [e for e in events if e["event"] == "stopped"]
    assert len(stopped) == 1 and stopped[0]["reason"] == "lanelet cost"
    assert stopped[0]["source"] == "safety_gate"
    archived = [json.loads(line) for relative in current["files"]["events"]
                for line in (journal.root / relative).read_text().splitlines()]
    assert next(e for e in archived if e["event"] == "stopped")["evidence"]["gate"]["source"] == "safety_gate"
    assert len([e for e in events if e["event"] == "resumed"]) == 1
    assert len([e for e in events if e["event"] == "stop_reason_changed"]) == 1


REAL_GATE_MESSAGE = (
    "state=SAFETY_HOLD reasons=obstacle charging=false "
    "charging_departure_auth=none drop_zone_auth_phase=none "
    "drop_zone_auth_age_s=1.00 battery=80.0% route_clear_s=0.00 "
    "route_releases=0/3 route_episode_progress_m=0.000 "
    "recovery_motion_observed=false recovery_candidate=none recovery_reason=blocked"
)


def test_real_gate_numeric_churn_keeps_latest_evidence_without_spam(fixture):
    # HH_260915 - Exercise the actual gate's key/value shape, not a static mock
    # reason that would hide age-only events evicting the UI's 50-event tail.
    journal, driver, _ = fixture
    driver.event("mission_started")
    gate = {"level": 1, "operating_state": "SAFETY_HOLD",
            "source": "/control/cmd_vel_safety_gate/status", "message": REAL_GATE_MESSAGE}
    assert journal.observe_gate(gate)
    latest = REAL_GATE_MESSAGE.replace("1.00", "1.50").replace("80.0%", "79.9%")
    latest = latest.replace("route_clear_s=0.00", "route_clear_s=0.50")
    latest = latest.replace("route_episode_progress_m=0.000", "route_episode_progress_m=0.123")
    assert not journal.observe_gate({**gate, "message": latest})
    driver.sample(0, vx=0)
    driver.sample(0.5, vx=0)
    assert not journal.observe_gate({**gate, "message": latest.replace("1.50", "2.00")})
    changed = latest.replace("reasons=obstacle", "reasons=estop")
    assert journal.observe_gate({**gate, "message": changed})
    current = journal.snapshot()["current_mission"]
    events = current["events"]
    assert len([event for event in events if event["event"] == "gate_changed"]) == 2
    assert len([event for event in events if event["event"] == "stop_reason_changed"]) == 1
    archived = [json.loads(line) for relative in current["files"]["events"]
                for line in (journal.root / relative).read_text().splitlines()]
    stopped = next(event for event in archived if event["event"] == "stopped")
    assert stopped["reason"] == latest
    assert stopped["evidence"]["gate"]["message"] == latest
    assert next(event for event in archived if event["event"] == "stop_reason_changed")["reason"] == changed


@pytest.mark.parametrize("before,after", [
    ("reasons=obstacle", "reasons=estop"),
    ("charging=false", "charging=true"),
    ("charging_departure_auth=none", "charging_departure_auth=mission_key"),
    ("drop_zone_auth_phase=none", "drop_zone_auth_phase=departing"),
    ("route_releases=0/3", "route_releases=1/3"),
    ("recovery_motion_observed=false", "recovery_motion_observed=true"),
    ("recovery_candidate=none", "recovery_candidate=backtrack"),
    ("recovery_reason=blocked", "recovery_reason=clear"),
    ("battery=80.0%", "battery=unknown"),
    ("drop_zone_auth_age_s=1.00", "drop_zone_auth_age_s=nan"),
    ("recovery_reason=blocked", "recovery_reason=blocked error_code=12"),
    ("recovery_reason=blocked", "recovery_reason=blocked route_hit_world=1.0,2.0"),
])
def test_gate_semantic_and_unknown_details_are_never_normalized_away(fixture, before, after):
    journal, driver, _ = fixture
    driver.event("mission_started")
    gate = {"level": 1, "operating_state": "SAFETY_HOLD", "source": "gate",
            "message": REAL_GATE_MESSAGE}
    assert journal.observe_gate(gate)
    assert journal.observe_gate({**gate, "message": REAL_GATE_MESSAGE.replace(before, after)})


@pytest.mark.parametrize("field,value", [
    ("level", 2), ("operating_state", "FAULT_HOLD"),
    ("source", "other_gate"), ("reason_codes", ["estop"]),
])
def test_gate_nonmessage_reason_identity_remains_observable(fixture, field, value):
    journal, driver, _ = fixture
    driver.event("mission_started")
    gate = {"level": 1, "operating_state": "SAFETY_HOLD", "source": "gate",
            "message": REAL_GATE_MESSAGE}
    assert journal.observe_gate(gate)
    assert journal.observe_gate({**gate, field: value})


def test_idle_and_outside_manual_distances_not_mislabelled_as_missions(fixture):
    journal, driver, _ = fixture
    driver.sample(0, 0, vx=0)
    driver.sample(1, 0, vx=0)
    assert journal.snapshot()["outside_missions"]["session_count"] == 0
    driver.sample(2, 0)
    driver.sample(3, 0)
    result = journal.snapshot()
    assert result["outside_missions"]["manual_m"] == 1.5
    assert result["outside_missions"]["session_count"] == 1
    assert result["lifetime"]["total_m"] == 1.5
    assert result["lifetime"]["mission_count"] == 0
    driver.event("mission_started")
    assert driver.sample(4) == 0
    driver.sample(5)
    assert journal.snapshot()["lifetime"]["total_m"] == 2.5


def test_jsonl_rotation_quota_preserves_old_files_and_marks_incomplete(tmp_path):
    clock = Clock()
    root = tmp_path / "journal"
    journal = MissionJournal(root, environment="test", now_fn=clock, rotation_bytes=1024, quota_bytes=4096)
    driver = Driver(journal, clock)
    driver.event("mission_started")
    for stamp in range(30):
        driver.sample(stamp)
    result = journal.snapshot()
    assert result["recorder"]["status"] == "DEGRADED"
    assert "quota" in result["recorder"]["error"]
    paths = list(root.rglob("*.jsonl"))
    assert len(paths) > 1
    assert all(path.stat().st_size <= 1024 for path in paths)
    before = {str(path): path.read_bytes() for path in paths}
    assert sum(map(len, before.values())) <= 4096
    journal.observe_raw_frame({"data": "a" * 2000})
    assert all(Path(path).read_bytes() == data for path, data in before.items())
    assert journal.flush_snapshot()
    assert read_snapshot(root)["current_mission"]["incomplete"] is True
    journal.close()


def test_writer_exclusion_profile_validation_and_no_legacy_store_access(tmp_path):
    root = tmp_path / "journal"
    root.mkdir()
    old = root / "service_metrics.sqlite3"
    old.write_bytes(b"legacy sentinel, deliberately not a SQLite file")
    journal = MissionJournal(root, robot_id="one", environment="test")
    with pytest.raises((BlockingIOError, OSError)):
        MissionJournal(root, robot_id="one", environment="test")
    journal.close()
    with pytest.raises(ValueError, match="robot/environment"):
        MissionJournal(root, robot_id="two", environment="test")
    assert old.read_bytes() == b"legacy sentinel, deliberately not a SQLite file"
    # Failed profile check released the writer lock.
    second = MissionJournal(root, robot_id="one", environment="test")
    second.close()


def test_name_safe_paths_raw_frame_and_atomic_snapshot_health(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started", site="../../outside/한글", mission_id="../../not-a-path")
    driver.sample(0)
    journal.set_raw_can_status({"state": "listening", "interface": "test-only"})
    assert journal.observe_raw_frame({"can_id": 0x201, "data": "001122", "direction": "bus_observed"})
    journal.set_recorder_status({"platform_stale": False, "queue_dropped_total": 3, "robot_id": "wrong"})
    assert journal.flush_snapshot()
    snapshot = read_snapshot(journal.root, robot_id="test-robot", environment="test")
    assert snapshot["recorder"]["queue_dropped_total"] == 3
    assert snapshot["recorder"]["raw_can_status"] == "listening"
    assert snapshot["recorder"]["robot_id"] == "test-robot"
    assert snapshot["current_mission"]["files"]["raw_can"]
    for paths in snapshot["current_mission"]["files"].values():
        assert all(".." not in Path(path).parts and not Path(path).is_absolute() for path in paths)
    assert not list(journal.root.glob(".snapshot-*.tmp"))


def test_standalone_return_without_prior_mission_and_closed_rows_survive(fixture):
    journal, driver, _ = fixture
    assert driver.event("return_requested", mission_id="", intent="return", site="DROP_ZONE", final_return=True)
    identity = journal.snapshot()["current_mission"]["id"]
    driver.sample(0)
    driver.sample(1)
    driver.event(mission_id=identity, state=0)
    assert journal.snapshot()["missions"][0]["intent"] == "return"
    assert journal.snapshot()["lifetime"]["completed_count"] == 1
    assert journal.flush_snapshot()
    with sqlite3.connect(journal.root / "mission_journal.sqlite3") as db:
        assert db.execute("SELECT COUNT(*) FROM missions").fetchone()[0] == 1


def test_source_freshness_gap_is_recorded_without_claiming_stop_duration(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    driver.sample(0, vx=0)
    driver.sample(1, vx=0)
    journal.mark_incomplete("platform_status_stale")
    driver.sample(101, vx=0)
    assert journal.snapshot()["current_mission"]["stop_duration_s"] == 1
    assert journal.snapshot()["recorder"]["status"] == "DEGRADED"


def test_failed_atomic_export_preserves_previous_snapshot(fixture, monkeypatch):
    journal, driver, _ = fixture
    driver.event("mission_started")
    assert journal.flush_snapshot()
    original = (journal.root / "snapshot.json").read_bytes()
    driver.sample(0)
    driver.sample(1)
    import camrod_ui.mission_journal as module
    replace = module.os.replace

    def failed_replace(*args):
        raise OSError("synthetic export failure")

    monkeypatch.setattr(module.os, "replace", failed_replace)
    assert not journal.flush_snapshot()
    assert (journal.root / "snapshot.json").read_bytes() == original
    assert journal.snapshot()["recorder"]["status"] == "DEGRADED"
    assert not list(journal.root.glob(".snapshot-*.tmp"))
    monkeypatch.setattr(module.os, "replace", replace)
    assert journal.flush_snapshot()
    assert read_snapshot(journal.root)["current_mission"]["total_m"] == 1


def test_lifecycle_gap_and_orphan_mark_incomplete_without_adopting(fixture):
    journal, driver, _ = fixture
    assert not driver.event(state=3, seq=3, mission_id="lost-start")
    assert journal.snapshot()["current_mission"] is None
    assert "orphan_lifecycle_event" in journal.snapshot()["recorder"]["error"]
    driver.event("mission_started", seq=4)
    driver.sample(0)
    driver.event(phase="site", seq=6)
    assert driver.sample(1) == 0
    assert journal.snapshot()["current_mission"]["incomplete"] is True
    assert "lifecycle_sequence_gap" in journal.snapshot()["recorder"]["error"]


def test_event_export_tail_is_explicit_and_full_jsonl_preserved(fixture):
    journal, driver, _ = fixture
    driver.event("mission_started")
    for index in range(60):
        journal.observe_gate({"source": "test", "message": f"gate event {index}"})
    current = journal.snapshot()["current_mission"]
    assert current["event_count"] == 62
    assert current["events_exported_count"] == 50
    assert current["events_truncated"] is True
    archived = [line for relative in current["files"]["events"]
                for line in (journal.root / relative).read_text().splitlines()]
    assert len(archived) == 62


@pytest.mark.parametrize("kwargs", [
    {"environment": "guessed"}, {"minimum_speed_mps": -1},
    {"maximum_speed_mps": math.nan}, {"maximum_sample_gap_s": 0},
    {"stop_dwell_s": -1}, {"rotation_bytes": 1},
    {"rotation_bytes": 200, "quota_bytes": 100},
])
def test_invalid_configuration_is_rejected(tmp_path, kwargs):
    with pytest.raises(ValueError):
        MissionJournal(tmp_path / "journal", **kwargs)
