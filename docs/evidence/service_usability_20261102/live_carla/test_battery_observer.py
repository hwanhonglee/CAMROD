"""HH_261102 - Keep a passive evidence helper bounded without requiring ROS or a vehicle."""

import ast
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SOURCE = Path(__file__).with_name("battery_observer.py")
spec = importlib.util.spec_from_file_location("battery_observer", SOURCE)
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


def test_writer_never_overwrites_and_stops_before_limit(tmp_path):
    path = tmp_path / "evidence.jsonl"
    writer = observer.JsonlWriter(path, 64)
    writer.write({"actual": 0.35, "invalid": float("nan")})
    assert json.loads(path.read_text()) == {"actual": 0.35, "invalid": None}
    with pytest.raises(observer.EvidenceFull):
        writer.write({"too_large": "x" * 80})
    writer.stream.close()
    assert len(path.read_bytes()) <= 64
    with pytest.raises(FileExistsError):
        observer.JsonlWriter(path, 64)


def test_compact_apis_exclude_geometry_but_keep_mission_and_freshness():
    result = observer.api_summary("/api/driving", {
        "mission": {"generation": 4, "active": False}, "motion": {"age_s": 3},
        "base_map": {"polylines": [[1] * 10000]}, "perception": {"points": [1] * 10000}})
    assert result == {"mission": {"generation": 4, "active": False}, "motion": {"age_s": 3}}
    assert observer.api_summary("/api/charging-access", {"platform_feedback_fresh": False}) == {
        "platform_feedback_fresh": False}


def test_raw_fraction_stamps_and_invalid_values_are_not_promoted_to_readiness():
    result = observer.compact_message("/platform/status", {
        "header": {"stamp": {"sec": 2, "nanosec": 5}}, "battery_percentage": 0.349999994,
        "is_charging": True, "battery_state_available": False, "unused": [1] * 10000})
    assert result["battery_percentage"] == 0.349999994
    assert result["battery_state_available"] is False
    assert observer.source_stamp(result) == {"sec": 2, "nanosec": 5}
    assert "unused" not in result
    assert len(observer.bounded([1] * 100)) == 64


def test_http_is_get_only_and_no_publisher_or_service_client_exists():
    tree = ast.parse(SOURCE.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert not any(isinstance(node.func, ast.Attribute) and node.func.attr in {
        "create_publisher", "create_client", "publish", "async_send_request"} for node in calls)
    request = next(node for node in calls if isinstance(node.func, ast.Name) and node.func.id == "Request")
    assert next(arg.value.value for arg in request.keywords if arg.arg == "method") == "GET"


@pytest.mark.parametrize("charging,error", [(True, ""), (False, "actual controller failure")])
def test_battery_watch_stops_on_actual_charging_or_failure_without_controls(tmp_path, monkeypatch, charging, error):
    # HH_261102 - A live window records either outcome once; it never retries or manufactures success.
    monkeypatch.setitem(sys.modules, "battery_observer", observer)
    watch_spec = importlib.util.spec_from_file_location("battery_watch", SOURCE.with_name("battery_watch.py"))
    watch = importlib.util.module_from_spec(watch_spec)
    watch_spec.loader.exec_module(watch)
    path = tmp_path / "window.jsonl"
    monkeypatch.setattr(sys, "argv", ["battery_watch.py", "--output", str(path)])
    monkeypatch.setattr(watch, "fetch_api", lambda _base, route: {"ok": True, "data": {
        "platform_is_charging": charging, "mission_execution_error": error,
        "service_state_name": "CHARGING" if charging else "DROP_ZONE_PARKING",
        "module_states": [{"name": "parking", "message": "actual status"}],
        "diagnostics": ["not copied"],
    }} if route == "/ui/state" else {"ok": True, "data": {"motion": {"speed_mps": 0}}})
    watch.main()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(rows) == 1
    state = rows[0]["state"]["data"]
    assert state["platform_is_charging"] is charging
    assert state["mission_execution_error"] == error
    assert "diagnostics" not in state
    assert state["controllers"] == [{"name": "parking", "message": "actual status"}]
