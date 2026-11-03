"""HH_261103 - Check receipt latency and truthful quantitative reports without importing CARLA/ROS or moving an actor."""
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys

import pytest


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name+".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


camera = module("sensor_camera_watch")
compare = module("sensor_compare")


def test_backlog_is_drained_without_spinning_once_per_sample_and_partial_line_waits():
    source = io.StringIO('{"event":"sample"}\n'*400+'{"event":"measured_placement","case":"front_10cm"}\n'+
        '{"event":"complete"')
    assert camera.drain_receipts(source) == [{"event": "measured_placement", "case": "front_10cm"}]
    assert source.read() == '{"event":"complete"'


@pytest.mark.parametrize("stamp,received,expected", [(100.4, 4.9, "waiting"), (100.8, 4.9, "capture"),
    (99.9, 4.9, "waiting"), (100.8, 4, "waiting"), (float("nan"), 4.9, "waiting")])
def test_only_settled_source_stamped_and_receipt_fresh_jpeg_can_capture(stamp, received, expected):
    event = {"utc": "1970-01-01T00:01:40+00:00"}
    image = {"stamp_s": stamp, "received": received, "data": b"\xff\xd8real-camera-bytes"}
    assert camera.capture_status(event, image, 101, 5) == expected


def test_expired_or_future_case_never_gets_a_later_unrelated_photo():
    event = {"utc": "1970-01-01T00:01:40+00:00"}
    assert camera.capture_status(event, {}, 104, 5).startswith("unavailable:")
    assert camera.capture_status(event, {}, 99, 5).startswith("unavailable:")


def test_non_jpeg_or_future_source_stamp_is_not_saved():
    event = {"utc": "1970-01-01T00:01:40+00:00"}
    image = {"stamp_s": 100.8, "received": 4.9, "data": b"not-a-camera-jpeg"}
    assert camera.capture_status(event, image, 101, 5) == "waiting"
    image.update(stamp_s=101.1, data=b"\xff\xd8real-camera-bytes")
    assert camera.capture_status(event, image, 101, 5) == "waiting"


def test_default_camera_is_plan_only_without_ros():
    result = subprocess.run([sys.executable, camera.__file__], capture_output=True, text=True, check=True)
    assert json.loads(result.stdout)["mode"] == "PLAN_ONLY_NO_CONNECTION"
    assert "create_publisher(" not in Path(camera.__file__).read_text()


def test_counts_panel_reuses_original_reducer_and_preserves_unavailable():
    result = compare.summarize(compare.LEGACY/"nearfield_actual_02.jsonl")
    row = compare.comparison_row("actual02", result["case_results"][0])
    assert row[1] == "10.0000"
    assert row[-2] == "24 / 0 / 1"
    assert result["gate_operating_states"] == ["FAULT_HOLD"]
    assert len(result["post_removal_transition_hits"]) == 1
    assert result["source_sha256"] == "56a4d526d8d391a766720276188844edffceeb627cbdc981ec27d3e3d523b868"


def test_comparison_rejects_same_trace_as_two_different_runs(tmp_path):
    trace = str(compare.LEGACY/"nearfield_actual_02.jsonl")
    result = subprocess.run([sys.executable, compare.__file__, trace, trace,
        "--output-prefix", str(tmp_path/"comparison")], capture_output=True, text=True)
    assert result.returncode != 0
    assert "distinct actual trace" in result.stderr
    assert not list(tmp_path.iterdir())


def test_photo_sheet_loads_seven_actual_originals_and_rejects_sha_change(tmp_path):
    sheet = module("sensor_photo_sheet")
    source = Path(__file__).with_name("nearfield_actual_03_camera.jsonl")
    photos = sheet.load_photos(source)
    assert len(photos) == 7
    assert photos[3]["case"] == "front_30cm"
    assert photos[3]["capture_utc"].startswith("2026-11-03 08:17:")
    assert "30.0003 cm" in photos[3]["label"]
    rows = [json.loads(line) for line in source.read_text().splitlines()]
    next(row for row in rows if row.get("image"))["image_sha256"] = "0"*64
    altered = tmp_path/"invalid_manifest.jsonl"
    altered.write_text("\n".join(map(json.dumps, rows)))
    with pytest.raises(ValueError, match="SHA mismatch"):
        sheet.load_photos(altered)
