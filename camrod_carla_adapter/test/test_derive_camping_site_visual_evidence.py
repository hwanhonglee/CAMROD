"""Host-only contracts for camping-site visual evidence timestamp mapping."""
# flake8: noqa

from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "derive_camping_site_visual_evidence.py"
)
SPEC = importlib.util.spec_from_file_location(
    "derive_camping_site_visual_evidence", SCRIPT
)
assert SPEC and SPEC.loader
visual = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = visual
SPEC.loader.exec_module(visual)


CAPTURE_START = datetime(2026, 9, 3, 0, 0, 0, tzinfo=timezone.utc)
VIDEO = visual.VideoInfo(
    duration_s=140.0,
    fps=5.0,
    frame_count=700,
    width_px=3840,
    height_px=1800,
    codec_name="h264",
    start_time_s=0.0,
)


def _utc(seconds):
    return (CAPTURE_START + timedelta(seconds=seconds)).isoformat().replace(
        "+00:00", "Z"
    )


def _report(status="PASS", site_status="PASS", start=10, finish=110):
    return {
        "schema": visual.MATRIX_SCHEMA,
        "status": status,
        "sites": [
            {
                "site": "B2",
                "status": site_status,
                "started_at_utc": _utc(start),
                "finished_at_utc": _utc(finish),
                "milestones": [
                    {"at_utc": _utc(11), "event": "dispatch accepted"},
                    {
                        "at_utc": _utc(42),
                        "event": "WAITING_FOR_RETURN_REQUEST with ordered WAIT_RETURN phase",
                    },
                    {"at_utc": _utc(42), "event": "return request accepted"},
                    {
                        "at_utc": _utc(98),
                        "event": "WAITING_FOR_CHARGING at Drop Zone",
                    },
                    {"at_utc": _utc(109), "event": "PARKED and CHARGING"},
                ],
            }
        ],
    }


def test_timestamp_selection_uses_milestones_and_stays_inside_site():
    window = visual.build_site_windows(_report(), CAPTURE_START, VIDEO)[0]
    samples = visual.choose_representative_samples(
        window, CAPTURE_START, VIDEO
    )
    intervals = visual.build_gif_intervals(samples, window, VIDEO, 1.25)

    assert len(samples) == 6
    assert len({sample.frame for sample in samples}) == 6
    assert samples == tuple(sorted(samples, key=lambda sample: sample.frame))
    assert all(window.start_frame <= sample.frame <= window.finish_frame for sample in samples)
    assert any("milestone" in sample.source for sample in samples)
    selected_events = {event for sample in samples for event in sample.events}
    assert "dispatch accepted" in selected_events
    assert "PARKED and CHARGING" in selected_events
    assert all(
        window.start_offset_s <= interval.start_s < interval.end_s <= window.finish_offset_s
        for interval in intervals
    )


def test_failed_site_is_preserved_for_root_cause_visuals():
    report = _report(status="FAIL", site_status="FAIL")
    window = visual.build_site_windows(report, CAPTURE_START, VIDEO)[0]
    assert window.status == "FAIL"
    assert visual.choose_representative_samples(window, CAPTURE_START, VIDEO)


def test_non_utc_and_unfinished_matrix_are_rejected(tmp_path):
    with pytest.raises(visual.EvidenceError, match="must use UTC"):
        visual.parse_utc("2026-09-03T09:00:00+09:00", "capture")

    report_path = tmp_path / "running.json"
    report = _report()
    report["status"] = "RUNNING"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(visual.EvidenceError, match="completed PASS or FAIL"):
        visual.read_matrix(report_path)


def test_site_or_milestone_outside_capture_contract_fails_closed():
    outside = _report(start=130, finish=150)
    with pytest.raises(visual.EvidenceError, match="outside source video"):
        visual.build_site_windows(outside, CAPTURE_START, VIDEO)

    starts_before_capture = _report(start=-0.1, finish=10)
    with pytest.raises(visual.EvidenceError, match="outside source video"):
        visual.build_site_windows(starts_before_capture, CAPTURE_START, VIDEO)

    bad_milestone = _report()
    bad_milestone["sites"][0]["milestones"][0]["at_utc"] = _utc(9)
    with pytest.raises(visual.EvidenceError, match="outside the site's measured"):
        visual.build_site_windows(bad_milestone, CAPTURE_START, VIDEO)


def test_probe_requires_cfr_frame_count_and_consistent_duration():
    probe = {
        "streams": [
            {
                "codec_name": "h264",
                "width": 3840,
                "height": 1800,
                "start_time": "0.000000",
                "avg_frame_rate": "5/1",
                "r_frame_rate": "5/1",
                "nb_read_frames": "700",
            }
        ],
        "format": {"duration": "140.000000"},
    }
    assert visual.video_info_from_probe(probe) == VIDEO

    vfr = json.loads(json.dumps(probe))
    vfr["streams"][0]["avg_frame_rate"] = "499/100"
    with pytest.raises(visual.EvidenceError, match="constant-frame-rate"):
        visual.video_info_from_probe(vfr)

    inconsistent = json.loads(json.dumps(probe))
    inconsistent["streams"][0]["nb_read_frames"] = "600"
    with pytest.raises(visual.EvidenceError, match="disagree"):
        visual.video_info_from_probe(inconsistent)


def test_duplicate_json_keys_are_rejected(tmp_path):
    report = tmp_path / "duplicate.json"
    report.write_text(
        '{"schema":"%s","schema":"%s","status":"PASS","sites":[]}'
        % (visual.MATRIX_SCHEMA, visual.MATRIX_SCHEMA),
        encoding="utf-8",
    )
    with pytest.raises(visual.EvidenceError, match="duplicate JSON key"):
        visual.read_matrix(report)


@pytest.mark.skipif(
    shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None,
    reason="ffmpeg/ffprobe are not installed on this host",
)
def test_offline_ffmpeg_derivation_emits_hashed_png_gif_and_manifest(tmp_path):
    video = tmp_path / "actual_x11_capture_fixture.mp4"
    subprocess.run(
        [
            "ffmpeg", "-n", "-hide_banner", "-loglevel", "error",
            "-f", "lavfi", "-i", "testsrc=size=640x360:rate=5:duration=8",
            "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video),
        ],
        check=True,
    )
    report = _report(start=1, finish=7)
    report["sites"][0]["milestones"] = [
        {"at_utc": _utc(2), "event": "dispatch accepted"},
        {
            "at_utc": _utc(4),
            "event": "WAITING_FOR_RETURN_REQUEST with ordered WAIT_RETURN phase",
        },
        {"at_utc": _utc(6), "event": "PARKED and CHARGING"},
    ]
    matrix = tmp_path / "camping_site_matrix.json"
    matrix.write_text(json.dumps(report), encoding="utf-8")
    output = tmp_path / "visual_evidence"

    result = visual.derive(
        SimpleNamespace(
            video=video,
            capture_start_utc=_utc(0),
            matrix_report=matrix,
            output_dir=output,
            derived_width=320,
            gif_fps=5,
            clip_seconds=0.5,
        )
    )

    assert result == output
    assert (output / "B2" / "representative_contact_sheet.png").stat().st_size > 0
    assert (output / "B2" / "representative_motion.gif").stat().st_size > 0
    manifest = json.loads((output / "derivation_manifest.json").read_text())
    assert manifest["status"] == "PASS"
    assert manifest["scope"]["ai_generated_or_enhanced"] is False
    assert manifest["sites"][0]["site"] == "B2"
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, relative = line.split("  ", 1)
        assert visual._sha256(output / relative) == digest
