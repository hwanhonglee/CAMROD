"""Behavioral tests for the bounded CARLA visual-stream preflight."""

import importlib.util
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import time

import pytest
from sensor_msgs.msg import PointCloud2, PointField


REPO_ROOT = Path(__file__).resolve().parents[2]
PROBE_PATH = REPO_ROOT / "scripts" / "virtual_carla" / "check_carla_sensor_streams.py"


def _load_probe_module():
    spec = importlib.util.spec_from_file_location(
        "carla_sensor_stream_preflight_test", PROBE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bare_probe(module, rate_hz: float):
    probe = object.__new__(module.SensorStreamProbe)
    probe.min_rate_hz = 8.0
    probe.min_observation_seconds = 1.0
    probe.max_sample_age_seconds = 0.5
    probe.required = {"stream": "/fixture/stream"}
    probe.received = {"stream": {"topic": "/fixture/stream"}}
    probe.invalid = {}
    now = time.monotonic()
    probe.sample_times = {
        "stream": [now - (10 - index) / rate_hz for index in range(11)]
    }
    probe.payload_pairs_ready = lambda: True
    return probe


def test_probe_requires_sustained_rate_instead_of_one_payload():
    module = _load_probe_module()

    assert _bare_probe(module, 10.0).complete()
    assert not _bare_probe(module, 5.0).complete()

    one_shot = _bare_probe(module, 10.0)
    one_shot.sample_times["stream"] = [1.0]
    assert not one_shot.complete()

    stale = _bare_probe(module, 10.0)
    stale.sample_times["stream"] = [
        value - 5.0 for value in stale.sample_times["stream"]
    ]
    assert stale.stream_metrics("stream")["rate_ready"]
    assert not stale.stream_metrics("stream")["fresh_ready"]
    assert not stale.complete()


def test_invalid_latest_payload_revokes_an_earlier_valid_stream():
    module = _load_probe_module()
    probe = _bare_probe(module, 10.0)

    probe._reject("stream", "empty payload")

    assert "stream" not in probe.received
    assert probe.invalid == {"stream": "empty payload"}
    assert probe.sample_times["stream"] == []
    assert not probe.complete()


def _azimuth_cloud(angles_deg):
    message = PointCloud2()
    message.height = 1
    message.width = len(angles_deg)
    message.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
    ]
    message.point_step = 16
    message.row_step = message.width * message.point_step
    message.data = b"".join(
        struct.pack(
            "<ffff",
            2.0 * math.cos(math.radians(angle)),
            2.0 * math.sin(math.radians(angle)),
            -0.5,
            1.0,
        )
        for angle in angles_deg
    )
    return message


def test_lidar_extent_rejects_alternating_half_scan_regression():
    module = _load_probe_module()

    assert module.lidar_azimuth_extent_deg(
        _azimuth_cloud((-60.0, 0.0, 60.0))
    ) == pytest.approx((-60.0, 60.0), abs=1e-5)
    left_extent = module.lidar_azimuth_extent_deg(
        _azimuth_cloud((-60.0, -30.0, 0.0))
    )
    right_extent = module.lidar_azimuth_extent_deg(
        _azimuth_cloud((0.0, 30.0, 60.0))
    )
    assert left_extent[1] < 55.0
    assert right_extent[0] > -55.0


def test_sigint_exits_130_without_double_shutdown_traceback():
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "227"
    process = subprocess.Popen(
        [
            sys.executable,
            str(PROBE_PATH),
            "--timeout-seconds",
            "20",
            "--min-observation-seconds",
            "1",
            "--emit-ready-signal",
        ],
        cwd=REPO_ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    readable, _, _ = select.select([process.stdout], [], [], 5.0)
    assert readable, "sensor preflight did not finish ROS initialization"
    ready_line = process.stdout.readline().strip()
    assert ready_line == "CARLA_SENSOR_PREFLIGHT_READY"
    process.send_signal(signal.SIGINT)
    remaining_stdout, stderr = process.communicate(timeout=5.0)
    stdout = ready_line + "\n" + remaining_stdout

    assert process.returncode == 130, (stdout, stderr)
    assert "Traceback" not in stderr
