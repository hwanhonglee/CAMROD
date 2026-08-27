"""Behavioral tests for the bounded CARLA visual-stream preflight."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


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
        ],
        cwd=REPO_ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.5)
    process.send_signal(signal.SIGINT)
    stdout, stderr = process.communicate(timeout=5.0)

    assert process.returncode == 130, (stdout, stderr)
    assert "Traceback" not in stderr
