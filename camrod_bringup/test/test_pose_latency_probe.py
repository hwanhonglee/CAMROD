import importlib.util
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "pose_latency_probe.py"
)
SPEC = importlib.util.spec_from_file_location("pose_latency_probe", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_nearest_rank_percentile_handles_boundaries_and_empty_input():
    assert MODULE.nearest_rank_percentile([], 0.95) is None
    assert MODULE.nearest_rank_percentile([4.0, 1.0, 3.0, 2.0], 0.50) == 2.0
    assert MODULE.nearest_rank_percentile([4.0, 1.0, 3.0, 2.0], 0.95) == 4.0


def test_summarize_samples_calculates_rate_gaps_and_header_age():
    summary = MODULE.summarize_samples(
        [10.0, 10.1, 10.2, 10.4],
        [2.0, 4.0, 6.0, 20.0],
    )

    assert summary["count"] == 4
    assert abs(summary["rate_hz"] - 7.5) < 1.0e-9
    assert abs(summary["interarrival_ms"]["p50"] - 100.0) < 1.0e-9
    assert abs(summary["interarrival_ms"]["p95"] - 200.0) < 1.0e-9
    assert abs(summary["interarrival_ms"]["max"] - 200.0) < 1.0e-9
    assert summary["header_age_ms"]["p50"] == 5.0
    assert summary["header_age_ms"]["p95"] == 20.0
    assert summary["header_age_ms"]["max"] == 20.0


def test_summarize_samples_keeps_missing_statistics_visible():
    summary = MODULE.summarize_samples([], [])

    assert summary["count"] == 0
    assert summary["rate_hz"] is None
    assert summary["interarrival_ms"] == {
        "p50": None,
        "p95": None,
        "max": None,
    }
    assert summary["header_age_ms"] == {
        "p50": None,
        "p95": None,
        "max": None,
    }
