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


def test_motion_summary_unwraps_yaw_and_counts_crab_samples():
    pose_samples = [
        {"stamp": 1.0, "x": 0.0, "y": 0.0, "yaw": 3.10},
        {"stamp": 2.0, "x": 0.0, "y": 0.4, "yaw": -3.10},
    ]
    twist_samples = [
        {
            "stamp": 1.0,
            "linear_x": 0.0,
            "linear_y": 0.2,
            "angular_z": 0.01,
        },
        {
            "stamp": 2.0,
            "linear_x": 0.1,
            "linear_y": 0.0,
            "angular_z": 0.02,
        },
    ]

    summary = MODULE.summarize_motion_samples(
        pose_samples,
        twist_samples,
    )

    assert summary["position_displacement_m"] == 0.4
    assert summary["yaw_pose_count"] == 2
    assert 4.0 < summary["yaw_change_deg"] < 5.0
    assert summary["yaw_span_deg"] == summary["yaw_change_deg"]
    assert summary["crab_sample_count"] == 1
    assert summary["angular_z_abs_radps"]["max"] == 0.02


def test_pose_comparison_pairs_by_stamp_and_wraps_yaw_error():
    reference = [
        {"stamp": 1.0, "x": 1.0, "y": 2.0, "yaw": 3.13},
        {"stamp": 2.0, "x": 2.0, "y": 2.0, "yaw": -3.13},
    ]
    target = [
        {"stamp": 1.02, "x": 1.3, "y": 2.4, "yaw": -3.13},
        {"stamp": 2.02, "x": 2.0, "y": 2.1, "yaw": 3.13},
    ]

    comparison = MODULE.compare_pose_samples(reference, target)

    assert comparison["pair_count"] == 2
    assert comparison["yaw_pair_count"] == 2
    assert abs(comparison["position_error_m"]["max"] - 0.5) < 1.0e-9
    assert comparison["yaw_error_deg"]["max"] < 2.0
    assert comparison["stamp_delta_ms"]["max"] < 21.0


def test_pose_comparison_ignores_unavailable_covariance_yaw_only():
    reference = [
        {
            "stamp": 1.0,
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0,
            "yaw_valid": False,
        },
    ]
    target = [
        {
            "stamp": 1.0,
            "x": 1.1,
            "y": 2.0,
            "yaw": 0.2,
            "yaw_valid": True,
        },
    ]

    comparison = MODULE.compare_pose_samples(reference, target)

    assert comparison["pair_count"] == 1
    assert comparison["yaw_pair_count"] == 0
    assert comparison["position_error_m"]["max"] > 0.09
    assert comparison["yaw_error_deg"]["max"] is None
