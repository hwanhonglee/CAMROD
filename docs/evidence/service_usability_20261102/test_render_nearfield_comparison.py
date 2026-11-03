"""HH_261102 - Validate report input using unit fixtures, never publish them as measurements."""

import copy
import pytest

pytest.importorskip("matplotlib")
from render_nearfield_comparison import MODES, SCENARIOS, distance_rows


def fixture():
    return {"metadata": {"runtime": "isolated_ros_actual_nodes"}, "rows": [
        {"mode": mode, "scenario": scenario, "output_vx": 0.0, "blocked": True}
        for mode in MODES for scenario in SCENARIOS]}


def test_all_sixteen_measurements_are_required():
    data = fixture()
    assert len(distance_rows(data)) == 16
    data["rows"].pop()
    with pytest.raises(ValueError, match="16"):
        distance_rows(data)


def test_duplicate_samples_cannot_hide_a_missing_condition():
    data = fixture()
    data["rows"][-1] = copy.deepcopy(data["rows"][0])
    with pytest.raises(ValueError, match="duplicate"):
        distance_rows(data)


@pytest.mark.parametrize("output", [0.2, float("nan"), float("inf")])
def test_moving_or_invalid_outputs_cannot_be_colored_as_stopped(output):
    data = fixture()
    data["rows"][0]["output_vx"] = output
    with pytest.raises(ValueError):
        distance_rows(data)


def test_an_unlabelled_runtime_cannot_be_presented_as_native_ros():
    data = fixture()
    data["metadata"]["runtime"] = "unknown"
    with pytest.raises(ValueError, match="native ROS"):
        distance_rows(data)
