"""Execute the capture script's actual sampling code without a live display."""
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/virtual_carla/capture_ui_evidence.sh"


@pytest.mark.parametrize("duration,fps", [(12, 1), (60, 5), (900, 5), (1800, 2)])
def test_terminal_frame_is_preserved_with_six_distinct_samples(duration, fps):
    source = SCRIPT.read_text(encoding="utf-8")
    code = source.split('"${CLIP_SECONDS}" <<\'PY\'\n', 1)[1].split("\nPY\n", 1)[0]
    result = subprocess.run(
        [sys.executable, "-", str(duration), str(fps), "1.25"],
        input=code, text=True, capture_output=True, check=True,
    )
    time_line, frame_line, interval_line = result.stdout.strip().splitlines()
    times = [float(value) for value in time_line.split(",")]
    frames = [int(value) for value in frame_line.split(",")]
    intervals = [tuple(map(float, value.split(":"))) for value in interval_line.split(",")]
    assert len(times) == len(set(frames)) == len(intervals) == 6
    assert frames == sorted(frames)
    assert all(0 <= frame < duration * fps for frame in frames)
    assert 0 < duration - times[-1] <= 1.0
    assert all(0 <= start < end <= duration for start, end in intervals)
    assert all(left[1] < right[0] for left, right in zip(intervals, intervals[1:]))
