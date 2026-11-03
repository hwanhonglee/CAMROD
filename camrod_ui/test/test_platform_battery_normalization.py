"""HH_261102 - Robot/Guest conversion must agree at physical float32 SOC thresholds."""

from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime/python"))
from camrod_ui.battery_policy import normalize_platform_battery_percent


@pytest.mark.parametrize("value,expected", [
    (float("nan"), -1), (float("inf"), -1), (float("-inf"), -1),
    (-0.1, 0), (0.0, 0), (0.249, 24), (0.25, 25),
    (0.349, 34), (0.35, 35), (0.80, 80), (1.0, 100), (1.20, 100),
])
def test_existing_unknown_clamp_and_floor_contract(value, expected):
    assert normalize_platform_battery_percent(value) == expected


@pytest.mark.parametrize("ratio,expected", [(0.25, 25), (0.35, 35)])
def test_wire_threshold_passes_but_its_adjacent_lower_sample_does_not(ratio, expected):
    wire = struct.unpack("<f", struct.pack("<f", ratio))[0]
    bits = struct.unpack("<I", struct.pack("<f", ratio))[0]
    lower = struct.unpack("<f", struct.pack("<I", bits - 1))[0]
    assert normalize_platform_battery_percent(wire) == expected
    assert normalize_platform_battery_percent(lower) == expected - 1
