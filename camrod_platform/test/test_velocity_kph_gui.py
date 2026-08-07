"""Lock the installed physical ground-speed GUI to the production contract."""

import ast
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "velocity_kph_gui.py"


def _speed_helpers() -> dict:
    """Load pure helpers without requiring a built ROS message environment."""
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"), filename=str(SCRIPT))
    wanted = {"mps_to_kph", "ground_speed_mps"}
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in wanted
    ]
    assert {node.name for node in functions} == wanted
    namespace = {"math": math}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SCRIPT), "exec"), namespace)
    return namespace


def test_speed_helpers_report_vector_ground_speed() -> None:
    helpers = _speed_helpers()

    assert helpers["ground_speed_mps"](3.0, 4.0) == pytest.approx(5.0)
    assert helpers["mps_to_kph"](0.555556) == pytest.approx(2.0, abs=2.0e-6)


def test_default_gauge_cruise_matches_two_kph_release() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'declare_parameter("cruise_speed_kph", 2.0)' in source
    assert 'declare_parameter("cruise_speed_kph", 3.0)' not in source
    assert "ros2 run camrod_platform velocity_kph_gui.py" in source
