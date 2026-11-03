"""HH_261102 - Retiring an unused scanner must preserve the actual soak recovery check."""

import ast
from pathlib import Path

import pytest


def recovery_progress(events):
    source = Path(__file__).resolve().parents[1] / "scripts/sim_validation_runner.py"
    tree = ast.parse(source.read_text())
    runner = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == "SimValidationRunner")
    # Exercise the real pure helper without constructing its ROS validation node.
    helper = next(node for node in runner.body
                  if isinstance(node, ast.FunctionDef) and node.name == "_route_recovery_progress")
    helper.decorator_list = []
    namespace = {}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[helper.name](events)


@pytest.mark.parametrize("events,expected", [
    ([], (False, False)),
    (["IDLE|initial"], (False, False)),
    (["IDLE|initial", "CRAB_LEFT|moving"], (True, False)),
    (["REVERSE|moving", "IDLE|released"], (True, True)),
    (["REVERSE_YAW_RIGHT|moving", "ERROR|failed"], (True, False)),
])
def test_recovery_requires_idle_after_real_motion(events, expected):
    assert recovery_progress(events) == expected
