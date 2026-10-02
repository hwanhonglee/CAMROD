"""HH_261002 - Keep passive mission recording separate from motion authority."""

import ast
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "launch/_bringup_impl.py"


def _argument_default(tree, name):
    return next(node.elts[1] for node in ast.walk(tree)
                if isinstance(node, ast.Tuple) and len(node.elts) == 3
                and isinstance(node.elts[0], ast.Constant)
                and node.elts[0].value == name)


def test_recorder_defaults_to_real_independent_storage_and_no_raw_can():
    # HH_261002 - Adding a journal must not silently open a physical CAN socket
    # or reuse the pre-existing service metrics database.
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    enabled = _argument_default(tree, "enable_mission_recorder")
    assert isinstance(enabled, ast.Call)
    assert ast.literal_eval(enabled.args[1]) == "system/enable_mission_recorder"
    assert ast.literal_eval(enabled.args[2]) is True
    expected = {
        "mission_recorder_environment": ("CAMROD_RECORDING_ENVIRONMENT", "real"),
        "mission_recorder_raw_can_interface": ("CAMROD_MISSION_RAW_CAN_INTERFACE", ""),
    }
    for name, values in expected.items():
        node = _argument_default(tree, name)
        assert ast.unparse(node.func) == "os.environ.get"
        assert tuple(ast.literal_eval(arg) for arg in node.args) == values
    root = _argument_default(tree, "mission_records_root")
    assert ast.literal_eval(root.args[0]) == "CAMROD_MISSION_RECORDS_ROOT"
    assert ast.unparse(root.args[1].func) == "os.path.expanduser"
    assert ast.literal_eval(root.args[1].args[0]) == "~/.local/state/camrod/mission_records"


def test_recorder_arguments_are_forwarded_to_ui_without_platform_overrides():
    # HH_261002 - The journal is owned by camrod_ui; bringup passes values only.
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    api_args = next(node.value for node in ast.walk(tree)
                    if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "api_args"
                            for target in node.targets))
    values = {ast.literal_eval(key): value for key, value in zip(api_args.keys, api_args.values)}
    for name in ("enable_mission_recorder", "mission_records_root",
                 "mission_recorder_environment", "mission_recorder_raw_can_interface"):
        value = values[name]
        assert isinstance(value, ast.Subscript)
        assert isinstance(value.value, ast.Name) and value.value.id == "lc"
        assert ast.literal_eval(value.slice) == name
