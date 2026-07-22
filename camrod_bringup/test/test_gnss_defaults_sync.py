"""Regression tests for bringup-to-sensing GNSS default synchronization."""

import ast
from pathlib import Path

import yaml


_BRINGUP_ROOT = Path(__file__).resolve().parents[1]
_WORKSPACE_SRC = _BRINGUP_ROOT.parent
_IMPL_PATH = _BRINGUP_ROOT / "launch" / "_bringup_impl.py"

_EXPECTED_DEFAULTS = {
    "ublox_dual_antenna": True,
    "ublox_dual_forward_ntrip_to_rover": False,
    "ublox_dual_warm_start_on_startup": False,
    "ublox_dual_base_rtcm_device": "/dev/ttyUSB0",
    "ublox_dual_base_rtcm_baud": 115200,
}


def _implementation_tree():
    return ast.parse(_IMPL_PATH.read_text(encoding="utf-8"))


def _gnss_cfg_get_fallbacks():
    fallbacks = {}
    for node in ast.walk(_implementation_tree()):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "cfg_get":
            continue
        if len(node.args) < 3 or not isinstance(node.args[1], ast.Constant):
            continue
        key = node.args[1].value
        if not isinstance(key, str) or not key.startswith("sensing/ublox_dual_"):
            continue
        if isinstance(node.args[2], ast.Constant):
            fallbacks[key.removeprefix("sensing/")] = node.args[2].value
    return fallbacks


def _sensing_argument_keys():
    for node in ast.walk(_implementation_tree()):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == "sensing_args" for target in node.targets):
            continue
        if not isinstance(node.value, ast.Dict):
            continue
        return {
            key.value
            for key in node.value.keys
            if isinstance(key, ast.Constant) and isinstance(key.value, str)
        }
    return set()


# HH_260722 - Lock the production two-port values at the top-level deployment
# entry point, including the direct-rover and warm-start safety defaults.
def test_launch_defaults_select_verified_two_port_route():
    config_path = _BRINGUP_ROOT / "config" / "bringup" / "launch_defaults.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    sensing = config["bringup"]["sensing"]
    assert {key: sensing[key] for key in _EXPECTED_DEFAULTS} == _EXPECTED_DEFAULTS


# HH_260722 - Keep code fallbacks and the full child-launch pass-through aligned
# with launch_defaults.yaml even when a custom config omits these fields.
def test_bringup_implementation_forwards_every_gnss_default():
    assert _gnss_cfg_get_fallbacks() == _EXPECTED_DEFAULTS
    assert _EXPECTED_DEFAULTS.keys() <= _sensing_argument_keys()


# HH_260722 - Treat package GNSS YAML as canonical and the bringup copies as
# byte-identical deployment mirrors checked by the field-test tool.
def test_bringup_gnss_configs_match_sensing_package():
    for filename in ("ntrip_client.yaml", "zed_f9p_rover.yaml"):
        bringup_config = _BRINGUP_ROOT / "config" / "sensing" / "gnss" / filename
        sensing_config = _WORKSPACE_SRC / "camrod_sensing" / "config" / "gnss" / filename
        assert bringup_config.read_bytes() == sensing_config.read_bytes()
