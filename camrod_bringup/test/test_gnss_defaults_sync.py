"""Regression tests for bringup-to-sensing GNSS default synchronization."""

import ast
from pathlib import Path

import yaml


_BRINGUP_ROOT = Path(__file__).resolve().parents[1]
_WORKSPACE_SRC = _BRINGUP_ROOT.parent
_IMPL_PATH = _BRINGUP_ROOT / "launch" / "_bringup_impl.py"

_EXPECTED_ROUTE_DEFAULTS = {
    "ublox_dual_antenna": True,
    "ublox_dual_forward_ntrip_to_rover": False,
    "ublox_dual_warm_start_on_startup": False,
}

_EXPECTED_BRINGUP_FALLBACKS = {
    **_EXPECTED_ROUTE_DEFAULTS,
    "ublox_dual_base_rtcm_device": "__config__",
    "ublox_dual_base_rtcm_baud": "__config__",
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
    assert {
        key: sensing[key] for key in _EXPECTED_ROUTE_DEFAULTS
    } == _EXPECTED_ROUTE_DEFAULTS
    assert sensing["gnss_param_file"] == "sensing/gnss/zed_f9p_rover.yaml"
    # Device and baud now have one source: the selected GNSS parameter file.
    assert "ublox_dual_base_rtcm_device" not in sensing
    assert "ublox_dual_base_rtcm_baud" not in sensing


# HH_260722 - Keep code fallbacks and the full child-launch pass-through aligned
# with launch_defaults.yaml even when a custom config omits these fields.
def test_bringup_implementation_forwards_every_gnss_default():
    assert _gnss_cfg_get_fallbacks() == _EXPECTED_BRINGUP_FALLBACKS
    assert _EXPECTED_BRINGUP_FALLBACKS.keys() <= _sensing_argument_keys()


# HH_260722 - Treat package GNSS YAML as canonical and the bringup copies as
# byte-identical deployment mirrors checked by the field-test tool.
def test_bringup_gnss_configs_match_sensing_package():
    for filename in ("ntrip_client.yaml", "zed_f9p_rover.yaml"):
        bringup_config = _BRINGUP_ROOT / "config" / "sensing" / "gnss" / filename
        sensing_config = _WORKSPACE_SRC / "camrod_sensing" / "config" / "gnss" / filename
        assert bringup_config.read_bytes() == sensing_config.read_bytes()


def test_synced_gnss_config_owns_both_receiver_ports():
    config_path = (
        _BRINGUP_ROOT / "config" / "sensing" / "gnss" / "zed_f9p_rover.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rover = config["/**/ublox_gps_node"]["ros__parameters"]
    moving_base = config["/**/moving_base_rtcm_writer"]["ros__parameters"]

    assert rover["device"] == "/dev/ttyACM0"
    # HH_260818 - Lock the antenna frame and the receiver cadence at the
    # deployment mirror; nav_rate is an epoch divisor, not a frequency in Hz.
    # The FTDI identity and baud track the sensing package: the mirror still
    # carried DN03DF8V after the cable swap, which is the stale path that let
    # the moving base run without CORS while the rover reported carrSoln=0.
    assert rover["frame_id"] == "gnss_link"
    assert rover["rate"] == 10.0
    assert rover["nav_rate"] == 1
    assert moving_base["device"] == (
        "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN05Y9E7-if00-port0"
    )
    assert moving_base["baud"] == 460800


def test_cleanup_removes_stale_moving_base_writer():
    cleanup_path = _BRINGUP_ROOT / "config" / "bringup" / "cleanup_patterns.yaml"
    cleanup = yaml.safe_load(cleanup_path.read_text(encoding="utf-8"))
    assert "__node:=moving_base_rtcm_writer" in cleanup["patterns"]
