"""Regression tests for CAMROD GNSS correction routing."""

import ast
import importlib.util
from pathlib import Path

import sys
import types

import yaml


# HH_260722 - Stub only the launch API surface used by gnss.launch.py so routing
# contracts remain testable on build machines without a sourced ROS environment.
class _StubLaunchConfiguration:
    def __init__(self, name):
        self.name = name


class _StubDeclareLaunchArgument:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _StubLaunchDescription:
    def __init__(self, actions):
        self.actions = actions


class _StubNode:
    def __init__(self, **kwargs):
        self.node_executable = kwargs.get("executable")
        self.node_name = kwargs.get("name")
        self.node_namespace = kwargs.get("namespace")
        self.node_parameters = kwargs.get("parameters", [])


def _install_launch_stubs():
    ament = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = lambda package: f"/tmp/{package}"
    ament.packages = packages
    launch = types.ModuleType("launch")
    launch.LaunchDescription = _StubLaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = _StubDeclareLaunchArgument
    actions.OpaqueFunction = _StubDeclareLaunchArgument
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = _StubLaunchConfiguration
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = _StubNode
    launch_ros.actions = launch_ros_actions
    # HH_260722 - Replace these modules deliberately: ROS pytest plugins may
    # import the real launch API before collection, while this unit test needs
    # deterministic stubs and no package-index lookup.
    sys.modules["ament_index_python"] = ament
    sys.modules["ament_index_python.packages"] = packages
    sys.modules["launch"] = launch
    sys.modules["launch.actions"] = actions
    sys.modules["launch.substitutions"] = substitutions
    sys.modules["launch_ros"] = launch_ros
    sys.modules["launch_ros.actions"] = launch_ros_actions


_install_launch_stubs()


# HH_260722 - Load the launch file by path because its dotted filename is not a
# normal Python module name.
_GNSS_LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "gnss.launch.py"
_SPEC = importlib.util.spec_from_file_location("camrod_gnss_launch", _GNSS_LAUNCH_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_GNSS_LAUNCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_GNSS_LAUNCH)


class _StubLaunchContext:
    def __init__(self, **values):
        self._values = values

    def perform_substitution(self, launch_configuration):
        return str(self._values[launch_configuration.name])


def _dual_launch_context(
    forward_ntrip_to_rover,
    base_rtcm_device="__config__",
    base_rtcm_baud="__config__",
):
    return _StubLaunchContext(
        gnss_namespace="gnss",
        rtcm_topic="rtcm",
        gnss_log_level="error",
        enable_ntrip="true",
        ntrip_param_file="/tmp/ntrip.yaml",
        ublox_dual_base_rtcm_device=base_rtcm_device,
        ublox_dual_base_rtcm_baud=base_rtcm_baud,
        ublox_param_file="/tmp/rover.yaml",
        ublox_dual_antenna="true",
        ublox_dual_forward_ntrip_to_rover=str(forward_ntrip_to_rover).lower(),
        ublox_dual_warm_start_on_startup="false",
    )


def _literal_launch_defaults(path):
    """Read direct DeclareLaunchArgument string defaults without importing ROS."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                defaults[node.args[0].value] = keyword.value.value
    return defaults


def _gnss_include_keywords(path):
    """Return explicit keyword arguments forwarded by the GNSS include."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "_inc":
            continue
        if node.args and isinstance(node.args[0], ast.Name) and node.args[0].id == "gnss_launch":
            return {keyword.arg for keyword in node.keywords if keyword.arg is not None}
    return set()


def test_single_antenna_keeps_existing_ntrip_forwarding():
    """Single antenna keeps publisher and receiver on the requested topic."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=False, forward_ntrip_to_rover=False
    )
    assert ntrip_topic == "rtcm"
    assert ublox_topic == "rtcm"


def test_dual_cascade_keeps_ntrip_separate_and_rover_usb_rtcm_off():
    """Corrected moving-base mode keeps the external stream off rover USB."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=True, forward_ntrip_to_rover=False
    )
    params = _GNSS_LAUNCH._dual_antenna_runtime_params(False)
    assert ntrip_topic == "ntrip_client/rtcm"
    assert ublox_topic == "rtcm"
    assert params["dual_antenna.usb_rtcm_in"] is False
    assert params["dual_antenna.warm_start_on_startup"] is False
    # HH_260807 - The active physical validation profile uses 200 ms epochs.
    assert params["rate"] == 5.0
    assert params["nav_rate"] == 1


def test_dual_direct_injection_fallback_enables_rover_usb_rtcm():
    """The diagnostic fallback can still feed CORS directly to rover USB."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=True, forward_ntrip_to_rover=True
    )
    params = _GNSS_LAUNCH._dual_antenna_runtime_params(True)
    assert ntrip_topic == "ntrip_client/rtcm"
    assert ublox_topic == ntrip_topic
    assert params["dual_antenna.usb_rtcm_in"] is True
    assert params["dual_antenna.configure_usb"] is True
    assert params["dual_antenna.configure_navigation"] is False
    assert params["dual_antenna.warm_start_on_startup"] is False
    assert params["dual_antenna.block_rtcm_ids"] == [4072]


def test_external_corrections_have_exactly_one_receiver_destination():
    """Direct-rover diagnostics and base correction are mutually exclusive."""
    select = _GNSS_LAUNCH._use_base_rtcm_writer
    assert select(True, False, "/dev/ttyUSB0") is True
    assert select(True, True, "/dev/ttyUSB0") is False
    assert select(True, False, "") is False
    assert select(False, False, "/dev/ttyUSB0") is False


def test_base_rtcm_writer_targets_ntrip_topic_and_device():
    """An explicit CLI port overrides the selected GNSS config."""
    node = _GNSS_LAUNCH._base_rtcm_writer_node(
        "gnss",
        "/tmp/rover.yaml",
        "/dev/test-moving-base",
        "230400",
        "ntrip_client/rtcm",
        "error",
    )
    assert node.node_executable == "rtcm_serial_writer.py"
    assert node.node_name == "moving_base_rtcm_writer"
    assert node.node_parameters[0] == "/tmp/rover.yaml"
    params = node.node_parameters[1]
    assert params["device"] == "/dev/test-moving-base"
    assert params["baud"] == 230400
    assert params["rtcm_topic"] == "ntrip_client/rtcm"


def test_base_rtcm_writer_uses_selected_gnss_config_by_default():
    """Sentinel defaults leave device and baud owned by the GNSS YAML."""
    node = _GNSS_LAUNCH._base_rtcm_writer_node(
        "gnss",
        "/tmp/rover.yaml",
        "__config__",
        "__config__",
        "ntrip_client/rtcm",
        "error",
    )
    assert node.node_parameters == [
        "/tmp/rover.yaml",
        {"rtcm_topic": "ntrip_client/rtcm"},
    ]


# HH_260722 - Verify the complete launch setup, not only helper return values:
# production owns the base writer while the direct diagnostic owns rover USB.
def test_dual_launch_creates_exactly_one_external_correction_owner():
    """Production and diagnostic routes never run both correction owners."""
    production = _GNSS_LAUNCH._launch_setup(_dual_launch_context(False))
    production_names = {node.node_name for node in production}
    assert production_names == {
        "ublox_gps_node",
        "ntrip_client",
        "moving_base_rtcm_writer",
    }
    production_ublox = next(node for node in production if node.node_name == "ublox_gps_node")
    assert production_ublox.node_parameters[-1]["dual_antenna.usb_rtcm_in"] is False
    production_writer = next(
        node for node in production if node.node_name == "moving_base_rtcm_writer"
    )
    assert production_writer.node_parameters == [
        "/tmp/rover.yaml",
        {"rtcm_topic": "ntrip_client/rtcm"},
    ]

    diagnostic = _GNSS_LAUNCH._launch_setup(_dual_launch_context(True))
    diagnostic_names = {node.node_name for node in diagnostic}
    assert diagnostic_names == {"ublox_gps_node", "ntrip_client"}
    diagnostic_ublox = next(node for node in diagnostic if node.node_name == "ublox_gps_node")
    assert diagnostic_ublox.node_parameters[-1]["dual_antenna.usb_rtcm_in"] is True


def test_dual_defaults_correct_moving_base_without_direct_rover_injection():
    """Field defaults improve absolute RTK without sacrificing MB heading."""
    declarations = {
        action.args[0]: action.kwargs.get("default_value")
        for action in _GNSS_LAUNCH.generate_launch_description().actions
        if action.args
    }
    assert declarations["ublox_dual_antenna"] == "true"
    assert declarations["ublox_dual_forward_ntrip_to_rover"] == "false"
    assert declarations["ublox_dual_warm_start_on_startup"] == "false"
    assert declarations["ublox_dual_base_rtcm_device"] == "__config__"
    assert declarations["ublox_dual_base_rtcm_baud"] == "__config__"
    assert declarations["gnss_namespace"] == "sensing/gnss"


# HH_260722 - Lock the aggregate sensing defaults and pass-through names to the
# standalone GNSS contract so a future bringup refactor cannot split the route.
def test_sensing_launch_matches_and_forwards_dual_gnss_defaults():
    """Aggregate sensing launch preserves every production GNSS default."""
    sensing_launch = _GNSS_LAUNCH_PATH.with_name("sensing.launch.py")
    defaults = _literal_launch_defaults(sensing_launch)
    expected = {
        "ublox_dual_antenna": "true",
        "ublox_dual_forward_ntrip_to_rover": "false",
        "ublox_dual_warm_start_on_startup": "false",
        "ublox_dual_base_rtcm_device": "__config__",
        "ublox_dual_base_rtcm_baud": "__config__",
    }
    assert {key: defaults[key] for key in expected} == expected
    assert expected.keys() <= _gnss_include_keywords(sensing_launch)


def test_dual_warm_start_is_explicit_recovery_only():
    """Recovery can reset the rover without changing the field default."""
    params = _GNSS_LAUNCH._dual_antenna_runtime_params(
        usb_rtcm_in=False, warm_start_on_startup=True
    )
    assert params["dual_antenna.warm_start_on_startup"] is True


def test_gnss_config_owns_distinct_rover_and_moving_base_ports():
    """One YAML safely configures both physical receivers by node name."""
    config_path = _GNSS_LAUNCH_PATH.parents[1] / "config" / "gnss" / "zed_f9p_rover.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rover = config["/**/ublox_gps_node"]["ros__parameters"]
    moving_base = config["/**/moving_base_rtcm_writer"]["ros__parameters"]

    assert rover["device"] == "/dev/ttyACM0"
    # HH_260807 - Raw receiver messages identify the antenna TF and the shared
    # profile retains the same 200 ms cadence as the dual runtime overlay.
    assert rover["frame_id"] == "gnss_link"
    assert rover["rate"] == 5.0
    assert rover["nav_rate"] == 1
    assert moving_base["device"] == (
        "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03DF8V-if00-port0"
    )
    assert moving_base["baud"] == 115200
    assert rover["device"] != moving_base["device"]


def test_explicit_empty_base_device_disables_writer():
    """Bench heading mode can still opt out without editing the YAML."""
    nodes = _GNSS_LAUNCH._launch_setup(
        _dual_launch_context(False, base_rtcm_device="")
    )
    assert {node.node_name for node in nodes} == {"ublox_gps_node", "ntrip_client"}
