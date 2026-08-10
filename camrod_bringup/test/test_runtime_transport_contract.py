"""Lock v2.1.5 runtime composition and optional LiDAR-grid contracts."""

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from launch import LaunchContext
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
BRINGUP_DEFAULTS = (
    SRC_ROOT / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
)
BRINGUP_IMPL = SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
CYCLONEDDS_SHM = (
    SRC_ROOT / "camrod_bringup" / "config" / "middleware" / "cyclonedds_shm.xml"
)
SENSING_CYCLONEDDS_SHM = (
    SRC_ROOT / "camrod_sensing" / "config" / "middleware"
    / "cyclonedds_lidar_shm.xml"
)
BRINGUP_SENSING_CYCLONEDDS_SHM = (
    SRC_ROOT / "camrod_bringup" / "config" / "sensing" / "middleware"
    / "cyclonedds_lidar_shm.xml"
)
ROUDI_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "middleware" / "iceoryx_roudi.toml"
)
LIDAR_LAUNCH = SRC_ROOT / "camrod_sensing" / "launch" / "lidar_driver.launch.py"
GROUND_SEGMENTATION = (
    SRC_ROOT / "camrod_sensing" / "external" / "ground_segmentation_ros2"
    / "src" / "ground_segmentation_ros2_node.cpp"
)
SYSTEM_LAUNCH = SRC_ROOT / "camrod_system" / "launch" / "system.launch.py"
RECOVERY_PROBE = (
    SRC_ROOT / "camrod_bringup" / "scripts" / "automatic_route_recovery_probe.py"
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _component_plugins(module, monkeypatch, *, cost_grid: bool) -> list[str]:
    captured = []

    def component(**kwargs):
        captured.append(kwargs)
        return kwargs

    monkeypatch.setattr(module, "ComposableNode", component)
    monkeypatch.setattr(module, "ComposableNodeContainer", lambda **kwargs: kwargs)

    context = LaunchContext()
    context.launch_configurations.update({
        "use_lidar_processing_container": "true",
        "enable_lidar_driver": "true",
        "enable_lidar_preprocessor": "true",
        "enable_lidar_cost_grid": "true" if cost_grid else "false",
    })
    actions = module._build_lidar_processing_container(context)
    assert len(actions) == 1
    return [item["plugin"] for item in captured]


def test_production_defaults_use_working_ui_renderer_and_scope_shared_memory() -> None:
    """The central deployment profile must select one explicit runtime policy."""
    defaults = yaml.safe_load(BRINGUP_DEFAULTS.read_text(encoding="utf-8"))["bringup"]

    # HH_260810 - The managed UI owns routine visualization and manual goals;
    # RViz remains available only through an explicit rviz:=true override.
    assert defaults["runtime"]["rviz"] is False
    # HH_260805 - SHM remains opt-in pending Jetson measurement and must never
    # be exported to the complete graph on Humble.
    assert defaults["runtime"]["enable_dds_shared_memory"] is False
    assert defaults["sensing"]["use_lidar_processing_container"] is True
    assert defaults["sensing"]["enable_lidar_cost_grid"] is False
    assert defaults["system"]["operator_ui_window_engine"] == "webkit"
    assert defaults["system"]["enable_operator_telemetry"] is True
    assert defaults["system"]["operator_telemetry_stream_rate_hz"] == 10.0

    bringup_source = BRINGUP_IMPL.read_text(encoding="utf-8")
    lidar_source = LIDAR_LAUNCH.read_text(encoding="utf-8")
    assert "SetEnvironmentVariable(" not in bringup_source
    assert "lidar_dds_shared_memory_active" in bringup_source
    assert "lc['sim']" in bringup_source
    assert "SetEnvironmentVariable(" in lidar_source
    assert '"RMW_IMPLEMENTATION"' in lidar_source
    assert '"CYCLONEDDS_URI"' in lidar_source
    assert "GroupAction([" in lidar_source
    assert "iox-roudi" in bringup_source
    assert "'enable_operator_telemetry': lc['enable_operator_telemetry']" in bringup_source
    assert (
        "'operator_telemetry_stream_rate_hz': "
        "lc['operator_telemetry_stream_rate_hz']"
    ) in bringup_source
    assert "cfg_get(launch_cfg, 'runtime/rviz', False)" in bringup_source


def test_shared_memory_profiles_enable_iceoryx_with_large_message_pools() -> None:
    """CycloneDDS and RouDi must agree on SHM and image/cloud-sized chunks."""
    root = ET.parse(CYCLONEDDS_SHM).getroot()
    sensing_root = ET.parse(SENSING_CYCLONEDDS_SHM).getroot()
    bringup_sensing_root = ET.parse(BRINGUP_SENSING_CYCLONEDDS_SHM).getroot()
    assert root.findtext("./Domain/SharedMemory/Enable") == "true"
    assert sensing_root.findtext("./Domain/SharedMemory/Enable") == "true"
    assert ET.tostring(root) == ET.tostring(sensing_root)
    # HH_260805 - The field config audit compares package and deployment trees
    # byte-for-byte, including the opt-in physical LiDAR transport profile.
    assert SENSING_CYCLONEDDS_SHM.read_bytes() == (
        BRINGUP_SENSING_CYCLONEDDS_SHM.read_bytes()
    )
    assert ET.tostring(sensing_root) == ET.tostring(bringup_sensing_root)
    # HH_260805 - A forced socket-buffer minimum can make DDS fail before SHM
    # starts on hosts whose kernel wmem_max is lower than that requested value.
    assert root.find("./Domain/Internal/SocketSendBufferSize") is None
    assert root.find("./Domain/Internal/SocketReceiveBufferSize") is None

    roudi = ROUDI_CONFIG.read_text(encoding="utf-8")
    assert "size = 4194304" in roudi
    assert "size = 8388608" in roudi


def test_lidar_container_loads_cost_grid_only_when_requested(monkeypatch) -> None:
    """Default LiDAR processing must not construct the optional rasterizer."""
    launch = _load_module("camrod_lidar_launch_contract", LIDAR_LAUNCH)

    without_grid = _component_plugins(launch, monkeypatch, cost_grid=False)
    assert without_grid == [
        "camrod::sensing::LidarPreprocessorNode",
        "GroundSegmentationNode",
    ]

    with_grid = _component_plugins(launch, monkeypatch, cost_grid=True)
    assert with_grid[-1] == "camrod::sensing::LidarCostGridNode"
    assert len(with_grid) == 3


def test_lidar_grid_diagnostics_follow_the_same_toggle() -> None:
    """An unloaded optional grid cannot create false checker errors."""
    launch = _load_module("camrod_system_launch_contract", SYSTEM_LAUNCH)
    config_root = SRC_ROOT / "camrod_system" / "config" / "diagnostics" / "default"

    disabled = launch._checker_parameters(
        str(config_root), str(config_root), "sensing", "cost_grid_checker",
        "cost_grid_checker.yaml", False,
    )
    enabled = launch._checker_parameters(
        str(config_root), str(config_root), "sensing", "cost_grid_checker",
        "cost_grid_checker.yaml", True,
    )

    assert disabled["grid_names"] == ["radar", "inflation"]
    assert enabled["grid_names"] == ["lidar", "radar", "inflation"]

    source = SYSTEM_LAUNCH.read_text(encoding="utf-8")
    assert "/sensing/lidar/lidar_cost_grid" in source
    assert "/sensing/cost_grid/lidar" in source
    assert "disabled_nodes_csv" in source
    assert "disabled_topics_csv" in source


def test_ground_segmentation_tf_listener_uses_component_context() -> None:
    """The scoped LiDAR container has no usable global default ROS context."""
    source = GROUND_SEGMENTATION.read_text(encoding="utf-8")
    assert "*buffer, this, false" in source
    assert "TransformListener>(*buffer);" not in source
    assert "CallbackGroupType::Reentrant" in source
    assert "buffer->setUsingDedicatedThread(true);" in source


def test_sensor_hot_paths_use_components_and_move_ownership() -> None:
    """Intra-process transport needs component registration and movable messages."""
    sensing_cmake = (SRC_ROOT / "camrod_sensing" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    lidar_source = LIDAR_LAUNCH.read_text(encoding="utf-8")
    preprocessor = (
        SRC_ROOT / "camrod_sensing" / "src" / "lidar_preprocessor_node.cpp"
    ).read_text(encoding="utf-8")
    front_camera = (
        SRC_ROOT / "camrod_sensing" / "src" / "camera_front_publisher_node.cpp"
    ).read_text(encoding="utf-8")

    assert "lidar_preprocessor_component SHARED" in sensing_cmake
    assert "lidar_cost_grid_component SHARED" in sensing_cmake
    assert lidar_source.count('"use_intra_process_comms": True') == 1
    assert lidar_source.count('"use_intra_process_comms": False') == 2
    assert 'package="camrod_runtime"' in lidar_source
    assert 'executable="scoped_component_container_mt"' in lidar_source
    # HH_260805 - Three components and their containing process share the namespace.
    assert lidar_source.count('namespace=LaunchConfiguration("module_namespace")') == 4
    assert "transient-local" in lidar_source
    assert "pub_->publish(std::move(out_msg))" in preprocessor
    assert "rect_pub_->publish(std::move(rect_message))" in front_camera


def test_recovery_probe_observes_bounded_yaw_stages() -> None:
    """Runtime evidence must recognize and measure reverse-yaw recovery."""
    source = RECOVERY_PROBE.read_text(encoding="utf-8")

    # HH_260805 - Keep dynamic evidence aligned with the production owner's
    # complete moving-state contract rather than timing out on a valid yaw arc.
    assert '"REVERSE_YAW_LEFT"' in source
    assert '"REVERSE_YAW_RIGHT"' in source
    assert '"maximum_recovery_abs_angular_z_radps"' in source
