"""Static launch/data contracts that prevent silent plugin/topic drift."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module(filename):
    path = PACKAGE_ROOT / "launch" / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace(".", "_"), str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_role_topics_follow_non_default_vehicle_role():
    module = _load_launch_module("adapter.launch.py")
    context = LaunchContext()
    context.launch_configurations["role_name"] = "test_vehicle"
    assert perform_substitutions(
        context, module._role_topic("/extended_ackermann_cmd")
    ) == "/carla/test_vehicle/extended_ackermann_cmd"
    assert perform_substitutions(
        context, module._role_topic("/odometry")
    ) == "/carla/test_vehicle/odometry"
    assert perform_substitutions(
        context, module._role_topic("/imu")
    ) == "/carla/test_vehicle/imu"
    assert perform_substitutions(
        context, module._role_topic("/gnss")
    ) == "/carla/test_vehicle/gnss"


def test_through_poses_bt_uses_loaded_selector_plugin_ids():
    path = (
        Path(get_package_share_directory("camrod_planning"))
        / "config"
        / "bt"
        / "navigate_through_poses_w_planner_selector.xml"
    )
    root = ET.parse(path).getroot()

    planner_selectors = root.findall(".//PlannerSelector")
    controller_selectors = root.findall(".//ControllerSelector")
    compute_nodes = root.findall(".//ComputePathThroughPoses")
    follow_nodes = root.findall(".//FollowPath")

    assert len(planner_selectors) == 1
    assert planner_selectors[0].attrib["default_planner"] == "LaneletRoute"
    assert len(controller_selectors) == 1
    assert controller_selectors[0].attrib["default_controller"] == "RPP"
    assert compute_nodes
    assert all(
        node.attrib["planner_id"] == "{selected_planner}"
        for node in compute_nodes
    )
    assert follow_nodes
    assert all(
        node.attrib["controller_id"] == "{selected_controller}"
        for node in follow_nodes
    )

    xml_text = path.read_text(encoding="utf-8")
    assert 'planner_id="GridBased"' not in xml_text
    assert 'controller_id="FollowPath"' not in xml_text
    compatibility_copy = (
        PACKAGE_ROOT
        / "config"
        / "navigate_through_poses_w_camrod_selectors.xml"
    )
    assert compatibility_copy.read_bytes() == path.read_bytes()
