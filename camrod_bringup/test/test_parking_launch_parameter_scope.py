"""Execute parking launch scopes without starting ROS processes."""

from pathlib import Path

import ament_index_python.packages
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import visit_all_entities_and_collect_futures
import launch_ros.actions
import pytest
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "method,detector_enabled,expected_controllers,expect_detector",
    [
        ("auto", "true", {"parking_dispatcher", "reverse_parking_controller",
                          "apriltag_parking_controller"}, True),
        ("apriltag", "true", {"apriltag_parking_controller"}, True),
        ("reverse", "true", {"reverse_parking_controller"}, False),
        ("auto", "false", {"parking_dispatcher", "reverse_parking_controller",
                           "apriltag_parking_controller"}, False),
    ],
)
def test_detector_include_cannot_replace_controller_yaml(
    monkeypatch, method, detector_enabled, expected_controllers, expect_detector,
):
    """The included detector owns its YAML; later controllers keep parking YAML."""
    controller_file = SRC_ROOT / "camrod_control/config/parking.yaml"
    detector_file = SRC_ROOT / "camrod_perception/config/apriltag_parking_detector.yaml"
    captured = {}

    def capture_node(**kwargs):
        def capture(context):
            parameters = kwargs.get("parameters", [])
            if parameters:
                captured[kwargs["name"]] = Path(parameters[0].perform(context))
            return []

        return OpaqueFunction(function=capture, condition=kwargs.get("condition"))

    monkeypatch.setattr(
        ament_index_python.packages, "get_package_share_directory",
        lambda package: str(SRC_ROOT / package),
    )
    # Preserve real launch inclusion, argument evaluation, conditions and scope
    # push/pop. Only the process-starting actions are replaced by observations.
    monkeypatch.setattr(launch_ros.actions, "Node", capture_node)
    monkeypatch.setattr(launch_ros.actions, "ComposableNodeContainer", capture_node)
    context = LaunchContext()
    context.launch_configurations.update({
        "parameter_file": str(controller_file),
        "apriltag_parameter_file": str(detector_file),
        "parking_method": method,
        "launch_apriltag_detector": detector_enabled,
        "apriltag_launch_rectify": "false",
        "launch_rectify": "true",
    })
    description = PythonLaunchDescriptionSource(str(
        SRC_ROOT / "camrod_control/launch/parking.launch.py"
    )).get_launch_description(context)
    visit_all_entities_and_collect_futures(description, context)

    assert set(captured) == expected_controllers | (
        {"apriltag_parking_detector"} if expect_detector else set()
    )
    if expect_detector:
        assert captured["apriltag_parking_detector"] == detector_file
    for controller in expected_controllers:
        assert captured[controller] == controller_file
    assert context.launch_configurations["parameter_file"] == str(controller_file)
    assert context.launch_configurations["launch_rectify"] == "true"
    if "reverse_parking_controller" in captured:
        reverse = yaml.safe_load(captured["reverse_parking_controller"].read_text())[
            "/parking/reverse_parking_controller"
        ]["ros__parameters"]
        assert reverse["complete_without_charging"] is True
        assert reverse["maximum_reverse_distance_m"] == 5.0
