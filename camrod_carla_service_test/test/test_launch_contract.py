"""HH_261103 - Verify explicit simulator selection while ordinary native launch defaults remain unchanged."""

import hashlib
import importlib.util
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml


ROOT = Path(__file__).parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def text(value, context):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def parking_context(overrides=None):
    module = load(ROOT.parent / "camrod_control/launch/parking.launch.py", "service_native_parking")
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    april = next(node for node in nodes if "apriltag" in text(node.node_executable, context))
    return context, april


def test_native_april_selection_and_empty_overlay_are_default():
    context, node = parking_context()
    assert text(node.node_package, context) == "camrod_control"
    assert text(node.node_executable, context) == "apriltag_parking_controller_node"
    disabled = yaml.safe_load((ROOT.parent / "camrod_control/config/parking_runtime_profiles/disabled.yaml").read_text())
    assert all(value["ros__parameters"] == {} for value in disabled.values())


def test_explicit_package_overlay_retains_private_dispatcher_topics():
    overlay = str(ROOT / "config/parking_retry.yaml")
    context, node = parking_context({"apriltag_controller_package": "camrod_carla_service_test",
                                     "runtime_override_parameter_file": overlay})
    assert text(node.node_package, context) == "camrod_carla_service_test"
    parameters = evaluate_parameters(context, node._Node__parameters)
    assert Path(parameters[1]) == Path(overlay)
    assert parameters[-1]["command_topic"] == "/parking/private/apriltag/cmd_vel"
    assert parameters[-1]["service_state_topic"] == "/parking/private/apriltag/service_state"


def test_wrapper_disables_original_contact_and_uses_sparse_retry_profile():
    module = load(ROOT / "launch/service_test.launch.py", "service_test_wrapper")
    _profile, arguments = module.service_test_arguments(get_package_share_directory("camrod_carla_adapter"), str(ROOT))
    assert arguments["launch_charging_contact_emulator"] == "false"
    assert arguments["carla_parking_runtime_override_param_file"] == str(ROOT / "config/parking_retry.yaml")
    source = (ROOT / "launch/service_test.launch.py").read_text()
    assert 'GroupAction(scoped=True' in source
    assert '"charging_station.yaml"' in source
    assert '"parking_status_topic": "/parking/status"' in source
    assert '"drop_zones_yaml"' not in source.split("arguments.update", 1)[1].split("return profile", 1)[0]


def test_retry_profile_does_not_relax_acceptance_and_vendor_is_portable():
    values = yaml.safe_load((ROOT / "config/parking_retry.yaml").read_text())["/parking/apriltag_parking_controller"]["ros__parameters"]
    assert values["final_lateral_tolerance_m"] == .03
    assert values["translation_stop_tag_distance_m"] == .40
    assert values["maximum_retries"] == 2
    assert values["retry_forward_distance_m"] == .8
    assert values["retry_maximum_odometry_step_m"] == .10
    provenance = json.loads((ROOT / "PROVENANCE.json").read_text())
    for record in provenance["files"]:
        if record["destination"].startswith("test/"):
            continue
        body = (ROOT / record["destination"]).read_bytes().split(b"\n", 1)[1]
        assert hashlib.sha256(body).hexdigest() == record["source_sha256"]
