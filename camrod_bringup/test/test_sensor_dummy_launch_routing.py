"""Cross-package launch routing tests for disabled-hardware dummies."""

import ast
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions


BRINGUP_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = BRINGUP_ROOT.parent
BRINGUP_IMPL_PATH = BRINGUP_ROOT / "launch/_bringup_impl.py"
DEFAULTS_PATH = BRINGUP_ROOT / "config/bringup/launch_defaults.yaml"


def _load_bringup_module():
    spec = importlib.util.spec_from_file_location(
        "camrod_bringup_impl_dummy_test", BRINGUP_IMPL_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


BRINGUP_IMPL = _load_bringup_module()
BRINGUP_TREE = ast.parse(
    BRINGUP_IMPL_PATH.read_text(encoding="utf-8")
)
BRINGUP_DESCRIPTION = BRINGUP_IMPL.generate_launch_description()


def _assigned_value(variable_name):
    matches = [
        node.value
        for node in ast.walk(BRINGUP_TREE)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == variable_name
            for target in node.targets
        )
    ]
    assert len(matches) == 1
    return matches[0]


def _dictionary_value(variable_name, key):
    dictionary = _assigned_value(variable_name)
    assert isinstance(dictionary, ast.Dict)
    for key_node, value_node in zip(dictionary.keys, dictionary.values):
        if isinstance(key_node, ast.Constant) and key_node.value == key:
            return value_node
    raise AssertionError(f"{key!r} missing from {variable_name}")


def _walk_owned_entities(entity):
    """Walk bringup-owned actions without expanding child launch sources."""
    yield entity
    if isinstance(entity, IncludeLaunchDescription):
        return
    if hasattr(entity, "entities"):
        children = entity.entities
    elif isinstance(entity, TimerAction):
        children = entity.actions
    elif hasattr(entity, "get_sub_entities"):
        children = entity.get_sub_entities()
    else:
        children = ()
    for child in children:
        yield from _walk_owned_entities(child)


def _module_include(filename):
    context = LaunchContext()
    matches = []
    for entity in _walk_owned_entities(BRINGUP_DESCRIPTION):
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        location = perform_substitutions(
            context,
            entity.launch_description_source
            ._LaunchDescriptionSource__location,
        )
        if location.endswith(f"/{filename}"):
            matches.append(entity)
    assert len(matches) == 1
    return matches[0]


def _camera_owner_resolution_action(
    name="camera_yolo_container_active_resolved",
):
    context = LaunchContext()
    matches = [
        entity
        for entity in BRINGUP_DESCRIPTION.entities
        if isinstance(entity, SetLaunchConfiguration)
        and perform_substitutions(
            context,
            entity._SetLaunchConfiguration__name,
        ) == name
    ]
    assert len(matches) == 1
    return matches[0]


def _base_rear_owner_context(**overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": "false",
        "use_rear_camera_apriltag_container": "true",
        "enable_sensing_module": "true",
        "enable_camera": "true",
        "enable_rear_camera": "true",
        "enable_parking": "true",
        "parking_method": "apriltag",
        **overrides,
    })
    return context


@pytest.mark.parametrize(
    "truthy_sim", ("1", "true", "True", "yes", "YES", "on", "ON")
)
def test_sim_switch_forces_dummy_policy_off_for_all_truthy_spellings(
    truthy_sim,
):
    expression = BRINGUP_IMPL.sim_switch(
        LaunchConfiguration("sim"),
        "false",
        LaunchConfiguration("real_dummy_policy"),
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": truthy_sim,
        "real_dummy_policy": "true",
    })

    assert context.perform_substitution(expression) == "false"


@pytest.mark.parametrize("real_sim", ("0", "false", "no", "off"))
def test_non_sim_preserves_requested_dummy_policy(real_sim):
    expression = BRINGUP_IMPL.sim_switch(
        LaunchConfiguration("sim"),
        "false",
        LaunchConfiguration("real_dummy_policy"),
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": real_sim,
        "real_dummy_policy": "true",
    })

    assert context.perform_substitution(expression) == "true"


def test_bringup_forces_both_sensing_and_platform_dummies_off_in_sim():
    sensing_policy = ast.unparse(
        _dictionary_value(
            "sensing_args", "publish_sensor_dummies_when_disabled"
        )
    )
    platform_policy = ast.unparse(
        _dictionary_value(
            "platform_args", "ranger_dummy_when_disabled"
        )
    )

    expected = (
        "sim_switch(lc['sim'], 'false', "
        "lc['publish_sensor_dummies_when_disabled'])"
    )
    assert sensing_policy == expected
    assert platform_policy == expected


@pytest.mark.parametrize("truthy_sim", ("1", "true", "yes", "on"))
def test_launch_description_passes_dummy_off_to_both_modules_in_sim(
    truthy_sim,
):
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": truthy_sim,
        "publish_sensor_dummies_when_disabled": "true",
    })
    sensing_arguments = dict(
        _module_include("sensing.launch.py").launch_arguments
    )
    platform_arguments = dict(
        _module_include("platform.launch.py").launch_arguments
    )

    assert context.perform_substitution(
        sensing_arguments["publish_sensor_dummies_when_disabled"]
    ) == "false"
    assert context.perform_substitution(
        platform_arguments["ranger_dummy_when_disabled"]
    ) == "false"


def test_sim_selects_non_ranger_platform_and_disables_physical_can():
    platform_type = ast.unparse(
        _dictionary_value("platform_args", "platform_type")
    )
    ranger_driver = ast.unparse(
        _dictionary_value("platform_args", "ranger_driver_enable")
    )

    assert platform_type == (
        "sim_switch(lc['sim'], 'rmp401', lc['platform_type'])"
    )
    assert ranger_driver == (
        "sim_switch(lc['sim'], 'false', "
        "lc['platform_ranger_driver_enable'])"
    )


def test_external_component_camera_owns_front_without_dummy_overlap():
    front_external = ast.unparse(
        _dictionary_value(
            "sensing_args", "front_camera_source_external"
        )
    )
    component_expression = ast.unparse(
        _assigned_value("camera_yolo_container_active_expression")
    )
    component_active = ast.unparse(
        _assigned_value("camera_yolo_container_active")
    )
    regular_front = ast.unparse(
        _assigned_value("regular_front_camera_enable")
    )

    assert front_external == "camera_yolo_container_active"
    assert "lc['sim']" in component_expression
    assert "not in ['1', 'true', 'yes', 'on']" in component_expression
    assert component_active == (
        "LaunchConfiguration('camera_yolo_container_active_resolved')"
    )
    assert "camera_yolo_container_active" in regular_front
    assert "lc['enable_front_camera']" in regular_front
    assert "in ['1', 'true', 'yes', 'on']" in regular_front
    assert "'false' if" in regular_front


@pytest.mark.parametrize(
    ("sim", "use_container", "expected_external", "expected_regular"),
    (
        ("false", "true", "true", "false"),
        ("false", "false", "false", "true"),
        ("1", "true", "false", "false"),
        ("yes", "true", "false", "false"),
        ("on", "true", "false", "false"),
    ),
)
def test_launch_description_resolves_component_front_camera_ownership(
    sim, use_container, expected_external, expected_regular
):
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": sim,
        "use_camera_yolo_container": use_container,
        "enable_camera": "true",
        "enable_front_camera": "true",
        "perception_enable_yolo": "true",
    })
    _camera_owner_resolution_action().execute(context)
    arguments = dict(
        _module_include("sensing.launch.py").launch_arguments
    )

    assert context.perform_substitution(
        arguments["front_camera_source_external"]
    ) == expected_external
    assert context.perform_substitution(
        arguments["enable_front_camera"]
    ) == expected_regular


def test_component_camera_owner_survives_scoped_child_front_override():
    context = LaunchContext()
    context.launch_configurations.update({
        "sim": "false",
        "use_camera_yolo_container": "true",
        "enable_camera": "true",
        "enable_front_camera": "true",
        "perception_enable_yolo": "true",
    })
    _camera_owner_resolution_action().execute(context)
    arguments = dict(
        _module_include("sensing.launch.py").launch_arguments
    )

    assert (
        context.launch_configurations[
            "camera_yolo_container_active_resolved"
        ]
        == "true"
    )
    # HH_260729 - IncludeLaunchDescription scopes the child's declared launch
    # arguments. This reproduces the value sensing.launch receives for its
    # regular front publisher and proves the external-owner flag is no longer
    # recomputed from that shadowed false value.
    context.launch_configurations["enable_front_camera"] = "false"
    assert context.perform_substitution(
        arguments["front_camera_source_external"]
    ) == "true"


@pytest.mark.parametrize(
    ("overrides", "expected_active"),
    (
        ({}, "true"),
        ({"sim": "true"}, "false"),
        ({"parking_method": "reverse"}, "false"),
        ({"enable_rear_camera": "false"}, "false"),
        ({"use_rear_camera_apriltag_container": "false"}, "false"),
    ),
)
def test_rear_apriltag_container_has_one_physical_owner(
    overrides, expected_active
):
    context = _base_rear_owner_context(**overrides)
    _camera_owner_resolution_action(
        "rear_camera_apriltag_container_active_resolved"
    ).execute(context)
    sensing_arguments = dict(
        _module_include("sensing.launch.py").launch_arguments
    )
    parking_arguments = dict(
        _module_include("parking.launch.py").launch_arguments
    )

    assert context.perform_substitution(
        sensing_arguments["rear_camera_source_external"]
    ) == expected_active
    expected_regular = "false" if expected_active == "true" else (
        "false" if overrides.get("sim") == "true" else
        overrides.get("enable_rear_camera", "true")
    )
    assert context.perform_substitution(
        sensing_arguments["enable_rear_camera"]
    ) == expected_regular
    assert context.perform_substitution(
        parking_arguments["launch_apriltag_detector"]
    ) == ("false" if expected_active == "true" else "true")


def test_rear_apriltag_container_keeps_the_complete_intra_process_chain():
    """Capture, rectification, and detection must share one owner/process."""
    source = (
        SRC_ROOT / "camrod_bringup" / "launch" /
        "rear_camera_apriltag_container.launch.py"
    ).read_text(encoding="utf-8")
    defaults = yaml.safe_load(DEFAULTS_PATH.read_text(encoding="utf-8"))

    # HH_260805 - Lock all three high-bandwidth stages and prevent a future
    # fallback edit from silently reintroducing a duplicate rear publisher.
    for plugin in (
        "camrod::sensing::CameraRearPublisherNode",
        "image_proc::RectifyNode",
        "camrod::perception::AprilTagParkingDetectorNode",
    ):
        assert f'plugin="{plugin}"' in source
    assert source.count('"use_intra_process_comms": True') == 3
    assert 'package="camrod_runtime"' in source
    assert 'executable="scoped_component_container_mt"' in source
    assert defaults["bringup"]["perception"][
        "use_rear_camera_apriltag_container"
    ] is True


def test_master_dummy_policy_default_is_enabled_only_for_non_sim_runs():
    defaults = yaml.safe_load(DEFAULTS_PATH.read_text(encoding="utf-8"))

    assert (
        defaults["bringup"]["sensing"][
            "publish_sensor_dummies_when_disabled"
        ]
        is True
    )
