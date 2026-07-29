"""Launch-condition tests for real versus disabled-sensor publishers."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name, relative_path):
    path = PACKAGE_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


SENSING_LAUNCH = _load_module(
    "camrod_sensing_launch", "launch/sensing.launch.py"
)
RADAR_LAUNCH = _load_module(
    "camrod_radar_launch", "launch/radar.launch.py"
)

GROUP_PARAMETER_NAMES = {
    "publish_gnss": "gnss",
    "publish_imu": "imu",
    "publish_lidar": "lidar",
    "publish_front_camera": "front_camera",
    "publish_rear_camera": "rear_camera",
}


def _walk_entities(entity):
    yield entity
    if hasattr(entity, "entities"):
        children = entity.entities
    else:
        children = entity.get_sub_entities()
    for child in children:
        yield from _walk_entities(child)


def _sensing_dummy_node():
    description = SENSING_LAUNCH.generate_launch_description()
    matches = [
        entity
        for entity in _walk_entities(description)
        if isinstance(entity, Node)
        and entity.node_executable == "sensing_dummy_publisher.py"
    ]
    assert len(matches) == 1
    return matches[0]


def _camera_include():
    description = SENSING_LAUNCH.generate_launch_description()
    matches = []
    context = LaunchContext()
    for entity in _walk_entities(description):
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        location = perform_substitutions(
            context,
            entity.launch_description_source
            ._LaunchDescriptionSource__location,
        )
        if location.endswith("camera.launch.py"):
            matches.append(entity)
    assert len(matches) == 1
    return matches[0]


def _dummy_selection(**overrides):
    configuration = {
        "publish_sensor_dummies_when_disabled": "true",
        "enable_gnss": "true",
        "enable_imu": "true",
        "enable_lidar_driver": "true",
        "front_camera_source_external": "false",
        "enable_front_camera_effective": "true",
        "enable_rear_camera_effective": "true",
    }
    configuration.update({
        name: "true" if value is True else "false" if value is False else value
        for name, value in overrides.items()
    })
    context = LaunchContext()
    context.launch_configurations.update(configuration)
    node = _sensing_dummy_node()

    values = {}
    parameter_dictionary = node._Node__parameters[0]
    for key_substitutions, value in parameter_dictionary.items():
        key = perform_substitutions(context, list(key_substitutions))
        values[key] = value.evaluate(context) if hasattr(
            value, "evaluate"
        ) else value
    groups = tuple(
        group
        for parameter_name, group in GROUP_PARAMETER_NAMES.items()
        if values[parameter_name]
    )
    return node.condition.evaluate(context), groups, values


@pytest.mark.parametrize(
    ("overrides", "expected_active", "expected_groups"),
    (
        ({}, False, ()),
        (
            {"publish_sensor_dummies_when_disabled": False,
             "enable_gnss": False, "enable_imu": False,
             "enable_lidar_driver": False,
             "enable_front_camera_effective": False,
             "enable_rear_camera_effective": False},
            False,
            (),
        ),
        ({"enable_gnss": False}, True, ("gnss",)),
        ({"enable_imu": False}, True, ("imu",)),
        ({"enable_lidar_driver": False}, True, ("lidar",)),
        (
            {"enable_front_camera_effective": False},
            True,
            ("front_camera",),
        ),
        (
            {"enable_rear_camera_effective": False},
            True,
            ("rear_camera",),
        ),
        (
            {"enable_gnss": False, "enable_imu": False,
             "enable_lidar_driver": False,
             "enable_front_camera_effective": False,
             "enable_rear_camera_effective": False},
            True,
            ("gnss", "imu", "lidar", "front_camera", "rear_camera"),
        ),
    ),
)
def test_only_disabled_real_sensor_groups_select_dummy_outputs(
    overrides, expected_active, expected_groups
):
    active, groups, values = _dummy_selection(**overrides)

    assert active is expected_active
    assert groups == expected_groups
    assert all(
        isinstance(values[parameter_name], bool)
        for parameter_name in GROUP_PARAMETER_NAMES
    )


def test_external_component_front_camera_never_gets_duplicate_dummy():
    active, groups, _ = _dummy_selection(
        front_camera_source_external=True,
        enable_front_camera_effective=False,
    )
    assert active is False
    assert groups == ()

    active, groups, _ = _dummy_selection(
        front_camera_source_external=True,
        enable_front_camera_effective=False,
        enable_rear_camera_effective=False,
    )
    assert active is True
    assert groups == ("rear_camera",)


@pytest.mark.parametrize("truthy", ("1", "true", "True", "yes", "on"))
def test_truthy_real_flags_never_select_a_duplicate_dummy(truthy):
    active, groups, _ = _dummy_selection(
        publish_sensor_dummies_when_disabled=truthy,
        enable_gnss=truthy,
        enable_imu=truthy,
        enable_lidar_driver=truthy,
        enable_front_camera_effective=truthy,
        enable_rear_camera_effective=truthy,
    )

    assert active is False
    assert groups == ()


@pytest.mark.parametrize("disabled", ("0", "false", "no", "off"))
def test_all_false_spellings_select_only_the_disabled_group(disabled):
    active, groups, _ = _dummy_selection(enable_gnss=disabled)

    assert active is True
    assert groups == ("gnss",)


def _resolve_camera(
    monkeypatch,
    *,
    enable_camera,
    enable_front,
    enable_rear,
    device_exists,
    front_executable,
    rear_executable,
):
    camera_device = "/virtual/econ-camera"
    real_exists = SENSING_LAUNCH.os.path.exists

    def fake_exists(path):
        if path == camera_device:
            return device_exists
        return real_exists(path)

    def fake_executable_exists(executable):
        if executable == "camera_front_publisher_node":
            return front_executable
        if executable == "camera_rear_publisher_node":
            return rear_executable
        raise AssertionError(f"unexpected executable: {executable}")

    monkeypatch.setattr(SENSING_LAUNCH.os.path, "exists", fake_exists)
    monkeypatch.setattr(
        SENSING_LAUNCH,
        "_camera_executable_exists",
        fake_executable_exists,
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "enable_camera": "true" if enable_camera else "false",
        "enable_front_camera": "true" if enable_front else "false",
        "enable_rear_camera": "true" if enable_rear else "false",
        "camera_device_path": camera_device,
    })
    for action in SENSING_LAUNCH._resolve_camera_enable(context):
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    return context


@pytest.mark.parametrize(
    (
        "enable_camera", "enable_front", "enable_rear", "device_exists",
        "front_executable", "rear_executable", "expected",
    ),
    (
        (False, True, True, True, True, True, (False, False, False)),
        (True, True, True, False, True, True, (False, False, False)),
        (True, True, True, True, True, True, (True, True, True)),
        (True, True, True, True, True, False, (True, True, False)),
        (True, True, True, True, False, True, (True, False, True)),
        (True, True, True, True, False, False, (False, False, False)),
        (True, True, False, True, True, False, (True, True, False)),
        (True, False, True, True, False, True, (True, False, True)),
    ),
)
def test_camera_effective_flags_are_resolved_per_physical_channel(
    monkeypatch,
    enable_camera,
    enable_front,
    enable_rear,
    device_exists,
    front_executable,
    rear_executable,
    expected,
):
    context = _resolve_camera(
        monkeypatch,
        enable_camera=enable_camera,
        enable_front=enable_front,
        enable_rear=enable_rear,
        device_exists=device_exists,
        front_executable=front_executable,
        rear_executable=rear_executable,
    )
    actual = tuple(
        context.launch_configurations[name] == "true"
        for name in (
            "enable_camera_effective",
            "enable_front_camera_effective",
            "enable_rear_camera_effective",
        )
    )
    assert actual == expected


def test_camera_include_uses_aggregate_and_per_channel_effective_flags():
    include = _camera_include()
    context = LaunchContext()
    context.launch_configurations.update({
        "enable_camera_effective": "true",
        "enable_front_camera_effective": "true",
        "enable_rear_camera_effective": "false",
        "camera_params_file": "/tmp/camera.yaml",
    })
    assert include.condition.evaluate(context) is True
    arguments = dict(include.launch_arguments)
    assert context.perform_substitution(
        arguments["enable_front_camera"]
    ) == "true"
    assert context.perform_substitution(
        arguments["enable_rear_camera"]
    ) == "false"

    context.launch_configurations["enable_camera_effective"] = "false"
    assert include.condition.evaluate(context) is False


@pytest.mark.parametrize(
    ("real_enabled", "dummy_enabled", "expected"),
    (
        ("true", "true", (True, False)),
        ("false", "true", (False, True)),
        ("false", "false", (False, False)),
    ),
)
def test_radar_real_and_dummy_launch_actions_are_mutually_exclusive(
    real_enabled, dummy_enabled, expected
):
    description = RADAR_LAUNCH.generate_launch_description()
    real_include = next(
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    )
    dummy_node = next(
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_executable == "radar_dummy_publisher.py"
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "enable_radar": real_enabled,
        "enable_radar_dummy_when_disabled": dummy_enabled,
    })

    actual = (
        real_include.condition.evaluate(context),
        dummy_node.condition.evaluate(context),
    )
    assert actual == expected
