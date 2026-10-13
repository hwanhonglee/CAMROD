"""HH_261008 - Keep tighter CARLA pre-parking correction out of native defaults."""

import copy
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import SetLaunchConfiguration
from launch.utilities import perform_substitutions
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
CONTROL_ROOT = REPO_ROOT / "camrod_control"
WRAPPER = PACKAGE_ROOT / "launch" / "camrod_carla_develop_site_geometry.launch.py"
DROP_ZONE_NODE = "/control/drop_zone_maneuver_controller"
APPROACH_PARAMETER = "parking_approach_position_tolerance_m"


def _load_wrapper():
    spec = importlib.util.spec_from_file_location("site_parking_launch", WRAPPER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_site_rewrite_changes_exactly_one_qualified_parameter():
    module = _load_wrapper()
    source = CONTROL_ROOT / "config" / "control.yaml"
    original_bytes = source.read_bytes()
    native = yaml.safe_load(original_bytes)
    expected = copy.deepcopy(native)
    assert expected[DROP_ZONE_NODE]["ros__parameters"][APPROACH_PARAMETER] == 0.20
    expected[DROP_ZONE_NODE]["ros__parameters"][APPROACH_PARAMETER] = 0.05

    rewrite = module.develop_site_geometry_control_params(str(CONTROL_ROOT))
    context = LaunchContext()
    assert rewrite.resolve_rewrites(context) == ({
        f"{DROP_ZONE_NODE}.ros__parameters.{APPROACH_PARAMETER}": "0.05",
    }, {})
    generated = Path(rewrite.perform(context))
    try:
        assert generated != source
        assert yaml.safe_load(generated.read_text(encoding="utf-8")) == expected
        assert source.read_bytes() == original_bytes
    finally:
        generated.unlink()

    # Both native deployment mirrors and final reverse-parking acceptance
    # remain independent of this CARLA-only approach precision experiment.
    for root in (CONTROL_ROOT / "config", REPO_ROOT / "camrod_bringup/config/control"):
        control = yaml.safe_load((root / "control.yaml").read_text(encoding="utf-8"))
        assert control[DROP_ZONE_NODE]["ros__parameters"][APPROACH_PARAMETER] == 0.20
        parking = yaml.safe_load((root / "parking.yaml").read_text(encoding="utf-8"))
        reverse = parking["/parking/reverse_parking_controller"]["ros__parameters"]
        assert reverse["station_axis_tolerance_m"] == 0.25
        assert reverse["maximum_reverse_distance_m"] == 5.0
        assert reverse["reverse_timeout_s"] == 30.0


def test_site_control_file_is_selected_before_nested_full_launch(monkeypatch):
    module = _load_wrapper()
    shares = {
        "camrod_carla_adapter": str(PACKAGE_ROOT),
        "camrod_control": str(CONTROL_ROOT),
    }
    monkeypatch.setattr(module, "get_package_share_directory", shares.__getitem__)
    entities = module.generate_launch_description().entities
    context = LaunchContext()
    setters = [
        (index, action) for index, action in enumerate(entities)
        if isinstance(action, SetLaunchConfiguration)
        and perform_substitutions(context, action.name) == "control_param_file"
    ]
    includes = [
        index for index, action in enumerate(entities)
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(setters) == len(includes) == 1
    index, setter = setters[0]
    assert index < includes[0]
    assert any(isinstance(action, LogInfo) for action in entities[index + 1:includes[0]])
    setter.execute(context)
    generated = Path(context.launch_configurations["control_param_file"])
    try:
        # Mirror the nested bringup declaration: an existing profile choice
        # must survive its ordinary package-default argument declaration.
        DeclareLaunchArgument(
            "control_param_file", default_value=str(CONTROL_ROOT / "config/control.yaml")
        ).execute(context)
        assert context.launch_configurations["control_param_file"] == str(generated)
        selected = yaml.safe_load(generated.read_text(encoding="utf-8"))
        assert selected[DROP_ZONE_NODE]["ros__parameters"][APPROACH_PARAMETER] == 0.05
    finally:
        generated.unlink()

    for name in ("camrod_carla_full.launch.py", "camrod_carla_woraksan_tuned.launch.py"):
        source = (PACKAGE_ROOT / "launch" / name).read_text(encoding="utf-8")
        assert "develop_site_geometry_control_params" not in source
        assert APPROACH_PARAMETER not in source
