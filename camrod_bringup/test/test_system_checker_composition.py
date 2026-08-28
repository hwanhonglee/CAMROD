"""Lock the granular production checker-composition boundary."""

import importlib.util
from pathlib import Path
import sys
import tempfile

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
SYSTEM_LAUNCH = SRC_ROOT / "camrod_system" / "launch" / "system.launch.py"
BRINGUP_DEFAULTS = (
    SRC_ROOT / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
)
BRINGUP_IMPL = SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"


def _load_system_launch():
    spec = importlib.util.spec_from_file_location("camrod_system_launch", SYSTEM_LAUNCH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_every_checker_has_one_verified_runtime_boundary() -> None:
    """Optional composition must not remove or duplicate health coverage."""
    launch = _load_system_launch()
    specs = launch.CHECKER_NODE_SPECS

    assert len(specs) == 24
    assert len({spec[2] for spec in specs}) == 24
    assert set(launch.CHECKER_COMPONENT_PLUGINS) == {spec[2] for spec in specs}

    grouped_counts = {
        group: len(checker_names)
        for group, checker_names in launch.CHECKER_COMPONENT_GROUPS.items()
    }
    assert grouped_counts == {
        "hardware_sensing": 11,
        "localization": 6,
        "autonomy_topics": 6,
        "planning_lifecycle": 1,
    }
    default_composed = set().union(*(
        launch.CHECKER_COMPONENT_GROUPS[group]
        for group in launch.DEFAULT_CHECKER_COMPONENT_GROUPS
    ))
    default_standalone = [spec[2] for spec in specs if spec[2] not in default_composed]
    assert default_standalone == []


def test_component_group_parser_is_ordered_and_strict() -> None:
    launch = _load_system_launch()
    assert launch._parse_checker_component_groups(
        "autonomy_topics,hardware_sensing,autonomy_topics"
    ) == ("autonomy_topics", "hardware_sensing")

    try:
        launch._parse_checker_component_groups("hardware_sensing,unknown")
    except ValueError as error:
        assert "unknown" in str(error)
    else:
        raise AssertionError("unknown checker groups must fail at launch")


def test_sparse_diagnostics_profiles_follow_ordered_fallback_chain() -> None:
    launch = _load_system_launch()
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        carla = root / "carla"
        sim = root / "sim"
        default = root / "default"
        for profile in (carla, sim, default):
            (profile / "sensing").mkdir(parents=True)

        sim_file = sim / "sensing" / "imu_checker.yaml"
        default_file = default / "sensing" / "camera_checker.yaml"
        sim_file.write_text("sim", encoding="utf-8")
        default_file.write_text("default", encoding="utf-8")

        assert launch._profile_param_file(
            str(carla), (str(sim), str(default)), "sensing", "imu_checker.yaml"
        ) == str(sim_file)
        assert launch._profile_param_file(
            str(carla), (str(sim), str(default)), "sensing", "camera_checker.yaml"
        ) == str(default_file)

        carla_file = carla / "sensing" / "imu_checker.yaml"
        carla_file.write_text("carla", encoding="utf-8")
        assert launch._profile_param_file(
            str(carla), (str(sim), str(default)), "sensing", "imu_checker.yaml"
        ) == str(carla_file)


def test_composed_checkers_keep_standalone_entrypoints() -> None:
    """Composition must not remove per-checker field-debug commands."""
    launch = _load_system_launch()
    diagnostics_dir = SRC_ROOT / "camrod_system" / "src" / "diagnostics"

    for _category, executable, _name, _params in launch.CHECKER_NODE_SPECS:
        source = diagnostics_dir / executable.replace("_node", "_node.cpp")
        assert source.is_file(), executable
        assert "CAMROD_SYSTEM_CHECKER_ENTRYPOINT" in source.read_text(
            encoding="utf-8"
        )


def test_bringup_guards_and_forwards_checker_composition() -> None:
    """Bringup selects scoped checker groups and preserves explicit fallback."""
    defaults = yaml.safe_load(BRINGUP_DEFAULTS.read_text(encoding="utf-8"))
    system = defaults["bringup"]["system"]
    assert system["use_system_tools_container"] is True
    assert system["use_checker_components"] is True
    assert system["checker_component_groups"] == (
        "hardware_sensing,localization,autonomy_topics,planning_lifecycle"
    )
    assert system["checker_component_threads"] == 1
    assert system["diagnostics_profile_fallback"] == "default"

    source = BRINGUP_IMPL.read_text(encoding="utf-8")
    assert "'use_system_tools_container': lc['use_system_tools_container']" in source
    assert "'use_checker_components': lc['use_checker_components']" in source
    assert "'checker_component_groups': lc['checker_component_groups']" in source
    assert "'checker_component_threads': lc['checker_component_threads']" in source
    assert "'config_profile_fallback': lc['diagnostics_profile_fallback']" in source

    system_launch = SYSTEM_LAUNCH.read_text(encoding="utf-8")
    assert 'name="system_core_container"' in system_launch
    assert 'plugin="camrod_system::SystemDiagnosticNode"' in system_launch
    assert 'package="camrod_runtime"' in system_launch
    assert 'executable="scoped_component_container"' in system_launch
    assert 'executable="scoped_component_container_mt"' not in system_launch


def test_obstacle_cloud_checker_accepts_sensor_data_qos_publishers() -> None:
    """A diagnostic checker must match BEST_EFFORT LiDAR/CARLA clouds."""
    source = (
        SRC_ROOT
        / "camrod_system"
        / "src"
        / "diagnostics"
        / "perception_obstacle_checker_node.cpp"
    ).read_text(encoding="utf-8")

    point_cloud_branch = source.split(
        "// PointCloud2 (기본)", maxsplit=1
    )[1].split("RCLCPP_INFO", maxsplit=1)[0]
    assert "rclcpp::SensorDataQoS()" in point_cloud_branch
    assert "src->topic, rclcpp::QoS(10)" not in point_cloud_branch
