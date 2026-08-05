"""Lock the stable production default and bench checker composition boundary."""

import importlib.util
from pathlib import Path
import sys

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
    """Composition fallback must not remove or duplicate health coverage."""
    launch = _load_system_launch()
    specs = launch.CHECKER_NODE_SPECS

    assert len(specs) == 24
    assert len({spec[2] for spec in specs}) == 24
    assert set(launch.CHECKER_COMPONENT_PLUGINS) == {spec[2] for spec in specs}

    grouped_counts = {
        group: sum(spec[0] in categories for spec in specs)
        for group, categories in launch.CHECKER_COMPONENT_GROUPS.items()
    }
    assert grouped_counts == {
        "hardware_sensing": 11,
        "autonomy": 7,
    }
    composed_categories = set().union(*launch.CHECKER_COMPONENT_GROUPS.values())
    standalone = [spec[2] for spec in specs if spec[0] not in composed_categories]
    # HH_260805 - This is the reduced bench topology. Production keeps all 24
    # checkers standalone because other composed groups also failed repeated
    # full-stack shutdown tests on Humble.
    assert standalone == [
        "localization_gnss_checker",
        "localization_mode_checker",
        "localization_pose_checker",
        "localization_init_checker",
        "localization_source_checker",
        "localization_lanelet_checker",
    ]


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
    """Composition stays available without becoming the unstable default."""
    defaults = yaml.safe_load(BRINGUP_DEFAULTS.read_text(encoding="utf-8"))
    system = defaults["bringup"]["system"]
    # HH_260805 - Full-stack Humble shutdown tests reproduced intermittent
    # component-library -11 exits; all standalone checker exits were clean.
    assert system["use_checker_components"] is False
    assert system["checker_component_threads"] == 1

    source = BRINGUP_IMPL.read_text(encoding="utf-8")
    assert "'use_checker_components': lc['use_checker_components']" in source
    assert "'checker_component_threads': lc['checker_component_threads']" in source

    system_launch = SYSTEM_LAUNCH.read_text(encoding="utf-8")
    assert 'executable="component_container"' in system_launch
    assert 'executable="component_container_mt"' not in system_launch
