"""Lock the LiDAR preprocessor parameter-file and launch wiring contract."""

import ast
from pathlib import Path
import re

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
SENSING_CONFIG = (
    SRC_ROOT / "camrod_sensing" / "config" / "lidar" / "preprocessor.yaml"
)
BRINGUP_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "sensing" / "lidar"
    / "preprocessor.yaml"
)
PREPROCESSOR_SOURCE = (
    SRC_ROOT / "camrod_sensing" / "src" / "lidar_preprocessor_node.cpp"
)
LIDAR_DRIVER_LAUNCH = (
    SRC_ROOT / "camrod_sensing" / "launch" / "lidar_driver.launch.py"
)
GROUND_SEGMENTATION_CMAKE = (
    SRC_ROOT / "camrod_sensing" / "external" / "ground_segmentation_ros2"
    / "CMakeLists.txt"
)


def _declared_cpp_parameters() -> set[str]:
    source = PREPROCESSOR_SOURCE.read_text(encoding="utf-8")
    return set(re.findall(
        r'declare_parameter(?:<[^>]+>)?\s*\(\s*"([^"]+)"',
        source,
    ))


def _function_return_dict_keys(tree: ast.AST, function_name: str) -> set[str]:
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != function_name:
            continue
        for statement in node.body:
            if not isinstance(statement, ast.Return):
                continue
            assert isinstance(statement.value, ast.Dict)
            return {
                key.value
                for key in statement.value.keys
                if isinstance(key, ast.Constant) and isinstance(key.value, str)
            }
    raise AssertionError(f"missing function: {function_name}")


def _preprocessor_parameter_lists(tree: ast.AST) -> list[ast.List]:
    parameter_lists = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id not in {"ComposableNode", "Node"}:
            continue
        keywords = {item.arg: item.value for item in node.keywords if item.arg}
        name = keywords.get("name")
        if not isinstance(name, ast.Constant) or name.value != "lidar_preprocessor":
            continue
        parameters = keywords.get("parameters")
        assert isinstance(parameters, ast.List)
        parameter_lists.append(parameters)
    return parameter_lists


def test_package_config_matches_node_fqn_and_declared_schema() -> None:
    config = yaml.safe_load(SENSING_CONFIG.read_text(encoding="utf-8"))
    assert set(config) == {"/sensing/lidar/lidar_preprocessor"}
    parameters = config["/sensing/lidar/lidar_preprocessor"]["ros__parameters"]

    assert set(parameters) == _declared_cpp_parameters()
    assert parameters["input_topic"] == "vanjee/points_raw"
    assert parameters["output_topic"] == "filtered_cloud"
    assert parameters["max_process_hz"] == 0.0


def test_bringup_preprocessor_config_is_byte_identical_mirror() -> None:
    assert BRINGUP_CONFIG.read_bytes() == SENSING_CONFIG.read_bytes()


def test_standalone_and_composed_nodes_load_config_before_wiring_overrides() -> None:
    tree = ast.parse(LIDAR_DRIVER_LAUNCH.read_text(encoding="utf-8"))
    assert _function_return_dict_keys(tree, "_lidar_runtime_overrides") == {
        "input_topic",
        "output_topic",
    }

    parameter_lists = _preprocessor_parameter_lists(tree)
    assert len(parameter_lists) == 2
    for parameters in parameter_lists:
        assert len(parameters.elts) == 2
        config_file, runtime_overrides = parameters.elts
        assert isinstance(config_file, ast.Call)
        assert isinstance(config_file.func, ast.Name)
        assert config_file.func.id == "LaunchConfiguration"
        assert isinstance(config_file.args[0], ast.Constant)
        assert config_file.args[0].value == "lidar_preprocessor_param_file"
        assert isinstance(runtime_overrides, ast.Call)
        assert isinstance(runtime_overrides.func, ast.Name)
        assert runtime_overrides.func.id == "_lidar_runtime_overrides"


def test_aggregate_and_bringup_forward_the_same_parameter_file() -> None:
    lidar_launch = (
        SRC_ROOT / "camrod_sensing" / "launch" / "lidar.launch.py"
    ).read_text(encoding="utf-8")
    sensing_launch = (
        SRC_ROOT / "camrod_sensing" / "launch" / "sensing.launch.py"
    ).read_text(encoding="utf-8")
    bringup_impl = (
        SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    defaults = yaml.safe_load((
        SRC_ROOT / "camrod_bringup" / "config" / "bringup"
        / "launch_defaults.yaml"
    ).read_text(encoding="utf-8"))["bringup"]["sensing"]

    assert "lidar_preprocessor_param_file" in lidar_launch
    assert "lidar_preprocessor_param_file" in sensing_launch
    assert "'lidar_preprocessor_param_file'" in bringup_impl
    assert defaults["lidar_preprocessor_param_file"] == (
        "sensing/lidar/preprocessor.yaml"
    )


def test_ground_segmentation_standalone_uses_multithreaded_executor() -> None:
    """Keep TF reception live while standalone cloud callbacks wait on TF."""
    cmake = GROUND_SEGMENTATION_CMAKE.read_text(encoding="utf-8")
    registration = re.search(
        r"rclcpp_components_register_node\(\s*"
        r"ground_segmentation_ros2_component(?P<body>.*?)\n\)",
        cmake,
        flags=re.DOTALL,
    )
    assert registration is not None
    assert "EXECUTOR MultiThreadedExecutor" in registration.group("body")
