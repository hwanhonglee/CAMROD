"""Contract tests for the workspace build wrapper's CMake build-type policy."""

from pathlib import Path
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
BUILD_WRAPPER = SRC_ROOT / "colcon_build.sh"


def _function_source(source: str, name: str) -> str:
    """Extract one top-level Bash function without sourcing the build wrapper."""
    start = source.index(f"{name}() {{")
    end = source.index("\n}\n", start) + len("\n}\n")
    return source[start:end]


def _prepared_args(*args: str) -> tuple[str, list[str]]:
    source = BUILD_WRAPPER.read_text(encoding="utf-8")
    functions = "\n".join(
        _function_source(source, name)
        for name in (
            "_has_explicit_cmake_build_type",
            "_prepare_colcon_build_args",
        )
    )
    script = f"""
set -euo pipefail
{functions}
_prepare_colcon_build_args "$@"
printf '%s\\0' "${{COLCON_BUILD_TYPE_SOURCE}}" "${{COLCON_BUILD_ARGS[@]}}"
"""
    result = subprocess.run(
        ["bash", "-c", script, "build-policy-test", *args],
        check=True,
        capture_output=True,
    )
    fields = result.stdout.decode("utf-8").split("\0")
    assert fields[-1] == ""
    return fields[0], fields[1:-1]


def test_normal_build_defaults_to_release() -> None:
    source, args = _prepared_args("--packages-select", "ground_segmentation_ros2")

    assert source == "default-release"
    assert args == [
        "--packages-select",
        "ground_segmentation_ros2",
        "--cmake-args",
        "-DCMAKE_BUILD_TYPE=Release",
    ]


def test_release_default_joins_existing_cmake_argument_group() -> None:
    source, args = _prepared_args(
        "--packages-select",
        "ground_segmentation_ros2",
        "--cmake-args",
        "-DBUILD_TESTING=OFF",
    )

    assert source == "default-release"
    cmake_index = args.index("--cmake-args")
    assert args[cmake_index + 1:cmake_index + 3] == [
        "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_TESTING=OFF",
    ]


def test_explicit_debug_and_relwithdebinfo_remain_authoritative() -> None:
    for original in (
        ["--cmake-args", "-DCMAKE_BUILD_TYPE=Debug", "-DBUILD_TESTING=ON"],
        [
            "--cmake-args",
            "-DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo",
            "-DBUILD_TESTING=ON",
        ],
        ["--cmake-args=-DCMAKE_BUILD_TYPE=Debug"],
        ["--cmake-args", "-D", "CMAKE_BUILD_TYPE=RelWithDebInfo"],
    ):
        source, args = _prepared_args(*original)

        assert source == "user"
        assert args == original
        assert "-DCMAKE_BUILD_TYPE=Release" not in args


def test_inline_cmake_argument_is_preserved_when_adding_release() -> None:
    source, args = _prepared_args("--cmake-args=-DBUILD_TESTING=OFF")

    assert source == "default-release"
    assert args == [
        "--cmake-args",
        "-DBUILD_TESTING=OFF",
        "-DCMAKE_BUILD_TYPE=Release",
    ]


def test_only_explicit_virtual_carla_underlays_survive_sanitization(
    tmp_path: Path,
) -> None:
    source = BUILD_WRAPPER.read_text(encoding="utf-8")
    functions = "\n".join(
        _function_source(source, name)
        for name in ("_is_allowed_extra_prefix", "sanitize_path_var")
    )
    workspace = tmp_path / "camrod_ws"
    bridge_install = (
        tmp_path / "ranger" / ".work" / "ros-bridge-ws" / "install"
    )
    ranger_install = tmp_path / "ranger" / "ros_ws" / "install"
    unrelated = tmp_path / "ambient_autoware" / "install"
    bridge_package = bridge_install / "carla_ros_bridge"
    ranger_package = ranger_install / "carla_extended_ackermann_control"
    for path in (bridge_package, ranger_package, unrelated):
        path.mkdir(parents=True)

    script = f"""
set -euo pipefail
{functions}
WS_ROOT="$1"
export CAMROD_EXTRA_PREFIX_ROOTS="$2:$3"
export TEST_PREFIX_PATH="$4:$5:$6"
sanitize_path_var TEST_PREFIX_PATH
printf '%s' "${{TEST_PREFIX_PATH}}"
"""
    result = subprocess.run(
        [
            "bash",
            "-c",
            script,
            "build-prefix-test",
            str(workspace),
            str(bridge_install),
            str(ranger_install),
            str(bridge_package),
            str(ranger_package),
            str(unrelated),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.split(":") == [
        str(bridge_package),
        str(ranger_package),
    ]
