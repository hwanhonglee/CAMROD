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


def _scope_includes(package: str, *args: str) -> bool:
    source = BUILD_WRAPPER.read_text(encoding="utf-8")
    function = _function_source(source, "_build_scope_includes_pkg")
    script = f"""
set -euo pipefail
SRC_ROOT={str(SRC_ROOT)!r}
{function}
if _build_scope_includes_pkg "$1" "${{@:2}}"; then
  printf included
else
  printf excluded
fi
"""
    result = subprocess.run(
        ["bash", "-c", script, "build-policy-test", package, *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout == "included"


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


def test_ui_frontend_build_scope_honors_explicit_select_and_skip() -> None:
    assert _scope_includes("camrod_ui")
    assert _scope_includes(
        "camrod_ui", "--packages-select", "camrod_planning", "camrod_ui"
    )
    assert not _scope_includes(
        "camrod_ui", "--packages-select", "camrod_planning"
    )
    assert not _scope_includes("camrod_ui", "--packages-skip", "camrod_ui")


def test_ui_frontend_build_scope_honors_packages_up_to_dependency_closure() -> None:
    assert _scope_includes(
        "camrod_ui", "--packages-up-to", "camrod_bringup"
    )
    assert not _scope_includes(
        "camrod_ui", "--packages-up-to", "camrod_planning"
    )
    # Colcon combines selectors. An explicit selection that excludes camrod_ui
    # must remain authoritative even when another target depends on the UI.
    assert not _scope_includes(
        "camrod_ui",
        "--packages-up-to",
        "camrod_bringup",
        "--packages-select",
        "camrod_planning",
    )


def test_ui_frontend_install_verification_is_content_and_freshness_closed() -> None:
    source = BUILD_WRAPPER.read_text(encoding="utf-8")
    fingerprint = _function_source(
        source, "_camrod_ui_frontend_input_fingerprint"
    )
    verification = _function_source(source, "_verify_camrod_ui_frontend_install")

    for contract in (
        "find src public -type f -print0",
        "package.json package-lock.json",
        "camrod-build-env.json",
        "sort -z",
        "sha256sum",
    ):
        assert contract in fingerprint

    for contract in (
        '"${frontend_source}" -nt "${source_bundle_path}"',
        'cmp -s "${source_bundle_path}" "${installed_bundle_path}"',
        '_camrod_ui_frontend_input_fingerprint "${frontend_dir}"',
        'cmp -s "${source_fingerprint}" "${installed_fingerprint}"',
        "grep -Fq 'operator-open-destination'",
        "frontend bundle is older than App.js",
        "frontend bundle does not match current src/public/manifests",
        "installed frontend bundle content differs from source build",
        "frontend canonical build environment differs from source/install evidence",
    ):
        assert contract in verification


def test_ui_frontend_build_environment_is_canonical_and_rejects_dotenv(
    tmp_path: Path,
) -> None:
    source = BUILD_WRAPPER.read_text(encoding="utf-8")
    function = _function_source(
        source, "_prepare_camrod_ui_build_environment"
    )
    frontend = tmp_path / "frontend"
    frontend.mkdir()
    script = f"""
set -euo pipefail
log() {{ :; }}
{function}
export PUBLIC_URL=/ambient
export REACT_APP_OPERATING_HOURS_GATE_ENABLED=true
export REACT_APP_OPERATING_HOURS_START=18
export REACT_APP_OPERATING_HOURS_END=19
export REACT_APP_UNRELATED=host-dependent
_prepare_camrod_ui_build_environment "$1"
printf '%s\\0' \
  "$PUBLIC_URL" \
  "$REACT_APP_OPERATING_HOURS_GATE_ENABLED" \
  "$REACT_APP_OPERATING_HOURS_START" \
  "$REACT_APP_OPERATING_HOURS_END" \
  "${{REACT_APP_UNRELATED+x}}" \
  "$BUILD_PATH" "$NODE_ENV" "$GENERATE_SOURCEMAP"
"""
    result = subprocess.run(
        ["bash", "-c", script, "ui-env-test", str(frontend)],
        check=True,
        capture_output=True,
    )
    assert result.stdout.decode().split("\0") == [
        "",
        "false",
        "3",
        "23",
        "",
        "build",
        "production",
        "false",
        "",
    ]

    (frontend / ".env.production").write_text(
        "PUBLIC_URL=/noncanonical\n", encoding="utf-8"
    )
    rejected = subprocess.run(
        ["bash", "-c", script, "ui-env-test", str(frontend)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert rejected.returncode != 0


def test_wrapper_pins_colcon_outputs_outside_source_checkout() -> None:
    """Following maintained build docs must not recreate src/build/install/log."""
    source = BUILD_WRAPPER.read_text(encoding="utf-8")

    # HH_260825 - Raw colcon defaults all three bases to its current directory.
    # Require both the workspace-root cwd and explicit bases so a later refactor
    # cannot silently reintroduce generated trees below the source checkout.
    assert 'cd "${WS_ROOT}"' in source
    assert '--log-base "${WS_ROOT}/log" build' in source
    assert '--build-base  "${WS_ROOT}/build"' in source
    assert '--install-base "${WS_ROOT}/install"' in source

    for relative_path in (
        "camrod_common/README.md",
        "camrod_ui/README.md",
        "camrod_voice/README.md",
    ):
        readme = (SRC_ROOT / relative_path).read_text(encoding="utf-8")
        assert "./colcon_build.sh --packages-select" in readme
        assert "colcon build --packages-select" not in readme


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
