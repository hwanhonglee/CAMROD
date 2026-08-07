"""Regression tests for intentional hardware-dummy diagnostic contracts."""

from pathlib import Path

import yaml


_BRINGUP_ROOT = Path(__file__).resolve().parents[1]
_WORKSPACE_SRC = _BRINGUP_ROOT.parent
_SYSTEM_ROOT = _WORKSPACE_SRC / "camrod_system"

_CHECKER_FILES = (
    ("sensing", "camera_checker.yaml"),
    ("sensing", "gnss_checker.yaml"),
    ("sensing", "imu_checker.yaml"),
    ("sensing", "lidar_checker.yaml"),
    ("sensing", "radar_checker.yaml"),
    ("sensing", "velocity_converter_checker.yaml"),
    ("sensing", "wheel_odometry_checker.yaml"),
    ("localization", "localization_gnss_checker.yaml"),
)


def _checker_params(category, filename):
    path = (
        _SYSTEM_ROOT
        / "config"
        / "diagnostics"
        / "default"
        / category
        / filename
    )
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    node = next(iter(data.values()))
    return node["ros__parameters"]


# HH_260729 - Full bringup deployment overrides must preserve the package
# checker's exact dummy heartbeat paths and freshness limits.
def test_dummy_checker_configs_are_byte_identical_bringup_mirrors():
    for category, filename in _CHECKER_FILES:
        system_path = (
            _SYSTEM_ROOT
            / "config"
            / "diagnostics"
            / "default"
            / category
            / filename
        )
        bringup_path = (
            _BRINGUP_ROOT
            / "config"
            / "system"
            / "diagnostics"
            / "default"
            / category
            / filename
        )
        assert bringup_path.read_bytes() == system_path.read_bytes()


def test_every_hardware_source_has_an_explicit_freshness_contract():
    expected = {
        ("sensing", "camera_checker.yaml"): {
            "econ_front": "/sensing/camera/econ_front/dummy_active",
            "econ_rear": "/sensing/camera/econ_rear/dummy_active",
        },
        ("sensing", "gnss_checker.yaml"): {
            "main": "/sensing/gnss/dummy_active",
        },
        ("sensing", "imu_checker.yaml"): {
            "main": "/sensing/imu/dummy_active",
        },
        ("sensing", "lidar_checker.yaml"): {
            "main": "/sensing/lidar/dummy_active",
            "filtered": "/sensing/lidar/dummy_active",
        },
        ("sensing", "wheel_odometry_checker.yaml"): {
            "main": "/platform/dummy_active",
        },
        ("localization", "localization_gnss_checker.yaml"): {
            "main": "/sensing/gnss/dummy_active",
        },
    }

    for checker, sources in expected.items():
        params = _checker_params(*checker)
        for source, topic in sources.items():
            assert params[source]["dummy_active_topic"] == topic
            assert params[source]["dummy_active_timeout_s"] == 1.0

    velocity = _checker_params(
        "sensing", "velocity_converter_checker.yaml"
    )
    assert velocity["platform_dummy_active_topic"] == "/platform/dummy_active"
    assert velocity["platform_dummy_active_timeout_s"] == 1.0
    assert velocity["imu_dummy_active_topic"] == "/sensing/imu/dummy_active"
    assert velocity["imu_dummy_active_timeout_s"] == 1.0

    # HH_260729 - The global marker covers enable_radar:=false, while each
    # channel marker covers only its sensor_enabled[i]:=false replacement.
    radar = _checker_params("sensing", "radar_checker.yaml")
    assert radar["dummy_active_topic"] == "/sensing/radar/dummy_active"
    assert radar["dummy_active_timeout_s"] == 1.0
    for name in radar["radar_names"]:
        lower_name = name.lower()
        assert (
            radar[name]["dummy_active_topic"]
            == f"/sensing/radar/{lower_name}/dummy_active"
        )
        assert radar[name]["dummy_active_timeout_s"] == 1.0


# HH_260807 - Diagnostics must reject the former 1 Hz rover profile while the
# current physical validation run intentionally uses 200 ms receiver epochs.
def test_physical_gnss_checkers_expect_five_hz():
    sensing = _checker_params("sensing", "gnss_checker.yaml")
    localization = _checker_params(
        "localization", "localization_gnss_checker.yaml"
    )

    assert sensing["main"]["expected_hz"] == 5.0
    assert localization["main"]["expected_hz"] == 5.0


# HH_260807 - The raw sensor and motion-relevant ground-segmented output are
# both required to preserve the physical LiDAR's 10 Hz cadence end to end.
def test_physical_lidar_checker_expects_ten_hz_end_to_end():
    sensing = _checker_params("sensing", "lidar_checker.yaml")

    assert sensing["main"]["expected_hz"] == 10.0
    assert sensing["filtered"]["topic"] == "/sensing/lidar/points_filtered"
    assert sensing["filtered"]["expected_hz"] == 10.0


def test_checker_sources_keep_dummy_mode_explicit_and_fail_visible():
    source_names = (
        "camera_checker_node.cpp",
        "gnss_checker_node.cpp",
        "imu_checker_node.cpp",
        "lidar_checker_node.cpp",
        "localization_gnss_checker_node.cpp",
        "radar_checker_node.cpp",
        "velocity_converter_checker_node.cpp",
        "wheel_odometry_checker_node.cpp",
    )
    for source_name in source_names:
        source = (
            _SYSTEM_ROOT / "src" / "diagnostics" / source_name
        ).read_text(encoding="utf-8")
        assert "DUMMY DATA" in source
        assert ".isActive" in source
        assert "DiagnosticStatus::WARN" in source


def test_radar_checker_keeps_global_and_channel_dummy_scopes_separate():
    source = (
        _SYSTEM_ROOT / "src" / "diagnostics" / "radar_checker_node.cpp"
    ).read_text(encoding="utf-8")
    assert "global_dummy_monitor_.isActive" in source
    assert "radar.dummy_monitor.isActive" in source
    assert "global_dummy_active || channel_dummy_active" in source
    assert '"global+channel"' in source
    assert '"channel"' in source
