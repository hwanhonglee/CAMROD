"""Contracts for the full CAMROD-on-CARLA composition."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent


def test_full_launch_keeps_carla_lifecycle_external_and_enables_full_bringup():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    assert 'get_package_share_directory("carla_ros_bridge")' not in source
    assert 'get_package_share_directory("carla_spawn_objects")' not in source
    assert '"sim": "true"' in source
    assert '"external_simulator": "true"' in source
    assert '"sim_platform_status_enable": "true"' in source
    assert '"clean_before_launch": "false"' in source
    assert '"platform_odometry_topic": "/odom"' in source
    assert '"platform_ranger_bridge_enable": "true"' in source
    assert '"planning_nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert '"enable_api_ui": LaunchConfiguration("enable_api_ui")' in source
    assert 'controller_share, "full_stack.launch.py"' in source
    assert 'controller_share, "launch", "full_stack.launch.py"' not in source


def test_fake_sensor_rate_isolated_from_platform_heartbeat_launch_scope():
    """A sibling 5 Hz include must not down-rate 10 Hz readiness topics."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    heartbeat_launch = (
        PACKAGE_ROOT / "launch" / "platform_heartbeat.launch.py"
    ).read_text(encoding="utf-8")
    fake_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    bringup_impl = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert (
        'DeclareLaunchArgument("publish_rate_hz", default_value="5.0")'
        in heartbeat_launch
    )
    assert '"sim_fake_sensor_publish_rate_hz": "10.0"' in full_launch
    assert "'fake_sensor_publish_rate_hz'" in fake_launch
    assert "LaunchConfiguration('fake_sensor_publish_rate_hz')" in fake_launch
    assert "LaunchConfiguration('publish_rate_hz')" not in fake_launch
    assert "'sim_fake_sensor_publish_rate_hz'" in bringup_impl
    assert "'fake_sensor_publish_rate_hz': lc[" in bringup_impl
    assert defaults["sim"]["fake_sensor_publish_rate_hz"] == 10.0


def test_subset_launch_forwards_multi_pose_selector_tree():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    assert '"nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert (
        '"nav2_bt_xml_nav_through_poses": LaunchConfiguration('
        in source
    )


def test_composition_launches_have_no_host_absolute_paths_and_use_env_gates():
    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert "/home/hong" not in source
        assert "/home/nvidia" not in source
        assert "RANGER_CARLA_ROOT" in source
        assert "RANGER_BASELINE_MANIFEST" in source
        assert "RANGER_PHYSICAL_MANIFEST" in source
        assert "RANGER_CARLA_PYTHON_EGG" in source
        assert "RANGER_PYTHON_EGG_CACHE" in source
        assert "ranger_ros_backend_gate_20260814" not in source
        assert "ranger_physical_4ws_acceptance_gate_20260823" not in source


def test_composition_launches_forward_the_explicit_camrod_map_path():
    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert '"camrod_map_path", default_value=camrod_map_path' in source
        assert '"map_path": LaunchConfiguration("camrod_map_path")' in source


def test_sensor_relay_uses_camrod_canonical_topics():
    source = (
        PACKAGE_ROOT
        / "src"
        / "camrod_carla_adapter"
        / "sensor_relay_node.py"
    ).read_text(encoding="utf-8")
    for topic in (
        "/sensing/camera/econ_front/image_raw",
        "/sensing/camera/econ_front/image_rect",
        "/sensing/camera/econ_front/camera_info",
        "/sensing/camera/econ_rear/image_raw",
        "/sensing/camera/econ_rear/image_rect",
        "/sensing/camera/econ_rear/camera_info",
        "/sensing/lidar/vanjee/points_raw",
    ):
        assert topic in source
