"""Lock the robot_center_link migration and its physical-geometry invariants."""

import ast
from pathlib import Path

import pytest
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_SENSOR_CONFIG = (
    SRC_ROOT / "camrod_sensor_kit" / "config" / "robot_params.yaml"
)
BRINGUP_SENSOR_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "sensor_kit" / "robot_params.yaml"
)
PACKAGE_LOCALIZATION_CONFIG = (
    SRC_ROOT / "camrod_localization" / "config" / "source" / "input_adapter.yaml"
)
BRINGUP_LOCALIZATION_CONFIG = (
    SRC_ROOT
    / "camrod_bringup"
    / "config"
    / "localization"
    / "source"
    / "input_adapter.yaml"
)
CENTER_OFFSET_M = 0.443
GNSS_LEFT_ANTENNA_OFFSET_M = 0.45
MEASURED_BODY_EXTENTS_M = {
    "front": 0.70837,
    "rear": 0.68323,
    "left": 0.53505,
    "right": 0.53495,
}
BOUNDARY_SHAPE_M = {
    "front_taper": 0.12,
    "front_shoulder_depth": 0.12,
    "corner_radius": 0.05,
    "corner_samples": 4,
}

# HH_260803 - These are the deployed rear-axle-relative X values before the
# frame migration. Keeping the unchanged mounts here makes their conversion auditable.
# HH_260807 - The four side radars were remeasured directly from robot_center_link,
# so they are locked separately below instead of preserving their superseded X values.
LEGACY_SENSOR_X_M = {
    ("imu",): 1.131,
    ("lidar",): 1.20636,
    ("camera", "front"): 1.20637,
    ("camera", "rear"): -0.17633,
    ("radar", "front1"): 1.07087,
    ("radar", "front2"): 1.07087,
    ("radar", "rear"): -0.17433,
}

REMEASURED_SIDE_RADAR_XY_M = {
    "left1": (0.38, 0.53),
    "left2": (-0.38, 0.53),
    "right1": (0.38, -0.53),
    "right2": (-0.38, -0.53),
}


def _yaml(path: Path) -> dict:
    """Load one ROS parameter YAML file."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _wildcard_parameters(path: Path) -> dict:
    """Return parameters from a /** ROS parameter file."""
    return _yaml(path)["/**"]["ros__parameters"]


def _nested(mapping: dict, keys: tuple[str, ...]) -> dict:
    """Return a nested mapping selected by keys."""
    for key in keys:
        mapping = mapping[key]
    return mapping


def test_sensor_config_mirror_is_byte_synchronized() -> None:
    """Full bringup must deploy the same canonical geometry as the package."""
    assert PACKAGE_SENSOR_CONFIG.read_bytes() == BRINGUP_SENSOR_CONFIG.read_bytes()


def test_center_offset_and_measured_body_are_auditable() -> None:
    """Keep the axle midpoint and the fabrication-inclusive measured envelope."""
    robot = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)["robot"]

    assert robot["center_offset_from_rear_axle"] == pytest.approx(CENTER_OFFSET_M)
    assert robot["wheelbase"] / 2.0 == pytest.approx(CENTER_OFFSET_M)
    for side, measured_extent in MEASURED_BODY_EXTENTS_M.items():
        assert robot["body_extents"][side] == pytest.approx(measured_extent)
    assert (
        robot["body_extents"]["front"] + robot["body_extents"]["rear"]
    ) == pytest.approx(robot["length"])
    assert (
        robot["body_extents"]["left"] + robot["body_extents"]["right"]
    ) == pytest.approx(robot["width"])


def test_planning_margin_is_ten_centimeters_on_every_side() -> None:
    """The recoverable planning envelope adds 10 cm around the measured body."""
    extents = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)["robot"]["body_extents"]

    assert extents["planning_margin"] == pytest.approx(0.10)
    assert extents["planning_lateral_margin"] == pytest.approx(0.10)
    assert extents["front"] + extents["planning_margin"] == pytest.approx(0.80837)
    assert extents["rear"] + extents["planning_margin"] == pytest.approx(0.78323)
    assert extents["left"] + extents["planning_lateral_margin"] == pytest.approx(
        0.63505
    )
    assert extents["right"] + extents["planning_lateral_margin"] == pytest.approx(
        0.63495
    )


@pytest.mark.parametrize("keys,legacy_x", LEGACY_SENSOR_X_M.items())
def test_sensor_x_shift_preserves_physical_mount(
    keys: tuple[str, ...], legacy_x: float
) -> None:
    """Each sensor keeps its mount after changing only the reference origin."""
    parameters = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)
    pose = _nested(parameters, keys)

    assert pose["x"] == pytest.approx(legacy_x - CENTER_OFFSET_M)


@pytest.mark.parametrize("sensor,expected_xy", REMEASURED_SIDE_RADAR_XY_M.items())
def test_remeasured_side_radar_mounts_use_robot_center(
    sensor: str, expected_xy: tuple[float, float]
) -> None:
    """The field-measured side mounts are expressed directly in robot_center_link."""
    pose = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)["radar"][sensor]

    assert (pose["x"], pose["y"]) == pytest.approx(expected_xy)
    assert pose["z"] == pytest.approx(0.29013)


def test_gnss_left_antenna_and_center_correction_share_one_lever_arm() -> None:
    """TF exposes the left antenna while localization publishes robot center."""
    gnss = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)["gnss"]
    localization = _wildcard_parameters(PACKAGE_LOCALIZATION_CONFIG)

    assert gnss["x"] == pytest.approx(0.0)
    assert gnss["y"] == pytest.approx(GNSS_LEFT_ANTENNA_OFFSET_M)
    assert gnss["z"] == pytest.approx(0.0)
    assert localization["enable_gnss_lever_arm_correction"] is True
    assert localization["gnss_antenna_offset_x_m"] == pytest.approx(gnss["x"])
    assert localization["gnss_antenna_offset_y_m"] == pytest.approx(gnss["y"])
    assert localization["gnss_lever_arm_require_fresh_heading"] is True
    assert localization["enable_gnss_lever_arm_ekf_heading_fallback"] is True
    assert localization["gnss_lever_arm_fallback_anchor_max_age_s"] == pytest.approx(3.0)
    assert localization["gnss_lever_arm_fallback_history_s"] == pytest.approx(5.0)
    assert localization["gnss_lever_arm_fallback_match_tolerance_s"] == pytest.approx(0.2)
    assert localization["gnss_lever_arm_fallback_max_ekf_yaw_covariance"] == pytest.approx(1.0)
    assert PACKAGE_LOCALIZATION_CONFIG.read_bytes() == (
        BRINGUP_LOCALIZATION_CONFIG.read_bytes()
    )


def test_real_ekf_lag_window_and_deployment_mirror() -> None:
    """Keep the measured-delay rewind window identical in package and bringup."""
    package_path = SRC_ROOT / "camrod_localization" / "config" / "filter" / "ekf.yaml"
    bringup_path = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "localization"
        / "filter"
        / "ekf.yaml"
    )

    assert package_path.read_bytes() == bringup_path.read_bytes()
    parameters = _wildcard_parameters(package_path)
    # HH_260806 - Cover the measured 747.6 ms selected-pose age while keeping
    # the real EKF and Nav2 controller on the same 20 Hz prediction cadence.
    assert parameters["frequency"] == pytest.approx(20.0)
    assert parameters["smooth_lagged_data"] is True
    assert parameters["predict_to_current_time"] is True
    assert parameters["history_length"] == pytest.approx(1.0)


def test_sim_gnss_models_the_same_raw_antenna_and_heading_contract() -> None:
    """Full simulation must exercise rather than bypass lever-arm correction."""
    localization = _wildcard_parameters(PACKAGE_LOCALIZATION_CONFIG)
    fake = _yaml(SRC_ROOT / "camrod_bringup" / "config" / "sim" / "fake_sensors.yaml")
    fake = fake["/bringup/fake_sensor_publisher"]["ros__parameters"]

    assert fake["gnss_antenna_offset_x_m"] == pytest.approx(
        localization["gnss_antenna_offset_x_m"]
    )
    assert fake["gnss_antenna_offset_y_m"] == pytest.approx(
        localization["gnss_antenna_offset_y_m"]
    )
    assert (
        fake["gnss_heading_raw_yaw_bias_deg"]
        + localization["gnss_heading_yaw_offset_deg"]
    ) == pytest.approx(0.0)


def test_nav2_and_gate_share_center_based_planning_footprint() -> None:
    """Planning and final command authorization must use one occupied boundary."""
    # HH_260809 - This is the generated 10 cm offset of the canonical tapered,
    # rounded physical body, rounded to the six decimals stored in Nav2 YAML.
    expected = [
        [0.764436, 0.500405], [0.783090, 0.477675],
        [0.796952, 0.451742], [0.805488, 0.423603],
        [0.808370, 0.394339], [0.808370, -0.394239],
        [0.805488, -0.423503], [0.796952, -0.451642],
        [0.783090, -0.477575], [0.764436, -0.500305],
        [0.673725, -0.591016], [0.650995, -0.609670],
        [0.625062, -0.623532], [0.596923, -0.632068],
        [0.567659, -0.634950], [-0.633230, -0.634950],
        [-0.690633, -0.623532], [-0.739296, -0.591016],
        [-0.771812, -0.542353], [-0.783230, -0.484950],
        [-0.783230, 0.485050], [-0.771812, 0.542453],
        [-0.739296, 0.591116], [-0.690633, 0.623632],
        [-0.633230, 0.635050], [0.567659, 0.635050],
        [0.596923, 0.632168], [0.625062, 0.623632],
        [0.650995, 0.609770], [0.673725, 0.591116],
    ]
    nav2 = _yaml(
        SRC_ROOT / "camrod_planning" / "config" / "nav2_vehicle.yaml"
    )
    local_costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    gate = _yaml(
        SRC_ROOT / "camrod_control" / "config" / "cmd_vel_safety_gate.yaml"
    )["/**"]["ros__parameters"]
    launch_control = _yaml(
        SRC_ROOT / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
    )["bringup"]["control"]

    assert ast.literal_eval(local_costmap["footprint"]) == expected
    assert ast.literal_eval(global_costmap["footprint"]) == expected
    extents = _wildcard_parameters(PACKAGE_SENSOR_CONFIG)["robot"]["body_extents"]
    for key, value in BOUNDARY_SHAPE_M.items():
        assert extents[key] == pytest.approx(value)
    assert gate["robot_base_frame"] == "robot_center_link"
    assert gate["lanelet_safety_body_hard_stop_enable"] is True
    assert gate["lanelet_safety_body_hard_stop_threshold"] == 100
    assert gate["lanelet_safety_body_front_m"] == pytest.approx(0.70837)
    assert gate["lanelet_safety_body_rear_m"] == pytest.approx(0.68323)
    assert gate["lanelet_safety_body_left_m"] == pytest.approx(0.53505)
    assert gate["lanelet_safety_body_right_m"] == pytest.approx(0.53495)
    assert gate["lanelet_safety_tapered_rounded_boundary_enable"] is True
    assert gate["lanelet_safety_boundary_front_taper_m"] == pytest.approx(0.12)
    assert gate["lanelet_safety_boundary_front_shoulder_depth_m"] == pytest.approx(
        0.12
    )
    assert gate["lanelet_safety_boundary_corner_radius_m"] == pytest.approx(0.05)
    assert gate["lanelet_safety_boundary_corner_samples"] == 4
    assert gate["lanelet_safety_footprint_front_m"] == pytest.approx(0.80837)
    assert gate["lanelet_safety_footprint_rear_m"] == pytest.approx(0.78323)
    assert gate["lanelet_safety_footprint_left_m"] == pytest.approx(0.63505)
    assert gate["lanelet_safety_footprint_right_m"] == pytest.approx(0.63495)
    for side, extent in {
        "front": 0.80837,
        "rear": 0.78323,
        "left": 0.63505,
        "right": 0.63495,
    }.items():
        assert launch_control[
            f"cmd_vel_gate_lanelet_safety_footprint_{side}_m"
        ] == pytest.approx(extent)
    assert launch_control[
        "cmd_vel_gate_lanelet_safety_tapered_rounded_boundary_enable"
    ] is True
    assert launch_control[
        "cmd_vel_gate_lanelet_safety_boundary_front_taper_m"
    ] == pytest.approx(0.12)
    assert launch_control[
        "cmd_vel_gate_lanelet_safety_boundary_front_shoulder_depth_m"
    ] == pytest.approx(0.12)
    assert launch_control[
        "cmd_vel_gate_lanelet_safety_boundary_corner_radius_m"
    ] == pytest.approx(0.05)
    assert launch_control["cmd_vel_gate_lanelet_safety_boundary_corner_samples"] == 4


def test_visual_boundary_uses_ten_centimeter_margins() -> None:
    """Verify RViz and the gate share one robot-center-local boundary contract."""
    package_path = SRC_ROOT / "camrod_platform" / "config" / "robot_visualization.yaml"
    bringup_path = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "platform"
        / "robot_visualization.yaml"
    )

    assert package_path.read_bytes() == bringup_path.read_bytes()
    parameters = _yaml(package_path)["/platform/robot_visualization"]["ros__parameters"]
    assert parameters["base_frame_id"] == "robot_center_link"
    assert parameters["planning_boundary_margin"] == pytest.approx(0.10)
    assert parameters["planning_boundary_lateral_margin"] == pytest.approx(0.10)

    visualizer_source = (
        SRC_ROOT / "camrod_platform" / "src" / "robot_visualization_node.cpp"
    ).read_text()
    gate_source = (
        SRC_ROOT / "camrod_control" / "src" / "cmd_vel_safety_gate_node.cpp"
    ).read_text()
    motion_source = (
        SRC_ROOT / "camrod_control" / "src" / "motion_cost_stop.cpp"
    ).read_text()
    assert "polygon_msg.header.frame_id = base_frame_id_;" in visualizer_source
    assert "message->header.frame_id != robot_base_frame_" in gate_source
    assert "setFootprintPolygonLocal(polygon)" in gate_source
    assert "setFootprintPolygonWorld" not in motion_source


def test_route_safety_retry_policy_is_identical_in_package_and_bringup() -> None:
    """The deployed gate must retain the package-owned retry containment."""
    package_path = (
        SRC_ROOT / "camrod_control" / "config" / "cmd_vel_safety_gate.yaml"
    )
    bringup_path = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "control"
        / "cmd_vel_safety_gate.yaml"
    )

    assert package_path.read_bytes() == bringup_path.read_bytes()
    parameters = _yaml(package_path)["/**"]["ros__parameters"]
    assert parameters["route_safety_recovery_max_auto_releases"] == 50
    assert parameters["route_safety_recovery_recontact_window_s"] == pytest.approx(5.0)
    assert parameters[
        "route_safety_recovery_progress_reset_distance_m"
    ] == pytest.approx(0.75)


def test_high_resolution_lanelet_safety_grid_and_nav2_owner_contract() -> None:
    """Keep final safety resolution separate from Nav2 without stop-go arbitration."""
    package_grid = (
        SRC_ROOT / "camrod_map" / "config" / "lanelet_safety_cost_grid.yaml"
    )
    bringup_grid = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "map"
        / "lanelet_safety_cost_grid.yaml"
    )
    package_gate = (
        SRC_ROOT / "camrod_control" / "config" / "cmd_vel_safety_gate.yaml"
    )
    defaults_path = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "bringup"
        / "launch_defaults.yaml"
    )

    # HH_260806 - A coarse 0.25 m cell produced a visible false stop even with
    # 12.4 cm exact planning clearance. Lock the independent local grid and
    # prevent the removed Nav2 rotation/translation handoff from returning.
    assert package_grid.read_bytes() == bringup_grid.read_bytes()
    grid = _yaml(package_grid)["/map/lanelet_safety_cost_grid"]["ros__parameters"]
    assert grid["output_topic"] == "/map/cost_grid/lanelet_safety"
    assert grid["resolution"] == pytest.approx(0.05)
    assert grid["width"] == 600
    assert grid["height"] == 600
    assert grid["window_mode"] == "robot_centered"
    assert grid["outside_value"] == 100
    assert grid["lanelet_boundary_value"] == 98

    gate = _wildcard_parameters(package_gate)
    assert gate["lanelet_safety_grid_topic"] == "/map/cost_grid/lanelet_safety"
    assert gate["maneuver_command_release_hold_s"] == pytest.approx(0.5)
    for obsolete in (
        "navigation_rotation_settle_s",
        "navigation_translation_epsilon_mps",
        "navigation_rotation_min_radps",
    ):
        assert obsolete not in gate

    defaults = _yaml(defaults_path)["bringup"]
    assert defaults["control"]["cmd_vel_gate_lanelet_safety_grid_topic"] == (
        "/map/cost_grid/lanelet_safety"
    )
    assert defaults["map"]["lanelet_safety_cost_grid_param_file"] == (
        "map/lanelet_safety_cost_grid.yaml"
    )


def test_apriltag_longitudinal_thresholds_preserve_rear_axle_stop_points() -> None:
    """Moving the measured base forward must not move the physical dock stop."""
    parking = _yaml(
        SRC_ROOT / "camrod_control" / "config" / "parking.yaml"
    )["/parking/apriltag_parking_controller"]["ros__parameters"]
    old_values = {
        "slowdown_start_distance_m": 1.0,
        "final_insertion_start_distance_m": 0.55,
        "parked_distance_from_tag_m": 0.5,
        "axis_full_trust_distance_m": 0.3,
        "axis_minimum_trust_distance_m": 4.0,
    }

    assert parking["base_frame_id"] == "robot_center_link"
    for key, old_value in old_values.items():
        assert parking[key] - CENTER_OFFSET_M == pytest.approx(old_value)


@pytest.mark.parametrize(
    "path,node,key",
    [
        ("camrod_platform/config/ranger_driver.yaml", "/**", "base_frame"),
        ("camrod_localization/config/filter/ekf.yaml", "/**", "base_link_frame"),
        ("camrod_localization/config/filter/ekf_sim.yaml", "/**", "base_link_frame"),
        ("camrod_localization/config/filter/pose_selector.yaml", "/**", "base_frame_id"),
        ("camrod_localization/config/source/input_adapter.yaml", "/**", "wheel_base_frame_id"),
        (
            "camrod_sensing/config/inflation_cost_grid.yaml",
            "/sensing/inflation_cost_grid",
            "base_frame_id",
        ),
        (
            "camrod_sensing/config/lidar/cost_grid.yaml",
            "/sensing/lidar/lidar_cost_grid",
            "base_frame_id",
        ),
        (
            "camrod_sensing/config/radar/cost_grid.yaml",
            "/sensing/radar/radar_cost_grid",
            "base_frame_id",
        ),
        (
            "camrod_bringup/config/sim/fake_sensors.yaml",
            "/bringup/fake_sensor_publisher",
            "base_frame_id",
        ),
    ],
)
def test_active_runtime_consumers_use_robot_center_link(
    path: str, node: str, key: str
) -> None:
    """Odom, localization, sensing, and simulation share one dynamic base."""
    parameters = _yaml(SRC_ROOT / path)[node]["ros__parameters"]

    assert parameters[key] == "robot_center_link"


def test_urdf_has_one_center_root_and_a_rear_axle_compatibility_child() -> None:
    """The compatibility frame must not create a second TF parent."""
    xacro = (
        SRC_ROOT
        / "camrod_sensor_kit"
        / "urdf"
        / "camrod_sensor_kit.xacro"
    ).read_text(encoding="utf-8")

    assert '<xacro:arg name="robot_center_link" default="robot_center_link"/>' in xacro
    assert '<xacro:arg name="robot_base_link" default="robot_base_link"/>' in xacro
    assert '<xacro:arg name="gnss_xyz" default="0.0 0.45 0.0"/>' in xacro
    assert '<parent link="${center_link_name}"/>' in xacro
    assert '<child link="${rear_base_link_name}"/>' in xacro
    assert 'xyz="-${rear_axle_offset_x_val} 0 0"' in xacro


@pytest.mark.parametrize("profile", ("default", "sim"))
def test_diagnostic_mount_metadata_uses_center_coordinates(profile: str) -> None:
    """Operator diagnostics must report the same mounts as TF and RobotParams."""
    config = _yaml(
        SRC_ROOT
        / "camrod_system"
        / "config"
        / "diagnostics"
        / profile
        / "aggregator"
        / "diagnostics_config.yaml"
    )
    actual = {
        topic["metadata"]["component_id"]: topic["metadata"].get("mount_xyz_m")
        for topic in config["topics"]
        if "metadata" in topic and "component_id" in topic["metadata"]
    }
    expected = {
        "gnss.main": "0.00000,0.45000,0.00000",
        "imu.main": "0.68800,0.00000,0.75600",
        "lidar.main": "0.76336,0.00000,0.59538",
        "lidar.filtered": "0.76336,0.00000,0.59538",
        "radar.front1": "0.62787,-0.11005,0.33378",
        "radar.front2": "0.62787,0.11005,0.33378",
        "radar.left1": "0.38,0.53,0.29013",
        "radar.left2": "-0.38,0.53,0.29013",
        "radar.right1": "0.38,-0.53,0.29013",
        "radar.right2": "-0.38,-0.53,0.29013",
        "radar.rear": "-0.61733,0.00000,0.33978",
        "camera.econ_front": "0.76337,0.00000,0.49568",
        "camera.econ_rear": "-0.61933,0.00000,0.30013",
    }

    for component, xyz in expected.items():
        assert actual[component] == xyz

    gnss_metadata = next(
        topic["metadata"]
        for topic in config["topics"]
        if topic.get("metadata", {}).get("component_id") == "gnss.main"
    )
    assert gnss_metadata["location"] == "left_antenna"
    assert gnss_metadata["pose_verified"] == "false"

    localization_metadata = next(
        topic["metadata"]
        for topic in config["topics"]
        if topic.get("metadata", {}).get("component_id")
        == "gnss.localization_input"
    )
    assert localization_metadata["location"] == "robot_center_lever_arm_corrected"
    assert localization_metadata["frame_id"] == "robot_center_link"
    assert localization_metadata["mount_xyz_m"] == "0.00000,0.00000,0.00000"
    assert localization_metadata["source_mount_xyz_m"] == (
        "0.00000,0.45000,0.00000"
    )
