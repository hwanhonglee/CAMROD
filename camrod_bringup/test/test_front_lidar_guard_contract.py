"""HH_261102 - Keep the experimental raw front guard bounded, opt-in, and tied to measured calibration."""

from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_front_guard_uses_ground_segmented_raw_points_and_measured_body():
    sensing = ROOT / "camrod_sensing/config/lidar/front_guard.yaml"
    mirror = ROOT / "camrod_bringup/config/sensing/lidar/front_guard.yaml"
    assert sensing.read_bytes() == mirror.read_bytes()
    guard = yaml.safe_load(sensing.read_text())["/**"]["ros__parameters"]
    robot = yaml.safe_load((ROOT / "camrod_bringup/config/sensor_kit/robot_params.yaml").read_text())["/**"]["ros__parameters"]["robot"]["body_extents"]
    # HJ_261110 - Box is 0.30 m ahead of / beside / below the LiDAR mount
    # (no mount rotation); it starts at the body front and keeps the body top.
    lidar = yaml.safe_load((ROOT / "camrod_bringup/config/sensor_kit/robot_params.yaml").read_text())["/**"]["ros__parameters"]["lidar"]
    assert (lidar["roll"], lidar["pitch"], lidar["yaw"]) == (0.0, 0.0, 0.0)
    assert guard["front_guard_x_min_m"] == robot["front"]
    assert guard["front_guard_x_max_m"] == pytest.approx(lidar["x"] + 0.30, abs=1e-5)
    assert guard["front_guard_y_min_m"] == pytest.approx(lidar["y"] - 0.30, abs=1e-5)
    assert guard["front_guard_y_max_m"] == pytest.approx(lidar["y"] + 0.30, abs=1e-5)
    assert guard["front_guard_z_min_m"] == pytest.approx(lidar["z"] - 0.30, abs=1e-5)
    assert guard["front_guard_z_max_m"] == robot["top_z"]
    assert guard["input_topics"] == ["/sensing/lidar/points_filtered"]
    assert guard["raw_lidar_cost_enabled"] is False
    assert not any(key.startswith(("front_guard_body_", "front_guard_distance", "front_guard_ground"))
                   for key in guard)
    ground = yaml.safe_load((ROOT / "camrod_sensing/config/lidar/ground_seg_params.yaml").read_text())["/sensing/lidar/ground_segmentation"]["ros__parameters"]
    assert guard["front_guard_z_min_m"] > robot["bottom_z"] + ground["groundInlierThreshold"]
    assert guard["route_lanelet_filter_enable"] is False
    assert guard["ego_clear_radius_m"] == 0.0
    # HJ_261110 - Single-point LiDAR spikes must not stop the robot.
    assert guard["front_guard_min_points"] == 5
    assert guard["front_guard_min_consecutive_scans"] == 2


def test_front_guard_opt_in_is_forwarded_to_both_sensing_and_final_gate():
    defaults = yaml.safe_load((ROOT / "camrod_bringup/config/bringup/launch_defaults.yaml").read_text())
    # HJ_261110 - Bringup enables the guard; package launch defaults stay off.
    assert defaults["bringup"]["sensing"]["enable_lidar_front_guard"] is True
    gate = yaml.safe_load((ROOT / "camrod_control/config/cmd_vel_safety_gate.yaml").read_text())["/**"]["ros__parameters"]
    assert gate["lidar_front_guard_stop_enable"] is False
    assert gate["lidar_front_guard_max_age_s"] == 0.35
    source = (ROOT / "camrod_bringup/launch/_bringup_impl.py").read_text()
    assert "'enable_lidar_front_guard': lc['enable_lidar_front_guard']" in source
    assert "safety_gate_args['cmd_vel_gate_lidar_front_guard_stop_enable'] = lc['enable_lidar_front_guard']" in source
    for path in ("camrod_sensing/launch/sensing.launch.py", "camrod_sensing/launch/lidar.launch.py"):
        text = (ROOT / path).read_text()
        assert 'DeclareLaunchArgument("enable_lidar_front_guard", default_value="false")' in text
    semantic = yaml.safe_load((ROOT / "camrod_sensing/config/lidar/cost_grid.yaml").read_text())["/sensing/lidar/lidar_cost_grid"]["ros__parameters"]
    assert semantic["raw_lidar_cost_enabled"] is False


def test_ground_pipeline_preserves_raw_acquisition_stamp_for_empty_and_occupied_scans():
    preprocessor = (ROOT / "camrod_sensing/src/lidar_preprocessor_node.cpp").read_text()
    segmentation = (ROOT / "camrod_sensing/external/ground_segmentation_ros2/src/ground_segmentation_ros2_node.cpp").read_text()
    assert "out_msg->header = msg->header;" in preprocessor
    assert "obstacle_points->header.stamp = pointcloud_msg->header.stamp;" in segmentation
    assert "obstacle_points->header.stamp = this->now();" not in segmentation


def test_front_guard_grid_is_merged_into_the_cost_grid_map():
    # HJ_261110 - The guard grid is a dynamic (cost_grid/lidar*) merged input.
    for path in ("camrod_sensing/config/inflation_cost_grid.yaml",
                 "camrod_bringup/config/sensing/inflation_cost_grid.yaml"):
        params = yaml.safe_load((ROOT / path).read_text())["/sensing/inflation_cost_grid"]["ros__parameters"]
        topics = params["input_topics"]
        assert "/sensing/cost_grid/lidar_front_guard" in topics
        assert len(params["input_max_ages_s"]) == len(topics)
        assert params["input_max_ages_s"][topics.index("/sensing/cost_grid/lidar_front_guard")] == 0.50
