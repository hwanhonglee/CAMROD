"""HH_261102 - Keep the experimental raw front guard bounded, opt-in, and tied to measured calibration."""

from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_front_guard_uses_ground_segmented_raw_points_and_measured_body():
    sensing = ROOT / "camrod_sensing/config/lidar/front_guard.yaml"
    mirror = ROOT / "camrod_bringup/config/sensing/lidar/front_guard.yaml"
    assert sensing.read_bytes() == mirror.read_bytes()
    guard = yaml.safe_load(sensing.read_text())["/**"]["ros__parameters"]
    robot = yaml.safe_load((ROOT / "camrod_bringup/config/sensor_kit/robot_params.yaml").read_text())["/**"]["ros__parameters"]["robot"]["body_extents"]
    for parameter, dimension in (("front", "front"), ("left", "left"), ("right", "right"),
                                  ("bottom", "bottom_z"), ("top", "top_z")):
        assert guard[f"front_guard_body_{parameter}_m"] == robot[dimension]
    assert guard["input_topics"] == ["/sensing/lidar/points_filtered"]
    assert guard["raw_lidar_cost_enabled"] is False
    assert guard["front_guard_distance_m"] == 0.30
    ground = yaml.safe_load((ROOT / "camrod_sensing/config/lidar/ground_seg_params.yaml").read_text())["/sensing/lidar/ground_segmentation"]["ros__parameters"]
    assert guard["front_guard_ground_clearance_m"] == ground["groundInlierThreshold"]
    assert guard["route_lanelet_filter_enable"] is False
    assert guard["ego_clear_radius_m"] == 0.0


def test_front_guard_opt_in_is_forwarded_to_both_sensing_and_final_gate():
    defaults = yaml.safe_load((ROOT / "camrod_bringup/config/bringup/launch_defaults.yaml").read_text())
    assert defaults["bringup"]["sensing"]["enable_lidar_front_guard"] is False
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
