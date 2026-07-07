---
name: workspace-layout
description: "CAMROD package layout, roles, and where key nodes/configs live"
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

Git repo + source is under `/home/avg/camrod_develop/src/`. 12 `camrod_*` packages + shared msgs. Each package has its own README (well-documented repo). Every package pulls large upstream deps into an `external/` dir (Nav2, robot_localization, lanelet2, apriltag_ros, ublox, vanjee_lidar_sdk, ranger_ros2/ugv_sdk, yolov9mit, etc.).

Packages & roles:
- `camrod_bringup` — top-level orchestrator; launch = `bringup.launch.py` (impl in `_bringup_impl.py`); holds a **synchronized copy** of every package's config under `config/<pkg>/`.
- `camrod_map` — Lanelet2 map load, lane cost grids, RViz markers. **`config/map_info.yaml` is the single source of truth for coordinate frames/offsets** (synced to bringup).
- `camrod_sensing` — sensor drivers + near-range cost grids. C++ nodes in `src/`: `lidar_cost_grid_node`, `radar_cost_grid_node`, `inflation_cost_grid_node`, `sen0592_radar_node`, camera pub/preproc, `lidar_preprocessor_node`, `platform_velocity_converter_node`.
- `camrod_localization` — GNSS+IMU+wheel EKF (default) / ESKF (optional) fusion, DR-only fallback. C++ nodes: `localization_eskf_node`, `_pose_selector_`, `_input_adapter_`, `_monitor_`, `_map_helper_`, `_gnss_reattach_`.
- `camrod_perception` — LiDAR obstacle clustering + optional YOLOv9 TensorRT camera fusion.
- `camrod_planning` — Nav2 runtime + goal snapping + safety gate + state machine. **C++ plugins/nodes** in `src/`: `lanelet_route_planner`, `goal_snapper_node`, `centerline_snapper_node`, `local_path_extractor_node`, `engage_aware_progress_checker`, `goal_replanner_node`, `nav2_lifecycle_startup_retry_node`, `path_tracking_error_node`. **Python nodes** in `scripts/`: `planning_cmd_vel_gate_node.py`, `planning_state_machine_node.py`, `obstacle_replan_monitor_node.py`, `nav2_selector_latch_node.py`, `path_visualizer_node.py`, `planning_progress_node.py`.
- `camrod_platform` — final cmd_vel gate, Ranger CAN bridge, URDF/TF.
- `camrod_sensor_kit` — URDF/xacro + static TF backbone + RobotParams lib.
- `camrod_system` — 20+ per-module diagnostic checkers + aggregator; `system_checker_node`, `diagnostics_aggregator_node` in `src/`. Publishes `/system/status`, `/system/msgs`.
- `camrod_ui` — FastAPI backend (`runtime/python/camrod_ui/ui_backend_node.py`, port 8010) + React frontend (`camrod_ui_robot/assets/frontend/`); also `camrod_ui_guest`.
- `camrod_docking` — AprilTag detection + opennav_docking.
- `camrod_parking` (Python ament pkg) — rule-based campsite crab maneuver + drop-zone reverse parking. **Mutually exclusive with camrod_docking** via `parking_method:={rule_based|docking}`.
- `camrod_common/avg_msgs` — shared ROS 2 msg/srv/action interfaces; the **internal CAMROD-facing message surface** with conversion helpers (`conversions.hpp`) to/from standard ROS msgs.

Disabled (`disable/`, marked `COLCON_IGNORE`): `vio_bridge`, `kimera_vio_bridge`, `config_archive`. Also `camrod_voice`, `todo/`, `docs/` present. See [[project-overview]], [[safety-critical-path]].
