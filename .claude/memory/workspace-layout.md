---
name: workspace-layout
description: "CAMROD package layout, roles, and where key nodes/configs live"
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

<!-- HH_260720 - Describe the current package ownership after control consolidation. -->
Git repo + source is under `/home/hong/camrod_ws/src/`. The workspace contains 12 `camrod_*` packages plus `camrod_common/avg_msgs`. Package-owned dependencies live under each package's `external/` directory when a source build is required.

Packages & roles:
- `camrod_bringup` — top-level orchestrator; launch = `bringup.launch.py` (impl in `_bringup_impl.py`); carries deployment overrides under `config/<pkg>/`.
- `camrod_map` — Lanelet2 map load, lane cost grids, RViz markers. **`config/map_info.yaml` is the single source of truth for coordinate frames/offsets** (synced to bringup).
- `camrod_sensing` — sensor drivers + near-range cost grids. C++ nodes in `src/`: `lidar_cost_grid_node`, `radar_cost_grid_node`, `inflation_cost_grid_node`, `sen0592_radar_node`, camera pub/preproc, `lidar_preprocessor_node`, `platform_velocity_converter_node`.
- `camrod_localization` — GNSS+IMU+wheel EKF (default) / ESKF (optional) fusion, DR-only fallback. C++ nodes: `localization_eskf_node`, `_pose_selector_`, `_input_adapter_`, `_monitor_`, `_map_helper_`, `_gnss_reattach_`.
- `camrod_perception` — LiDAR obstacle clustering, optional YOLOv9 TensorRT camera fusion, and AprilTag parking pose detection.
- `camrod_planning` — Nav2 runtime, lanelet route/goal planning, path monitoring, and mission state machine. It produces motion candidates but does not own the command gate or site maneuvers.
- `camrod_control` — the single command safety gate, campsite crab/zero-turn maneuver, drop-zone exit/yaw maneuver, reverse parking, and AprilTag parking controller placeholder.
- `camrod_platform` — Ranger CAN/status bridge, light control, and robot visualization. Ranger consumes the explicit `/control/cmd_vel_ros` driver boundary directly; there is no platform gate.
- `camrod_sensor_kit` — URDF/xacro + static TF backbone + RobotParams lib.
- `camrod_system` — 20+ per-module diagnostic checkers + aggregator; `system_checker_node`, `diagnostics_aggregator_node` in `src/`. Publishes `/system/status`, `/system/msgs`.
- `camrod_ui` — FastAPI backend (`runtime/python/camrod_ui/ui_backend_node.py`, port 8010) + React frontend (`camrod_ui_robot/assets/frontend/`); also `camrod_ui_guest`.
- `camrod_common/avg_msgs` — generated ROS 2 msg/srv interfaces for internal CAMROD contracts, with explicit standard-ROS boundary conversion helpers. It contains no standard-message alias headers.

Disabled (`disable/`, marked `COLCON_IGNORE`): `vio_bridge`, `kimera_vio_bridge`, `config_archive`. See [[project-overview]], [[safety-critical-path]].
