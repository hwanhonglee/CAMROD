# Camrod Runtime Node/Data Flow (Bringup 기준)

Updated: 2026-03-14

## 1) Bringup Orchestration

Main entry:
- `camrod_bringup/launch/bringup.launch.py`

Bringup includes module launches in this order:
1. `camrod_platform/launch/platform.launch.py`
2. `camrod_map/launch/map.launch.py`
3. `camrod_bringup/launch/fake_sensors.launch.py` (sim=true only)
4. `camrod_sensing/launch/sensing.launch.py`
5. `camrod_perception/launch/perception.launch.py`
6. `camrod_localization/launch/localization.launch.py`
7. `camrod_planning/launch/planning.launch.py`
8. `camrod_system/launch/module_checkers.launch.py`
9. `camrod_bringup/scripts/bringup_diagnostic_node.py`

## 2) Module-by-Module Runtime Graph

### 2.1 sensing

Primary launch:
- `camrod_sensing/launch/sensing.launch.py`

Runtime nodes:
- `camrod_sensing/lidar_preprocessor_node`
- `camrod_sensing/camera_preprocessor_node`
- `camrod_sensing/platform_velocity_converter_node`
- `camrod_sensing/sen0592_radar_node` (when `enable_radar=true`)
- `camrod_sensing/lidar_cost_grid_node` (when `enable_lidar_cost_grid=true`)
- `camrod_sensing/radar_cost_grid_node` (when `enable_radar_cost_grid=true`)
- `vanjee_lidar_sdk/vanjee_lidar_sdk_node` via `camrod_sensing/launch/lidar.launch.py` (when installed + enabled)
- `camrod_sensing/sensing_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

Key topic flow:
- Raw LiDAR:
  `/sensing/lidar/vanjee/points_raw`
  -> `lidar_preprocessor_node`
  -> `/sensing/lidar/points_filtered`
- Camera:
  `/sensing/camera/image_raw`, `/sensing/camera/camera_info`
  -> `camera_preprocessor_node`
  -> `/sensing/camera/processed/image`, `/sensing/camera/processed/camera_info`
- Platform velocity + IMU:
  `/platform/status/velocity`, `/sensing/imu/data`
  -> `platform_velocity_converter_node`
  -> `/sensing/platform_velocity_converter/twist_with_covariance`
- Radar ranges:
  `/sensing/radar/*/range`
  -> `radar_cost_grid_node`
  -> `/sensing/radar/near_cost_grid`
- Perception obstacles:
  `/perception/obstacles`
  -> `lidar_cost_grid_node`
  -> `/sensing/lidar/near_cost_grid`

### 2.2 perception

Primary launch:
- `camrod_perception/launch/perception.launch.py`

Runtime nodes:
- `camrod_perception/obstacle_lidar_node`
- `camrod_perception/obstacle_fusion_node`
- `camrod_perception/perception_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

Key topic flow:
- `/sensing/lidar/points_filtered` -> `obstacle_lidar_node` -> `/perception/obstacles/lidar`
- `/perception/obstacles/lidar` (+ optional other sources) -> `obstacle_fusion_node` -> `/perception/obstacles`

### 2.3 localization

Primary launch:
- `camrod_localization/launch/localization.launch.py`

Runtime nodes (depending on args):
- `camrod_localization/navsat_to_pose_node`
- `camrod_localization/localization_eskf_node` or `robot_localization/ekf_node`
- `camrod_localization/localization_supervisor_node`
- `camrod_localization/localization_health_monitor_node`
- `camrod_localization/drop_zone_matcher_node`
- `camrod_localization/wheel_odometry_bridge_node` (optional)
- `camrod_localization/kimera_csv_bridge_node` (optional)
- `camrod_localization/localization_pose_selector_node` (optional)
- `camrod_localization/odometry_to_pose_node` (optional path)
- `camrod_localization/localization_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

Key topic flow:
- GNSS:
  `/sensing/gnss/navsatfix`
  -> `navsat_to_pose_node`
  -> `/sensing/gnss/pose`, `/sensing/gnss/pose_with_covariance`
- Fusion:
  `/sensing/imu/data`, `/platform/wheel/odometry`, `/sensing/gnss/pose_with_covariance`
  -> `localization_eskf_node`
  -> `/localization/pose`, `/localization/pose_with_covariance`, `/tf` (`map->robot_base_link` or map/odom chain config-dependent)

### 2.4 map

Primary launch:
- `camrod_map/launch/map.launch.py`

Runtime nodes:
- `camrod_map/lanelet2_map_node`
- `camrod_map/lanelet_cost_grid_node` (base map grid)
- `camrod_map/cost_field_node`
- `camrod_map/cost_field_marker_node` (multiple instances)
- `camrod_map/marker_array_aggregator_node`
- `camrod_map/map_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

Key topic flow:
- Lanelet source -> `/map/lanelet2_map` + visualization markers
- Cost grids/markers -> `/map/cost_grid/*`
- Aggregated inflation markers -> `/map/cost_grid/inflation_markers`

### 2.5 planning

Primary launch:
- `camrod_planning/launch/planning.launch.py`

Runtime nodes:
- `camrod_planning/goal_snapper_node`
- `camrod_planning/centerline_snapper_node`
- `camrod_planning/local_path_extractor_node`
- `camrod_planning/goal_replanner_node`
- `nav2_*` stack via `camrod_planning/launch/nav2_lanelet.launch.py`
  - `planner_server`
  - `controller_server`
  - `behavior_server`
  - `bt_navigator`
  - `lifecycle_manager`
- `camrod_planning/planning_state_machine_node.py` (when enabled)
- `camrod_planning/planning_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

Key topic flow:
- `/planning/goal_pose` + `/planning/lanelet_pose`
  -> `goal_replanner_node` -> Nav2 `compute_path_to_pose`
  -> `/planning/global_path`
- `/planning/global_path` + `/planning/lanelet_pose`
  -> `local_path_extractor_node`
  -> `/planning/local_path`
- Cost-grid markers:
  `/planning/cost_grid/global_path_markers`
  `/planning/cost_grid/local_path_markers`

### 2.6 platform

Primary launch:
- `camrod_platform/launch/platform.launch.py`

Runtime nodes:
- `camrod_platform/robot_visualization_node`
- `tf2_ros/static_transform_publisher` (`robot_base_link` -> `base_link` alias, enabled by config)
- `camrod_platform/platform_diagnostic_node.py`
- `camrod_system/module_checker_node.py` (optional checker)

### 2.7 sensor_kit

Primary launch:
- `camrod_sensor_kit/launch/sensor_kit.launch.py`

Runtime nodes:
- `robot_state_publisher`
- `camrod_sensor_kit/sensor_kit_diagnostic_node.py`

### 2.8 system

Primary launches:
- `camrod_system/launch/module_checkers.launch.py`
- `camrod_system/launch/system_checker.launch.py`

Runtime nodes:
- `camrod_system/module_checker_node.py` (per-module instances)
- `camrod_system/system_diagnostic_node.py`
- `camrod_system/system_checker_node.py` (single-node mode launch)

## 3) C++ Code That Exists But Is Not Executed

Detected source files under `src/` not registered by CMake `add_executable`:
- `camrod_localization/src/centerline_snapper_node.cpp`
- `camrod_localization/src/pose_cov_bridge_node.cpp`

Implication:
- These files are not built as runtime nodes.
- They will never run from launch unless CMake target + launch wiring is added.

## 4) Launch Files With Placeholder/Non-Operational Content

- `camrod_sensing/launch/camera.launch.py`
  - Placeholder only (LogInfo)
- `camrod_sensing/launch/lidar_concat.launch.py`
  - Placeholder only (LogInfo), intentionally no nonexistent executable

## 5) Static Topic Cleanup Candidates

These are review targets, not automatic deletions:
- Legacy + new path coexistence:
  - `sensing_param_file` (legacy monolithic) and sensor-specific files now both exist by design.
- SDK internal topics that may be unused downstream:
  - `/vanjee_packet`, `/vanjee_device_ctrl_state`, `/vanjee_lidar_parameter_*`
- Nav2 internal topics (`/_action/*`, lifecycle internals)
  - Usually should not be removed manually; these are framework-managed.

## 6) What Was Changed In This Pass

1. Sensing configs were restructured into per-sensor folders:
   - `camrod_sensing/config/lidar|camera|imu|gnss|radar/...`
   - `camrod_bringup/config/sensing/lidar|camera|imu|gnss|radar/...`
2. Bringup compatibility preserved:
   - Legacy flat files remain.
   - Launch now supports layered parameter override (legacy + sensor-specific).
3. Topic naming consistency improved:
   - LiDAR raw default is now `/sensing/lidar/vanjee/points_raw` in `lidar.launch.py`.
4. Broken/duplicated launch content cleaned:
   - `camera.launch.py` fixed (removed accidental mixed LiDAR content).
   - `lidar_concat.launch.py` converted to explicit placeholder.
