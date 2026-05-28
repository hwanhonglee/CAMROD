# Camrod Node Execution Audit

> **Last updated: HH_260528**
> Auto-generated from CMake + launch files. Manually maintained after that.

---

## camrod_localization

- C++ executables in CMake: 9
- All referenced by launch

### Launched C++ executables

| Executable | Launch file |
|---|---|
| `drop_zone_matcher_node` | `localization.launch.py` |
| `kimera_csv_bridge_node` | `localization.launch.py` |
| `localization_eskf_node` | `localization.launch.py` (filter_type=eskf) |
| `localization_health_monitor_node` | `localization.launch.py` |
| `localization_pose_selector_node` | `localization.launch.py` |
| `localization_supervisor_node` | `localization.launch.py` |
| `navsat_to_pose_node` | `localization.launch.py` |
| `odometry_to_pose_node` | `localization.launch.py` |
| `wheel_odometry_bridge_node` | `localization.launch.py` |

> `robot_localization/ekf_node` (external) launched as `ekf_filter` when `filter_type=ekf`.
> Log level forced to WARN in `filter.launch.py` (HH_260528).

---

## camrod_map

| Executable | Launch file |
|---|---|
| `cost_field_marker_node` | `map.launch.py`, `path_cost_grids.launch.py` |
| `cost_field_node` | `map.launch.py` |
| `area_exporter_node` | `area_export.launch.py` |
| `lanelet2_map_node` | `lanelet2_map.launch.py` |
| `lanelet_cost_grid_node` | `map.launch.py`, `path_cost_grids.launch.py` |
| `marker_array_aggregator_node` | `map.launch.py` |

> `LaneletCostLayer` Nav2 plugin: added `start_current` param (HH_260528).
> - local_costmap: `start_current: true` (non-blocking cold start)
> - global_costmap: `start_current: false` (strict, waits for map grid)

---

## camrod_perception

| Executable | Launch file |
|---|---|
| `obstacle_fusion_node` | `perception.launch.py` |
| `obstacle_lidar_node` | `perception.launch.py` |

---

## camrod_planning

| Executable | Launch file |
|---|---|
| `centerline_snapper_node` | `planning.launch.py` |
| `goal_replanner_node` | `planning.launch.py` |
| `goal_snapper_node` | `planning.launch.py` |
| `local_path_extractor_node` | `planning.launch.py` |

---

## camrod_platform

| Executable | Launch file |
|---|---|
| `robot_visualization_node` | `platform.launch.py` |

---

## camrod_sensing (HH_260528 — dual camera)

| Executable | Launch file | Notes |
|---|---|---|
| `camera_front_publisher_node` | `sensing.launch.py` (via `camera.launch.py`) | GPU VPI + NvJPEG, Jetson only (ARM64) |
| `camera_rear_publisher_node` | `sensing.launch.py` (via `camera.launch.py`) | CPU OpenCV + GStreamer, Jetson only (ARM64) |
| `lidar_cost_grid_node` | `sensing.launch.py`, `fake_sensors.launch.py` | |
| `lidar_preprocessor_node` | `lidar.launch.py` | |
| `platform_velocity_converter_node` | `sensing.launch.py`, `platform_velocity_converter.launch.py` | |
| `radar_cost_grid_node` | `sensing.launch.py`, `fake_sensors.launch.py` | |
| `sen0592_radar_node` | `radar.launch.py`, `sensing.launch.py` | |

> **Deleted (HH_260528):** `camera_preprocessor_node` — replaced by `camera_front_publisher_node` + `camera_rear_publisher_node`.
> Old topics `/sensing/camera/processed/image` and `/sensing/camera/camera_info` no longer exist.
> Current camera topics:
> - `/sensing/camera/econ_front/image_rect/compressed`
> - `/sensing/camera/econ_rear/image_raw`

---

## Python Nodes Referenced by Launch

| Script | Package | Launch file |
|---|---|---|
| `bringup_diagnostic_node.py` | `camrod_bringup` | `bringup.launch.py` |
| `fake_sensor_publisher.py` | `camrod_bringup` | `fake_sensors.launch.py` |
| `localization_diagnostic_node.py` | `camrod_localization` | `localization.launch.py` |
| `map_diagnostic_node.py` | `camrod_map` | `map.launch.py` |
| `perception_diagnostic_node.py` | `camrod_perception` | `perception.launch.py` |
| `planning_diagnostic_node.py` | `camrod_planning` | `planning.launch.py` |
| `planning_state_machine_node.py` | `camrod_planning` | `planning.launch.py` |
| `platform_diagnostic_node.py` | `camrod_platform` | `platform.launch.py` |
| `sensing_diagnostic_node.py` | `camrod_sensing` | `sensing.launch.py` |
| `sensor_kit_diagnostic_node.py` | `camrod_sensor_kit` | `sensor_kit.launch.py` |
| `module_checker_node.py` | `camrod_system` | multiple modules |
| `system_checker_node.py` | `camrod_system` | `system_checker.launch.py` |
| `system_diagnostic_node.py` | `camrod_system` | `module_checkers.launch.py`, `system_checker.launch.py` |

---

## C++ Source Files Not Built

| File | Status |
|---|---|
| `camrod_localization/src/centerline_snapper_node.cpp` | Source only — no CMake target, never executed |
| `camrod_localization/src/pose_cov_bridge_node.cpp` | Source only — no CMake target, never executed |
