# Bringup Summary

_Last updated: HH_260528_

## Launch Entry

```
camrod_bringup/launch/bringup.launch.py
```

## Module Launch Files Included

| Order | File |
|-------|------|
| 1 | `camrod_platform/launch/platform.launch.py` |
| 2 | `camrod_sensor_kit/launch/sensor_kit.launch.py` |
| 3 | `camrod_map/launch/map.launch.py` |
| 4 | `camrod_sensing/launch/sensing.launch.py` |
| 5 | `camrod_perception/launch/perception.launch.py` |
| 6 | `camrod_localization/launch/localization.launch.py` |
| 7 | `camrod_planning/launch/planning.launch.py` |
| 8 | `camrod_system/launch/module_checkers.launch.py` |
| 9 | `camrod_bringup/scripts/bringup_diagnostic_node.py` (inline) |

Sim-only addition: `camrod_bringup/launch/fake_sensors.launch.py`

## Key Runtime Inputs (Expected at Startup)

| Topic | Producer | Notes |
|-------|----------|-------|
| `/sensing/gnss/pose_with_covariance` | `navsat_to_pose_node` | RTK GNSS fix |
| `/sensing/imu/data` | `microstrain_inertial_driver` | CV7 or GQ7 |
| `/platform/status/wheel_odometry` | platform driver | Forward velocity + yaw rate |
| `/platform/wheel/nav_odometry` | platform driver | nav_msgs/Odometry |
| `/sensing/camera/econ_front/image_rect` | `camera_front_publisher_node` | GPU VPI pipeline |
| `/sensing/camera/econ_rear/image_raw` | `camera_rear_publisher_node` | CPU GStreamer pipeline |
| `/sensing/lidar/vanjee/points_raw` | vanjee SDK | Raw point cloud |

## Key Outputs

| Topic | Producer |
|-------|----------|
| `/localization/primary/odometry` | `ekf_filter` or `eskf` |
| `/localization/pose` | `odometry_to_pose_node` |
| `/localization/lanelet_pose` | `map_helper` / adapter chain |
| `/map/cost_grid/lanelet` | `lanelet_cost_grid_node` |
| `/perception/obstacles` | `obstacle_fusion_node` |
| `/planning/global_path` | Nav2 planner_server |
| `/planning/local_path` | `local_path_extractor_node` |

## Bringup Override Config Files (camrod_bringup/config/)

| Section | Key overrides |
|---------|--------------|
| `bringup/launch_defaults.yaml` | All module enable flags, param file paths |
| `sensing/camera/camera_params.yaml` | Dual econ camera device paths + calibration |
| `sensing/imu/*.yaml` | IMU model (`imu_model: cv7` or `gq7`) |
| `localization/filter/ekf.yaml` | EKF tuning (frequency, Q, covariance) |
| `planning/nav2_base.yaml` | Costmap timeouts, planner max time |
| `planning/goal_replanner.yaml` | request_timeout_s |
| `planning/bt/*.xml` | BT tree with costmap race guard (Wait 0.4s) |

## Sensing Subsystem (HH_260528)

### Dual Camera Architecture

| Camera | Node | Pipeline | Topics |
|--------|------|----------|--------|
| Front (econ_front) | `camera_front_publisher_node` | GPU VPI + NvJPEG (Jetson only) | `image_rect/compressed`, `camera_info` |
| Rear (econ_rear) | `camera_rear_publisher_node` | CPU OpenCV + GStreamer | `image_raw`, `image_raw/compressed`, `camera_info` |

Camera enable flags: `enable_front_camera`, `enable_rear_camera` (in `launch_defaults.yaml`)

### Unified IMU

Single `imu.launch.py` replaces the old `imu_cv7.launch.py` + `imu_gq7_ntrip.launch.py`.
Select model with `imu_model: cv7` or `imu_model: gq7` in `launch_defaults.yaml`.

## Not Used / Not Yet Implemented

- Real VIO odometry producer (real `/localization/vio/odometry`)
- `isCharging()` hardware integration in docking plugin — currently returns `isDocked()`
- Reverse docking: `rear_bumper_to_camera_x` not measured yet
