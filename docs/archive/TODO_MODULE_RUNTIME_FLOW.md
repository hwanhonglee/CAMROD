# CAMROD Runtime Node / Data Flow

_Last updated: HH_260528_

---

## 1. Bringup Orchestration

Main entry:
```
camrod_bringup/launch/bringup.launch.py
```

Launch order:
1. `camrod_platform/launch/platform.launch.py`
2. `camrod_sensor_kit/launch/sensor_kit.launch.py`
3. `camrod_map/launch/map.launch.py`
4. `camrod_bringup/launch/fake_sensors.launch.py` (sim=true only)
5. `camrod_sensing/launch/sensing.launch.py`
6. `camrod_perception/launch/perception.launch.py`
7. `camrod_localization/launch/localization.launch.py`
8. `camrod_planning/launch/planning.launch.py`
9. `camrod_system/launch/module_checkers.launch.py`
10. `camrod_bringup/scripts/bringup_diagnostic_node.py`

---

## 2. Module Runtime Graphs

### 2.1 sensing

Primary launch: `camrod_sensing/launch/sensing.launch.py`

**Camera (HH_260528 — dual econ camera):**
```
/dev/video0  →  camera_front_publisher_node  →  /sensing/camera/econ_front/image_rect/compressed
                (GPU: VPI VIC + NvJPEG)          /sensing/camera/econ_front/camera_info

/dev/video1  →  camera_rear_publisher_node   →  /sensing/camera/econ_rear/image_raw
                (CPU: OpenCV + GStreamer)          /sensing/camera/econ_rear/image_raw/compressed
                                                  /sensing/camera/econ_rear/camera_info
```

**LiDAR:**
```
/sensing/lidar/vanjee/points_raw
  →  lidar_preprocessor_node  →  /sensing/lidar/points_filtered
```

**IMU (HH_260528 — unified imu.launch.py):**
```
imu_model=cv7  →  microstrain_inertial_driver_node  →  /sensing/imu/data
imu_model=gq7  →  microstrain_inertial_driver (SDK launch) + ntrip_client_node
```

**Platform velocity:**
```
/platform/status/velocity  →  platform_velocity_converter_node
  →  /sensing/platform_velocity_converter/twist_with_covariance
```

**Radar:**
```
/sensing/radar/*/range  →  radar_cost_grid_node  →  /sensing/radar/near_cost_grid
```

**LiDAR cost grid:**
```
/perception/obstacles  →  lidar_cost_grid_node  →  /sensing/lidar/near_cost_grid
```

---

### 2.2 perception

Primary launch: `camrod_perception/launch/perception.launch.py`

```
/sensing/lidar/points_filtered  →  obstacle_lidar_node  →  /perception/obstacles/lidar
/perception/obstacles/lidar (+others)  →  obstacle_fusion_node  →  /perception/obstacles
```

---

### 2.3 localization

Primary launch: `camrod_localization/launch/localization.launch.py`

```
/sensing/gnss/ublox_gps_node/fix  →  navsat_to_pose_node
  →  /sensing/gnss/pose_with_covariance

/sensing/imu/data
/sensing/gnss/pose_with_covariance
/platform/status/wheel_odometry
  →  ekf_filter (robot_localization/ekf_node, filter_type=ekf)
  →  /localization/primary/odometry

/localization/primary/odometry  →  odometry_to_pose_node  →  /localization/pose
/localization/primary/odometry  →  localization_pose_selector_node
```

EKF log level set to WARN (suppresses verbose INFO like set_pose request).

---

### 2.4 map

Primary launch: `camrod_map/launch/map.launch.py`

```
lanelet2 map file  →  lanelet2_map_node  →  /map/lanelet2_map
/map/lanelet2_map  →  lanelet_cost_grid_node  →  /map/cost_grid/lanelet
/map/cost_grid/*   →  cost_field_node + cost_field_marker_node  →  /map/cost_grid/inflation_markers
```

---

### 2.5 planning

Primary launch: `camrod_planning/launch/planning.launch.py`

```
/planning/goal_pose + /localization/lanelet_pose
  →  goal_replanner_node  →  Nav2 compute_path_to_pose
  →  /planning/global_path

/planning/global_path + /localization/lanelet_pose
  →  local_path_extractor_node  →  /planning/local_path

Nav2 BT tree: navigate_to_pose_w_planner_selector*.xml
  (includes Wait 0.4s after ClearEntireCostmap — HH_260528 costmap race guard)
```

---

### 2.6 platform

Primary launch: `camrod_platform/launch/platform.launch.py`

Args (HH_260528):
- `platform_type` — selects platform hardware variant
- `ranger_bridge_enable` — enables RMP401 ranger bridge
- `sensor_kit_bridge_enable` — enables sensor kit bridge node

```
robot_visualization_node
static_transform_publisher  (robot_base_link → base_link alias, if enabled)
```

---

### 2.7 sensor_kit

Primary launch: `camrod_sensor_kit/launch/sensor_kit.launch.py`

```
camrod_sensor_kit.xacro
  →  robot_state_publisher
  →  /tf_static  (sensor_kit_base_link → camera_front_link → camera_front)
                  (sensor_kit_base_link → camera_rear_link  → camera_rear)
                  (sensor_kit_base_link → lidar_top_link → ...)
                  (sensor_kit_base_link → imu_link → ...)
```

Camera TF ownership moved here from `camrod_docking` (HH_260528).

---

### 2.8 system

Primary launches:
- `camrod_system/launch/module_checkers.launch.py`
- `camrod_system/launch/system_checker.launch.py`

```
module_checker_node.py  (one per module)  →  /diagnostics
system_diagnostic_node.py                →  /diagnostics/system
```

---

## 3. Source Files Not Wired to CMake (Not Executed)

| File | Status |
|------|--------|
| `camrod_localization/src/centerline_snapper_node.cpp` | Source exists, no CMake target — not built |
| `camrod_localization/src/pose_cov_bridge_node.cpp` | Source exists, no CMake target — not built |

---

## 4. Deleted / Replaced Files (HH_260528)

| Old | Replaced by |
|-----|------------|
| `camrod_sensing/launch/imu_cv7.launch.py` | `imu.launch.py` (imu_model=cv7) |
| `camrod_sensing/launch/imu_gq7_ntrip.launch.py` | `imu.launch.py` (imu_model=gq7) |
| `camrod_sensing/src/camera_publisher_node.cpp` | `camera_front_publisher_node.cpp` |
| All `*copy_org*` / `*copy_750*` / `*copy_722*` backup files | Deleted |
