# CAMROD Module Launch Command Reference

This document summarizes:
- basic run commands for each module package
- important launch options and defaults
- practical override examples

## 0) Common Prerequisites

```bash
cd /home/hong/camrod_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Build (recommended before run):
```bash
# If GNSS ublox source stack is used, include external ublox base path.
colcon build --symlink-install \
  --base-paths src src/camrod_sensing/external/ublox \
  --packages-up-to camrod_bringup ublox_gps
source install/setup.bash
```

Inspect all launch arguments of any launch file:
```bash
ros2 launch <package_name> <launch_file.py> -s
```

---

## 1) Full System Bringup

### 1.1 Entrypoint (recommended)
```bash
ros2 launch camrod_bringup bringup.launch.py
```

### Main options (`bringup.launch.py`)

| Arg | Default | Description |
|---|---|---|
| `sim` | `true` | Simulation mode |
| `rviz` | `true` | Enable RViz |
| `use_eskf` | `true` | Use ESKF localization |
| `map_path` | `""` | Lanelet map override |
| `enable_goal_replanner` | `true` | Enable goal replanner |
| `enable_state_machine` | `false` | Enable planning state machine |
| `enable_module_checkers` | `true` | Enable module/system checkers |
| `clean_before_launch` | `true` | Kill stale processes before launch |

Example:
```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=false rviz:=true use_eskf:=true \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm \
  enable_goal_replanner:=true enable_state_machine:=false
```

`bringup.launch.py` has additional advanced args (namespaces/topics/nav2 profile toggles).  
Use `ros2 launch camrod_bringup bringup.launch.py -s` to see full list.

---

## 2) Map Module (`camrod_map`)

### 2.1 Recommended
```bash
ros2 launch camrod_map map_top.launch.py
```

### 2.2 Full
```bash
ros2 launch camrod_map map.launch.py
```

### Main options (`map_top.launch.py`)

| Arg | Default | Description |
|---|---|---|
| `module_namespace` | `map` | Map namespace |
| `system_namespace` | `system` | Checker namespace |
| `map_path` | `""` | Lanelet map override |
| `enable_module_checker` | `true` | Enable checker |

### Main options (`map.launch.py`)

| Arg | Default source | Description |
|---|---|---|
| `map_param_file` | `camrod_bringup/config/map/map_info.yaml` | Lanelet2 map loader params |
| `map_path` | from `map_info.yaml` | Lanelet OSM path |
| `origin_lat/lon/alt` | from `map_info.yaml` | map origin |
| `map_visualization_param_file` | `camrod_bringup/config/map/map_visualization.yaml` | marker settings |
| `enable_nav2_inflation_debug_marker` | `false` | heavy debug marker |
| `module_namespace` | `map` | namespace |
| `system_namespace` | `system` | checker namespace |

Standalone map loader only:
```bash
ros2 launch camrod_map lanelet2_map.launch.py
```

Drop-zone export utility:
```bash
ros2 launch camrod_map drop_zone_export.launch.py \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm \
  output_yaml_path:=/home/hong/camrod_ws/src/camrod_bringup/config/localization/drop_zones.yaml
```

---

## 3) Sensing Module (`camrod_sensing`)

### 3.1 Recommended
```bash
ros2 launch camrod_sensing sensing_top.launch.py
```

### 3.2 Full
```bash
ros2 launch camrod_sensing sensing.launch.py
```

### Main options (`sensing_top.launch.py`)

| Arg | Default | Description |
|---|---|---|
| `module_namespace` | `sensing` | sensing namespace |
| `system_namespace` | `system` | checker namespace |
| `gnss_namespace` | `sensing/gnss` | GNSS namespace |
| `enable_lidar_driver` | `false` | Vanjee SDK driver |
| `enable_radar` | `false` | serial radar node |
| `enable_gnss` | `true` | ublox stack |
| `enable_ntrip` | `false` | ntrip client |
| `enable_module_checker` | `true` | checker |

### Important options (`sensing.launch.py`)

| Group | Key args |
|---|---|
| Namespace | `module_namespace`, `system_namespace`, `gnss_namespace` |
| Sensor enable | `enable_lidar_driver`, `enable_radar`, `enable_gnss`, `enable_ntrip` |
| Cost grid | `enable_lidar_cost_grid`, `enable_radar_cost_grid` |
| GNSS topics | `gnss_navsatfix_topic`, `gnss_rtcm_topic` |
| LiDAR topics | `lidar_raw_topic`, `lidar_filtered_topic`, `lidar_imu_packets_topic`, `lidar_diagnostic_topic` |
| Camera topics | `camera_input_*`, `camera_output_*`, `camera_diagnostic_topic` |
| IMU topics | `imu_input_topic`, `imu_output_topic`, `imu_diagnostic_topic` |

### Sensor-specific standalone launches

GNSS (ublox + optional ntrip):
```bash
ros2 launch camrod_sensing gnss.launch.py \
  gnss_namespace:=gnss enable_ntrip:=false \
  navsatfix_topic:=navsatfix rtcm_topic:=rtcm
```

LiDAR (Vanjee + preprocessor):
```bash
ros2 launch camrod_sensing lidar.launch.py \
  module_namespace:=lidar \
  enable_lidar_driver:=false \
  lidar_raw_topic:=vanjee/points_raw \
  lidar_filtered_topic:=points_filtered
```

Camera preprocessor:
```bash
ros2 launch camrod_sensing camera.launch.py module_namespace:=camera
```

IMU (Microstrain include):
```bash
ros2 launch camrod_sensing imu.launch.py \
  port:=/dev/ttyACM0 namespace:=imu
```

Radar:
```bash
ros2 launch camrod_sensing radar.launch.py module_namespace:=radar
```

Platform velocity converter:
```bash
ros2 launch camrod_sensing platform_velocity_converter.launch.py \
  module_namespace:=imu \
  velocity_topic:=/platform/status/velocity \
  imu_topic:=data \
  output_topic:=twist_with_covariance
```

---

## 4) Localization Module (`camrod_localization`)

### 4.1 Recommended
```bash
ros2 launch camrod_localization localization_top.launch.py
```

### 4.2 Full
```bash
ros2 launch camrod_localization localization.launch.py
```

### Main options (`localization_top.launch.py`)

| Arg | Default | Description |
|---|---|---|
| `module_namespace` | `localization` | localization namespace |
| `platform_namespace` | `platform` | platform namespace |
| `system_namespace` | `system` | checker namespace |
| `navsat_topic` | `/sensing/gnss/navsatfix` | navsat input |
| `use_eskf` | `true` | use ESKF or legacy EKF |
| `enable_module_checker` | `true` | checker |

### Main options (`localization.launch.py`)

| Group | Key args |
|---|---|
| Input topics | `navsat_topic`, `gnss_pose_topic`, `gnss_pose_cov_topic` |
| Origin/alignment | `origin_lat`, `origin_lon`, `origin_alt`, `yaw_offset_deg` |
| Filter | `use_eskf`, `eskf_param_file`, `supervisor_param_file` |
| Bridges/selectors | `wheel_bridge_enable`, `kimera_bridge_enable`, `pose_selector_enable` |
| Files | `drop_zone_param_file`, `kimera_bridge_param_file`, `pose_selector_param_file` |
| Namespaces/checkers | `module_namespace`, `platform_namespace`, `system_namespace`, `enable_module_checker` |

Example (real GNSS):
```bash
ros2 launch camrod_localization localization_top.launch.py \
  navsat_topic:=/sensing/gnss/navsatfix use_eskf:=true
```

---

## 5) Planning Module (`camrod_planning`)

### 5.1 Recommended
```bash
ros2 launch camrod_planning planning_top.launch.py
```

### 5.2 Full
```bash
ros2 launch camrod_planning planning.launch.py
```

### Main options (`planning_top.launch.py`)

| Arg | Default | Description |
|---|---|---|
| `module_namespace` | `planning` | planning namespace |
| `system_namespace` | `system` | checker namespace |
| `map_path` | `""` | map path override |
| `enable_goal_replanner` | `true` | goal replanner |
| `enable_state_machine` | `false` | planning state machine |
| `enable_module_checker` | `true` | checker |

### Important options (`planning.launch.py`)

| Group | Key args |
|---|---|
| Nav2 overlay files | `nav2_base_param_file`, `nav2_vehicle_param_file`, `nav2_lanelet_param_file`, `nav2_behavior_param_file` |
| Planner helpers | `enable_path_cost_grids`, `enable_goal_replanner`, `enable_nav2_lifecycle_retry`, `enable_state_machine` |
| Path helper params | `local_path_extractor_param_file`, `planning_state_machine_param_file` |
| Map/origin | `map_path`, `origin_lat`, `origin_lon`, `origin_alt` |
| Base frame | `nav2_robot_base_frame` (default `robot_base_link`) |
| Namespace/checker | `module_namespace`, `system_namespace`, `enable_module_checker` |

Nav2-only launch:
```bash
ros2 launch camrod_planning nav2_lanelet.launch.py
```

Path cost-grid helper only:
```bash
ros2 launch camrod_planning path_cost_grids.launch.py \
  module_namespace:=planning \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm
```

---

## 6) Platform Module (`camrod_platform`)

```bash
ros2 launch camrod_platform platform.launch.py
```

Main options:

| Arg | Default | Description |
|---|---|---|
| `map_frame_id` | `map` | map frame |
| `base_frame_id` | `robot_base_link` | base frame |
| `sensor_kit_base_frame_id` | `sensor_kit_base_link` | sensor kit frame |
| `params_file` | `camrod_bringup/config/sensor_kit/robot_params.yaml` | sensor kit robot params |
| `robot_visualization_param_file` | `camrod_bringup/config/platform/robot_visualization.yaml` | visualization params |
| `publish_base_link_alias` | `true` | publish `robot_base_link -> base_link` |
| `enable_module_checker` | `true` | checker |
| `module_namespace` | `platform` | namespace |
| `system_namespace` | `system` | checker namespace |
| `sensor_kit_namespace` | `sensor_kit` | included sensor-kit namespace |

---

## 7) Perception Module (`camrod_perception`)

```bash
ros2 launch camrod_perception perception.launch.py
```

Main options:

| Arg | Default | Description |
|---|---|---|
| `perception_param_file` | `camrod_bringup/config/perception/perception_params.yaml` | perception params |
| `enable_module_checker` | `true` | checker |
| `enable_lidar_obstacle` | `true` | LiDAR DBSCAN obstacle node |
| `module_namespace` | `perception` | namespace |
| `system_namespace` | `system` | checker namespace |

---

## 8) Sensor-kit Module (`camrod_sensor_kit`)

```bash
ros2 launch camrod_sensor_kit sensor_kit.launch.py
```

Main options:

| Arg | Default | Description |
|---|---|---|
| `params_file` | `camrod_bringup/config/sensor_kit/robot_params.yaml` | robot/sensor geometry |
| `module_namespace` | `sensor_kit` | namespace |
| `base_frame_id` | `robot_base_link` | base frame |
| `sensor_kit_base_frame_id` | `sensor_kit_base_link` | sensor-kit frame |
| `map_frame_id` | `map` | map frame |
| `enable_diagnostic` | `true` | sensor-kit diagnostic node |

---

## 9) System Module (`camrod_system`)

### 9.1 Recommended
```bash
ros2 launch camrod_system system_top.launch.py
```

### 9.2 Per-module checker orchestrator
```bash
ros2 launch camrod_system module_checkers.launch.py
```

### 9.3 Legacy system checker launch
```bash
ros2 launch camrod_system system_checker.launch.py
```

Main options:

| Launch | Args |
|---|---|
| `system_top.launch.py` | `system_namespace`, `enable_checkers` |
| `module_checkers.launch.py` | `system_namespace`, `enable_checkers` |
| `system_checker.launch.py` | `system_namespace`, `param_file` |

---

## 10) Bringup Utilities (`camrod_bringup`)

Fake sensors:
```bash
ros2 launch camrod_bringup fake_sensors.launch.py \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm \
  lanelet_id:=-1 speed_mps:=1.4 publish_rate_hz:=20.0
```

Useful options:
- `loop`, `frame_id`, `base_frame_id`
- `fake_enable_cost_grids`
- `bringup_namespace`, `sensing_namespace`
- map/origin overrides

---

## 11) Non-runtime Packages

### `avg_msgs`
- Interface package (messages/services/actions)
- no standalone launch

### `camrod_common`
- shared common resources/utilities
- no standalone runtime launch

---

## 12) Recommended Operator Profiles

### A) Simulation quick start
```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true
```

### B) Real hardware baseline (no LiDAR SDK / no NTRIP)
```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=false rviz:=true \
  enable_module_checkers:=true
```

### C) Real GNSS + NTRIP
```bash
ros2 launch camrod_sensing sensing_top.launch.py \
  enable_gnss:=true enable_ntrip:=true \
  gnss_namespace:=sensing/gnss
```

---

## 13) Troubleshooting Commands

Show active nodes:
```bash
ros2 node list | sort
```

Show active topics:
```bash
ros2 topic list | sort
```

Show launch arguments for any launch:
```bash
ros2 launch <pkg> <launch.py> -s
```

Check Nav2 lifecycle:
```bash
ros2 lifecycle get /planning/planner_server
ros2 lifecycle get /planning/controller_server
ros2 lifecycle get /planning/bt_navigator
```
