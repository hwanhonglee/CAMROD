# CAMROD Workspace (`camrod_ws/src`)

This is the integrated ROS 2 Humble workspace for CAMROD runtime.

## System Architecture
```text
                +--------------------+
                |   camrod_map       |
                | lanelet + cost map |
                +----------+---------+
                           |
                           v
+-------------------+   +--+------------------+   +--------------------+
|  camrod_sensing   +---> camrod_localization +---> camrod_planning    |
| lidar/camera/imu  |   | navsat->pose, ESKF  |   | Nav2 + snap/extract|
| gnss/radar        |   | supervisor           |   | global/local path  |
+---------+---------+   +----------+-----------+   +----------+---------+
          |                        |                          |
          v                        |                          v
+-------------------+              |                +--------------------+
| camrod_perception |--------------+--------------->| camrod_platform    |
| obstacle fusion   |                               | robot viz / TF     |
+-------------------+                               +--------------------+

                +--------------------------------------------+
                | camrod_system (/diagnostics aggregation)   |
                +--------------------------------------------+
```

## Package Map
- `camrod_bringup`: top-level orchestration launch
- `camrod_sensing`: sensor data ingestion + preprocessing + near-range grids
- `camrod_localization`: localization pipeline (GNSS/ESKF/supervisor)
- `camrod_map`: Lanelet2 map, base/planning cost grids, marker visualization
- `camrod_planning`: Nav2 runtime + lanelet goal/pose snapping + path extraction
- `camrod_perception`: obstacle generation/fusion
- `camrod_platform`: platform-side visualization and legacy TF alias
- `camrod_sensor_kit`: robot URDF + static sensor frame layout
- `camrod_system`: module checkers and system diagnostics
- `camrod_common/avg_msgs`: shared interface package (`avg_msgs`)

## Build
```bash
cd ~/camrod_ws
rosdep install --from-paths src --ignore-src -r -y
# NOTE:
# `camrod_sensing/external/*` packages are nested under another package tree,
# so include those base paths explicitly when building.
colcon build --symlink-install \
  --base-paths src src/camrod_sensing/external/ublox src/camrod_sensing/external/vanjee_lidar \
  --packages-up-to camrod_bringup ublox_gps vanjee_lidar_sdk
source install/setup.bash
```

## Run Full Stack
```bash
ros2 launch camrod_bringup bringup.launch.py
```

## Run Module Standalone (Examples)
```bash
ros2 launch camrod_map map.launch.py
ros2 launch camrod_sensing sensing.launch.py
ros2 launch camrod_localization localization.launch.py
ros2 launch camrod_planning planning.launch.py
```

## Topic / Namespace Rules
- Module namespace: `/<module>/...`
- Sensor sub-domain: `/<module>/<sensor>/<topic>`
- Diagnostics:
  - module-local: `/<module>/diagnostic`
  - aggregated: `/diagnostics`

## Key Runtime Topics
- Global path: `/planning/global_path`
- Local path: `/planning/local_path`
- Planning marker grids:
  - `/planning/cost_grid/global_path_markers`
  - `/planning/cost_grid/local_path_markers`
- Map inflation marker:
  - `/map/cost_grid/inflation_markers`

## Required TF Backbone
```text
map -> robot_base_link -> sensor_kit_base_link
                                   -> imu_link
                                   -> gnss_link
                                   -> lidar_link
                                   -> camera_front_link
```
