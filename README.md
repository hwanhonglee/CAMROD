# CAMROD Workspace (`camrod_ws/src`)

Integrated ROS 2 Humble workspace for the CAMROD autonomous platform stack.

## 1) Workspace Architecture
```mermaid
graph LR
  BR[camrod_bringup] --> MAP[camrod_map]
  BR --> SENS[camrod_sensing]
  BR --> LOC[camrod_localization]
  BR --> PER[camrod_perception]
  BR --> PLAN[camrod_planning]
  BR --> PLAT[camrod_platform]
  BR --> SK[camrod_sensor_kit]
  BR --> SYS[camrod_system]
  BR --> API[camrod_api]

  MAP --> LOC
  MAP --> PLAN
  SENS --> LOC
  SENS --> PER
  LOC --> PLAN
  PLAN --> PLAT

  AVG[camrod_common/avg_msgs] --> MAP
  AVG --> SENS
  AVG --> LOC
  AVG --> PER
  AVG --> PLAN
  AVG --> PLAT
  AVG --> SYS
  AVG --> API
```

## 2) End-to-End Runtime Flow
```mermaid
graph TD
  A[Sensors: camera/lidar/radar/imu/gnss] --> B[camrod_sensing]
  B --> C[camrod_localization]
  B --> D[camrod_perception]
  E[camrod_map] --> C
  E --> F[camrod_planning]
  C --> F
  D --> F
  F --> G[/planning/cmd_vel_raw]
  H[/planning/engage + estop conditions] --> I[planning/platform cmd_vel gate]
  G --> I
  I --> J[/platform/cmd_vel]
  J --> K[Vehicle motion]
  L[camrod_system diagnostics] --> M[/system/diagnostics_agg]
  M --> N[camrod_api / UI]
```

## 3) Main Operational Scenario
1. System starts and required module nodes launch automatically.
2. Localization initializes using map reference (`camrod_map/config/map_info.yaml`).
3. Robot can move to `drop_zone` keypoint and wait.
4. When a target key (for example `camping_site_3`) is requested, planning generates global/local path.
5. `/planning/cmd_vel` is allowed only when gate conditions are valid (engage true, estop false).
6. Platform consumes command and moves to target.
7. At destination, robot waits or returns to `drop_zone` by return command.
8. Manual goal navigation through `/goal_pose` is also supported.

## 4) Package Responsibilities
| Package | Role |
|---|---|
| `camrod_bringup` | top-level orchestration and cross-package argument/config wiring |
| `camrod_map` | lanelet map load, map cost grids, map markers, map reference source |
| `camrod_sensing` | camera/lidar/radar/imu/gnss pipelines + near-range cost grids |
| `camrod_localization` | GNSS/IMU/wheel fusion, localization state, map helper |
| `camrod_perception` | obstacle outputs from lidar-only and camera+lidar fusion |
| `camrod_planning` | Nav2 runtime, snapping, global/local path, cmd_vel gating, state machine |
| `camrod_platform` | final cmd_vel gate, robot visualization, sensor kit launch include |
| `camrod_sensor_kit` | URDF/xacro and TF backbone publication |
| `camrod_system` | module diagnostics checkers and diagnostics aggregation |
| `camrod_api` | API bridge and lightweight UI backend |
| `camrod_common/avg_msgs` | shared ROS interfaces used across modules |

## 5) Build

### 5.1 Bootstrap external dependencies (from `src`)
```bash
cd ~/camrod_ws/src
./bootstrap_module_externals.sh
```

### 5.2 Install system dependencies
```bash
cd ~/camrod_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5.3 Build workspace
```bash
cd ~/camrod_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src src/camrod_sensing/external/ublox src/camrod_sensing/external/vanjee_lidar src/camrod_map/external/lanelet2 \
  --packages-up-to camrod_bringup
source install/setup.bash
```

## 6) Run

### 6.1 Full stack (recommended)
```bash
ros2 launch camrod_bringup bringup.launch.py
```

### 6.2 Simulation mode
```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true
```

### 6.3 Real-data mode (no fake sensors)
```bash
ros2 launch camrod_bringup bringup.launch.py sim:=false
```

### 6.4 Override map path from bringup
```bash
ros2 launch camrod_bringup bringup.launch.py \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm
```

### 6.5 Standalone module launches
```bash
ros2 launch camrod_map map.launch.py
ros2 launch camrod_sensing sensing.launch.py
ros2 launch camrod_localization localization.launch.py
ros2 launch camrod_planning planning.launch.py
ros2 launch camrod_platform platform.launch.py
ros2 launch camrod_system system.launch.py
```

## 7) Key Topics and Signals
| Topic | Purpose |
|---|---|
| `/planning/global_path` | global route from planner |
| `/planning/local_path` | local route for tracking |
| `/planning/cmd_vel_raw` | raw controller velocity |
| `/planning/cmd_vel` | gated velocity command for platform |
| `/platform/cmd_vel` | final platform command |
| `/map/cost_grid/lanelet` | lanelet-based base cost grid |
| `/map/cost_grid/planning_base` | planning-oriented secondary cost grid |
| `/map/cost_grid/inflation_markers` | merged map/planning/sensing marker visualization |
| `/localization/pose` | canonical fused pose |
| `/localization/initial_match_ok` | localization readiness signal used by planning startup gate |
| `/system/diagnostics` | raw system diagnostics stream |
| `/system/diagnostics_agg` | aggregated diagnostics stream |

## 8) Shared Map Reference Rule
`camrod_map/config/map_info.yaml` is the shared reference source for map and localization alignment.

Main fields include:
- `map_path`
- `offset_lat`, `offset_lon`, `offset_alt`
- `offset_utm_easting`, `offset_utm_northing`, `offset_utm_alt`
- `yaw_offset_deg`
- `rotate_latlon_xy_by_yaw_offset`
- `world_frame_id`, `map_frame_id`

Launch-level overrides (for example `map_path:=...`) are still supported from `bringup.launch.py` and module launches.

## 9) RViz
Default operator-style RViz used in bringup:
- config: `camrod_map/rviz/camrod_operator.rviz`
- stylesheet: `camrod_map/rviz/operator_theme.qss`
