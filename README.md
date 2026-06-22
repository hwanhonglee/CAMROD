# CAMROD — Autonomous Camping Delivery Robot

ROS 2 Humble workspace for the CAMROD autonomous mobile platform.  
Built on the **Agilex Ranger** base, CAMROD navigates pre-mapped campground sites, delivers goods, and returns autonomously with GNSS/IMU/wheel localization and Lanelet2 lane-aware planning.

> Current release: **v1.15**

---

## Documentation Map

| Package | Role | README |
|---------|------|--------|
| `camrod_bringup` | Top-level orchestrator | [README](camrod_bringup/README.md) |
| `camrod_map` | Lanelet2 map + cost grids + markers | [README](camrod_map/README.md) |
| `camrod_sensing` | Camera/LiDAR/radar/IMU/GNSS + near-range cost grids | [README](camrod_sensing/README.md) |
| `camrod_localization` | GNSS/IMU/wheel fusion (EKF default; ESKF optional) + map helper | [README](camrod_localization/README.md) |
| `camrod_perception` | LiDAR obstacles + optional YOLOv9 camera fusion | [README](camrod_perception/README.md) |
| `camrod_planning` | Nav2 runtime + cmd\_vel gating + state machine | [README](camrod_planning/README.md) |
| `camrod_parking` | Rule-based campsite crab maneuver + drop-zone reverse parking | [README](camrod_parking/README.md) |
| `camrod_docking` | AprilTag dock detection + opennav\_docking | [README](camrod_docking/README.md) |
| `camrod_platform` | Final cmd\_vel gate + robot visualization | [README](camrod_platform/README.md) |
| `camrod_sensor_kit` | URDF + static TF tree + RobotParams lib | [README](camrod_sensor_kit/README.md) |
| `camrod_system` | Diagnostic checkers + aggregator | [README](camrod_system/README.md) |
| `camrod_ui` | HTTP backend + operator web UI | [README](camrod_ui/README.md) |
| `avg_msgs` | Shared ROS 2 message/service interfaces | [README](camrod_common/avg_msgs/README.md) |

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware & Software Requirements](#2-hardware--software-requirements)
3. [Package Architecture](#3-package-architecture)
4. [Runtime Data Flow](#4-runtime-data-flow)
5. [External Dependencies](#5-external-dependencies)
6. [First Run Guide](#6-first-run-guide)
7. [Build](#7-build)
8. [Docker](#8-docker)
9. [Run](#9-run)
10. [Key Topics & Signals](#10-key-topics--signals)
11. [Planning Profiles](#11-planning-profiles)
12. [Operator UI](#12-operator-ui)
13. [Camping Sites Configuration](#13-camping-sites-configuration)
14. [Map Reference](#14-map-reference)
15. [Diagnostics](#15-diagnostics)
16. [Glossary](#16-glossary)
17. [Documentation Standards](#17-documentation-standards)
18. [Disabled / Optional Packages](#18-disabled--optional-packages)

---

## 1. System Overview

CAMROD is a supervised-autonomy delivery platform designed for controlled outdoor environments (campgrounds, parks, warehouses).

**Core capabilities:**
- Autonomous point-to-point navigation via pre-surveyed Lanelet2 maps
- Multi-sensor obstacle detection (LiDAR + camera + mmWave radar)
- GNSS/RTK localization with IMU + wheel dead-reckoning fallback
- AprilTag-based autonomous docking
- Web operator UI for site selection, diagnostics, and manual override
- Mission state machine: deliver → dwell → recall → return

---

## 2. Hardware & Software Requirements

| Component | Model / Notes |
|-----------|---------------|
| Mobile base | Agilex Ranger (4WD skid-steer, CAN bus) |
| GNSS | SparkFun ZED-F9P (single antenna, RTK via NTRIP) + ArduSimple simpleRTK2B Heading (dual antenna, moving-baseline heading) |
| IMU | Microstrain CV7 or GQ7 (9-axis; GQ7 has embedded GNSS) |
| LiDAR | Vanjee 3D LiDAR |
| Camera | USB camera (V4L2, UYVY → ROS Image) |
| Radar | SEN0592 mmWave (near-range obstacle detection, ×6) |
| Dock markers | AprilTag 36h11 family (autonomous docking) |
| Compute | Onboard Linux x86\_64 / ARM64, Ubuntu 22.04 + ROS 2 Humble |

**Software requirements:**

| Requirement | Version / Notes |
|-------------|----------------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Nav2 | Humble release (included in `camrod_planning/external/`) |
| opennav\_docking | Included in `camrod_docking/external/` |
| apriltag\_ros | Included in `camrod_docking/external/` |
| robot\_localization | Included in `camrod_localization/external/` |
| Python | 3.10+ (FastAPI, uvicorn for camrod\_ui) |
| Node.js | 18+ (frontend build only) |
| Tested architectures | `amd64`, `arm64` (see Dockerfiles) |

---

## 3. Package Architecture

> Each package has a detailed README — see the [Documentation Map](#documentation-map) above.

```mermaid
%%{init: {
  'theme': 'base',
  'themeVariables': {
    'fontFamily': 'ui-sans-serif, system-ui, sans-serif',
    'fontSize': '14px',
    'primaryColor': '#EEF2FF',
    'primaryTextColor': '#0F172A',
    'primaryBorderColor': '#6366F1',
    'lineColor': '#475569'
  },
  'flowchart': { 'curve': 'basis', 'htmlLabels': true, 'padding': 12 }
}}%%
graph TD
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef docking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef iface        fill:#F0FDFA,stroke:#14B8A6,stroke-width:1.5px,color:#115E59;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  subgraph ORCH ["🚀 Orchestration"]
    BR([🚀 camrod_bringup])
  end

  subgraph PERC ["🌐 Perception & Map stack"]
    SEN([🎯 camrod_sensing])
    PER([👁️ camrod_perception])
    MAP([🗺️ camrod_map])
  end

  subgraph STATE ["📍 State & Planning"]
    LOC([📍 camrod_localization])
    PLN([🧭 camrod_planning])
  end

  subgraph ACT ["🤖 Actuation"]
    PLT([🤖 camrod_platform])
    KIT([🔧 camrod_sensor_kit])
    PARK([🅿️ camrod_docking])
  end

  subgraph OPS ["🩺 Ops & UI"]
    SYS([🩺 camrod_system])
    UI([🖥️ camrod_ui])
  end

  subgraph IF ["📨 Interfaces"]
    AVG[(📨 avg_msgs)]
  end

  BR --> MAP & SEN & LOC & PER & PLN & PLT & KIT & SYS & UI & PARK
  MAP --> LOC & PLN
  SEN ==> LOC
  LOC ==> PLN
  PLN ==> PLT
  SEN --> PER & PARK
  PER --> PLN
  PLN --> PARK
  PLT --> PARK
  SYS --> UI

  AVG -.-> MAP & SEN & LOC & PER & PLN & PLT & SYS & UI & PARK

  class BR highlight
  class SEN sensing
  class MAP mapping
  class LOC localization
  class PER perception
  class PLN planning
  class PLT platform
  class KIT system
  class PARK docking
  class SYS system
  class UI ui
  class AVG iface

  linkStyle 5,6,7 stroke:#6366F1,stroke-width:2.5px;
```

> **Diagram legend**
> 📦 Package (stadium) · 🧩 ROS node (round-rect) · 📡 Topic (circle) · ⚙️ Config · 🛠️ Hardware · 📨 Interface
> Solid `-->` runtime dependency · `==>` critical path · `-.->` interface-only dependency
> Color = package/layer — see [docs/templates/DIAGRAM_PALETTE.md](docs/templates/DIAGRAM_PALETTE.md)

*Figure 1 — CAMROD workspace architecture. The `==>` chain (sensing → localization → planning → platform) is the safety-critical runtime path. Dashed arrows mark interface-only dependencies on `avg_msgs`.*

### Package Responsibilities

| Package | Role |
|---------|------|
| `camrod_bringup` | Top-level orchestrator; cross-package argument and config wiring |
| `camrod_map` | Lanelet2 map loading, lane cost grids, RViz markers |
| `camrod_sensing` | Camera / LiDAR / GNSS / IMU / radar drivers + near-range cost grids |
| `camrod_localization` | GNSS+IMU+wheel EKF fusion by default; optional ESKF backend; DR-mode fallback; drop-zone init |
| `camrod_perception` | LiDAR obstacle clustering, YOLOv9 camera detection, obstacle fusion |
| `camrod_planning` | Nav2 runtime, Lanelet2 goal/path snapping, cmd\_vel safety gate, mission state machine |
| `camrod_platform` | Final cmd\_vel gate, Ranger CAN bridge, URDF/TF publisher |
| `camrod_sensor_kit` | Robot URDF/xacro and static TF backbone |
| `camrod_system` | Per-module diagnostic checkers + aggregator (20+ checkers) |
| `camrod_ui` | FastAPI HTTP backend (port 8010) + React operator web UI |
| `camrod_docking` | AprilTag detection bridge + opennav\_docking integration |
| `camrod_common/avg_msgs` | Shared ROS 2 message/service/action definitions |

---

## 4. Runtime Data Flow

```mermaid
%%{init: {
  'theme': 'base',
  'themeVariables': {
    'fontFamily': 'ui-sans-serif, system-ui, sans-serif',
    'fontSize': '14px',
    'primaryColor': '#EEF2FF',
    'primaryTextColor': '#0F172A',
    'primaryBorderColor': '#6366F1',
    'lineColor': '#475569'
  },
  'flowchart': { 'curve': 'basis', 'htmlLabels': true, 'padding': 12 }
}}%%
graph LR
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  subgraph SRC ["🛠️ Hardware"]
    HW{{🛠️ Sensors}}
    VEH{{🚗 Ranger CAN}}
  end

  subgraph SENSE ["🎯 Sensing & Map"]
    SEN([🎯 camrod_sensing])
    MAP([🗺️ camrod_map])
  end

  subgraph EST ["📍 State Estimation"]
    LOC([📍 camrod_localization])
  end

  subgraph OBS ["👁️ Perception"]
    PER([👁️ camrod_perception])
  end

  subgraph NAV ["🧭 Planning & Control"]
    PLN([🧭 camrod_planning])
    GATE(🚦 cmd_vel_gate)
  end

  subgraph MON ["🩺 Monitoring & UI"]
    SYS([🩺 camrod_system])
    AGG((diag_agg))
    UI([🖥️ camrod_ui])
  end

  HW --> SEN
  SEN ==> LOC
  MAP --> LOC & PLN
  SEN --> PER
  LOC ==> PLN
  PER --> PLN
  PLN ==> GATE
  GATE ==> VEH
  SYS --> AGG
  AGG --> UI
  UI --> PLN

  class HW hardware
  class VEH hardware
  class SEN sensing
  class MAP mapping
  class LOC localization
  class PER perception
  class PLN planning
  class GATE highlight
  class SYS system
  class AGG topic
  class UI ui
```

> **Diagram legend**
> `([...])` Package (stadium) · `(...)` ROS node (round-rect) · `((...))` Topic (circle) · `{{...}}` Hardware
> Solid `-->` runtime data flow · `==>` safety-critical path · `-.->` interface/config dependency
> Color = package/layer — see [docs/templates/DIAGRAM_PALETTE.md](docs/templates/DIAGRAM_PALETTE.md)

*Figure 2 — Runtime data flow. The `==>` chain (sensing → localization → planning → cmd\_vel\_gate → Ranger CAN) is the control-critical path. `cmd_vel_gate` is highlighted because it is the final safety interlock before wheel commands reach the vehicle.*

**Nominal mission loop:**

1. System starts; all modules launch with staggered delays.
2. Localization initializes from `camrod_map/config/map_info.yaml` (single map reference source).
3. State machine sends robot to `drop_zone` (startup goal).
4. Operator selects a camping site via UI → `/planning/mission_key` published.
5. Nav2 generates global path; cmd\_vel gate enforces engage / e-stop / cost-stop / GNSS-recovery hold.
6. Robot arrives; waits `goal_reached_dwell_s` (default 600 s) or until recall trigger.
7. On recall: navigates to site's lanelet coordinate for loading, then auto-returns to `drop_zone`.

**GNSS outage:**
- Localization switches to `DR_ONLY`; robot continues on IMU + wheel odometry.
- On GNSS recovery: 2 s `gnss_recovery_hold_s` stop lets Nav2 and costmap settle.

**Manual override:** `/goal_pose` (RViz 2D Nav Goal) works at any time.

---

## 5. External Dependencies

| Package group | Location | Purpose |
|---------------|----------|---------|
| `ublox` | `camrod_sensing/external/` | u-blox GNSS driver |
| `vanjee_lidar_sdk` | `camrod_sensing/external/` | Vanjee LiDAR driver |
| `ntrip_client` | `camrod_sensing/external/` | RTK correction client |
| `perception_pcl` | `camrod_sensing/external/` | PCL ↔ ROS 2 bridge |
| `robot_localization` | `camrod_localization/external/` | EKF / ESKF state estimator |
| `lanelet2` | `camrod_localization/external/` | Lanelet2 map library core |
| `yolov9mit` | `camrod_perception/external/` | YOLOv9 TensorRT inference library |
| `yolov9mit_ros` | `camrod_perception/external/` | YOLOv9 ROS 2 wrapper node |
| `vision_opencv` | `camrod_perception/external/` | cv\_bridge / image\_transport |
| `nav2_*` (10 pkgs) | `camrod_planning/external/` | Nav2 navigation stack |
| `ranger_ros2` | `camrod_platform/external/` | Agilex Ranger CAN driver |
| `ugv_sdk` | `camrod_platform/external/` | Agilex UGV CAN SDK |
| `opennav_docking` | `camrod_docking/external/` | Docking station manager |
| `apriltag_ros` | `camrod_docking/external/` | AprilTag marker detection |
| `lanelet2` | `camrod_map/external/` | Map utilities |

---

## 6. First Run Guide

Step-by-step from a fresh clone to a running simulation.

```bash
# 1. Clone and enter workspace
git clone https://github.com/hwanhonglee/CAMROD.git ~/camrod_ws/src
cd ~/camrod_ws/src

# 2. Bootstrap externals + install system deps
./setup_camrod.sh

# 3. Build
cd ~/camrod_ws
colcon build --symlink-install --packages-up-to camrod_bringup

# 4. Source workspace
source install/setup.bash

# 5. Launch simulation with RViz
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true

# 6. Open operator UI
# http://127.0.0.1:8010

# 7. Send a goal via UI dropdown or RViz 2D Nav Goal
#    → select "B1" from the site picker
#    → or click the 2D Nav Goal button in RViz
```

---

## 7. Build

### 6.1 First-time setup (clone externals + rosdep)

```bash
cd ~/camrod_ws/src
./setup_camrod.sh
```

To update existing externals:

```bash
./setup_camrod.sh --update
```

### 6.2 Build (all packages)

```bash
cd ~/camrod_ws
colcon build --symlink-install
```

Or use the project wrapper (handles multi-base-paths for `external/` dirs):

```bash
./src/build_camrod.sh
```

Build a specific package and its dependencies:

```bash
./src/build_camrod.sh --packages-up-to camrod_bringup
```

Build only (skip rosdep / bootstrap):

```bash
./src/build_camrod.sh --build-only --packages-up-to camrod_bringup
```

Bootstrap only (no build):

```bash
./src/build_camrod.sh --bootstrap-only
```

### 6.3 Source workspace

```bash
source ~/camrod_ws/install/setup.bash
```

---

## 8. Docker

Build and run CAMROD in a container.

```bash
# Build for amd64
docker build -f Dockerfile.camrod.amd64 -t camrod:amd64 .

# Build for arm64 (cross-compile or on ARM host)
docker build -f Dockerfile.camrod.arm64 -t camrod:arm64 .
```

<!-- TODO: verify exact docker run command with CAN, USB, and GPU device mounts -->

---

## 9. Run

### 7.1 Full stack

```bash
ros2 launch camrod_bringup bringup.launch.py
```

### 7.2 Simulation mode (fake sensors + RViz)

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true
```

### 7.3 Real hardware

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=false
```

### 7.4 Override map

```bash
ros2 launch camrod_bringup bringup.launch.py \
  map_path:=/path/to/lanelet2_maps.osm
```

### 7.5 Individual module launches

```bash
ros2 launch camrod_map         map.launch.py
ros2 launch camrod_sensing     sensing.launch.py
ros2 launch camrod_localization localization.launch.py
ros2 launch camrod_perception  perception.launch.py
ros2 launch camrod_planning    planning.launch.py
ros2 launch camrod_platform    platform.launch.py
ros2 launch camrod_system      system.launch.py
ros2 launch camrod_ui          ui.launch.py
```

### 7.6 UI only

```bash
ros2 launch camrod_ui ui.launch.py ui_host:=0.0.0.0 ui_port:=8010
# Open http://<robot-ip>:8010
```

### 7.7 RViz

```bash
rviz2 -d ~/camrod_ws/src/camrod_map/rviz/camrod_operator.rviz \
  --stylesheet ~/camrod_ws/src/camrod_map/rviz/operator_theme.qss
```

---

## 10. Key Topics & Signals

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/sensing/lidar/points_filtered` | `PointCloud2` | sensing → perception/planning | Processed LiDAR cloud |
| `/sensing/gnss/fix` | `NavSatFix` | sensing → localization | GNSS position |
| `/sensing/imu/data` | `Imu` | sensing → localization | 9-axis inertial data |
| `/sensing/camera/color/image_rect` | `Image` | sensing → perception | Camera feed for YOLO |
| `/localization/pose` | `PoseStamped` | localization → planning | Canonical fused localization pose |
| `/localization/mode` | `AvgLocalizationMode` | localization → planning gate | NORMAL / DEGRADED / DR\_ONLY / INVALID |
| `/localization/initial_match_ok` | `Bool` | localization → planning | Drop-zone match readiness |
| `/perception/obstacles` | `PointCloud2` | perception → planning costmap | Fused obstacle cloud |
| `/planning/cost_grid/inflation` | `OccupancyGrid` | sensing/map → planning | Merged near-range cost grid |
| `/map/cost_grid/lanelet` | `OccupancyGrid` | map → sensing inflation | Lane traversability layer |
| `/planning/global_path` | `Path` | planning → RViz / diagnostics | Global Nav2 route |
| `/planning/local_path` | `Path` | planning → cmd\_vel\_gate | Active trajectory segment |
| `/planning/cmd_vel_raw` | `Twist` | Nav2 controller → gate | Raw controller output |
| `/planning/cmd_vel` | `Twist` | gate → platform | Gated velocity (safe to send) |
| `/platform/cmd_vel` | `Twist` | platform gate → Ranger | Final vehicle command |
| `/platform/status/estop` | `Bool` | Ranger CAN → gates | Hardware emergency stop |
| `/platform/status/odometry` | `Odometry` | Ranger CAN → localization | Wheel odometry |
| `/planning/engage` | `Bool` | UI / state machine → gate | Drive enable signal |
| `/planning/state_machine/state` | `String` | state machine → all | INIT / RUNNING / GOAL\_REACHED / … |
| `/planning/mission_key` | `String` | UI → state machine | Named site selector (e.g. "B3") |
| `/planning/state_machine/mission_source` | `String` | state machine → diagnostics | startup / recall:B3 / auto\_return |
| `/goal_pose` | `PoseStamped` | UI / RViz → Nav2 | Manual 2D Nav Goal |
| `/ui/selected_destination` | `String` | UI → backend | Operator site selection |
| `/system/diagnostics_agg` | `DiagnosticArray` | system → UI | Aggregated health status |

---

## 11. Planning Profiles

Nav2 planner × controller combinations are managed as standalone YAML overrides in:

```
camrod_planning/config/nav2_combo_profiles/
```

Each file encodes one `[planner]_[controller].yaml` pair (e.g. `smachybrid_graceful.yaml`) and overrides only the relevant Nav2 parameters on top of `nav2_base.yaml`.

**Available planners:** `navfn`, `smac2d`, `smachybrid`, `smaclattice`, `thetastar`  
**Available controllers:** `dwb`, `graceful`, `mppi`, `rotationshim`, `rpp`

Switch profile at launch:

```bash
ros2 launch camrod_planning planning.launch.py \
  nav2_combo_profile:=smachybrid_graceful
```

Key tuning parameters per controller:

| Controller | Key params |
|------------|-----------|
| Graceful | `v_linear_max`, `initial_rotation_min_angle`, `rotation_scaling_factor`, `allow_backward` |
| RotationShim | `angular_dist_threshold`, `max_angular_accel`, `rotate_to_goal_heading` |
| DWB | `sim_time`, `PathAlign.scale`, `GoalDist.scale`, `RotateToGoal.scale` |
| MPPI | `vx_max`, `prune_distance`, `temperature`, `gamma` |

---

## 12. Operator UI

The web UI runs at **http://\<robot-ip\>:8010** (default bind: `127.0.0.1:8010`).

**Features:**
- Camping site selection dropdown → dispatches goal to state machine
- System health dashboard (from `/system/diagnostics_agg`)
- Engage / stop button
- Real-time robot status (localization mode, planning state, battery)

**Backend architecture:**  
FastAPI (Python) in `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`  
React frontend built in `camrod_ui/camrod_ui_robot/assets/frontend/`

**Launch with external access:**

```bash
ros2 launch camrod_ui ui.launch.py ui_host:=0.0.0.0
```

**Rebuild frontend** (after editing `src/App.js`):

```bash
cd camrod_ui/camrod_ui_robot/assets/frontend
DISABLE_ESLINT_PLUGIN=true npm run build
```

---

## 13. Camping Sites Configuration

Site coordinates are defined in:

```
camrod_planning/config/camping_sites.yaml
```

Each entry maps a site key (e.g. `B1`) to a WGS84 lat/lon pose.  
The UI backend pre-loads this file and dispatches the corresponding `goal_pose` or `mission_key` when the operator selects a site.

Add a new site:

```yaml
B14:
  latitude: 37.123456
  longitude: 127.654321
  yaw_deg: 90.0
```

Then update `site_names` in `camrod_ui/launch/ui.launch.py`:

```python
'site_names': [f'B{i}' for i in range(1, 15)],
```

---

## 14. Map Reference

`camrod_map/config/map_info.yaml` is the **single source of truth** for coordinate frames.

Key fields:

```yaml
map_path:               # absolute path to .osm Lanelet2 file
offset_lat:             # WGS84 origin latitude
offset_lon:             # WGS84 origin longitude
offset_utm_easting:     # UTM easting offset
offset_utm_northing:    # UTM northing offset
yaw_offset_deg:         # map → world rotation
world_frame_id:         # typically "map"
map_frame_id:           # typically "map"
```

Override map path without editing the YAML:

```bash
ros2 launch camrod_bringup bringup.launch.py \
  map_path:=/data/maps/site_v2.osm
```

**GNSS RTK (NTRIP):** configured in `camrod_sensing/config/gnss/ntrip_client.yaml`.  
See `system_network_setting.md` for network / SIM card setup.

---

## 15. Diagnostics

The system health pipeline:

```
[per-module checker nodes]  →  /diagnostics
  ↓
diagnostics_aggregator  →  /system/diagnostics_agg
  ↓
camrod_ui dashboard
```

Checker categories: `hw`, `sensing` (GNSS, IMU, LiDAR, camera, radar, wheel), `localization` (pose, mode, GNSS, init, lanelet, source), `perception` (obstacles), `planning` (lifecycle, costmap, nav\_status, path), `map` (cost\_grid), `platform` (velocity\_converter).

View live diagnostics:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
# or
ros2 topic echo /system/diagnostics_agg
```

---

## 16. Glossary

Key terms used across all CAMROD packages.

| Term | Definition |
|------|-----------|
| **drop zone** | Fixed startup/return waypoint where the robot begins and ends every mission |
| **camping site** | Named delivery destination (e.g. "B3") defined in `camping_sites.yaml` |
| **recall** | Operator-triggered return to a camping site for a second delivery or pickup |
| **engage** | Boolean signal (`/planning/engage`) that enables the cmd\_vel gate to pass velocity commands to the platform |
| **cost-stop** | Safety behavior: robot stops when the inflation cost grid exceeds a threshold along its path |
| **DR\_ONLY** | Dead-reckoning-only localization mode (GNSS lost, running on IMU + wheel odometry) |
| **mission source** | String tag attached to each Nav2 goal describing why it was sent (`startup`, `recall:B3`, `auto_return`, `manual`) |
| **inflation cost grid** | Merged occupancy grid combining LiDAR, radar, lanelet, and path-proximity cost layers |
| **lanelet** | A directed road segment with left/right boundaries in the Lanelet2 map format |
| **road snap** | Projection of a goal or path onto the nearest lanelet centerline |
| **dwell** | Wait period at a camping site before auto-return triggers (`goal_reached_dwell_s`) |
| **dock** | Charging/loading station identified by an AprilTag marker |
| **staging offset** | Pre-approach waypoint in front of the dock, used before final alignment |

---

## 17. Documentation Standards

- Style guide: [docs/templates/README_STYLE_GUIDE.md](docs/templates/README_STYLE_GUIDE.md)
- Package template: [docs/templates/PACKAGE_README_TEMPLATE.md](docs/templates/PACKAGE_README_TEMPLATE.md)
- Parameter naming: [PARAMETER_NAMING_STANDARD.md](PARAMETER_NAMING_STANDARD.md)
- Docs changelog: [docs/DOCS_CHANGELOG.md](docs/DOCS_CHANGELOG.md)

---

## 18. Disabled / Optional Packages

Located in `disable/` (marked with `COLCON_IGNORE` — not built by default):

| Package | Description |
|---------|-------------|
| `vio_bridge` | Visual-Inertial Odometry (ZED SDK / Orbbec SDK) |
| `kimera_vio_bridge` | Kimera VIO integration |
| `config_archive` | Legacy configuration snapshots |

To enable VIO, install the required SDK and remove the `COLCON_IGNORE` file.

---

## Versioning

| Tag | Date | Summary |
|-----|------|---------|
| v1.15 | 2026-06-23 | Obstacle replan monitor (LiDAR/Radar persistent blockage → Smac2D fallback), extended AvgAmrServiceState/PlanningScenario (SITE_ENTRY/UNLOAD_WAIT/RECALL_TO_SITE_ROAD/GUEST_LOADING_WAIT/RETURN_WITH_CARGO/DROP_ZONE_PARKING), UI site-access reservation/occupancy gate, planning_state_machine parking-phase mirror from /AMR_service_state, dynamic-only cost stop gate, lanelet route re-entry bypass, goal_snapper uncontained-snap override, map profile auto-selection, area_exporter polygon centroid + corners export |
| v1.14 | 2026-06-19 | Mission-key semantic planning (PlanningState/MissionKey/Scenario msgs), lanelet raw cost safety stop, local path reset on goal change, goal_snapper pose-jump reissue, lanelet_route_planner + engage_aware_progress_checker plugins, front camera V4L2 fallback + image_raw publisher (PR#14), Ranger BMS charging detection, planning_state_checker, sim diagnostics profile, parking_method bringup arg |
| v1.13 | 2026-06-11 | GNSS dual-antenna heading stabilization (simpleRTK2B Heading moving-baseline RELPOSNED fix) |
| v1.12 | 2026-06-04 | Dual antenna GNSS heading (simpleRTK2B Heading, moving-baseline RELPOSNED), ublox_gps-based single/dual GNSS launch, Python NTRIP with GGA feedback for VRS, remove legacy dGNSS fallback and COG heading fallback |
| v1.11 | 2026-05-28 | Dual econ camera (front GPU/VPI + rear CPU/GStreamer), unified IMU launch (imu_model), camrod_parking → camrod_docking, rear camera calibration, EKF log suppression, costmap start_current |
| v1.10 | 2026-05-21 | Camera sensing refactor (V4L2 publisher), YOLOv9 perception, UI symlink fix, nav2 combo profiles, planning parameter stabilization |
| v1.9 | 2026-05-13 | Planning stability, radar angle fix, Smac2D re-enable |
| v1.8 | 2026-05-08 | ESKF stability, GNSS COG auto-init, DR timeout, platform fixes |
| v1.7 | 2026-04-28 | Ranger platform bridge, DBC status aggregation, planning/system nodes |

## 2026-06-17 Runtime Update

> HH_260617 - Current full-stack baseline after GNSS dual-antenna, system diagnostics, UI goal naming, and parking integration.
> HH_260618 - Final parking is method-selected: `parking_method:=rule_based` uses `camrod_parking`, `parking_method:=docking` uses `camrod_docking`, and launch/system checks require exactly one method to be active.

### Current Mission Flow

1. Operator UI or RViz publishes a raw site goal on `/goal_pose` or `/planning/goal_pose`.
2. `camrod_planning/goal_snapper` converts that site-center goal to the lanelet route goal on `/planning/goal_pose_snapped` and `/planning/goal_pose_snapped_ros`; the latest clicked/UI goal immediately replaces older goals.
3. `local_path_extractor` resets stale controller-local paths for a short hold after each new global route. HH_260618 - This keeps route-heading alignment pointed at the newest goal instead of the previous controller path.
4. Nav2 drives to the lanelet-snap pose and `planning_state_machine` publishes `avg_msgs/PlanningState` on `/planning/state_machine/state`.
5. `planning_cmd_vel_gate_node` checks the raw `/map/cost_grid/lanelet` grid before the ego-cleared inflation grid. HH_260618 - Forward translation is blocked on lane-boundary/off-lane cost while in-place rotation remains allowed.
6. `camrod_parking/site_maneuver` starts for `camping_site_*`: while active, it commands `Twist.linear.y` on `/planning/cmd_vel_raw` for crab entry, rotates 180 degrees, waits for unload, then crab-exits back to the snap pose. HH_260618 - Parking nodes stay silent on `/planning/cmd_vel_raw` while idle so they do not race Nav2.
7. `site_maneuver` requests return through `/planning/state_machine/return_to_drop_zone`.
8. Nav2 returns to the drop-zone snap pose.
9. Final parking is mutually exclusive: `camrod_parking/drop_zone_parking` aligns the parked robot front yaw to the configured charging-station yaw, reverses with yaw/lateral feedback, and stops on `/platform/status/is_charging`; `camrod_docking` remains the AprilTag/opennav method when `parking_method:=docking`.
10. `camrod_system` validates module graph/runtime diagnostics and publishes `/system/status` plus `/system/msgs`.

### Setup and Build

```bash
cd /home/hong/camrod_ws/src
./setup_camrod.sh
./colcon_build.sh
source /home/hong/camrod_ws/install/setup.bash
```

`setup_camrod.sh` is idempotent: it keeps existing external repositories, installs explicit system dependencies, skips Jetson-only docking packages on x86_64, and runs rosdep unless `--no-rosdep` is provided. `colcon_build.sh` builds source packages plus all `external/` package roots, skips x86_64-only unavailable docking/voice pieces only when their system dependencies are unavailable, and keeps the local `camrod_parking` package in the normal source build graph.

### Smoke Tests

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=false
ros2 topic echo /system/diagnostics --once   # check system_checker/final_parking
ros2 topic echo /parking/site_maneuver/status --once  # rule_based only
ros2 topic echo /parking/drop_zone/status --once       # rule_based only
ros2 topic echo /system/status --once
```
