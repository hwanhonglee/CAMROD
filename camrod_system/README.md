# 🩺 camrod_system — Diagnostic checkers & aggregator

**camrod_system** — Per-module diagnostic checkers and diagnostics aggregator for the full CAMROD stack.

---

## 📋 Summary

`camrod_system` provides the health monitoring layer for all CAMROD runtime packages. It runs more than 20 independent checker nodes, each subscribing to one module's runtime topics and publishing `diagnostic_msgs/DiagnosticStatus` entries to `/diagnostics`. A central `aggregator_node` collects all entries, detects stale reporters, groups them into a tree by subsystem, and publishes the consolidated result to `/diagnostics_agg`.

A separate lightweight system tools track (`enable_system_tools: true`) checks that required ROS 2 nodes and topics are alive and publishes a secondary summary to `/system/diagnostics_agg_tools`.

> **Non-goals:** Reports health status only — does **not** enforce safety actions (e-stop, speed reduction, disengagement). Does not contain autonomous decision logic; consumers (e.g., `camrod_ui`) decide what to do with the health data. Does not check `camrod_parking` yet (TODO: parking checker category).

---

## 🚀 Quick Start

```bash
# Build
cd ~/camrod_ws
colcon build --packages-select camrod_system
source install/setup.bash

# Full stack (all checkers + aggregator)
ros2 launch camrod_system system.launch.py

# Disable Ranger platform checker (default)
ros2 launch camrod_system system.launch.py enable_platform:=false

# Watch aggregated diagnostics
ros2 topic echo /system/diagnostics_agg

# Watch per-checker raw diagnostics
ros2 topic echo /system/diagnostics
```

---

## 🗺️ System Position

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph LR
  MAP([🗺️ camrod_map]):::mapping      -->|/map/...| SYS
  SENS([📡 camrod_sensing]):::sensing  -->|/sensing/...| SYS
  LOC([📍 camrod_localization]):::localization -->|/localization/...| SYS
  PER([👁️ camrod_perception]):::perception -->|/perception/...| SYS
  PLAN([🧭 camrod_planning]):::planning -->|/planning/...| SYS
  PLAT([🤖 camrod_platform]):::platform -->|/platform/...| SYS
  HW{{🖥️ CPU / GPU / Network}}:::hardware -->|metrics| SYS

  subgraph SYS_BOX["🩺 camrod_system"]
    SYS(🩺 aggregator\n+ checkers):::system
  end

  SYS -->|/diagnostics_agg| UI([🖥️ camrod_ui]):::ui

  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
```

> `camrod_system` is a **pure observer**. All upstream packages publish data regardless of whether the system checkers are running.

---

## 🏗️ Runtime Architecture

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph TD
  HW{{🖥️ CPU / GPU / Network}}:::hardware --> HWCHK(🧩 hw_checker\ngpu_checker\nnetwork_checker):::system
  SENS((📡 /sensing/*)):::topic --> SENSCHK(🧩 gnss/imu/lidar/radar/camera\nwheel_odometry/cost_grid\nvelocity_converter checkers):::sensing
  LOC((📍 /localization/*)):::topic --> LOCCHK(🧩 localization_gnss/mode/pose\ninit/source/lanelet checkers):::localization
  PER((👁️ /perception/*)):::topic --> PERCHK(🧩 perception_obstacle_checker):::perception
  MAP((🗺️ /map/cost_grid/lanelet)):::topic --> MAPCHK(🧩 map_cost_grid_checker):::mapping
  PLAN((🧭 /planning/*)):::topic --> PLANCHK(🧩 planning_lifecycle/costmap\nnav_status/path checkers):::planning
  PLAT((🤖 /platform/*)):::topic --> PLATOP(🧩 ranger_platform_checker\n⚠️ optional):::platform

  HWCHK & SENSCHK & LOCCHK & PERCHK & MAPCHK & PLANCHK & PLATOP --> DIAG((📡 /diagnostics)):::topic

  DIAG --> AGG(🩺 aggregator_node):::system
  AGG --> DIAGAGG((📡 /diagnostics_agg)):::topic

  DIAG --> SYSCHK(🩺 system_checker_node):::system
  SYSCHK --> SYSDIAG((📡 /system/diagnostics)):::topic
  SYSDIAG --> SYSAGG(🩺 diagnostics_aggregator_node):::system
  SYSAGG --> TOOLS((📡 /system/diagnostics_agg_tools)):::topic

  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
```

> When launched under the default namespace `system`, all topics are prefixed: `/system/diagnostics` and `/system/diagnostics_agg`.

---

## 🔑 Key Behaviors

### Module Readiness Decision Tree

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
flowchart TD
  A[/🔔 /diagnostics_agg received/]:::system --> B{msg.status\nempty?}:::system
  B -->|yes| C[🔴 ready = false\nmessage: 'no diagnostics yet']:::platform
  B -->|no| D{any ERROR\nlevel entries?}:::system
  D -->|yes| E[🔴 ready = false\nmessage: 'diagnostics errors: N']:::platform
  D -->|no| F[🟢 ready = true\nmessage: 'ready']:::localization

  C --> G[⛔ operation_mode = STOP]:::platform
  E --> G
  F --> H{engaged\n== true?}:::system
  H -->|yes| I[✅ operation_mode = AUTO]:::localization
  H -->|no| J[⛔ operation_mode = STOP]:::platform

  subgraph PARKING_NOTE["🅿️ Parking (TODO)"]
    PTODO[parking checker\nnot yet implemented]:::parking
  end

  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef parking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
```

### System Readiness Sequence

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}}}%%
sequenceDiagram
  autonumber
  participant SENS as 🎯 Sensing
  participant LOC as 📍 Localization
  participant PLAN as 🧭 Planning
  participant PARK as 🅿️ Parking
  participant PLAT as 🤖 Platform
  participant SYS as 🩺 System
  participant UI as 🖥️ UI

  SENS->>SYS: /sensing/* topics arrive
  SYS->>SYS: gnss/imu/lidar/radar/camera checkers → OK

  LOC->>SYS: /localization/initial_match_ok = true
  SYS->>SYS: localization_init_checker → OK

  LOC->>SYS: /localization/pose_with_covariance @ 30 Hz
  SYS->>SYS: localization_pose_checker → OK

  PLAN->>SYS: Nav2 lifecycle nodes active (get_state = active)
  SYS->>SYS: planning_lifecycle_checker → OK

  PLAT->>SYS: /platform/status/wheel_odometry arrives
  SYS->>SYS: wheel_odometry_checker → OK

  Note over PARK,SYS: 🅿️ Parking checker — TODO: not yet implemented

  SYS->>UI: /diagnostics_agg with zero ERROR entries
  UI->>UI: ready = true; if engaged → operation_mode = AUTO
```

---

## 🧩 Checker Categories

| Category | Checker nodes | Config subdir | Parking |
|---|---|---|---|
| `hw` | hw_checker, gpu_checker, network_checker | `config/diagnostics/default/hw/` | — |
| `sensing` | gnss, imu, lidar, radar, camera, wheel_odometry, cost_grid, velocity_converter | `config/diagnostics/default/sensing/` | — |
| `localization` | localization_gnss, localization_mode, localization_pose, localization_init, localization_source, localization_lanelet | `config/diagnostics/default/localization/` | — |
| `perception` | perception_obstacle | `config/diagnostics/default/perception/` | — |
| `map` | map_cost_grid | `config/diagnostics/default/map/` | — |
| `planning` | planning_lifecycle, planning_costmap, planning_nav_status, planning_path | `config/diagnostics/default/planning/` | — |
| `platform` | ranger_platform (optional) | `config/diagnostics/default/platform/` | — |
| `parking` | — | — | **TODO**: not yet implemented |

---

## 📡 Interface Contract

### Inputs

| Topic | Type | Required | Producer | Rate | Meaning |
|---|---|---|---|---|---|
| `/sensing/gnss/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | Yes | camrod_sensing | 5 Hz | GNSS fix for `gnss_checker` |
| `/sensing/imu/data` | `sensor_msgs/Imu` | Yes | camrod_sensing | 100 Hz | IMU data for `imu_checker` |
| `/sensing/lidar/*/points` | `sensor_msgs/PointCloud2` | Yes | camrod_sensing | 10 Hz | LiDAR points for `lidar_checker` |
| `/sensing/radar/*/range` | `sensor_msgs/Range` | Yes | camrod_sensing | variable | Radar ranges for `radar_checker` |
| `/sensing/camera/*/image_raw` | `sensor_msgs/Image` | Yes | camrod_sensing | 30 Hz | Camera frames for `camera_checker` |
| `/platform/status/wheel_odometry` | `nav_msgs/Odometry` | Yes | camrod_platform | variable | Wheel odometry for `wheel_odometry_checker` |
| `/planning/cost_grid/inflation` | `nav_msgs/OccupancyGrid` | Yes | camrod_planning | variable | Inflated cost grid for `cost_grid_checker` |
| `/sensing/platform_velocity_converter/twist_with_covariance` | `geometry_msgs/TwistWithCovarianceStamped` | Yes | camrod_sensing | variable | Velocity for `velocity_converter_checker` |
| `/sensing/gnss/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | Yes | camrod_localization | 5 Hz | GNSS-derived pose for `localization_gnss_checker` |
| `/localization/mode` | `avg_msgs/AvgLocalizationMode` | Yes | camrod_localization | variable | ESKF mode for `localization_mode_checker` |
| `/localization/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | Yes | camrod_localization | 30 Hz | Pose estimate for `localization_pose_checker` |
| `/localization/initial_match_ok` | `std_msgs/Bool` | Yes | camrod_localization | event | Drop zone init flag for `localization_init_checker` |
| `/localization/pose_source` | `std_msgs/String` | Yes | camrod_localization | event | Active pose source for `localization_source_checker` |
| `/localization/lanelet_pose` | `geometry_msgs/PoseStamped` | Yes | camrod_localization | variable | Lanelet-projected pose for `localization_lanelet_checker` |
| `/map/cost_grid/lanelet` | `nav_msgs/OccupancyGrid` | Yes | camrod_map | variable | Lanelet cost grid for `map_cost_grid_checker` |
| `/perception/obstacles` | `sensor_msgs/PointCloud2` | Yes | camrod_perception | 10 Hz | Fused obstacles for `perception_obstacle_checker` |
| `/perception/camera/detections_2d` | `vision_msgs/Detection2DArray` | Yes | camrod_perception | variable | Camera detections for `perception_obstacle_checker` |
| `/planning/global_path` | `nav_msgs/Path` | Yes | camrod_planning | variable | Global path for `planning_path_checker` |
| `/planning/local_path` | `nav_msgs/Path` | Yes | camrod_planning | variable | Local path for `planning_path_checker` |

### Outputs

| Topic | Type | Consumer | Rate | Meaning |
|---|---|---|---|---|
| `/system/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `aggregator_node` | ~1 Hz per checker | Raw per-checker status entries |
| `/system/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | `camrod_ui`, RViz | 1 Hz | Aggregated tree of all module statuses |
| `/system/diagnostics_agg_tools` | `diagnostic_msgs/DiagnosticArray` | monitoring tools | 1 Hz | System tools lightweight aggregation |

---

## 🚀 Launch Arguments

```bash
ros2 launch camrod_system system.launch.py [ARG:=VALUE ...]
```

| Argument | Default | Description |
|---|---|---|
| `module_namespace` | `system` | ROS 2 node namespace; topics appear as `/system/diagnostics` |
| `config_profile` | `default` | Config profile directory under `config/diagnostics/`; falls back to `default` if profile dir missing |
| `enable_checkers` | `true` | Launch all module checker nodes and `aggregator_node` |
| `enable_platform` | `false` | Launch `ranger_platform_checker_node` (only if binary is installed) |
| `enable_system_tools` | `true` | Launch `system_checker_node`, `system_diagnostic_node`, `diagnostics_aggregator_node` |
| `system_checker_param_file` | `config/system_checker.yaml` | Parameter file for `system_checker_node` |

---

## ⚙️ Config

All config files are under `config/diagnostics/default/`.

<details>
<summary><strong>Full diagnostics_agg YAML — sample payload</strong></summary>

```yaml
header:
  stamp: {sec: 1716300000, nanosec: 0}
status:
  - name: "Robot/Sensing/GNSS"
    level: 0          # 0=OK, 1=WARN, 2=ERROR
    message: "OK"
    hardware_id: ""
    values:
      - {key: "frequency", value: "5.1 Hz"}
      - {key: "status", value: "OK"}
  - name: "Robot/Localization/Init"
    level: 0
    message: "initial_match_ok received"
    hardware_id: ""
    values: []
  - name: "Robot/Planning/Lifecycle"
    level: 0
    message: "all lifecycle nodes active"
    hardware_id: ""
    values:
      - {key: "planner_server", value: "active"}
      - {key: "controller_server", value: "active"}
      - {key: "bt_navigator", value: "active"}
      - {key: "behavior_server", value: "active"}
```

</details>

| File | Purpose |
|---|---|
| `aggregator/diagnostics_config.yaml` | Master topic registry: `name`, `group`, `timeout_s` for each checker entry. `global.publish_rate_hz: 1.0`, `global.timeout_s: 5.0` |
| `aggregator/robot_diagnostics_aggregator.yaml` | `diagnostic_aggregator` analyzer tree configuration; `pub_rate: 1.0` |
| `hw/hw_gpu_checker.yaml` | CPU/memory/disk/GPU thresholds and container host paths |
| `hw/network_checker.yaml` | Network interface check config |
| `sensing/gnss_checker.yaml` | `expected_hz: 5.0`, `stale_timeout_s: 2.0` |
| `sensing/imu_checker.yaml` | `expected_hz: 100.0`, `stale_timeout_s: 0.5` |
| `sensing/lidar_checker.yaml` | `expected_hz: 10.0`, `min_point_count`, `max_point_count` |
| `sensing/radar_checker.yaml` | `stale_timeout_s: 1.0` |
| `sensing/camera_checker.yaml` | `expected_fps: 30.0`, `expected_width`, `expected_height` |
| `sensing/wheel_odometry_checker.yaml` | `stale_timeout_s: 1.0` |
| `sensing/cost_grid_checker.yaml` | `stale_timeout_s: 2.0` |
| `sensing/velocity_converter_checker.yaml` | `stale_timeout_s: 1.0` |
| `localization/localization_gnss_checker.yaml` | `expected_hz: 5.0`, `cov_warn_threshold` |
| `localization/localization_mode_checker.yaml` | `conf_warn: 0.6`, `innov_warn: 3.0` |
| `localization/localization_pose_checker.yaml` | `expected_hz: 30.0`, `cov_warn: 1.0`, `max_jump_m: 2.0` |
| `localization/localization_init_checker.yaml` | `stale_timeout_s: 5.0` |
| `localization/localization_source_checker.yaml` | Pose source switching detection |
| `localization/localization_lanelet_checker.yaml` | `stale_timeout_s: 2.0` |
| `map/map_cost_grid_checker.yaml` | `stale_timeout_s: 5.0` |
| `perception/perception_obstacle_checker.yaml` | `expected_hz: 10.0`, `min_count`, `max_count` |
| `planning/planning_lifecycle_checker.yaml` | Nav2 nodes to poll, `poll_rate_hz: 2.0` |
| `planning/planning_costmap_checker.yaml` | `stale_timeout_s: 3.0` |
| `planning/planning_nav_status_checker.yaml` | `abort_rate_warn` threshold |
| `planning/planning_path_checker.yaml` | `stale_timeout: 3.0`, `min_points_warn: 5` |
| `platform/ranger_platform_checker.yaml` | Ranger-specific platform diagnostics |

### Canonical Parameter Names

| Name | Meaning |
|---|---|
| `publish_rate_hz` | How often the aggregator publishes `/diagnostics_agg` |
| `poll_rate_hz` | How often a checker actively polls a service (e.g., Nav2 lifecycle) |
| `stale_timeout_s` | Seconds before a topic is considered stale and elevated to WARN |
| `grace_period_s` | Post-startup grace window before a checker raises errors |
| `startup_grace_s` | Grace period for system_checker_node after launch |
| `fallback_warn_s` | Seconds in fallback state before WARN is raised |
| `fallback_error_s` | Seconds in fallback state before ERROR is raised |

---

## 🔍 Validation

```bash
# Confirm aggregator is running
ros2 node list | grep diagnostics_agg

# Watch aggregated output
ros2 topic echo /system/diagnostics_agg

# Check individual checker rate
ros2 topic hz /system/diagnostics
```

---

## 🩺 Troubleshooting

<details>
<summary><strong>/diagnostics_agg is empty</strong></summary>

- Confirm `aggregator_node` is running: `ros2 node list | grep diagnostics_agg`
- Confirm at least one checker is publishing: `ros2 topic hz /system/diagnostics`
- Check that `diagnostics_config.yaml` exists at the resolved `config_profile` path.
- If `enable_checkers:=false` was used, no checker nodes will run.

</details>

<details>
<summary><strong>One module stuck in WARN forever</strong></summary>

- Identify which topic is stale: `ros2 topic hz <topic_name>` against the topic listed in the checker YAML.
- Check the source package is running: `ros2 node list | grep <package>`.
- If the source is dead, restart the upstream package — `camrod_system` does not restart it automatically.
- Increase `stale_timeout_s` temporarily if the module needs more time after startup.

</details>

<details>
<summary><strong>UI shows WAITING_FOR_READY indefinitely</strong></summary>

`camrod_ui` sets `ready = false` when `/diagnostics_agg` contains any ERROR-level entry. Run `ros2 topic echo /system/diagnostics_agg` and look for `level: 2` entries.

Common causes: LiDAR not publishing, localization not initialized (`initial_match_ok` not received), Nav2 lifecycle nodes not in active state.

</details>

<details>
<summary><strong>Profile change ignored</strong></summary>

The `config_profile` argument selects a subdirectory under `config/diagnostics/`. If the directory does not exist, the launch file silently falls back to `default`.

Verify: `ls $(ros2 pkg prefix camrod_system)/share/camrod_system/config/diagnostics/`

</details>

---

## 🔗 Related Docs

- [`../README.md`](../README.md) — Top-level CAMROD workspace overview
- [`../camrod_sensing/README.md`](../camrod_sensing/README.md)
- [`../camrod_localization/README.md`](../camrod_localization/README.md)
- [`../camrod_perception/README.md`](../camrod_perception/README.md)
- [`../camrod_map/README.md`](../camrod_map/README.md)
- [`../camrod_planning/README.md`](../camrod_planning/README.md)
- [`../camrod_platform/README.md`](../camrod_platform/README.md)
- [`../camrod_parking/README.md`](../camrod_parking/README.md)
- [`../camrod_ui/README.md`](../camrod_ui/README.md) — consumes `/diagnostics_agg`
- [`../PARAMETER_NAMING_STANDARD.md`](../PARAMETER_NAMING_STANDARD.md) — canonical parameter naming conventions
