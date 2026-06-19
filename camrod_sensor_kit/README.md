# 🔧 camrod_sensor_kit — URDF, static TF & RobotParams library

**camrod_sensor_kit** — Parameterized URDF generator, static TF publisher, and shared `RobotParams` C++ library for the CAMROD robot.

---

## 📋 Summary

`camrod_sensor_kit` has two responsibilities:

1. **Static TF tree** — expands `urdf/camrod_sensor_kit.xacro` with sensor pose parameters from `config/robot_params.yaml`, then launches `robot_state_publisher` to broadcast the complete sensor frame tree on `/tf_static` and `/robot_description`.
2. **Shared C++ library** — exports `camrod_sensor_kit_lib`, which provides the `RobotParams` struct and `loadRobotParams()` function used by all downstream C++ nodes that need robot geometry at runtime (footprint, wheelbase, sensor offsets).

> **Non-goals:** Does not publish dynamic transforms (all joints are fixed). Does not depend on runtime sensor data — the package completes its job at launch time. Does not perform any health monitoring or status reporting (status node removed in v1.10).

---

## 🚀 Quick Start

```bash
# Build
cd ~/camrod_ws
colcon build --packages-select camrod_sensor_kit
source install/setup.bash

# Standalone: publish /tf_static + /robot_description only
ros2 launch camrod_sensor_kit sensor_kit.launch.py

# Override geometry config
ros2 launch camrod_sensor_kit sensor_kit.launch.py \
  params_file:=/absolute/path/to/robot_params.yaml

# Custom frame names
ros2 launch camrod_sensor_kit sensor_kit.launch.py \
  base_frame_id:=robot_base_link \
  sensor_kit_base_frame_id:=sensor_kit_base_link

# Verify TF is live
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_echo robot_base_link lidar_link
```

---

## 🗺️ System Position

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph LR
  YAML[(robot_params.yaml)]:::config --> SKIT

  subgraph SKIT_BOX["🔧 camrod_sensor_kit"]
    SKIT[sensor_kit\nfoundation]:::system
  end

  SKIT -.->|/tf_static| PLAT([🤖 camrod_platform]):::platform
  SKIT -.->|/tf_static| SENS([🎯 camrod_sensing]):::sensing
  SKIT -.->|/tf_static| LOC([📍 camrod_localization]):::localization
  SKIT -.->|/tf_static| PLAN([🧭 camrod_planning]):::planning
  SKIT -.->|/tf_static| PARK([🅿️ camrod_docking]):::docking
  SKIT -.->|loadRobotParams| PLAT
  SKIT -.->|loadRobotParams| SENS

  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef docking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef config       fill:#FFFBEB,stroke:#D97706,stroke-width:1.5px,color:#92400E;
```

> Dashed arrows = dependency (TF lookup or compile-time `loadRobotParams`). All consumers require this package to be running at startup.

---

## ⚙️ Execution Modes

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
flowchart TD
  START([Launch sensor_kit?]):::system

  START --> Q1{Launched by\nparent bringup?}:::system
  Q1 -->|No — standalone| SA["`**Standalone Mode**
  ros2 launch camrod_sensor_kit
    sensor_kit.launch.py
  namespace: sensor_kit`"]:::system
  Q1 -->|Yes — included| SB["`**Platform Mode**
  Included by camrod_bringup /
    camrod_platform launch
  namespace: may be overridden`"]:::platform

  SA --> OUT([robot_state_publisher\npublishes /tf_static\n+ /robot_description]):::system
  SB --> OUT

  classDef system   fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef platform fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
```

| Mode | How launched | What runs |
|---|---|---|
| **Standalone** | `ros2 launch camrod_sensor_kit sensor_kit.launch.py` | `robot_state_publisher` only under namespace `sensor_kit` |
| **Inside platform launch** | Included by `camrod_bringup` / `camrod_platform` | Same `robot_state_publisher`; namespace may be overridden by the parent launch |

---

## 🏗️ Runtime Architecture

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph TD
  YAML[(robot_params.yaml)]:::config  --> LAUNCH
  XACRO[(camrod_sensor_kit.xacro)]:::config --> LAUNCH

  subgraph LAUNCH_BOX["sensor_kit.launch.py"]
    LAUNCH[🛠️ launch\nprocessing]:::system
  end

  LAUNCH -->|xacro expansion| DESC[📦 robot_description\nstring]:::system
  DESC --> RSP(robot_state_publisher):::system
  RSP --> TFSTATIC((/tf_static)):::topic
  RSP --> URDESC((/robot_description)):::topic

  YAML -. loadRobotParams .-> LIB[📦 camrod_sensor_kit_lib]:::system
  LIB -. RobotParams struct .-> PLAT([🤖 camrod_platform nodes]):::platform
  LIB -. RobotParams struct .-> SENS([🎯 camrod_sensing nodes]):::sensing

  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef config       fill:#FFFBEB,stroke:#D97706,stroke-width:1.5px,color:#92400E;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
```

The launch file reads `robot_params.yaml` at launch time, converts all degree-based pose fields to radians, then calls `xacro` to produce the robot description string. The URDF string is passed directly as a parameter to `robot_state_publisher`, which publishes all static joint transforms in a single `/tf_static` message.

---

## 🌳 URDF Frame Tree

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#F1F5F9', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#64748B', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph TD
  subgraph CONV["Convention frames (not published by this package)"]
    WORLD[🌐 world]:::system
    MAP[🗺️ map]:::localization
  end
  subgraph ROBOT["robot_base_link — Nav2 footprint origin"]
    BASE[🤖 robot_base_link\nfixed joints only]:::platform
    subgraph SKIT["sensor_kit_base_link — sensor mount origin"]
      SKB[🔧 sensor_kit_base_link]:::system
      IMU[📡 imu_link\n0.28, 0.0, 0.2]:::sensing
      GNSS[📡 gnss_link\n0.0, 0.0, 0.0]:::sensing
      LIDAR[📡 lidar_link\n0.68, 0.0, 0.45]:::sensing
      CAMF[📡 camera_front_link\n0.40, 0.0, 0.46]:::sensing
      CAMR[📡 camera_rear_link\n0.10, 0.0, 0.46]:::sensing
      subgraph RADAR["Radar sensors (direct to sensor_kit_base_link)"]
        RF[📡 radar_front_link]:::sensing
        RL1[📡 radar_left1_link]:::sensing
        RL2[📡 radar_left2_link]:::sensing
        RR1[📡 radar_right1_link]:::sensing
        RR2[📡 radar_right2_link]:::sensing
        RR[📡 radar_rear_link]:::sensing
      end
    end
  end

  WORLD --> MAP
  MAP --> BASE
  BASE --> SKB
  SKB --> IMU & GNSS & LIDAR & CAMF & CAMR
  SKB --> RF & RL1 & RL2 & RR1 & RR2 & RR

  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
```

> `world` and `map` are convention frames connected by the localization pipeline — not published by this package. `robot_base_link` is the physical footprint origin for Nav2 and collision checking.

---

## 📡 Interface Contract

### Outputs

| Topic | Type | Consumer | Rate | Meaning |
|---|---|---|---|---|
| `/tf_static` | `tf2_msgs/TFMessage` | all packages doing sensor frame lookups | once at startup | Full static joint tree from `world` to each sensor link |
| `/robot_description` | `std_msgs/String` | RViz, URDF consumers | once at startup | URDF XML string expanded from xacro |

### Library Interface

```cpp
// In any downstream CMakeLists.txt:
find_package(camrod_sensor_kit REQUIRED)
ament_target_dependencies(your_node camrod_sensor_kit)

// In C++ source:
#include "camrod_sensor_kit/robot_params.hpp"
camrod::RobotParams params = camrod::loadRobotParams(node);
```

`loadRobotParams(node)` declares ROS parameters on `node` and returns a populated `RobotParams` snapshot. The node must be passed the `robot_params.yaml` config file at launch.

---

## 🔑 Key Behaviors

### Static TF Publication

| Attribute | Detail |
|---|---|
| Trigger | `robot_state_publisher` startup |
| Internal logic | xacro expansion produces URDF; RSP reads `robot_description` param and latches all fixed joint transforms |
| Output effect | `/tf_static` latched once; all packages that call `tf2::lookupTransform` for sensor frames can resolve immediately |
| Operator-visible symptom | TF lookup errors in any downstream package after a fresh start indicate this package did not start correctly |
| Related params | `params_file`, `base_frame_id`, `sensor_kit_base_frame_id` |
| Related topics | `/tf_static`, `/robot_description` |

### RobotParams Load

| Attribute | Detail |
|---|---|
| Trigger | Downstream C++ node calls `camrod::loadRobotParams(node)` at construction |
| Internal logic | Calls `node->declare_parameter()` for each field; YAML degree values are converted to radians internally |
| Output effect | Returns a `RobotParams` struct used for footprint polygon, wheelbase, and sensor offset calculations |
| Operator-visible symptom | If `robot_params.yaml` is not loaded (missing params_file), default C++ struct values are silently used |
| Related params | All fields under `robot.*`, `imu.*`, `gnss.*`, `lidar.*`, `camera.*` |
| Related topics | None |

---

## 🚀 Launch Arguments

```bash
ros2 launch camrod_sensor_kit sensor_kit.launch.py [ARG:=VALUE ...]
```

| Argument | Default | Description |
|---|---|---|
| `params_file` | `config/robot_params.yaml` (from package share) | Robot geometry and sensor pose definitions |
| `module_namespace` | `sensor_kit` | ROS 2 node namespace |
| `base_frame_id` | `robot_base_link` | Parent robot body frame name |
| `sensor_kit_base_frame_id` | `sensor_kit_base_link` | Sensor kit base frame under `robot_base_link` |
| `map_frame_id` | `map` | Fixed world frame |

---

## ⚙️ Config — `config/robot_params.yaml`

All sensor poses are relative to `sensor_kit_base_link`. YAML angles are in **degrees**; `loadRobotParams()` converts them to radians internally.

### 🤖 Robot Body Parameters

| YAML key | Default (YAML) | C++ default | Unit | Notes |
|---|---|---|---|---|
| `robot.wheelbase` | `0.5` | `1.10` | m | **TODO:verify** — YAML (0.5 m) differs from C++ default (1.10 m) |
| `robot.track_width` | `3.2` | `0.65` | m | **TODO:verify** — see warning below |
| `robot.length` | `0.8` | `1.40` | m | Robot body length |
| `robot.width` | `0.4` | `0.70` | m | Robot body width |
| `robot.height` | `0.8` | `1.20` | m | Robot body height |
| `robot.wheel_radius` | `0.08` | `0.15` | m | Effective rolling radius |
| `robot.encoder_resolution` | `2048` | `2048` | ticks/rev | Encoder ticks per wheel revolution |
| `robot.drive_type` | `"ackermann"` | `"ackermann"` | — | Platform drive model |
| `ground_z_offset` | `5.3` | — | m | **TODO:verify** — see warning below |

> ⚠️ **Suspicious values in `robot_params.yaml` — verify before deployment:**
>
> - `robot.track_width: 3.2 m` — this is far wider than a typical CAMROD platform (C++ default is `0.65 m`). This value is likely a placeholder and will produce incorrect footprint polygons and collision geometry.
> - `ground_z_offset: 5.3 m` — this is the fallback Z offset used when map-ground estimation is unavailable. A 5.3 m offset is physically implausible for a ground robot and will corrupt any Z-dependent transform lookups.

### 📡 Sensor Mount Poses (relative to `sensor_kit_base_link`)

| Sensor | x (m) | y (m) | z (m) | roll (deg) | pitch (deg) | yaw (deg) |
|---|---|---|---|---|---|---|
| `imu` | 0.28 | 0.0 | 0.2 | 0.0 | 0.0 | 0.0 |
| `gnss` | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| `lidar` | 0.68 | 0.0 | 0.45 | 0.0 | 0.4 | 0.0 |
| `camera.front` | 0.40 | 0.0 | 0.46 | 0.0 | 0.0 | 0.0 |
| `camera.rear` | 0.10 | 0.0 | 0.46 | 0.0 | 0.0 | 180.0 |
| `radar.front` | 0.64 | 0.0 | 0.45 | 0.0 | 0.0 | 0.0 |
| `radar.left1` | -0.1 | 0.2 | 0.45 | 0.0 | 0.0 | 90.0 |
| `radar.left2` | 0.5 | 0.2 | 0.45 | 0.0 | 0.0 | 90.0 |
| `radar.right1` | -0.1 | -0.2 | 0.45 | 0.0 | 0.0 | -90.0 |
| `radar.right2` | 0.5 | -0.2 | 0.45 | 0.0 | 0.0 | -90.0 |
| `radar.rear` | -0.1 | 0.0 | 0.45 | 0.0 | 0.0 | 180.0 |

### `urdf/camrod_sensor_kit.xacro`

Parameterized URDF: `robot_base_link` body box, `sensor_kit_base_link` joint, and one link+joint macro per sensor. All pose arguments (`{sensor}_xyz`, `{sensor}_rpy`) are injected by the launch file from the values above.

---

## 🔍 Validation

```bash
# Confirm /tf_static is published
ros2 topic echo /tf_static --once | grep frame_id

# Lookup each sensor frame from robot_base_link
for frame in imu_link gnss_link lidar_link camera_front_link camera_rear_link \
  radar_front_link radar_rear_link; do
  ros2 run tf2_ros tf2_echo robot_base_link $frame
done

# Inspect URDF in RViz
ros2 run rviz2 rviz2

# Print loaded robot_params directly
ros2 param get /sensor_kit/robot_state_publisher robot_description | head -5
```

---

## 🩺 Troubleshooting

<details>
<summary><strong>TF lookup fails for sensor frame</strong></summary>

- Confirm `camrod_sensor_kit` is running: `ros2 node list | grep robot_state_publisher`
- Check that `params_file` points to the correct `robot_params.yaml`: `ros2 launch camrod_sensor_kit sensor_kit.launch.py --show-args`
- Ensure no TF tree cycles exist: `ros2 run tf2_tools view_frames`

</details>

<details>
<summary><strong>URDF wrong in RViz</strong></summary>

The URDF is generated at launch time from xacro. A stale `.yaml` file or wrong `params_file` argument will silently produce the wrong geometry. To force a rebuild: stop the node, edit `robot_params.yaml`, re-launch.

</details>

<details>
<summary><strong>`robot_params.yaml` change has no effect</strong></summary>

`robot_state_publisher` reads the URDF only at startup. Changes require a restart. Downstream C++ nodes calling `loadRobotParams()` also read parameters at construction time; they must be restarted to pick up YAML changes.

</details>

<details>
<summary><strong>Multiple `robot_state_publisher` conflicts</strong></summary>

If both `camrod_sensor_kit` standalone and a parent bringup launch start `robot_state_publisher`, they will conflict on `/robot_description` and `/tf_static`. Use `module_namespace` to isolate, or ensure only one instance runs per namespace.

</details>

---

## 🔗 Related Docs

- [`../README.md`](../README.md) — Top-level CAMROD workspace overview
- [`../camrod_platform/README.md`](../camrod_platform/README.md) — consumes `RobotParams` for footprint and drive model
- [`../camrod_sensing/README.md`](../camrod_sensing/README.md) — consumes `/tf_static` for all sensor frame lookups
- [`../camrod_docking/README.md`](../camrod_docking/README.md) — consumes `/tf_static` for parking geometry
- [`../PARAMETER_NAMING_STANDARD.md`](../PARAMETER_NAMING_STANDARD.md) — canonical parameter naming conventions

## 2026-06-17 Runtime Update

> HH_260617: Sensor-kit TF remains shared geometry for planning, platform, localization, sensing, and parking consumers.

`camrod_parking` relies on the same base frame convention as the rest of the stack: `Twist.linear.y > 0` means body-left crab motion in the robot base frame, and drop-zone yaw alignment compares the current `/localization/pose` yaw against the configured map-frame station yaw.
