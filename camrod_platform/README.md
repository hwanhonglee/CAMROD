# 🤖 camrod_platform — Final cmd_vel gate, visualization & sensor kit bridge

> ⚠️ **Safety boundary** — `/platform/cmd_vel` is the LAST topic before the robot moves. Verify `engage`, `drive_enable`, `estop`, and `cmd_vel` before debugging any motion issue. Do NOT disable the gate (`cmd_vel_gate_enable:=false`) on a live robot.

---

## 1. 📋 Summary

`camrod_platform` is the **last safety boundary before hardware**. It sits between the planning stack and the Ranger CAN driver, and it is responsible for three distinct concerns:

1. **`cmd_vel_gate_node`** — gates `/planning/cmd_vel` behind a four-signal safety interlock before the velocity command reaches the robot's drive motors.
2. **`robot_visualization_node`** — renders the robot body, sensor frames, planning boundary, and debug range rings as RViz MarkerArray at 20 Hz.
3. **`sensor_kit_bridge`** — includes `camrod_sensor_kit` to publish all static TF transforms and `/robot_description` (URDF) for every sensor frame.

**Safety interlock — `/platform/cmd_vel` is only published when ALL four conditions hold simultaneously:**
- `/platform/drive_enable` has published `true`
- `/planning/engage` has published `true` (when `engage_source_mode:=planning_engage`)
- `/platform/status/estop` is `false` (when `estop_source_mode:=platform_status`)
- The gate node itself is running (`cmd_vel_gate_enable:=true`)

When any condition is not met, the gate publishes a zero `Twist` on `/platform/cmd_vel` (`publish_zero_when_blocked: true`). The gate never passes through a stale velocity from a previous engage cycle.

---

## 2. ⚡ Quick Start

```bash
# Full platform stack (gate + visualization + Ranger CAN + sensor_kit TF)
ros2 launch camrod_platform platform.launch.py

# Without Ranger CAN driver (simulation or bench testing)
ros2 launch camrod_platform platform.launch.py ranger_driver_enable:=false

# Disable the cmd_vel gate (direct pass-through, use only for hardware bench tests)
ros2 launch camrod_platform platform.launch.py cmd_vel_gate_enable:=false
```

---

## 3. 🗺️ System Position

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph LR
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  PLAN([🧭 camrod_planning]) ==>|/planning/cmd_vel| PLAT
  LOC([📍 camrod_localization]) -->|/localization/pose| PLAT
  SENS([🎯 camrod_sensing]) -->|/sensing/gnss/pose| PLAT
  MAP([🗺️ camrod_map]) -.->|/map/markers| PLAT

  PLAT([🤖 camrod_platform]) ==>|/platform/cmd_vel| HW{{🛠️ Ranger CAN Driver}}
  PLAT -->|/platform/status/*| HW
  PLAT -->|odometry| LOC
  PLAT -->|/platform/status/velocity| SENS
  PLAT -->|drive_enabled| SYS([🩺 camrod_system])
  PLAT -->|TF + robot_description| ALL([📦 All Packages])

  class PLAN planning
  class LOC localization
  class SENS sensing
  class MAP mapping
  class SYS system
  class HW hardware
  class PLAT highlight
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · 🛠️ Hardware · 📦 External · ==> critical · -.-> optional

**Upstream:** camrod_planning, camrod_localization, camrod_sensing, camrod_map

**Downstream (platform outputs consumed by):** camrod_localization (odometry), camrod_sensing (velocity), camrod_system (diagnostics), Ranger CAN hardware driver

*Figure 1 — `camrod_platform` system position. The platform node is the sole choke-point between Nav2 cmd_vel and the physical drive motors.*

---

## 4. 🏗️ Runtime Architecture

`platform.launch.py` includes four sub-launches in parallel (no inter-launch stagger within platform):

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph TD
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;

  PLAN_IN((/planning/cmd_vel)) ==> GATE
  ENGAGE((/planning/engage)) ==> GATE
  DRIVE_EN((/platform/drive_enable)) ==> GATE
  ESTOP((/platform/status/estop)) ==> GATE

  subgraph PLAT_PKG [🤖 camrod_platform]
    subgraph GATE_SG [🚦 cmd_vel_gate_node]
      GATE(cmd_vel_gate_node\ncmd_vel_gate.launch.py)
    end

    subgraph VIZ_SG [🎨 robot_visualization_node]
      VIZ(robot_visualization_node\nrobot_visualization.launch.py)
    end

    subgraph BRIDGE_SG [🔧 sensor_kit_bridge]
      BRIDGE(sensor_kit_bridge\nsensor_kit_bridge.launch.py)
      RANGER[[ranger.launch.py\nranger_driver_enable:=true]]
    end
  end

  GATE ==>|always active| CMDVEL_OUT((/platform/cmd_vel))
  CMDVEL_OUT ==> HW{{🛠️ Ranger CAN Driver}}

  LOCPOSE((/localization/pose)) --> VIZ
  GNSSPOSE((/sensing/gnss/pose)) -.->|fallback| VIZ
  VIZ --> MARKERS((/platform/robot/markers))
  VIZ --> BOUNDARY((/platform/robot/planning_boundary))

  BRIDGE --> TF(TF static\n/robot_description)

  class GATE platform
  class VIZ platform
  class BRIDGE system
  class RANGER hardware
  class CMDVEL_OUT highlight
  class PLAN_IN,ENGAGE,DRIVE_EN,ESTOP,LOCPOSE,GNSSPOSE,MARKERS,BOUNDARY topic
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · 🛠️ Hardware · ==> critical · -.-> optional/fallback

| Sub-launch | File | Condition |
|---|---|---|
| `cmd_vel_gate.launch.py` | `launch/cmd_vel_gate.launch.py` | `cmd_vel_gate_enable:=true` (default) |
| `robot_visualization.launch.py` | `launch/robot_visualization.launch.py` | Always |
| `ranger.launch.py` | `launch/ranger.launch.py` | `ranger_driver_enable:=true` (default) |
| `sensor_kit_bridge.launch.py` | `launch/sensor_kit_bridge.launch.py` | Always |

*Figure 2 — Platform runtime architecture. Three logical subsystems run in parallel; `/platform/cmd_vel` is the only critical output path.*

---

## 5. 🔌 Interface Contract

### Inputs

| Topic | Type | Required | Producer | Rate | Meaning |
|---|---|---|---|---|---|
| `/planning/cmd_vel` | `geometry_msgs/Twist` | Yes | camrod_planning | ~10–20 Hz | Velocity command from Nav2 controller or planning gate |
| `/planning/engage` | `std_msgs/Bool` | Yes (if `engage_source_mode:=planning_engage`) | camrod_planning | On change | Operator or state-machine engage signal; `true` arms the gate |
| `/platform/drive_enable` | `std_msgs/Bool` | Yes | External (UI / operator) | On change | Hardware drive-enable signal; `true` arms the gate |
| `/platform/status/estop` | `std_msgs/Bool` | Yes (if `estop_source_mode:=platform_status`) | Ranger CAN driver (via bridge) | ~50 Hz | E-stop status from CAN bus; `true` blocks the gate |
| `/localization/pose` | `geometry_msgs/PoseStamped` | Yes | camrod_localization | ~20 Hz | Primary robot pose for visualization anchor |
| `/sensing/gnss/pose` | `geometry_msgs/PoseStamped` | No (fallback) | camrod_sensing | ~10 Hz | GNSS pose used when localization is stale |
| `/map/markers` | `visualization_msgs/MarkerArray` | No | camrod_map | On change | Lanelet markers; used to estimate ground Z for visualization |

### Outputs

| Topic | Type | Consumer | Rate | Meaning |
|---|---|---|---|---|
| `/platform/cmd_vel` | `geometry_msgs/Twist` | Ranger CAN driver | ~10–20 Hz | Gated velocity command; zero when gate blocked |
| `/platform/drive_enabled` | `std_msgs/Bool` | camrod_system (diagnostics) | On change | Current gate state (true = drive armed and not estopped) |
| `/platform/status/odometry` | `nav_msgs/Odometry` | camrod_localization (primary wheel) | 50 Hz | Normalized odometry from Ranger CAN bridge |
| `/platform/status/velocity` | `geometry_msgs/TwistStamped` | camrod_sensing (platform_velocity_converter) | 50 Hz | Velocity status from Ranger CAN bridge |
| `/platform/status/estop` | `std_msgs/Bool` | camrod_planning (cmd_vel gate), camrod_localization | 50 Hz | Derived e-stop from CAN bus |
| `/platform/status` | `avg_msgs/AvgPlatformStatus` | camrod_system | 50 Hz | Aggregated CAN status (vehicle_state, control_mode, battery, motor RPM) |
| `/rmp401/odom` | `nav_msgs/Odometry` | camrod_localization (wheel fallback) | 50 Hz | Fallback odometry (activated when `/odom` primary is silent) |
| `/platform/robot/markers` | `visualization_msgs/MarkerArray` | RViz | 20 Hz | Robot body, sensor axes, range rings, world origin |
| `/platform/robot/planning_boundary` | `geometry_msgs/PolygonStamped` | RViz, camrod_planning (collision reference) | 20 Hz | Body footprint + `planning_boundary_margin` polygon |
| TF static (all sensor frames) | `tf2_msgs/TFMessage` | All packages | Static | Sensor-to-base-link transforms from `robot_params.yaml` |
| `/robot_description` | `std_msgs/String` | All packages (TF, RViz) | Latched | URDF description from `camrod_sensor_kit` |

---

## 6. 🔬 Sub-System Details

### 6.1 🚦 cmd_vel_gate_node

The gate node (`camrod_platform/cmd_vel_gate_node`) subscribes to four signals and decides whether to forward or zero-out the incoming velocity command.

#### Gate Logic

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
flowchart TD
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;

  IN((/planning/cmd_vel)) ==> A
  A(Receive /planning/cmd_vel) --> B{❓ gate enabled?\ncmd_vel_gate_enable}
  B -- No --> Z(Pass through unchanged)
  B -- Yes --> C{❓ drive_enabled?}
  C -- No --> ZERO(Publish zero Twist)
  C -- Yes --> D{❓ estop active?\n/platform/status/estop}
  D -- Yes --> ZERO
  D -- No --> E{❓ engage active?\n/planning/engage}
  E -- No --> ZERO
  E -- Yes --> Z

  Z ==> OUT((/platform/cmd_vel → Ranger CAN))
  ZERO -.->|publish_zero_when_blocked| OUT

  subgraph ARMING [drive_enabled becomes True when ALL:]
    E1((/platform/drive_enable == true))
    E2((/planning/engage == true\nengage_source_mode=planning_engage))
  end

  subgraph ESTOP_SG [estop becomes True when:]
    F1((/platform/status/estop == true\nestop_source_mode=platform_status))
  end

  class A,B,C,D,E system
  class Z planning
  class ZERO platform
  class OUT highlight
  class IN,E1,E2,F1 topic
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · {❓} Decision · ==> critical path · -.-> zero/blocked path

> ⚠️ **The zero path (`ZERO`) is actively published**, not just silence. Any consumer expecting the gate output to stop on disable will still see a `Twist` message — it will contain all zeros.

*Figure 3 — cmd_vel gate logic. The gate has four independent trip conditions; any single failure zeros the output.*

**Key node parameters** (set via `cmd_vel_gate.launch.py`):

| Parameter | Default | Description |
|---|---|---|
| `input_cmd_vel_topic` | `/planning/cmd_vel` | Gate input topic |
| `output_cmd_vel_topic` | `/platform/cmd_vel` | Gate output topic |
| `enable_topic` | `/platform/drive_enable` | Drive-enable signal topic |
| `engage_topic` | `/planning/engage` | Planning engage signal topic |
| `engage_source_mode` | `planning_engage` | Engage source selector (`planning_engage` or `disabled`) |
| `state_topic` | `/platform/drive_enabled` | Published gate state |
| `estop_source_mode` | `platform_status` | E-stop source selector (`platform_status` or `disabled`) |
| `estop_topic` | `/platform/status/estop` | E-stop source topic |
| `allow_on_start` | `false` | Arm gate at startup without explicit enable signal |
| `publish_zero_when_blocked` | `true` | Actively publish zero Twist when gate is closed |

---

### 6.2 🎨 robot_visualization_node

Publishes a single `MarkerArray` on `/platform/robot/markers` at 20 Hz, anchored to the latest `/localization/pose`. Falls back to `/sensing/gnss/pose` if localization has been silent for longer than `localization_pose_timeout_s` (3.0 s default).

**Marker contents:**

- Robot footprint outline at ground plane (sized from `robot_params.yaml`)
- Planning boundary polygon (footprint + `planning_boundary_margin: 0.3 m`)
- Sensor frame axes labels (IMU, GNSS, LiDAR, camera)
- Debug range rings at radii `[2.0, 4.0, 6.0, 8.0]` m (configurable)
- World origin and map origin axes

**Key parameters** (from `config/robot_visualization.yaml`):

| Parameter | Default | Description |
|---|---|---|
| `publish_rate_hz` | `20.0` | Marker and boundary publish rate [Hz] |
| `localization_pose_timeout_s` | `3.0` | Max localization age before GNSS fallback [s] |
| `planning_boundary_margin` | `0.3` | Extra margin around footprint for boundary polygon [m] |
| `heading_yaw_offset_deg` | `0.0` | Heading correction [deg] applied to all markers |
| `range_ring_radii` | `[2.0, 4.0, 6.0, 8.0]` | Debug ring radii [m] |
| `pose_source_mode` | `localization_with_gnss_fallback` | `localization_only` or GNSS fallback mode when localization is stale |
| `ground_z_source` | `lanelet_map` | `fixed_offset` or lanelet-map sampled ground Z |

---

### 6.3 🔧 sensor_kit_bridge

Includes `camrod_sensor_kit/launch/sensor_kit.launch.py` with `enable_status:=false`. This publishes:

- All static TF transforms (sensor frame → `robot_base_link`) read from `robot_params.yaml`.
- `/robot_description` (URDF) for RViz and TF consumers.

The geometry source file is controlled by the `params_file` argument, which defaults to `camrod_sensor_kit/config/robot_params.yaml` and is overridden by `camrod_bringup` via `config/sensor_kit/robot_params.yaml` (mount-offset edits apply without rebuilding either package).

---

## 7. 🖥️ Hardware Dependency Matrix

| Component | Real Robot | Simulation (`sim:=true`) | Bench (no CAN) |
|---|:---:|:---:|:---:|
| Ranger CAN driver (`ranger.launch.py`) | ✅ Required | ❌ Not launched | ⚠️ Optional (`ranger_driver_enable:=false`) |
| cmd_vel_gate_node | ✅ Required | ✅ Active | ⚠️ Can disable (`cmd_vel_gate_enable:=false`) |
| robot_visualization_node | ✅ Active | ✅ Active | ✅ Active |
| sensor_kit_bridge (TF static) | ✅ Required | ✅ Required | ✅ Required |
| `/platform/status/estop` (from CAN) | ✅ Live signal | ❌ Not present | ⚠️ Simulated or `estop_source_mode:=disabled` |
| `/platform/status/odometry` (50 Hz) | ✅ Live signal | ✅ Fake sensor | ❌ Not available |
| `/localization/pose` anchor for VIZ | ✅ Live | ✅ Fake | ⚠️ Falls back to GNSS after 3 s |

---

## 8. 🔑 Key Behaviors

### Robot Does Not Move at Startup (by Design)

**Trigger:** Startup condition.

**Internal logic:** `allow_on_start: false` means the gate initializes in the closed (zeroed) state. Neither `/planning/engage` nor `/platform/drive_enable` is latched from a previous run.

**Output effect:** `/platform/cmd_vel` publishes zero Twist from startup until an explicit enable is received.

**Operator-visible symptom:** Robot is stationary even when Nav2 publishes a non-zero `/planning/cmd_vel`. This is the correct behavior.

**Related params:** `drive_allow_on_start`

**Related topics:** `/platform/drive_enable`, `/planning/engage`

---

### E-stop Overrides All Enables

**Trigger:** `/platform/status/estop` transitions to `true`.

**Internal logic:** Gate immediately publishes zero Twist regardless of the drive-enable or engage state. The estop signal comes from the Ranger CAN bridge (`ranger_params.yaml: estop_on_exception_state: true`).

**Output effect:** `/platform/cmd_vel` becomes zero within one gate processing cycle.

**Operator-visible symptom:** Robot decelerates and stops. `/platform/drive_enabled` transitions to `false`.

**Related params:** `estop_source_mode`, `estop_topic`, `estop_on_exception_state`

**Related topics:** `/platform/status/estop`, `/platform/drive_enabled`, `/platform/cmd_vel`

---

### Localization Fallback in Visualization

**Trigger:** `/localization/pose` has not been received for longer than `localization_pose_timeout_s` (3.0 s).

**Internal logic:** `robot_visualization_node` switches its pose anchor from `/localization/pose` to `/sensing/gnss/pose`.

**Output effect:** Robot marker in RViz continues to update using GNSS position, but marker color or a log warning may indicate the fallback state.

**Operator-visible symptom:** Robot marker does not freeze even if localization is restarting.

**Related params:** `localization_pose_timeout_s`, `pose_source_mode`

**Related topics:** `/localization/pose`, `/sensing/gnss/pose`, `/platform/robot/markers`

---

## 9. 🚀 Launch Arguments

### platform.launch.py arguments

| Argument | Default | Description |
|---|---|---|
| `cmd_vel_gate_enable` | `true` | Enable velocity gate node |
| `cmd_vel_in_topic` | `/planning/cmd_vel` | Gate input topic |
| `cmd_vel_out_topic` | `/platform/cmd_vel` | Gate output topic |
| `drive_enable_topic` | `/platform/drive_enable` | Drive-enable signal topic |
| `planning_engage_topic` | `/planning/engage` | Planning engage signal topic |
| `engage_source_mode` | `planning_engage` | Engage source selector (`planning_engage` or `disabled`) |
| `drive_state_topic` | `/platform/drive_enabled` | Gate state output topic |
| `estop_source_mode` | `platform_status` | E-stop source selector (`platform_status` or `disabled`) |
| `estop_topic` | `/platform/status/estop` | E-stop source topic |
| `drive_allow_on_start` | `false` | Arm gate at startup without explicit enable |
| `platform_type` | `ranger` | Platform type profile: `ranger` (CAN path) or `rmp401` (skip Ranger CAN, use external `/rmp401` topics) |
| `ranger_driver_enable` | `true` | Launch Ranger CAN base node |
| `ranger_bridge_enable` | `true` | Launch Ranger status bridge node (independent from CAN driver) |
| `sensor_kit_bridge_enable` | `true` | Include `sensor_kit_bridge.launch.py` (disable for debug without TF) |
| `ranger_params_file` | `config/ranger_driver.yaml` | Ranger driver parameter file |
| `params_file` | `camrod_sensor_kit/config/robot_params.yaml` | Robot geometry for TF and visualization |
| `robot_visualization_param_file` | `config/robot_visualization.yaml` | Robot visualization node parameters |
| `map_frame_id` | `map` | Global frame ID |
| `base_frame_id` | `robot_base_link` | Robot base frame ID |
| `sensor_kit_base_frame_id` | `sensor_kit_base_link` | Sensor kit base frame ID |

---

## 10. ⚙️ Config Files

| File | Purpose |
|---|---|
| `config/robot_visualization.yaml` | `robot_visualization_node` parameters: pose topics, timeout, heading offset, boundary margin, range rings, publish rate |
| `config/ranger_params.yaml` | Ranger CAN driver: `port_name: can0`, `update_rate: 50`, odom topics, e-stop behavior, platform status bridge topics |
| `config/vehicle_params.yaml` | Vehicle kinematics reference: wheelbase, track width, tire radius, max velocity/acceleration (read-only reference; not passed as a launch override automatically) |

---

## 11. ✅ Validation

After `ros2 launch camrod_platform platform.launch.py`:

```bash
# Confirm gate node is running
ros2 node list | grep cmd_vel_gate

# Check gate is publishing zeros (before engage)
ros2 topic echo /platform/cmd_vel --once

# Check drive state
ros2 topic echo /platform/drive_enabled --once

# Confirm sensor TF is up
ros2 run tf2_tools view_frames
# Expect: robot_base_link → sensor_kit_base_link → imu_link / gnss_link / lidar_link

# Check visualization markers are publishing
ros2 topic hz /platform/robot/markers   # expect ~20 Hz

# Check Ranger odometry (real hardware only)
ros2 topic hz /platform/status/odometry  # expect ~50 Hz
```

---

## 12. 🔧 Troubleshooting

### Robot Does Not Move

> ⚠️ Work through this checklist **in order** — each step rules out one gate trip condition.

**1. Is the gate armed?**
```bash
ros2 topic echo /platform/drive_enabled --once
```
Expected: `data: true`. If `false`, continue.

**2. Has `/planning/engage` been published?**
```bash
ros2 topic echo /planning/engage --once
```
If no message or `data: false`, the planning engage signal has not been sent. Use the UI or publish manually:
```bash
ros2 topic pub /planning/engage std_msgs/Bool "data: true" --once
```

**3. Has `/platform/drive_enable` been published?**
```bash
ros2 topic echo /platform/drive_enable --once
```
Same action as above if missing.

**4. Is e-stop active?**
```bash
ros2 topic echo /platform/status/estop --once
```
If `data: true`, the Ranger CAN driver is reporting a hardware fault. Check CAN bus (`ip link show can0`) and `ranger_params.yaml` (`estop_on_exception_state`, `estop_on_error_code`).

**5. Is `/planning/cmd_vel` non-zero?**
```bash
ros2 topic echo /planning/cmd_vel --once
```
If the planning gate itself is outputting zeros, the problem is upstream in `camrod_planning`.

---

### Robot Moves at Startup Unexpectedly

**Cause:** `drive_allow_on_start` was set to `true` in a previous launch or override.

**Fix:**
```bash
ros2 launch camrod_platform platform.launch.py drive_allow_on_start:=false
```
Or ensure `bringup/launch_defaults.yaml` has `platform/cmd_vel_gate_enable: false` only for the intended scenario.

---

### `/platform/cmd_vel` Publishing Zeros Forever (Gate Armed, No E-stop)

1. Verify `cmd_vel_gate_node` is running: `ros2 node list | grep cmd_vel_gate`.
2. Verify the input topic is correct: `ros2 param get /platform/cmd_vel_gate input_cmd_vel_topic`. It should be `/planning/cmd_vel`.
3. Check if the planning cmd_vel gate (in `camrod_planning`) is also blocked — it would appear as zeros on `/planning/cmd_vel`.
4. If `cmd_vel_gate_enable:=false` was set, `/platform/cmd_vel` is published directly from the platform sub-launch without gating. In this case there is no gate node to query.

---

### Robot Markers Missing in RViz

1. Confirm `robot_visualization_node` is running: `ros2 node list | grep robot_visualization`.
2. Check pose source: `ros2 topic hz /localization/pose` and `ros2 topic hz /sensing/gnss/pose`.
3. If both are silent, the visualization node has no pose anchor and will not publish.
4. Check `/platform/robot/markers` is being published: `ros2 topic hz /platform/robot/markers`.
5. In RViz, confirm the MarkerArray display is subscribed to `/platform/robot/markers` and the fixed frame is `map`.

---

### TF Tree Broken (sensor frames missing)

1. Check `sensor_kit_bridge` included `camrod_sensor_kit`: `ros2 node list | grep sensor_kit`.
2. Verify `/robot_description` is published: `ros2 topic echo /robot_description --once | head -5`.
3. Run `ros2 run tf2_tools view_frames` and look for disconnected subtrees.
4. Confirm `params_file` points to the correct `robot_params.yaml` with non-zero sensor offsets.
5. If `params_file` was overridden via `camrod_bringup/config/sensor_kit/robot_params.yaml`, verify that file has the correct mount offsets.

---

## 13. 📚 Related Docs

| Document | Location |
|---|---|
| Monorepo overview | `../README.md` |
| Parameter naming standard | `../PARAMETER_NAMING_STANDARD.md` |
| camrod_planning | `../camrod_planning/README.md` |
| camrod_localization | `../camrod_localization/README.md` |
| camrod_sensor_kit | `../camrod_sensor_kit/README.md` |
| camrod_system | `../camrod_system/README.md` |
| camrod_docking | `../camrod_docking/README.md` |
