# camrod_docking

ROS 2 Humble autonomous docking package for CAMROD robot.
AprilTag detection → Pose estimation → EgoPolar controller → charging station docking.
Supports both forward and reverse docking. Manual mode available for placement-first docking without Nav2.

---

## Package Structure

```
camrod_docking/
├── config/
│   ├── apriltag.yaml           # AprilTag detector configuration
│   ├── docking_server.yaml     # opennav_docking server parameters
│   ├── docks.yaml              # Docking station location database
│   └── manual_dock_server.yaml # Manual docking server parameters
├── include/camrod_docking/
│   ├── camrod_docking_plugin.hpp
│   └── manual_dock_server_node.hpp
├── launch/
│   ├── docking.launch.py       # Main launch (AprilTag + DockingServer + ManualDockServer; camera TF is in camrod_sensor_kit)
└── src/
    ├── camrod_docking_plugin.cpp       # ChargingDock plugin
    ├── docking_apriltag_bridge.cpp     # AprilTag → PoseStamped conversion node
    ├── manual_dock_server_node.cpp     # Manual docking action server
    ├── odom_yaw_corrector.cpp          # RMP401 odometry sign correction
    └── tag_distance_node.cpp           # Tag distance publisher node
```

---

## Docking Pipeline

```
econ_camera (rear or front)
  └─ image_proc/rectify_node
      └─ apriltag_node  (publishes dock_tag TF)
          └─ docking_apriltag_bridge  →  /docking/detected_dock_pose  (odom frame)
              └─ CamrodDockingPlugin.getRefinedPose()
                  └─ opennav_docking controller (EgoPolar)
                      └─ /rmp401/cmd_vel
```

---

## Nodes

### `docking_apriltag_bridge` (docking_apriltag_bridge.cpp)

Subscribes to AprilTag detections, looks up the `dock_tag` frame from TF, and publishes it as a `PoseStamped` in the odom frame with EMA filtering.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_tag_id` | 0 | AprilTag ID to track |
| `tag_frame` | `dock_tag` | TF frame name published by apriltag_ros |
| `fixed_frame` | `odom` | Output PoseStamped reference frame |
| `ema_alpha` | 0.4 | EMA filter coefficient (1.0 = passthrough) |
| `detection_timeout` | 0.5s | Stop publishing if no detection for this duration |
| `publish_rate_hz` | 10.0 | Output publish rate |

| Subscribed Topic | Type |
|------------------|------|
| `input_detection_topic` | `apriltag_msgs/AprilTagDetectionArray` |

| Published Topic | Type |
|-----------------|------|
| `output_detected_dock_pose_topic` | `geometry_msgs/PoseStamped` |
| `output_avg_pose_topic` | `avg_msgs/AvgAprilTagPose` |
| `output_avg_detection_topic` | `avg_msgs/AvgAprilTagDetectionArray` |

---

### `CamrodDockingPlugin` (camrod_docking_plugin.cpp)

`opennav_docking_core::ChargingDock` plugin loaded by DockingServer.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dock_backwards` | false | Reverse docking (shared param with DockingServer) |
| `staging_x_offset` | -0.70m | Staging pose offset from dock origin |
| `docking_threshold` | 0.05m | Distance tolerance for `isDocked()` |
| `external_detection_timeout` | 1.0s | Timeout for dock pose reception |
| `external_detection_translation_x` | -0.20m | Tag → base_link target offset (see TODO) |
| `external_detection_translation_y` | 0.0m | Lateral offset |
| `filter_coef` | 0.1 | In-plugin EMA coefficient |
| `detected_dock_pose_topic` | `/docking/detected_dock_pose` | Input PoseStamped topic |
| `base_frame` | `base_link` | Robot base frame |

**Behavior:**
- `getStagingPose()`: Computes staging position from dock database. If `dock_backwards=true`, rotates by +π.
- `getRefinedPose()`: On first call, locks robot's odom heading as `approach_yaw` to suppress PnP camera yaw drift. Applies EMA filter, then offsets by `external_detection_translation_x` to compute target pose.
- `isDocked()`: Accepts if `|base_link ↔ tag distance − |ext_translation_x||` < `docking_threshold`.
- `isCharging()`: Hardware not yet connected — returns `isDocked()` result. Replace with `/battery_state` current threshold after charger integration.

---

### `manual_dock_server_node` (manual_dock_server_node.cpp)

Action server that accepts dock requests from UI or CLI and internally calls DockingServer's `DockRobot` action.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `navigate_to_staging_pose` | false | false = robot is already in front of dock, skip Nav2 |
| `action_server_name` | `/docking/manual_dock` | Action name exposed by this node |
| `dock_action_name` | `/docking/dock_robot` | DockingServer action name |

**Manual docking:**
```bash
ros2 action send_goal /docking/manual_dock \
  avg_msgs/action/ManualDock "{dock_id: 'home_dock'}"
```

---

### `odom_yaw_corrector` (odom_yaw_corrector.cpp)

Corrects the odometry sign bug in the RMP401 `segwayrmp` driver.
Subscribes `/rmp401/odom` → publishes `odom → base_link` TF with `qz` and `position.y` sign-flipped.

---

## Camera TF Structure

```
sensor_kit_base_link
 ├─ camera_front_link  [x=+0.40m, z=0.46m, yaw=0  — forward-facing]
 │   └─ camera_front   [RPY(-π/2, 0, -π/2) — REP-103 optical frame]
 └─ camera_rear_link   [x=+0.10m, z=0.46m, yaw=π  — backward-facing]
     └─ camera_rear    [RPY(-π/2, 0, -π/2) — REP-103 optical frame]
```

> **HH_260528:** Static TF is now published by `camrod_sensor_kit` (`robot_state_publisher` via `camrod_sensor_kit.xacro`). `docking.launch.py` no longer runs `static_transform_publisher` nodes for camera frames.

---

## Launch

```bash
# Full docking stack (camera TF + AprilTag + DockingServer + ManualDockServer)
ros2 launch camrod_docking docking.launch.py

# Enable auto docking (default: manual only)
ros2 launch camrod_docking docking.launch.py enable_auto_docking:=true

# Disable manual docking (auto only)
ros2 launch camrod_docking docking.launch.py enable_manual_docking:=false
```

---

## TODO

- [ ] **Improve `external_detection_translation_x` parameter readability**

  The raw value (e.g., `-0.565`) gives no hint of its physical meaning.
  Two approaches:

  **Option A — split into meaningful parameters:**
  ```yaml
  # docking_server.yaml or robot_geometry.yaml
  robot_geometry:
    front_camera_to_base_link_x: 0.40    # base_link → front camera (m)
    front_bumper_to_camera_x:    0.165   # front camera → front bumper (m)
    rear_camera_to_base_link_x:  0.10    # base_link → rear camera (m)
    rear_bumper_to_camera_x:     0.0     # rear camera → rear bumper (m) — measure needed
  # external_detection_translation_x = -(camera_to_base_link + bumper_to_camera)
  ```
  Compute in plugin `configure()`:
  ```cpp
  ext_translation_x_ = -(camera_to_base_x + bumper_to_camera_x);
  ```

  **Option B — load from URDF:**
  Define full vehicle geometry in `rmp401_description` URDF and have the plugin read `base_link → bumper_link` from the TF tree.

- [ ] **Measure rear camera → rear bumper distance**
  Required for reverse docking re-enablement. Measure and set `rear_bumper_to_camera_x`.

- [ ] **Set actual station coordinates in `docks.yaml`**
  Current `pose: [1.0, 0.0, 0.0]` is a placeholder.
  Only affects retry behavior in manual mode (`navigate_to_staging_pose=false`), but must be replaced with measured values before enabling autonomous docking (`navigate_to_staging_pose=true`).

- [ ] **Wire real charging hardware to `isCharging()`**
  Currently returns `isDocked()`. After charger integration, replace with `/battery_state` current > `charging_current_threshold`.
