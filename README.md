# CAMROD

<!-- HH_260804 - Pair the concise source guide with real sim runtime captures
and keep the route-retry containment separate from physical acceptance. -->

ROS 2 Humble autonomous delivery robot stack for a Dual-Ackermann, crab, and
zero-turn Ranger platform. Current runtime baseline: **`v2.1.4`**.

![Full-stack mission contract](docs/assets/module-guides/bringup/full-stack-mission-contract.png)

## Actual Simulation Runtime

![Live full-stack B6 runtime](docs/assets/module-guides/bringup/runtime-full-stack-b6-20260804.png)

`SIM RUNTIME CAPTURE`: live `sim:=true rviz:=true` B6 graph showing the
lanelet map, robot, sensing layers, localization, and global/local planning
paths. This is not a generated diagram or a real-robot field claim.

## At A Glance

| Uses | Core function | Main outputs |
|---|---|---|
| ROS 2 Humble, Nav2, Lanelet2, robot_localization | Camping-site dispatch, navigation, local maneuver, parking, and charging lifecycle | `/service/state`, `/planning/state`, `/control/cmd_vel_ros`, `/system/status` |
| Vanjee LiDAR, 7 radars, dual GNSS, IMU, front/rear cameras | Sensor normalization, obstacle costs, localization, and perception | `/planning/cost_grid/inflation`, `/localization/pose`, `/perception/obstacles` |
| Robot UI and Guest UI | Mission commands, lifecycle display, safety hold, and diagnostics | `/ui/selected_destination`, `/planning/mission_key`, operation requests |

## Current Values

| Item | Active value | Meaning |
|---|---:|---|
| Navigation frame | `robot_center_link` | Axle midpoint used by localization, planning, control, and platform |
| Measured body | `1.49160 x 1.07000 m` | Physical length x width |
| Planning boundary | `1.59160 x 1.17000 m` | Body plus `0.05 m` on every side |
| New mission SOC | `>= 35%` | New campsite departure is admitted |
| Hard safety stop | `<= 20%` | Final command output is stopped |
| Planner/controller | `LaneletRoute + RPP` | Full-bringup default |
| Planner load set | `LaneletRoute + SmacLattice` | Fallback constructs only for a width-gated obstacle replan |
| Controller load set | `RPP + RotationShim` | Mission tracking plus manual clicked-yaw handling |
| Obstacle fallback hold | `20.0 s` | Safety stop is immediate; only planner preemption waits |
| Active Lanelet map | `map_version=15` | User-authored synchronized OSM pair |
| RPP preview | `1.1 m` | Current stable minimum lookahead |
| Recovery limit | `0.10 m/s`, `0.40 m`, `10 s` | Raw speed, travel, and duration; contact yaw command is zero |
| Rapid retry guard | `1 release`, `5 s` | A same-route recontact latches zero output until stop/replan |
| Parking methods | `reverse`, `apriltag` | Exactly one final parking controller is selected |

Configured values are not performance measurements. Package tables below mark
runtime evidence separately.

## Latest Evidence

| Check | Result | Scope |
|---|---|---|
| Release build/tests | **PASS** | 9 selected ROS packages: 380 xUnit cases, 0 errors/failures; 27 direct UI pytest cases and the React production build passed |
| Full stack startup/shutdown | **PASS** | 87 ROS nodes reached `[SYSTEM] OK`; controller, UIs, and all checker containers exited cleanly |
| Localization pose chain | **PASS** | 10 Hz inputs produced 20 Hz selected pose; header-age p95 `1.83 ms` |
| Reference-frame A/B | **PASS for compared segment** | Cross-track RMS `0.0588 -> 0.0549 m`; yaw RMS `2.901 -> 2.713 deg` |
| Automatic boundary recovery | **SIM-PASS containment, historical** | One release; map v14 recontacted in 0.276 s (v13: 0.267/0.372 s), then latched with final command zero |
| Guest/Robot UI contract | **PASS** | Dispatch, lifecycle, return, safety overlay, and operator stop observed |
| Physical radar disabled | **FIELD-PASS** | `600.063 s`; 5,976 clear grids; zero active/high-cost/stop evidence |
| Physical front camera/YOLO lifetime | **FIELD-PASS** | `300 s`; 2,750/2,750 JPEG decode; `9.167 Hz`; zero crash/restart |
| Physical rear camera rate | **FIELD-FAIL** | Raw `3.633 Hz` versus `10 Hz` target under field load |
| B6/B12 round trip | **NOT DEMONSTRATED** | Full footprint stops at missing surveyed service-access geometry |
| Physical Ranger/sensors/audio | **FIELD PENDING** | Jetson and real-robot measurements are still required |

![Measured full-bringup result](docs/assets/module-guides/bringup/simulation-evidence-20260804.png)

![Physical stationary field report](docs/assets/module-guides/bringup/field-stationary-report-20260731.png)

## Packages

| Package | Responsibility | Visual guide |
|---|---|---|
| [`camrod_bringup`](camrod_bringup/README.md) | Full-stack launch, defaults, simulation, and validation tools | Mission contract and measured smoke result |
| [`camrod_common/avg_msgs`](camrod_common/avg_msgs/README.md) | Generated shared messages and services | Interface inventory and dependency boundary |
| [`camrod_sensor_kit`](camrod_sensor_kit/README.md) | URDF, static TF, body geometry, and sensor mounts | Rear-axle/center comparison and side view |
| [`camrod_map`](camrod_map/README.md) | Lanelet2 map, semantic areas, and cost grids | Route mask and planning-grid values |
| [`camrod_sensing`](camrod_sensing/README.md) | Hardware acquisition, preprocessing, and cost fusion | Sensor matrix and ground-filter schematic |
| [`camrod_perception`](camrod_perception/README.md) | LiDAR obstacles, YOLO fusion, occupancy, and AprilTag | Field/simulation pipeline boundary |
| [`camrod_localization`](camrod_localization/README.md) | GNSS/IMU/wheel EKF and localization state | Pose generation and measured timing |
| [`camrod_planning`](camrod_planning/README.md) | Nav2, Lanelet routing, goal snapping, and mission state | Servers, selectors, and ownership handoffs |
| [`camrod_control`](camrod_control/README.md) | Final command gate, local maneuvers, parking, and recovery | Footprint safety and measured recovery |
| [`camrod_platform`](camrod_platform/README.md) | Ranger command/feedback and normalized platform status | Steering transition and CAN/BMS bridge |
| [`camrod_system`](camrod_system/README.md) | Diagnostics, graph readiness, and health summary | Severity and operator-state separation |
| [`camrod_ui`](camrod_ui/README.md) | Robot/Guest HTTP backends and browser interfaces | Shared mission/state contract |
| [`camrod_voice`](camrod_voice/README.md) | Event-to-audio policy and priority playback | Event and priority flow |

## Command Path

```text
Nav2 /control/nav2_cmd_vel_ros -----+
local maneuvers /control/cmd_vel_raw +--> cmd_vel_safety_gate
parking/recovery -------------------+        |
                                              +--> /control/cmd_vel
                                              +--> /control/cmd_vel_ros --> Ranger
```

The safety gate is the only final motion authority. Mission state
(`/service/state`), planner state (`/planning/state`), command authorization,
and health (`/system/status`) are separate contracts.

## Mission Lifecycle

```text
DROP_ZONE_WAIT -> DEPARTING_DROP_ZONE -> MOVING_TO_SITE -> SITE_ENTRY
-> UNLOAD_WAIT -> WAITING_FOR_RETURN_REQUEST -> RETURN_WITH_CARGO
-> RETURNING_TO_DROP_ZONE -> DROP_ZONE_PARKING
-> WAITING_FOR_CHARGING -> CHARGING
```

Parking-controller `PARKED` is an internal phase. Public state is `CHARGING`
when CAN charging feedback is true, `WAITING_FOR_CHARGING` while contact is
pending, and `DROP_ZONE_WAIT` when parked without charge feedback.

## Build And Run

```bash
cd ~/camrod_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Simulation with the operator RViz profile
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true

# Field profile on the configured Jetson/Ranger host
ros2 launch camrod_bringup bringup.launch.py
```

Do not tune physical CAN, camera, sensor-port, or Jetson performance parameters
from a workstation-only simulation result.

## Documentation

| Document | Purpose |
|---|---|
| [Module Visual Guide](docs/MODULE_VISUAL_GUIDE.md) | Evidence classes, asset sources, regeneration, and interpretation |
| [v2.1.4 release notes](docs/V2_1_4_RELEASE_NOTES.md) | Current runtime changes, exact values, validation, and field limits |
| [v2.1.3 release notes](docs/V2_1_3_RELEASE_NOTES.md) | Released runtime scope and verification |
| [Robot-center migration](docs/V2_1_3_ROBOT_CENTER_MIGRATION.md) | Exact before/after geometry and A/B results |
| [Boundary recovery validation](docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md) | Crab/reverse timelines, GIFs, and limitations |
| [Runtime capture evidence](docs/evidence/module-guides/bringup/runtime-visual-capture-20260804.json) | Live screens, retry timestamps, zero output, and operator-stop result |
| [Documentation changelog](docs/DOCS_CHANGELOG.md) | Post-release documentation-only updates |
| [`TODOLIST.txt`](TODOLIST.txt) | Remaining physical acceptance work |
