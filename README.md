# CAMROD

<!-- HH_260806 - Use the fabrication-inclusive measured body as the active
hard-stop envelope and retain earlier reduced-boundary runs as historical evidence. -->
<!-- HH_260806 - Replace the GNSS center placeholder with the measured
left-antenna lever arm and heading-aware center correction. -->
<!-- HH_260807 - Add map-v17 repeated service, B2 recovery, and persistent-obstacle evidence. -->

ROS 2 Humble autonomous delivery robot stack for a Dual-Ackermann, crab, and
zero-turn Ranger platform. Current runtime baseline: **`v2.1.5`**.

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
| GNSS position reference | left antenna `(0,+0.45,0) m` | Fresh dual-GNSS yaw rotates the lever arm; localization subtracts it to publish robot center |
| Physical body boundary | `1.39160 x 1.07000 m` | Fabrication-inclusive measured envelope; cost-100 contact is a hard stop with no automatic recovery |
| Planning boundary | `1.49160 x 1.17000 m` | Physical body plus `0.05 m` on all four sides; margin-only contact may use projected bounded recovery |
| Base platform dimensions | `1.19160 x 0.87000 m` | Chassis-only value retained separately from the fabrication-inclusive collision envelope |
| New mission SOC | `>= 35%` | New campsite departure is admitted |
| Hard safety stop | `<= 20%` | Final command output is stopped |
| Planner/controller | `LaneletRoute + RPP` | Full-bringup default |
| Planner load set | `LaneletRoute + SmacLattice` | Fallback constructs only for a width-gated obstacle replan |
| Controller load set | `RPP + RotationShim` | Mission tracking plus manual clicked-yaw handling |
| Obstacle fallback hold | `20.0 s` | Safety stop is immediate; only planner preemption waits |
| Active Lanelet map | `map_version=17`, SHA `8cd05c...5e021` | User-edited active OSM and named copy are byte-identical; SHA is a synchronization fingerprint, not a geometry lock |
| RPP preview | `1.1 m` | Current stable minimum lookahead |
| Recovery limit | `0.10 m/s`, `0.10 rad/s`, `12 deg`, `0.40 m`, `10 s` | Projected staged crab/reverse/reverse-yaw envelope |
| Rapid retry guard | `1 release`, `5 s` | A same-route recontact latches zero output until stop/replan |
| Parking methods | `reverse`, `apriltag` | Exactly one final parking controller is selected |
| LiDAR cost grid | default `OFF` | Optional component and its diagnostics activate together only when requested |
| Operator renderer | Chromium | GPU-enabled kiosk default; WebKit remains an explicit fallback |
| ROS transport | component intra-process; physical-LiDAR SHM opt-in | DDS-SHM is scoped to the LiDAR driver group and never exported to the full graph |
| System health tools | `4` nodes in `system_core_container` | Stable aggregate/status chain; standalone fallback remains available |
| System checker topology | `24` checkers in `4` serialized containers | Fault domains remain separate; standalone fallback remains available |
| Nav2 process topology | planner/controller scoped container | Vendor smoother/behavior/BT/lifecycle executables stay standalone |

Configured values are not performance measurements. Package tables below mark
runtime evidence separately.

## Latest Evidence

| Check | Result | Scope |
|---|---|---|
| Repeated campsite service | **AMD64 SIM PASS 3/3** | B1 -> B2 -> B3 completed route, site turnaround, explicit RETURN, drop alignment, reverse parking, charging, and next departure in 677.237 s with zero bringup restart |
| Current B2 boundary recovery | **AMD64 SIM PASS 3/3** | `REVERSE_YAW_RIGHT`, real recovery motion before 1.5 s release, mission complete, no second hold/retry latch |
| Persistent obstacle, map-v17 3.0 m lane | **SAFE-HOLD PASS** | Immediate stop; one Smac path preflight found no safe path; no selector/ABORT loop; obstacle clear resumed the original mission |
| v2.1.5 build/tests | **PASS** | 11 packages built; fresh CAMROD run: 58 CTest targets / 523 xUnit cases, 0 errors/failures, 13 existing static-analysis skips; UI 30/30; Ranger policy GTest 1/1 |
| Current map-v17 contract | **PASS** | Active/copy OSM SHA synchronized; 55 lanelets, 14 areas, 1,652 nodes |
| Boundary guard regression | **PASS** | Repository `colcon_build.sh` rebuilt 5 affected packages; 32/32 CTest targets passed; 334 aggregate xUnit records, 0 errors/failures, 8 cppcheck skips |
| Full stack startup/shutdown | **PASS** | Final topology reached Nav2 active and `[SYSTEM] OK`; all 6 component containers exited cleanly in 3/3 runs plus one default-argument run |
| amd64 runtime topology A/B | **SYSTEM CORE PASS; LIDAR TRADEOFF** | Core: 3 fewer processes, CPU -2.5 points, PSS -19.7 MiB, 3/3 controlled stop; LiDAR: CPU -17.5%, PSS +44.0 MiB, 10 Hz preserved |
| amd64 operator renderer A/B | **WEBKIT LIGHTER; PAGE-LOAD SPEED UNRESOLVED** | Chromium vs WebKit: PSS +327.3 MiB, CPU +1.44 points; system-OK mean -1.33 s is not browser first paint; both controlled-stop 3/3 |
| Localization pose chain | **PASS** | 10 Hz inputs produced 20 Hz selected pose; header-age p95 `1.83 ms` |
| Reference-frame A/B | **PASS for compared segment** | Cross-track RMS `0.0588 -> 0.0549 m`; yaw RMS `2.901 -> 2.713 deg` |
| Automatic boundary recovery | **v2.1.4 RELEASE-MAP SIM PASS, FAIL CLOSED** | Release map-v15 selected `REVERSE_YAW_RIGHT` at `0.05 rad/s` and `CRAB_LEFT`; rapid route recontact latched with final zero output |
| Earlier reduced-boundary policy | **AMD64 SIM PASS; HISTORICAL** | The `1.29160 x 0.87000 m` candidate produced normal-route, margin recovery, and physical-stop evidence retained for comparison; it is no longer the active body |
| Historical campsite sequencing | **PASS; B11-B13 RETURN FIELD-PENDING** | Map-v16 B1-B10 all completed crab, zero-turn, wait, explicit return, and crab-out; B11-B13 reached roadside `WAIT_RETURN` without any on-site turn |
| Guest/Robot UI contract | **PASS** | Dispatch, lifecycle, return, safety overlay, and operator stop observed |
| Physical radar disabled | **FIELD-PASS** | `600.063 s`; 5,976 clear grids; zero active/high-cost/stop evidence |
| Physical front camera/YOLO lifetime | **FIELD-PASS** | `300 s`; 2,750/2,750 JPEG decode; `9.167 Hz`; zero crash/restart |
| Physical rear camera rate | **FIELD-FAIL** | Raw `3.633 Hz` versus `10 Hz` target under field load |
| B1-B10 turnaround | **SIM PASS (10/10)** | All ten map-derived lateral entries (`1.79-5.31 m`) completed full round trips with maneuver-exclusive command ownership |
| B11-B13 roadside | **ARRIVAL SIM PASS; RETURN UNRESOLVED** | Each site stopped at `WAIT_RETURN` after a capped `0.60 m` crab; no RETURN was issued |
| Physical Ranger/sensors/audio | **FIELD PENDING** | Jetson and real-robot measurements are still required |

![Measured full-bringup result](docs/assets/module-guides/bringup/simulation-evidence-20260804.png)

![Physical stationary field report](docs/assets/module-guides/bringup/field-stationary-report-20260731.png)

![Measured amd64 runtime topology A/B](docs/assets/module-guides/system/runtime-topology-amd64-ab-20260805.png)

![v2.1.4 release-map staged boundary recovery](docs/assets/module-guides/control/map-v15-boundary-recovery-contact-sheet.png)

![Historical reduced-body and planning-margin validation](docs/assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)

![Current campsite maneuver validation](docs/assets/test_result/camping-site-sequencing-20260806/campsite-policy-validation.png)

![Current map-v17 repeated service](docs/assets/test_result/v2-1-5-service-validation-20260807/repeated-service-summary.png)

![Current persistent-obstacle safe hold](docs/assets/test_result/v2-1-5-service-validation-20260807/obstacle-safe-hold.png)

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
| [v2.1.5 release notes](docs/V2_1_5_RELEASE_NOTES.md) | Current runtime, GNSS center contract, exact validation, and field limits |
| [v2.1.5 field handoff](camrod_bringup/docs/v2_1_5_field_handoff_20260808.md) | Other-PC start, TODO crosswalk, and physical test order |
| [v2.1.5 service evidence](docs/assets/test_result/v2-1-5-service-validation-20260807/README.md) | Repeated service, obstacle, boundary JSON/log/PNG/GIF and limits |
| [v2.1.4 release notes](docs/V2_1_4_RELEASE_NOTES.md) | Previous map, boundary, UI, and transport baseline |
| [v2.1.3 release notes](docs/V2_1_3_RELEASE_NOTES.md) | Released runtime scope and verification |
| [Robot-center migration](docs/V2_1_3_ROBOT_CENTER_MIGRATION.md) | Exact before/after geometry and A/B results |
| [Boundary recovery validation](docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md) | Crab/reverse timelines, GIFs, and limitations |
| [Runtime capture evidence](docs/evidence/module-guides/bringup/runtime-visual-capture-20260804.json) | Live screens, retry timestamps, zero output, and operator-stop result |
| [amd64 runtime A/B evidence](docs/evidence/v2.1.5/runtime-topology/amd64-container-ab-20260805.json) | Per-run process/CPU/RSS/PSS/rate and controlled-stop samples |
| [Documentation changelog](docs/DOCS_CHANGELOG.md) | Post-release documentation-only updates |
| [`TODOLIST.txt`](TODOLIST.txt) | Remaining physical acceptance work |
