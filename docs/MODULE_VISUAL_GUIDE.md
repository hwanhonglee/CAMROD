# CAMROD Module Visual Guide

<!-- HH_260804 - Index generated diagrams and actual runtime screens separately,
including the rapid-recontact latch evidence and its field-claim boundary. -->

This guide explains what each README image proves. Runtime decisions still
come from ROS topics, controller state, diagnostics, and safety gates.

## Evidence Labels

| Label | Source | Valid claim |
|---|---|---|
| `SOURCE-DERIVED` | Checked-in YAML, messages, launch defaults, and code constants | Topology and displayed values match the repository |
| `SOURCE INVENTORY` | Files and package manifests | Interface/file/dependency counts match source |
| `MEASURED SIM` | Committed JSON or raw ROS logs from a running simulation | Listed rates, events, displacement, and state changes were observed in that run |
| `SIM RUNTIME CAPTURE` | RViz, ROS CLI, or browser connected to a live `sim:=true` graph | The displayed topics/layers/states existed in that run; no physical performance claim |
| `FIELD REPORT / RAW LOG EXTERNAL` | Committed normalized summary backed by a report that references Jetson-only raw paths | Listed stationary values were reported; the repository cannot independently replay the raw capture |
| `ALGORITHM SCHEMATIC` | Seeded synthetic data + active thresholds | Processing concept is reproducible; the points are not sensor evidence |
| `FIELD PENDING` | No matching physical capture | No physical accuracy, throughput, latency, or safety PASS is claimed |

Configured grid size, target rate, threshold, or timeout is **not** measured
performance. Every package README keeps those columns separate.

## Asset Index

| Package | Generated visual | Evidence/value source | Current verdict |
|---|---|---|---|
| `camrod_bringup` | `full-stack-mission-contract.png`, `mission-lifecycle-contract.gif` | Launch/state/battery/parking contracts | Expected sequence only |
| `camrod_bringup` | `simulation-evidence-20260804.png` | Full-bringup JSON + raw logs | 81-node startup and pose chain pass; B6/B12 round trip fails closed |
| `camrod_bringup` | `field-stationary-report-20260731.png` | Normalized JSON + physical test report | Radar-off/front-camera lifetime pass; rear rate/RTK/CPU limits visible; raw files external |
| `camrod_common/avg_msgs` | `interface-contract-and-dependencies.png` | Message/service files and manifests | 86 messages, 2 services, 12 direct package dependents |
| `camrod_control` | `command-safety-and-recovery.png`, boundary stop/recovery PNGs and GIFs | Gate/recovery YAML + pre-owner/current automatic-owner JSON | Bounded crab/reverse and RPP retry observed; mission incomplete |
| `camrod_localization` | `pose-generation-and-timing.png` | EKF YAML + 30-second probe | 10 Hz inputs -> 20 Hz selected pose; field accuracy pending |
| `camrod_map` | `lanelet-map-and-cost-grids.png` | Map/grid YAML | Route/planning grid values match source; service access missing |
| `camrod_perception` | `yolo-lidar-and-parking-pipelines.png` | Perception/AprilTag YAML | LiDAR sim path available; physical YOLO/fusion/tag pending |
| `camrod_planning` | `nav2-servers-and-mission-states.png`, `robot-center-narrow-route-risk-map.png` | Nav2 config, state contracts, and footprint sweep | Active topology documented; narrow corridor remains invalid |
| `camrod_platform` | `ranger-command-and-status.png` | Ranger/visualization YAML | Hardware boundary documented; CAN/actuator timing pending |
| `camrod_sensing` | `sensor-processing-and-cost-fusion.png` | Sensor/grid YAML | Source topology and target values only |
| `camrod_sensing` | `ground-segmentation-schematic.png` | Ground-filter YAML + seeded points | Algorithm schematic, not a point-cloud capture |
| `camrod_sensor_kit` | `reference-frame-before-after.png`, `rear-axle-vs-robot-center-drive.gif` | Geometry YAML + A/B JSON | Compared center-frame route metrics pass; narrow boundary remains |
| `camrod_sensor_kit` | `sensor-mount-side-view.png`, `sensor-x-before-after.png` | Geometry YAML | Physical side view and exact coordinate conversion |
| `camrod_system` | `diagnostic-severity-and-surfaces.png` | Manifests, aggregator, hardware thresholds | Health policy documented; utilization is not measured |
| `camrod_ui` | `robot-and-guest-mission-state.png`, Guest dispatch/hold screenshots | UI policy + browser/ROS JSON | Dispatch, return, hold, and operator stop observed |
| `camrod_voice` | `voice-events-and-priority.png` | Voice adapter YAML/policy | Queue/readiness policy documented; acoustic performance pending |

All files are under `docs/assets/module-guides/`. The main package renderer
creates **18 PNGs and one 10-frame GIF**. Release-evidence renderers add seven
PNGs and five GIFs. Fourteen manually captured live screens bring the checked-in
total to **39 PNGs and six GIFs**.

## Actual Runtime Screen Index

| Package | Live screen | Observed runtime content |
|---|---|---|
| root / `camrod_bringup` | `bringup/runtime-full-stack-b6-20260804.png` | B6 lanelet route, vehicle, sensing, localization, planning |
| `camrod_common`, `avg_msgs` | `common/runtime-interface-terminal-20260804.png` | Live topic type, generated `SystemStatus`, graph endpoints |
| `camrod_control` | `control/runtime-boundary-retry-latch-20260804.png`, `runtime-retry-latch-terminal-20260804.png` | Footprint contact, retry latch, all-zero final command |
| `camrod_localization` | `localization/runtime-pose-tf-20260804.png` | Selected poses, GNSS, center/base TF chain |
| `camrod_map` | `map/runtime-lanelet-map-20260804.png` | Published lanelet geometry and robot pose |
| `camrod_perception` | `perception/runtime-obstacle-bboxes-20260804.png` | Fused obstacle points and boxes |
| `camrod_planning` | `planning/runtime-b6-global-local-path-20260804.png` | B6 goal, global path, local path |
| `camrod_platform` | `platform/runtime-robot-geometry-20260804.png`, `runtime-status-terminal-20260804.png` | Robot boundary/frame and normalized simulated Ranger/BMS status |
| `camrod_sensing` | `sensing/runtime-lidar-radar-costs-20260804.png` | Filtered LiDAR, radar sectors, dynamic costs |
| `camrod_sensor_kit` | `sensor-kit/runtime-sensor-tf-20260804.png` | Loaded camera/GNSS/IMU/LiDAR/radar TF geometry |
| `camrod_system` | `system/runtime-health-terminal-20260804.png` | `SystemStatus`, UI state, live graph count |
| `camrod_ui` | `ui/guest-mission-dispatch-ready.png`, `guest-route-safety-hold.png` | Browser mission dispatch and safety overlay |
| `camrod_voice` | `voice/runtime-event-terminal-20260804.png` | Live `system.startup` request and adapter graph |

Capture metadata, exact launch command, event times, operator stop result, and
the concise raw excerpt are in
[`runtime-visual-capture-20260804.json`](evidence/module-guides/bringup/runtime-visual-capture-20260804.json).

## Key Measured Results

| Module | Measurement | Result | Limit |
|---|---|---|---|
| Bringup | Full graph | 81 nodes and `[SYSTEM] OK` | B6/B12 local entry still blocked |
| Localization | 30-second stationary probe | Selected pose `20.000 Hz`, age p95 `1.83 ms` | No field ground truth or motion disturbance |
| Sensor kit/planning | Common route A/B | Cross-track RMS `0.0588 -> 0.0549 m`; yaw RMS `2.901 -> 2.713 deg` | One simulated route segment |
| Control | One-sided automatic crab | `0.3375 m`; output `<= 0.05 m/s` | Mission did not complete |
| Control | Same-goal RPP retry | `0.4726 m`, yaw `-2.0008 deg` | A second boundary hold occurred |
| Control | Rapid retry containment | Map v14 recontact `0.276 s` (v13 `0.267/0.372 s`); one release; final Twist zero | Physical wheel response pending |
| UI | Browser/backend/ROS lifecycle | Mission, return, safety hold, and state 16 observed | No physical movement |
| Sensing/perception | Physical stationary report | Radar-off `600.063 s`; front camera `9.167 Hz` and `2750/2750` decode | Raw logs external; no accuracy or motion claim |
| Sensing | Rear camera field report | Raw `3.633 Hz` vs `10 Hz` target | Rate failed |
| Localization/system | Physical stationary report | Final pose `14.99 Hz`; CPU `99.26%` | Header-age p95 `352.5 ms`; CPU saturated |

## Boundary Contact And Recovery

| First complete-footprint contact | Current narrow-route risk |
|---|---|
| ![First boundary stop](assets/module-guides/control/first-route-boundary-stop-location.png) | ![Route risk map](assets/module-guides/planning/robot-center-narrow-route-risk-map.png) |

| Earlier manual gate probe | Current production command owner |
|---|---|
| ![Manual probe](assets/module-guides/control/pre-owner-robot-center-contact-sheet.png) | ![Automatic owner](assets/module-guides/control/automatic-owner-route-retry-contact-sheet.png) |

![Automatic route recovery](assets/module-guides/control/automatic-owner-route-retry.gif)

| Live RViz contact | Live gate status and zero output |
|---|---|
| ![Live boundary retry latch](assets/module-guides/control/runtime-boundary-retry-latch-20260804.png) | ![Live retry latch terminal](assets/module-guides/control/runtime-retry-latch-terminal-20260804.png) |

| Contact geometry | Allowed action |
|---|---|
| Exactly one lateral candidate clear | Pure crab away from contact |
| Both lateral candidates blocked and reverse clear | Reverse |
| Both lateral candidates clear or no candidate clear | Remain stopped |
| Fresh saved route clear for `1.0 s` | Release hold; retained RPP resumes yaw |
| Same route recontacts within `5.0 s` after one release | Latch hold, publish zero, require operator stop/replan |

Contact recovery is translation-only: raw speed `<= 0.10 m/s`, travel
`<= 0.40 m`, duration `<= 10 s`, angular command zero. Rotation resumes only
after route release because a swept-footprint contact proof is not implemented.

## Mission State Interpretation

| Layer | Examples | Priority/meaning |
|---|---|---|
| Parking internal phase | `PARKED` | Controller has reached its terminal pose |
| Public service state | `DROP_ZONE_WAIT`, `WAITING_FOR_CHARGING`, `CHARGING` | Charging feedback determines the visible state |
| Gate state | `ENABLED`, `ROUTE_SAFETY_HOLD`, battery stop | Final command authorization |
| Health state | `OK`, `WARN`, `ERROR` | Data/module health only |

`CHARGING` is public-state priority when CAN charging feedback is true even
though the parking controller is also internally `PARKED`. Expected site-entry
Nav2 cancellation is a control handoff, not a latched planning warning.

## Regeneration

Run from the repository root after changing referenced YAML, messages, code
constants, or evidence JSON:

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py
```

Render one package or a subset:

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py --module control
python3 camrod_bringup/scripts/render_module_readme_assets.py \
  --module sensing --module perception --module ui
```

Supported module names:

```text
bringup common control localization map planning perception platform
sensing sensor-kit system ui voice
```

Regression checks regenerate into a temporary directory and verify all asset
dimensions plus the 10 GIF frames:

```bash
pytest -q camrod_bringup/test/test_module_readme_assets.py
```

The renderer recreates source-derived diagrams only. `SIM RUNTIME CAPTURE`
files are intentionally not synthesized by it; repeat them from the exact
bringup command and record the matching metadata/raw log. Neither process
mutates runtime configuration.

## Field Evidence To Add

| Module | Required real-robot evidence |
|---|---|
| sensing/perception | Re-run and commit raw front-camera/radar-off evidence; rear-camera rate, seven-channel radar-on, calibration, detection/fusion/tag output |
| localization | Surveyed GNSS lever arm, GNSS-to-center residuals, yaw, wheel slip, vibration, moving latency |
| planning/control | All lane widths, physical oscillation, boundary stop distance, crab/reverse recovery, safe re-engage |
| platform | CAN latency, steering-angle settling, BMS current sign, actuator accuracy |
| voice | Speaker connection delay, audibility, interruption, and outdoor acceptance |
| bringup | Complete site arrival, explicit return, return route, selected parking, `PARKED`, and charging feedback |

Use production Jetson configuration unchanged while collecting these captures:

```bash
ros2 run camrod_bringup pose_latency_probe.py \
  --duration 60 --output-json "$HOME/camrod_field_logs/pose-chain.json"
ros2 run camrod_bringup camera_payload_probe.py --duration 300 --min-rate-hz 5
camrod_bringup/scripts/field_test_tool.sh snapshot
camrod_bringup/scripts/field_test_tool.sh record-recovery
```

Preserve raw log/bag/JSON first, then derive PNG/GIF with the command, baseline
commit, duration, and pass criteria. Until surveyed service access permits the
full `1.69160 x 1.27000 m` planning rectangle, the contract animation must stay
paired with the red `ROUND TRIP: NOT DEMONSTRATED` image.
