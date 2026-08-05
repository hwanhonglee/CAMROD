# camrod_bringup

<!-- HH_260805 - Synchronize Chromium, managed SHM, composed LiDAR processing,
and the optional-grid diagnostic contract. -->

Dependency-ordered full-stack launch, canonical configuration mirrors,
simulation profiles, and validation tools.

![Full-stack mission contract](../docs/assets/module-guides/bringup/full-stack-mission-contract.png)

## Actual Simulation Runtime

![Live full-stack B6 runtime](../docs/assets/module-guides/bringup/runtime-full-stack-b6-20260804.png)

`SIM RUNTIME CAPTURE`: actual RViz output from
`bringup.launch.py sim:=true rviz:=true`, not the contract renderer and not a
physical drive. The orange path is the dispatched B6 route.

## At A Glance

| Uses | Function | Outputs |
|---|---|---|
| ROS 2 launch, package-owned YAML, bringup mirrors | Starts platform through UI in dependency order | One complete ROS graph with selected hardware/simulation profile |
| Fake sensor/platform publishers | Exercises message, state, planner, control, and UI contracts | Deterministic simulation topics and normalized platform feedback |
| Probe and field-test scripts | Captures timing, payload, recovery, and mission evidence | JSON, rosbag, logs, PNG/GIF derived reports |

## Expected And Measured

The animation is a **contract**, not a runtime recording.

![Expected mission lifecycle](../docs/assets/module-guides/bringup/mission-lifecycle-contract.gif)

The latest full-bringup runtime result is shown separately.

![Full-bringup simulation evidence](../docs/assets/module-guides/bringup/simulation-evidence-20260804.png)

| Check | Result | Measured value |
|---|---|---:|
| Stack startup | PASS | 79 steady-state nodes; `[SYSTEM] OK` |
| Pose chain | PASS | 30 s probe; 20 Hz selected pose |
| B6 turnaround | FAIL CLOSED | `CRAB_IN -> lanelet_footprint_cost -> timeout` |
| B12 roadside stop | FAIL CLOSED | `CRAB_IN -> lanelet_footprint_cost -> timeout` |
| Rapid route recontact | PASS, FAIL CLOSED | One release; map v14 recontacted in 0.276 s (v13: 0.267/0.372 s), then latched with zero output |
| Map-v14 recovery rerun | PASS, FAIL CLOSED | route 0.366 s latch; reverse 0.0721 m; crab-left 0.3321 m; final commands zero |
| Map-v15 staged recovery | PASS, FAIL CLOSED | reverse-yaw max 0.05 rad/s; route recontact 0.335 s; crab-left 0.3378 m; final commands zero |
| Operator stop | PASS | `POST /ui/stop` returned HTTP 200; local owners and Nav2 canceled; state 16 published |
| Complete round trip | NOT DEMONSTRATED | Return, parking, and charging were not reached |

These 2026-08-04 captures and recovery records are historical map-v14 evidence.
The active synchronized OSM pair is now `map_version=15`. The active footprint
uses `0.05 m` on every body side while retaining the complete-footprint gate;
no map-v15 full-round-trip claim is inferred.

![Map v14 boundary recovery rerun](../docs/assets/module-guides/control/map-v14-boundary-recovery-contact-sheet.png)

[Open the map-v14 recovery GIF](../docs/assets/module-guides/control/map-v14-boundary-recovery.gif).

![Map v15 staged boundary recovery](../docs/assets/module-guides/control/map-v15-boundary-recovery-contact-sheet.png)

[Open the map-v15 recovery GIF](../docs/assets/module-guides/control/map-v15-boundary-recovery.gif).

## Physical Stationary Report

![Physical stationary field report](../docs/assets/module-guides/bringup/field-stationary-report-20260731.png)

The 2026-07-31 report records real hardware with physical E-stop held and no
motion. Its referenced raw files remain on the Jetson and were not committed;
the image therefore labels this as a field report, not self-contained raw
evidence.

## Launches

| Launch | Purpose | Default profile |
|---|---|---|
| `bringup.launch.py` | Physical Jetson/Ranger stack | Hardware drivers enabled by defaults |
| `bringup.launch.py sim:=true` | Full deterministic simulation | Fake sensors and raw Ranger/BMS boundaries |
| `bringup.launch.py sim:=true rviz:=true` | Simulation plus operator visualization | Shared CAMROD RViz config |

The physical operator window is a fullscreen Chromium kiosk by default. It
waits for backend readiness and uses an isolated profile, GPU rasterization,
zero-copy compositor, and disabled background throttling. WebKit remains
available with `operator_ui_window_engine:=webkit`; `auto` tries Chromium and
then WebKit. The launch-level `operator_ui_window_fullscreen:=false` override is
available for maintenance, and headless runs should set
`enable_operator_ui_window:=false`. Actual Jetson GPU use, frame pacing, and CPU
reduction remain field measurements.

Normal Ctrl+C shutdown lets ROS launch stop its own children. The optional
post-shutdown cleanup is disabled by default to avoid racing component and HTTP
server teardown; `clean_before_launch` still removes stale processes before the
next run.

```bash
source ~/camrod_ws/install/setup.bash

ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=true
ros2 launch camrod_bringup bringup.launch.py
```

## Startup Order

| Order | Package group | Required before next group |
|---:|---|---|
| 1 | platform, sensor kit | Robot frame and platform contracts |
| 2 | map | Lanelet map and static planning grid |
| 3 | sensing, perception | Sensor streams, dynamic costs, detections |
| 4 | localization | Selected `robot_center_link` pose |
| 5 | planning | Nav2 lifecycle and route servers |
| 6 | control | Final gate, maneuver, recovery, and parking owner |
| 7 | system, UI, voice | Standalone health checkers, health summary, and operator surfaces |

## Process And Transport Topology

| Scope | Production default | Opt-out / fallback |
|---|---|---|
| Camera -> YOLO | Existing component container with ROS intra-process | Separate package debug launches |
| LiDAR preprocessing -> ground segmentation | `lidar_processing_container` with ROS intra-process | `use_lidar_processing_container:=false` |
| LiDAR cost grid | Not loaded; node/topic omitted from diagnostics | `enable_lidar_cost_grid:=true` loads it in the LiDAR container with DDS transport for transient-local QoS |
| Low-rate diagnostics | 24 standalone checker processes | `use_checker_components:=true` retains a bench-only partial composition experiment |
| Inter-process DDS | Host RMW/environment; global SHM default `OFF` | `enable_dds_shared_memory:=true` is a bench-only CycloneDDS/iceoryx profile |

When explicitly enabled, RouDi starts before the delayed ROS module group. The
central CycloneDDS XML does not force host socket-buffer minima. The profile is
not a production default on ROS 2 Humble: a full CAMROD launch exhausts
iceoryx 2.0.5 static publisher/history capacity and can abort nodes. Small
bench graphs may use it; production keeps the host RMW path.

Checker component libraries are also retained for bounded experiments, but
`use_checker_components` is default `false`. Repeated complete-graph shutdowns
produced intermittent `-11` exits from more than one composed group on Humble;
the standalone topology reached `[SYSTEM] OK` and shut down cleanly.

## Canonical Configuration

| Path | Scope |
|---|---|
| `config/bringup/launch_defaults.yaml` | Feature selection and package parameter-file routing |
| `config/middleware/cyclonedds_shm.xml` | Bench-only CycloneDDS shared-memory profile |
| `config/middleware/iceoryx_roudi.toml` | RouDi pools through 8 MiB chunks for image/cloud traffic |
| `config/{package}/` | Synchronized deployment mirrors of package-owned YAML |
| `config/planning/nav2_planner_profiles/production.yaml` | Loads only `LaneletRoute` plus width-gated `SmacLattice` |
| `config/planning/nav2_controller_profiles/production.yaml` | Loads only mission `RPP` plus manual-goal `RotationShim` |
| `config/simulation/` | Fake sensor/platform inputs and simulation-only values |
| `config/system/diagnostics/{default,sim}/` | Field and simulation checker profiles |

Package-owned configuration remains the design source. Bringup mirrors are
deployment copies and are covered by synchronization tests.

## Validation Tools

| Tool | Measures |
|---|---|
| `pose_latency_probe.py` | Topic rate, age, and pose-chain continuity |
| `camera_payload_probe.py` | Physical payload decode, dimensions, and rate |
| `sim_validation_runner.py` | Mission/state scenarios and assertions |
| `field_test_tool.sh snapshot` | Nodes, topics, parameters, diagnostics, and platform state |
| `field_test_tool.sh record-recovery` | Boundary hold, candidate, owner command, and pose timeline |
| `render_module_readme_assets.py` | Reproducible source/evidence diagrams |
| `automatic_route_recovery_probe.py` | Production gate/controller crab, reverse, retry, latch, and final command timeline |
| `render_automatic_recovery_results.py` | Map-version-checked PNG/GIF from the three recovery timelines |

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py
pytest -q camrod_bringup/test/test_module_readme_assets.py
```

## Evidence

| Type | Location |
|---|---|
| Normalized module evidence | `docs/evidence/module-guides/` |
| Concise runtime excerpt | `docs/evidence/module-guides/bringup/raw/runtime-visual-capture-20260804.log` |
| Live runtime screen metadata | [`runtime-visual-capture-20260804.json`](../docs/evidence/module-guides/bringup/runtime-visual-capture-20260804.json) |
| Released v2.1.3 evidence | `docs/evidence/v2.1.3/` |
| Map-v14 recovery rerun | [`map-v14-boundary-recovery/`](../docs/evidence/v2.1.3/map-v14-boundary-recovery/) |
| Map-v15 staged recovery | [`map-v15-boundary-recovery/`](../docs/evidence/v2.1.4/map-v15-boundary-recovery/) |
| Interpretation and capture rules | [`docs/MODULE_VISUAL_GUIDE.md`](../docs/MODULE_VISUAL_GUIDE.md) |
