# camrod_bringup

<!-- HH_260805 - Synchronize scoped component shutdown, LiDAR-only SHM, and
the optional-grid diagnostic contract. -->
<!-- HH_260806 - Add map-v16 campsite sequencing, measured body geometry, and
arrival-only validation for constrained roadside sites. -->
<!-- HH_260806 - Record the B7 clear-road stop-go regression and independent lanelet safety grid. -->
<!-- HH_260806 - Record the B8 RPP curve-tracking and lookahead comparison. -->
<!-- HH_260806 - Record the 3 km/h active speed profile and bounded localization smoke result. -->
<!-- HH_260807 - Record map-v17 repeated service and safe persistent-obstacle handling. -->
<!-- HH_260807 - Record fixed-lookahead service A/B and single-owner platform status. -->
<!-- HH_260807 - Configure the current final 2 km/h production motion profile. -->
<!-- HH_260807 - Record the final B1-B10 no-restart lifecycle, route-snap
return handoff, diagnostic audit, and reproducible endurance media. -->

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

## Active v2.1.7 Contract

<!-- HH_260809 - Synchronize the full-bringup contract with the shared
tapered-front rounded body and exact planning offset. -->

| Item | Full-bringup value |
|---|---:|
| Physical GNSS / epoch | `5 Hz` / `200 ms` |
| EKF, selected pose, Nav2 control | `20 Hz` |
| Straight cruise | final `2.0 km/h` (`0.555556 m/s`) |
| Body / planning boundary | tapered rounded `1.39160 x 1.07000 m` / exact `0.10 m` offset `1.59160 x 1.27000 m` |
| LiDAR raw and filtered cloud | target `10 Hz`; no preprocessor throttling |
| Optional LiDAR cost grid | default `OFF` |
| Recovery release budget | `12` in the `5 s` recontact window; projected-safe inward escape remains eligible at budget |
| Radar display | `radar_status_gui.py` subscribes to seven real `/range_ros` streams; it does not publish dummy data |
| Operator window | WebKit fullscreen default |

![Current full-stack body and planning contours](../docs/assets/test_result/tapered-rounded-boundary-20260810/tapered-rounded-boundary-geometry.png)

![Full-stack boundary rigid motion](../docs/assets/test_result/tapered-rounded-boundary-20260810/tapered-rounded-boundary-motion.gif)

<!-- HH_260810 - Expose the active contour and frame motion without presenting
the source-derived GIF as full-stack runtime recovery evidence. -->
The [boundary visual record](../docs/assets/test_result/tapered-rounded-boundary-20260810/README.md)
is regenerated from package-owned geometry and checked against deployed Nav2
footprints. Runtime lanelet contact, candidate selection, and release still
require the separate simulation/field evidence below.

![Measured map-v17 road run with current contours](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/tapered-rounded-boundary-road-sim.png)

![Measured road drive contact recovery and completion](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/tapered-rounded-boundary-road-sim.gif)

<!-- HH_260810 - Bind current boundary documentation to one full ROS road run,
without converting workstation simulation into a physical-road claim. -->
The [current road-simulation record](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/README.md)
uses 511 live ROS pose samples over `29.700 s`: physical body clear, planning
margin cost 100, `REVERSE_YAW_RIGHT`, hold release, and original-route goal.

## Expected And Measured

The animation is a **contract**, not a runtime recording.

![Expected mission lifecycle](../docs/assets/module-guides/bringup/mission-lifecycle-contract.gif)

The latest full-bringup runtime result is shown separately.

![Full-bringup simulation evidence](../docs/assets/module-guides/bringup/simulation-evidence-20260804.png)

| Check | Result | Measured value |
|---|---|---:|
| Current tapered/rounded B2 road run | PASS (MEASURED ROS SIM) | body cost-100 contact no; planning contact yes; `0.0972 m`, `-4.545 deg`; no second hold; route complete |
| Stack startup | PASS | Fresh isolated map-v17 graph reached `[SYSTEM] OK` |
| Goal-independent UI startup | PASS | Zero RViz/UI goals; planning `WAIT_DZ`, UI `READY`; 5 authoritative READY frames in 2.2 s |
| B1 -> B2 -> B3 continuous service | PASS (3/3) | 677.237 s, zero restart; full site/RETURN/drop-zone/parking/charging/next-site lifecycle |
| B1-B10 service endurance | PASS (10/10) | `2210.611 s`, restart 0; cycle 1 seeded site handoff, cycles 2-10 full charger departure/outbound/RETURN/park/charge |
| Fixed-lookahead source profile | PASS (2/2) | B1/B2 in 422.848 s, zero restart; obstacle stop/resume, margin recovery, RETURN, park and charge |
| Charger departure | PASS | B2/B3 left CHARGING through `DEPARTING_CHARGER`; stale charge contact did not close drive authorization |
| B2 current-map boundary recovery | PASS (3/3) | `REVERSE_YAW_RIGHT`, mission complete, 1.5 s clear proof, no second hold or retry latch |
| Persistent obstacle on 3.0 m lane | SAFE-HOLD PASS | One no-path preflight, no selector/ABORT loop, original mission resumed after clear |
| Pose chain | PASS | 30 s probe; 20 Hz selected pose |
| B1-B10 site maneuver round trip | PASS (10/10) | Every map-derived entry (`1.79-5.31 m`) completed crab, 180-degree turn, explicit RETURN, crab-out, and `DONE` |
| B11-B13 roadside arrival | PASS | Capped `0.60 m` crab, unload wait, `WAIT_RETURN`; no zero-turn or RETURN issued |
| B11-B13 return | FIELD-PENDING | Earlier on-lane alignment attempt was safely blocked by `lanelet_physical_body_cost` |
| B7 clear-road command continuity | PASS | `224.92 s`, `27.1492 m`; Nav2 handoff `0`, post-start stale `0`, input/output `14.983/14.996 Hz` |
| B8 RPP curve tracking | PASS | `59.931 m`, `GOAL_REACHED`; raw R/T switches `403 -> 0`; one recovered planning-margin contact |
| B8 lookahead `1.2 m` comparison | REJECTED | first margin release then recontact in `0.999 s`; retry latched and route did not complete |
| 3 km/h command and pose smoke | PASS (AMD64 SIM) | `11.74 m`; final max `3.000001 km/h`; pose `20.024 Hz`; max step `6.485 cm`; `>20 cm` jumps `0` |
| Rapid route recontact | PASS, FAIL CLOSED | One release; map v14 recontacted in 0.276 s (v13: 0.267/0.372 s), then latched with zero output |
| Map-v14 recovery rerun | PASS, FAIL CLOSED | route 0.366 s latch; reverse 0.0721 m; crab-left 0.3321 m; final commands zero |
| v2.1.4 release-map staged recovery | PASS, FAIL CLOSED | release SHA `e0b50f...e36d`; reverse-yaw max 0.05 rad/s; route recontact 0.335 s; crab-left 0.3378 m; final commands zero |
| Reduced-boundary route/margin/body run | PASS, HISTORICAL | `1.29160 x 0.87000 m` candidate evidence retained; it is not the active body geometry |
| Operator stop | PASS | `POST /ui/stop` returned HTTP 200; local owners and Nav2 canceled; state 16 published |
| Drop-zone parking/charging continuation | PASS (AMD64 SIM) | All three continuous-service cycles reached `WAITING_FOR_CHARGING -> CHARGING` |
| Platform status ownership | PASS | `/platform/status` publisher count `1`; `ranger_platform_bridge` alone converts fake raw BMS feedback |
| Service handoff path diagnostic | PASS | Service-owned path clear is suppressed; WARN/ERROR timers each start at their own threshold with `3.0 s` grace |

The 2026-08-04 captures are historical map-v14 evidence, the staged map-v15
records are bound to the v2.1.4 release SHA, and the campsite policy media are
bound to map-v16. The current synchronized OSM pair is `map_version=17`, SHA
`8cd05c...5e021`; older results are intentionally not relabeled as current.
The active fabrication-inclusive physical body has `1.39160 x 1.07000 m`
bounding extents, a `0.12 m` tapered front shoulder, and `R0.05 m` corners. The
planning boundary is its exact `0.10 m` parallel offset, with
`1.59160 x 1.27000 m` extrema and `R0.15 m` corners. Physical-body cost 100 stops
ordinary motion; a virtual-boundary escape is eligible only while contact
overlap decreases monotonically and the swept body plus endpoint planning
footprint remain clear. Dynamic obstacles and interlocks remain fail-closed.
The current map-v17 continuous-service result separately covers complete
drop-zone parking, charging feedback, and departure to the next campsite.

![Package technology evidence matrix](../docs/assets/module-guides/bringup/package-technology-evidence.png)

![Package simulation/source evidence limits](../docs/assets/module-guides/bringup/package-technology-evidence.gif)

<!-- HH_260810 - Keep the cross-package evidence verdict beside the full-stack
runner while retaining package-specific figures in each package README. -->
The matrix records which package technologies have measured simulation output
and which remain source-only or field-pending. It does not change the map or
promote fake sensors into physical acceptance.

![B7 stop-go diagnosis](../docs/assets/test_result/cmd-vel-stop-go-20260806/b7-stop-go-diagnosis.png)

The [B7 regression record](../docs/assets/test_result/cmd-vel-stop-go-20260806/README.md)
also records the later real planning-margin contact; the full route is not
misreported as a completed campsite mission.

![3 km/h command and selected pose](../docs/assets/test_result/three-kph-localization-20260806/three-kph-command-pose.png)

The [3 km/h structured record](../docs/assets/test_result/three-kph-localization-20260806/README.md)
keeps the historical 3 km/h maneuver ratios, raw/final command trace, fusion inputs, bag
hash, and the explicit limitation that fake 10 Hz GNSS does not validate the
physical 5 Hz moving-base link or its epoch alignment.

![Historical reduced-boundary policy validation](../docs/assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)

![Historical map-v16 campsite validation](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-policy-validation.png)

[Open the campsite sequencing GIF](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-phase-sequence.gif).

![Current map-v17 repeated service](../docs/assets/test_result/v2-1-5-service-validation-20260807/repeated-service-summary.png)

[Open the continuous-service GIF](../docs/assets/test_result/v2-1-5-service-validation-20260807/repeated-service-timeline.gif).

![B1-B10 no-restart service endurance](../docs/assets/test_result/b1-b10-service-endurance-20260807/b1-b10-service-endurance.png)

[Open the B1-B10 endurance GIF](../docs/assets/test_result/b1-b10-service-endurance-20260807/b1-b10-service-endurance.gif).

The [final endurance record](../docs/assets/test_result/b1-b10-service-endurance-20260807/README.md)
contains the raw runner JSON, public state/recovery/charging log, B4 route-snap
geometry, path-diagnostic and Robot/Guest shutdown smokes, checksum manifest,
and exact claim boundary. B1 is a seeded site handoff; B2-B10 are nine complete
charger-departure services. The result does not claim physical-road acceptance
or a successful bypass on the current narrow map.

![3 km/h fixed-lookahead service A/B](../docs/assets/test_result/rpp-lookahead-service-ab-20260807/rpp-lookahead-service-ab.png)

The [RPP service A/B](../docs/assets/test_result/rpp-lookahead-service-ab-20260807/README.md)
records the rejected velocity-scaled run and the selected fixed `1.1 m` run.
The validation runner observes normalized platform status; it changes only the
fake raw-BMS battery input. This leaves `/platform/status` with one production-
equivalent owner and prevents `CHARGING`/`DROP_ZONE_WAIT` oscillation.

![Persistent obstacle safe hold](../docs/assets/test_result/v2-1-5-service-validation-20260807/obstacle-safe-hold.png)

![Map v14 boundary recovery rerun](../docs/assets/module-guides/control/map-v14-boundary-recovery-contact-sheet.png)

[Open the map-v14 recovery GIF](../docs/assets/module-guides/control/map-v14-boundary-recovery.gif).

![v2.1.4 release-map staged boundary recovery](../docs/assets/module-guides/control/map-v15-boundary-recovery-contact-sheet.png)

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

The physical operator window is a fullscreen GTK/WebKit surface by default on
the current Jetson image because its Snap Chromium cannot start in the robot
launch context. Chromium remains selectable on hosts with a working native
browser via `operator_ui_window_engine:=chromium`. The launch-level
`operator_ui_window_fullscreen:=false` override is
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
| 7 | system, UI, voice | Composed checker fault domains, health summary, and operator surfaces |

## Process And Transport Topology

| Scope | Production default | Opt-out / fallback |
|---|---|---|
| Camera -> YOLO | Existing component container with ROS intra-process | Separate package debug launches |
| Rear camera -> rectify -> AprilTag | One ROS intra-process container when physical rear camera, parking, and `parking_method:=apriltag` are active | `use_rear_camera_apriltag_container:=false` restores the three standalone owners |
| LiDAR preprocessing -> ground segmentation | `lidar_processing_container` with ROS intra-process | `use_lidar_processing_container:=false` |
| LiDAR cost grid | Not loaded; node/topic omitted from diagnostics | `enable_lidar_cost_grid:=true` loads it in the LiDAR container with DDS transport for transient-local QoS |
| Nav2 lifecycle servers | Planner/controller in `nav2_container`; vendor smoother/behavior/BT/lifecycle standalone | `use_nav2_container:=false` restores standalone planner/controller |
| System aggregate/status chain | Four nodes in serialized `system_core_container` | `use_system_tools_container:=false` restores standalone tools |
| Low-rate diagnostics | 24 checkers in four serialized fault-domain containers | `use_checker_components:=false` restores 24 standalone checker processes |
| Inter-process DDS | Host RMW; global SHM never applied | `enable_dds_shared_memory:=true` enables CycloneDDS/iceoryx only for a physical LiDAR driver group |

When explicitly enabled, RouDi starts only for non-simulation physical LiDAR.
The CycloneDDS environment is scoped to that launch group, so camera, Nav2,
system, UI, and the rest of the graph cannot consume iceoryx endpoints. This
removes the former full-graph capacity failure while keeping SHM default OFF
until the physical LiDAR path is measured on Jetson.

ROS 2 Humble retained checker entity/guard-condition references after context
shutdown and could unload RMW code while CycloneDDS `tev`/`recv` threads still
ran. `camrod_runtime` now owns one explicit context, detaches components,
retains plugin code through cleanup, and bypasses only the unsafe DSO static
destructor phase after explicit teardown. Four checker groups, system core,
and the Nav2 planner/controller container then reached `[SYSTEM] OK` and clean
exit in 3/3 final amd64 runs; one no-topology-override run also passed.
Standalone fallbacks remain available. Jetson resource and ten-cycle mission,
cancel, restart, camera, and physical-rate acceptance remain in `TODOLIST.txt`.

## Canonical Configuration

| Path | Scope |
|---|---|
| `config/bringup/launch_defaults.yaml` | Feature selection and package parameter-file routing |
| `config/middleware/cyclonedds_shm.xml` | Default-off CycloneDDS shared-memory profile scoped to the physical LiDAR driver group |
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
| `render_robot_boundary_validation.py` | Consolidated normal-route, margin, and physical-body simulation PNG |
| `render_tapered_rounded_boundary.py` | Current source-derived body/planning PNG, rigid-motion GIF, coordinates, and SHA manifest |
| `render_tapered_rounded_road_sim.py` | Current map-v17 ROS poses, gate states, exact contours, recovery result, and PNG/GIF |
| `render_camping_site_sequence_results.py` | Structured B1-B10 turnaround and B11-B13 roadside-arrival PNG/GIF |
| `render_v2_1_5_service_results.py` | Repeated-service, obstacle, and B2 recovery JSON -> PNG/GIF |
| `render_rpp_service_ab.py` | Fixed-versus-scaled RPP service decision JSON -> PNG |

```bash
python3 camrod_bringup/scripts/visualization/render_module_readme_assets.py
python3 camrod_bringup/scripts/visualization/render_tapered_rounded_boundary.py
python3 camrod_bringup/scripts/visualization/render_tapered_rounded_road_sim.py
pytest -q camrod_bringup/test/test_module_readme_assets.py
python3 -m pytest -q camrod_bringup/test/test_tapered_rounded_boundary_assets.py
python3 -m pytest -q camrod_bringup/test/test_tapered_rounded_road_sim_assets.py
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
| Current boundary geometry/motion | [`tapered-rounded-boundary-20260810/`](../docs/assets/test_result/tapered-rounded-boundary-20260810/) |
| Current boundary measured road simulation | [`tapered-rounded-boundary-road-sim-20260810/`](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/) |
| Current boundary runtime tests | [`robot-boundary-adjustment-20260806/`](../docs/assets/test_result/robot-boundary-adjustment-20260806/) |
| Current campsite sequencing tests | [`camping-site-sequencing-20260806/`](../docs/assets/test_result/camping-site-sequencing-20260806/) |
| Current RPP curve-tracking tests | [`rpp-curve-tracking-20260806/`](../docs/assets/test_result/rpp-curve-tracking-20260806/) |
| Current 3 km/h command/localization smoke | [`three-kph-localization-20260806/`](../docs/assets/test_result/three-kph-localization-20260806/) |
| Current map-v17 service/safety acceptance | [`v2-1-5-service-validation-20260807/`](../docs/assets/test_result/v2-1-5-service-validation-20260807/) |
| Current 3 km/h RPP service A/B | [`rpp-lookahead-service-ab-20260807/`](../docs/assets/test_result/rpp-lookahead-service-ab-20260807/) |
| Final B1-B10 service endurance | [`b1-b10-service-endurance-20260807/`](../docs/assets/test_result/b1-b10-service-endurance-20260807/) |
| Interpretation and capture rules | [`docs/MODULE_VISUAL_GUIDE.md`](../docs/MODULE_VISUAL_GUIDE.md) |
