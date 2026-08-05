# Documentation Changelog

<!-- HH_260805 - Publish the scoped runtime remediation and align the explicit
GNSS center assumption without claiming a physical antenna survey. -->
## [v2.1.5-release-sync] - 2026-08-05 (HH_260805)

### Changed

| Area | Released behavior and documentation basis |
|---|---|
| GNSS position contract | Changed the unmeasured converted placeholder from X `-0.443 m` to the explicit `robot_center_link` assumption `(0,0,0)` used by localization; retained `pose_verified=false` |
| Measurement boundary | Left all non-GNSS sensor coordinates unchanged and moved dual-antenna XYZ, baseline, receiver reference, and any needed lever-arm correction to the physical TODO |
| Runtime shutdown | Released explicit-context component containers, four checker fault domains, scoped planner/controller Nav2 composition, and normal Robot/Guest external-shutdown handling |
| DDS transport | Removed full-graph SHM injection; physical LiDAR-only DDS-SHM and the LiDAR cost grid both remain default-OFF pending Jetson validation |
| Evidence and visuals | Moved current runtime records under `v2.1.5`, regenerated sensor-kit figures, and marked 2026-08-04 runtime captures as historical geometry screens |
| Version baseline | Synchronized root, package, TODO/DONE, module guide, release notes, tests, and package/bringup configuration for `develop` and annotated `v2.1.5` |

### Validation

- GNSS frame, Xacro fallback, diagnostics, package/bringup mirrors, and visual
  references are protected by focused source contracts.
- Four final AMD64 full-simulation runs reached managed Nav2 active and
  `[SYSTEM] OK`; every run stopped six component containers without `-11`,
  forced kill, UI event-loop, iceoryx capacity, or residual-process failure.
- Eleven packages built; xUnit reported 487 tests with zero errors/failures and
  17 existing lint skips. Direct UI pytest passed 28/28 and focused source
  contracts passed 126/126; config audit matched 388/388 package, bringup, and
  install comparisons.
- The final domain-207 full simulation reached managed Nav2 active and
  `[SYSTEM] OK`, exited `0`, and cleanly stopped all six containers. Exact data
  is in `DONE.txt` and [`V2_1_5_RELEASE_NOTES.md`](V2_1_5_RELEASE_NOTES.md).

---

<!-- HH_260805 - Record the scoped Humble shutdown remediation separately from
the initial topology A/B so the failed pre-fix runs remain auditable. -->
## [v2.1.5-scoped-container-shutdown] - 2026-08-05 (HH_260805)

### Changed

| Area | Current behavior and basis |
|---|---|
| Component runtime | Added `camrod_runtime` with one explicit Context, ordered component/plugin teardown, and process-exit containment for the ROS 2 Humble Context/DSO lifetime race |
| System checkers | Enabled 24 checkers in four serialized fault-domain containers by default; retained all standalone executables as field-isolation fallbacks |
| Nav2 | Enabled the scoped planner/controller container by default; retained smoother, behavior, BT, and lifecycle servers standalone because they own private vendor executors |
| UI shutdown | Treats both `KeyboardInterrupt` and `ExternalShutdownException` as normal Robot/Guest launch shutdown |
| DDS shared memory | Removed full-graph environment injection; explicit SHM now applies only to a non-simulation physical LiDAR driver group and remains default-OFF |
| Field work | Kept Jetson resource measurement and ten mission/cancel/restart cycles in `TODOLIST.txt`; no AMD64 result is labeled physical acceptance |

### Validation

- Native tracing located the pre-fix `-11` in CycloneDDS `tev`/`recv` workers
  racing code unload after delayed Humble Context finalization.
- Three final full-simulation runs and one default-argument run reached managed
  Nav2 active and `[SYSTEM] OK`; all six component containers stopped cleanly
  with no `-11`, forced kill, event-loop error, or iceoryx capacity error.
- Rebuilt the six affected packages and passed 22 focused shutdown, launch,
  transport, and UI contracts.
- The superseding machine-readable record is
  [`amd64-scoped-container-shutdown-20260805.json`](evidence/v2.1.5/runtime-topology/amd64-scoped-container-shutdown-20260805.json).

---

<!-- HH_260805 - Synchronize the component topology docs and keep unmeasured
Jetson comparisons in the field TODO instead of presenting estimates. -->
## [v2.1.5-transport-and-recovery-final] - 2026-08-05 (HH_260805)

### Changed

| Area | Final behavior and basis |
|---|---|
| Operator renderer | Changed the production local-window default from WebKit to Chromium; retained explicit WebKit and Chromium-first `auto` fallback modes |
| ROS intra-process | Composed LiDAR preprocessing/ground segmentation, retained front camera/YOLO, and added the physical rear capture/rectify/AprilTag chain |
| DDS shared memory | The initial full-graph profile exhausted Humble ports/history; the scoped-container follow-up removed global injection and limited opt-in SHM to the physical LiDAR group |
| Optional LiDAR rasterizer | Set `enable_lidar_cost_grid=false` by default; the component, graph node/topic requirements, and cost-grid checker entry now activate together |
| Boundary recovery | Replaced the permanently latched first direction with projected staged reverse, reverse-yaw, and crab transitions; retained full-footprint, freshness, dynamic-obstacle, distance, time, yaw, and retry-latch limits |
| Active map copy | Recorded the byte-identical current-site pair fingerprint at SHA `d7b730...213f` (55 lanelets, 14 areas, 1658 nodes, 236 ways); this is a regression check, not a read-only map lock |
| Map evidence ownership | Kept v2.1.4 recovery media bound to release SHA `e0b50f...e36d`; the renderer must reject attempts to relabel it with the current-site map |
| System runtime | Initial A/B fell back to 24 standalone checkers after a shutdown fault; the scoped-container follow-up fixed that fault and enabled four checker containers |
| Nav2 runtime | Initial full-server composition failed shutdown (`-11`, `-11`, forced `-9`); the follow-up uses a cleanly validated planner/controller hybrid container |
| Runtime measurements | Added measured amd64 system-core and LiDAR A/B results; retained only the remaining Jetson camera/GPU/physical-rate and restart acceptance in `TODOLIST.txt` |
| Package guides | Updated bringup, sensing, system, UI, control, map, root, release, TODO, and generated architecture assets; historical map-v14, release-map-v15 evidence, and the current-site OSM are separated |

### Validation

- Added a normalized amd64 A/B record and chart. System-core composition reduced
  the full-sim process tree by about 3 processes, CPU by 2.5 one-core points,
  and PSS by 19.7 MiB with 3/3 controlled launch stops. LiDAR composition preserved
  10 Hz and reduced CPU by 17.5%, while isolated PSS increased by 44.0 MiB.
- Added three-run WebKit/Chromium workstation measurements. WebKit used 327.3
  MiB less PSS and 1.44 fewer one-core CPU points. Chromium's full stack reached
  system OK 1.33 seconds sooner, but this is not labeled browser load time.
  Host-wide GPU readings are retained but not used.
- Rebuilt the six modified packages and recorded 423 fresh xUnit cases with
  zero errors/failures and 12 existing slow-cppcheck skips; direct bringup
  pytest passed 134/134.
- Recorded the unchanged current-site map pair SHA `d7b730...213f`. The visual
  test proves release-SHA recovery JSON is rejected against this newer OSM;
  neither the test nor SHA makes the files read-only.
- All modified C++ packages compiled on the amd64 simulation workstation.
- Runtime contract tests cover Chromium defaults, component/intra-process
  registration, shared-memory profiles, and LiDAR grid OFF/ON diagnostics.
- RouDi loaded the checked-in pool and two isolated ROS processes exchanged a
  message. The initial full-graph bringup reproduced iceoryx endpoint/history
  exhaustion; global injection is now impossible and explicit opt-in is scoped
  to the physical LiDAR driver group.
- Current staged recovery is policy/geometry unit tested and dynamically rerun
  on map v15. Reverse-yaw, crab, rapid-recontact latch, and final zero output
  passed; route completion and real-robot response remain field-pending.
- The final changed-package rerun emitted 328 fresh CMake/xUnit cases with zero
  errors, failures, or skips; direct UI pytest passed 27/27 and the React
  production build synchronized successfully. Isolated LiDAR OFF/ON launches
  confirmed no rasterizer process when disabled and one transient-local grid
  publisher when enabled.

---

<!-- HH_260805 - Close the v2.1.4 source, configuration, visual, and field-limit
record without converting workstation checks into Jetson acceptance. -->
## [v2.1.4-release-sync] - 2026-08-05 (HH_260805)

### Changed

| Area | Released behavior and documentation basis |
|---|---|
| Runtime baseline | Updated root, package, TODO/DONE, field runbook, and release references to the synchronized v2.1.4 source/config baseline |
| Obstacle policy | Documented immediate safety stop separately from the `20.0 s` persistent-block requirement for width-gated SmacLattice preemption |
| Nav2 controller loading | Production constructs only `RPP` and manual-goal `RotationShim`; DWB/MPPI/Graceful remain in an opt-in development profile |
| Operator window | Documented WebKit static caching, smooth touch scrolling, always-on hardware-acceleration request, and disabled unused WebGL; retained Jetson GPU/CPU/FPS as field-pending |
| Shutdown ownership | Recorded orderly Robot/Guest uvicorn thread teardown, progress-plugin callback ownership cleanup, serialized checker executors, and disabled the default post-shutdown `pkill` pass that raced normal ROS launch cleanup |
| Footprint/map | Recorded the unchanged measured body, four-sided 5 cm planning envelope, user-authored map v15, and the limit of the offline geometry sweep |
| Package visuals | Regenerated source-derived architecture assets with a distinct paired palette per module and added a regression check for palette identity |
| Evidence ownership | Kept normalized JSON/log provenance consumed by renderers and tests; removed only redundant raw logs and unused timeline inputs |
| Radar | Explicitly made no new fixed-return exclusion because current clutter observations cannot separate self-return from real obstacles |

See [`V2_1_4_RELEASE_NOTES.md`](V2_1_4_RELEASE_NOTES.md) for exact active
values, verification, and remaining physical acceptance work.

### Validation

- The eight selected CAMROD packages and bundled `nav2_controller` built and
  completed 380 emitted xUnit cases with 0 errors, 0 failures, and 24 skips.
- Direct `camrod_ui` pytest execution passed all 27 contract tests, and the
  optimized React production build compiled and synchronized successfully.

---

<!-- HH_260805 - Synchronize the current static planning envelope to the
four-sided 0.05 m clearance requested after map-v15 visual review. -->
## [four-sided-five-centimeter-planning-envelope] - 2026-08-05 (HH_260805)

### Changed

| Area | Operational change and basis |
|---|---|
| Measured body | Kept the surveyed `1.49160 x 1.07000 m` body and its hard cost-100 stop contract unchanged |
| Static planning envelope | Reduced only front/rear clearance from `0.10 m` to `0.05 m`; left/right remain `0.05 m`, yielding `1.59160 x 1.17000 m` and center-frame extents `0.80837/0.78323/0.58505/0.58495 m` |
| Runtime synchronization | Applied the same polygon to sensor-kit geometry, Nav2 local/global costmaps, platform RViz boundary, command safety gate, package defaults, and bringup mirrors |
| Dynamic obstacle safety | Kept LiDAR/radar forward, rear, and side dynamic corridors unchanged; those conservative moving-obstacle windows are independent of the static lanelet footprint |
| Historical evidence | Kept v2.1.3 and map-v14 renderer dimensions at their recorded values and labelled them historical rather than silently regenerating old results |

### Validation Scope

An offline map-v15 geometry sweep sampled 58 poses on lanelets
`754/2751/2720`, with bounded lateral (`+/-0.4 m`) and yaw (`+/-20 deg`)
adjustment. Center-pose failures were `7` with the prior asymmetric envelope
and `6` with the current four-sided 5 cm envelope; both had `0` samples without
a tested adjustment. Five affected ROS packages built successfully and their
combined 152 tests passed. This proves configuration consistency and geometric
fit within the sampled search only, not dynamic completion or real-robot
safety. No lanelet map file was modified.

---

<!-- HH_260805 - Record the WebKit deployment default, bounded obstacle
fallback, and diagnostic-checker composition without claiming Jetson results. -->
## [webkit-wide-lane-replan-system-composition] - 2026-08-05 (HH_260805)

### Changed

| Area | Operational change and basis |
|---|---|
| Robot operator window | Made WebKit the explicit deployment default; added backend-readiness probing before first load and document/page caching, enabled smooth touch scrolling, requested always-on acceleration, and disabled only unused WebGL; retained explicit Chromium and Chromium-first `auto` modes |
| Nav2 planner loading | Production now constructs only `LaneletRoute` and reachable fallback `SmacLattice`; `Smac2D`, `NavFn`, `ThetaStar`, and `SmacHybrid` implementations remain configured and can be loaded through the opt-in `all.yaml` profile |
| Dynamic obstacle fallback | Kept immediate obstacle safety stopping and enabled planner preemption only after `20.0 s` persistent blockage when a fresh lanelet grid proves at least `2.50 m` contiguous mapped width and `0.60 m` clearance on each side; missing, stale, outside-map, and narrow cases retain LaneletRoute and fail closed |
| System diagnostics | Added component libraries for the 24 low-rate checkers and retained standalone executables; subsequent full-stack stress testing superseded this initial container-default plan |
| Map contracts | Updated active-map validation and package references to the user-authored synchronized `map_version=15`; preserved map-v14 runtime evidence against its own recorded SHA instead of comparing it to the current map |
| Package and bringup docs | Synchronized the active WebKit, planner-profile, obstacle-width, system-container, and map-revision behavior across module READMEs and central defaults |

### Validation

- WebKitGTK 2.50.4 loaded the live backend on its first render attempt and shut
  down cleanly on the amd64 workstation.
- An isolated Nav2 launch exposed exactly `LaneletRoute` and `SmacLattice` in
  `planner_server.planner_plugins`; all launch processes shut down cleanly.
- Isolated system component cycles loaded all 24 plugins and could stop cleanly.
  Later repeated full-stack shutdowns reproduced intermittent component exits,
  so the final production default was restored to standalone checkers; the
  component topology and `~82 MiB` sample remain bench-only observations.
- Synthetic wide/narrow grid tests, planner-profile synchronization tests,
  active map-v15 structure tests, and historical map-v14 evidence checks pass.

---

<!-- HH_260805 - Remove completed merge inputs and redundant raw evidence before
the next user-authored lanelet map revision. -->
## [workspace-cleanup-before-next-map] - 2026-08-05 (HH_260805)

### Removed

| Item | Reason |
|---|---|
| `for_merge/` | The 607 MB comparison snapshot had been fully integrated and validated |
| `/tmp/*camrod*` | Removed about 1.2 GB of temporary browser profiles, test logs, captures, and generated scratch files without touching other system temporary files |
| Six B6/B12 raw node logs | Their outcomes already exist in normalized campsite evidence and module-guide figures |
| Two pre-owner manual recovery timelines | Their comparison GIFs and summarized results remain; no current document or renderer consumed the raw JSON |

All referenced module-guide PNG/GIF assets, normalized JSON, the concise runtime
excerpt, and map-v14 versioned recovery evidence remain available. No lanelet
map file was changed during this cleanup. The evidence contract test now checks
the self-contained normalized provenance instead of requiring deleted raw logs.

---

<!-- HH_260804 - Record selective for_merge integration and bind regenerated
recovery visuals to the exact user-provided map revision. -->
## [post-v2.1.3-for-merge-and-map-v14-rerun] - 2026-08-04 (HH_260804)

### Changed

| Code, config, doc, or asset | What changed |
|-----|--------------|
| `for_merge` integration | Compared the duplicate bringup/UI trees file-by-file and selected newer kiosk, WebSocket, arrival-site, endpoint, icon, return-screen, and site-verification behavior; a second semantic audit corrected the initially missed Robot UI keypad hunks |
| Robot operator window | Kept fullscreen launch/config wiring, replaced the high-CPU forced WebKit path with portable Chromium auto-selection and an isolated kiosk profile, retained WebKit as fallback, and verified clean process shutdown |
| Guest WebSocket | Serialized outgoing frames, removed awaits under the thread lock, moved ROS work off the uvicorn event loop, added 10-second heartbeat/45-second stale close, and guaranteed slot cleanup |
| Robot backend/frontend | Preserved the active campsite through destination acknowledgement, made simple REST handlers nonblocking to the event loop, installed the updated facility/trail PNG assets, and restored the `B<N>` verification keypad plus Guest-return idle-screen exit |
| Robot keypad layout/test | Added uppercase virtual/physical input handling, function-based updates that retain queued touch keys, 1280x800-safe sizing, a 1920x1080 browser capture, 24 UI contract tests, and wrong/correct/cancel/Guest-return/admin-login browser checks |
| Robot UI render load | Replaced continuously repainting SVG range rings and moving/return text pulses with static status cues; the current Chromium browser/GPU/renderer sample fell from 15.8% to 0.5% combined while retaining one-second clock display |
| recovery probe/renderer | Records map version/hash, exact scenario lanelets, retry timing/latch, final output, and generates measured rather than hard-coded v14 PNG/GIF labels |
| map-v14 evidence | Added three full-bringup JSON timelines, two PNGs, and one GIF; route recontacted in 0.366 s and latched, static reverse moved 0.0721 m, crab-left moved 0.3321 m, and every final command was zero |
| local merge snapshot | Used `for_merge/COLCON_IGNORE` during review; the temporary snapshot was removed after integration on 2026-08-05 |
| [`FOR_MERGE_INTEGRATION_20260804.md`](FOR_MERGE_INTEGRATION_20260804.md) | Preserved the comparison counts, selected changes, retained current behavior, exclusions, and validation basis |

The v14 route remains fail-closed and no real-robot safety or completion claim
is inferred. UI production build and package tests passed on the workstation;
the changes do not retune Jetson controller, localization, or sensor values.

---

<!-- HH_260804 - Close the observed clear/release/recontact loop, repair the
operator-stop service call, and attach actual runtime screens to every module. -->
## [v2.1.3-runtime-captures-and-retry-latch] - 2026-08-04 (HH_260804)

### Changed

| Code, config, doc, or asset | What changed |
|-----|--------------|
| `camrod_control` route recovery policy | Allows one automatic route release, then latches a same-route recontact within 5 seconds; blocks further candidates/releases and keeps final output zero until stop/replan/re-engage |
| control and bringup `cmd_vel_safety_gate.yaml` | Added synchronized `route_safety_recovery_max_auto_releases: 1` and `route_safety_recovery_recontact_window_s: 5.0` deployment values |
| `camrod_ui` backend | Replaced invalid rclcpp-style `async_send_request()` with rclpy `call_async()` so `POST /ui/stop` cancels Nav2 and reaches `OPERATOR_STOPPED` instead of HTTP 500 |
| package READMEs and `MODULE_VISUAL_GUIDE.md` | Added 14 actual RViz/ROS CLI runtime screens, explicitly labelled simulation rather than generated or physical evidence |
| user-provided Lanelet2 map revision 14 | Preserved both byte-identical OSM files; ten boundary LineStrings were reshaped without changing relation IDs; full bringup loaded 55 lanelets/14 areas but B6 still contacted the boundary near `(4.3688, 45.0583)` |
| bringup runtime evidence | Preserved launch command, B6 dispatch, map-v14 0.276-second recontact (plus v13 history), retry latch, zero Twist, HTTP 200 stop, controller/Nav2 cancellation, and state 16 in JSON plus a concise raw excerpt |
| launch documentation | Removed nonexistent `bringup_sim.launch.py`, `bringup_minimal.launch.py`, and `rviz.launch.py` examples; documented the real `bringup.launch.py sim:=true rviz:=true` entrypoint |
| tests | Added policy coverage for rapid recontact and delayed retry, UI stop client coverage, runtime image/metadata checks, launch-document regression checks, and map-v14 synchronization/structure checks |

The retry-latch and UI-stop results are full-stack simulation evidence. The B6
route still fails closed at missing surveyed service-access geometry, and no
real wheel, sensor, charger, or speaker FIELD-PASS is inferred.

---

<!-- HH_260804 - Replace long package narratives with scan-first visual,
value, performance, and evidence tables without changing runtime behavior. -->
## [post-v2.1.3-all-package-visual-readmes] - 2026-08-04 (HH_260804)

### Changed

| Doc or asset | What changed |
|-----|--------------|
| root and 14 CAMROD package READMEs | Standardized the first-screen order as uses/function/output, active values, measured result, evidence limit, run command, and source links; removed repeated release prose and Mermaid diagrams |
| `camrod_common/avg_msgs` | Replaced the handwritten 86-row-style catalog/dependency narrative with a generated 86-message, 2-service, 12-dependent inventory and operational families |
| control, map, platform, system, UI, and voice READMEs | Added source-derived package diagrams; control and UI diagrams also consume committed simulation/browser evidence |
| sensing, perception, localization, planning, sensor-kit, and bringup READMEs | Retained existing visuals but converted long descriptions into compact numerical and validation tables |
| `docs/assets/module-guides/*` | Expanded the main renderer from ten to 18 PNGs, retained its 10-frame GIF, added an explicitly raw-external physical report, and moved seven release PNGs plus five release GIFs under their owning modules |
| `docs/evidence/module-guides/bringup/field-stationary-20260731.json` | Normalized existing radar, camera, GNSS/IMU, localization, and Jetson resource values while preserving `raw_files_committed=false` and external raw roots |
| `docs/MODULE_VISUAL_GUIDE.md` | Indexed every package, separated configuration from performance, retained boundary/yaw recovery comparisons, and listed missing physical evidence |
| visual renderer and test | Added package-target generation for `common`, `control`, `map`, `platform`, `system`, `ui`, and `voice`; full regeneration now checks every asset |

No runtime algorithm, package/bringup configuration, map, sensor mount, robot
geometry, safety threshold, or released `v2.1.3` tag changed. Values shown as
performance come only from committed evidence; all remaining hardware results
are labelled `FIELD PENDING`.

---

<!-- HH_260804 - Replace the crowded sensor-kit layout with a direct 4WS
rear-axle-versus-center comparison and compact value tables. -->
## [post-v2.1.3-sensor-kit-reference-cleanup] - 2026-08-04 (HH_260804)

### Changed

| Doc or asset | What changed |
|-----|--------------|
| `camrod_sensor_kit/README.md` | Consolidated four repetitive architecture sections into one runtime contract and shortened the frame-migration summary |
| sensor-kit module visuals | Replaced the dense TF/mount plot with a two-panel reference comparison, restored the useful physical side view as its own image, and retained a three-column sensor-X table |
| sensor-kit visual renderer/test | Added the committed A/B simulation metrics and retained deterministic package-only generation coverage |
| `docs/MODULE_VISUAL_GUIDE.md` | Added the first boundary-contact location, manual/current contact-sheet comparison, automatic crab/reverse GIF, yaw-release rule, and evidence ownership distinction |

No runtime frame, geometry, sensor pose, controller, or safety value changed.

---

<!-- HH_260804 - Add reproducible package-level visual guides without changing
the already-published v2.1.3 runtime baseline. -->
## [post-v2.1.3-module-visual-guides] - 2026-08-04 (HH_260804)

### Changed

| Doc or asset | What changed |
|-----|--------------|
| `camrod_bringup/README.md` | Paired the expected full mission contract/GIF with the measured B6/B12 fail-closed campsite result |
| sensor kit, localization, planning, perception, and sensing READMEs | Added package-specific frame/input/algorithm/output visuals and explicit source/simulation/field evidence limits |
| `docs/MODULE_VISUAL_GUIDE.md` | Added evidence classes, module interpretation, regeneration, and Jetson capture commands |
| `docs/assets/module-guides/*` | Added ten generated PNGs and one 10-frame contract GIF, including the rear-axle/center comparison, sensor side view, and sensor-coordinate table |
| `docs/evidence/module-guides/*` | Preserved the 30-second pose-chain measurement, full-bringup campsite verdict, and six source ROS node logs with checked line references |
| `render_module_readme_assets.py` and regression test | Made all module visuals reproducible from current YAML/messages and committed evidence |

This documentation checkpoint changes no runtime algorithm, package/bringup
configuration value, robot geometry, or safety threshold. The failed campsite
entry remains visible pending surveyed `service_access` polygons.

---

<!-- HH_260804 - Package the v2.1.3 release and normalize evidence ownership. -->
## [v2.1.3-release-and-evidence-layout] - 2026-08-04 (HH_260804)

### Changed

| Doc or asset | What changed |
|-----|--------------|
| root and all affected package `README.md` files | Synchronized the center-frame, bounded recovery, RPP, Guest UI, map-limit, parking-TF, diagnostics, and field-acceptance contracts across bringup, control, localization, map, perception, planning, platform, sensing, sensor kit, system, UI, and voice |
| `docs/V2_1_3_RELEASE_NOTES.md` | Added release scope, safety invariants, verification, evidence layout, and remaining physical acceptance |
| `docs/assets/module-guides/{sensor-kit,planning,control,ui}` | Organized reference-frame, boundary-geometry, pre-owner/current recovery, and Guest UI visuals by owning module rather than release folder |
| `docs/evidence/v2.1.3/*` | Added the previously workspace-local A/B drive, 0/5/10 cm geometry sweeps, and manual recovery timelines beside current automatic-owner evidence |
| `docs/V2_1_3_ROBOT_CENTER_MIGRATION.md` | Added A/B drive measurements, 0/5/10 cm geometry comparison, center-frame visuals, and source JSON links |
| `docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md` | Separated manual pre-owner runs from automatic-owner runs and linked the normalized evidence tree |
| `camrod_bringup` render tools | Installed both automatic-owner and earlier manual-candidate renderers and documented commands that regenerate the GIF/PNG evidence |
| local workspace generated files | Preserved unique root comparison artifacts under `docs`, removed byte-identical root duplicates, and cleared ignored `src/build`, `src/log`, pytest, and Python caches |
| root `TODOLIST.txt` | Advanced the review date while keeping all physical-motion acceptance explicitly pending |

Package and bringup configuration mirrors were rechecked after documentation
refresh. No Jetson-specific runtime value, measured body size, safety margin,
or physical FIELD-PASS claim was changed by this documentation packaging.

---

<!-- HH_260803 - Record Guest UI parity and automatic boundary recovery evidence. -->
## [guest-ui-automatic-boundary-recovery] - 2026-08-03 (HH_260803)

### Changed

| Doc | What changed |
|-----|--------------|
| `docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md` | Added Guest-to-Robot UI integration results, production-owned crab/reverse policy, measured simulation runs, visual artifacts, and map limitations |
| root `TODOLIST.txt` | Replaced the obsolete missing-owner note with the implemented bounded owner and retained real-robot acceptance separately |
| `camrod_control/README.md` | Documented candidate selection, direction latch, speed/distance/time bounds, and the no-rotation contact invariant |
| `camrod_ui/README.md` | Documented the shared backend contract, 35% Guest admission rule, and service-state/safety-overlay separation |
| `camrod_bringup/README.md`, `field_test_tool.sh` | Added simulation replay entrypoints and recovery candidate/controller topics to field rosbag evidence |

The Guest command/state contract and automatic recovery were validated in
simulation only. Narrow lanelets 754/2751/2720 still require map/corridor work,
and all physical motion acceptance remains open in `TODOLIST.txt`.

---

<!-- HH_260803 - Record the complete axle-midpoint frame migration. -->
## [robot-center-link-migration] - 2026-08-03 (HH_260803)

### Changed

| Doc | What changed |
|-----|--------------|
| `docs/V2_1_3_ROBOT_CENTER_MIGRATION.md` | Added the frame definition, exact sensor/body/footprint/parking before-and-after values, invariants, runtime consumers, and field acceptance |
| root `README.md`, `TODOLIST.txt` | Added the canonical center-frame summary and retained GNSS/TF/drive work as explicit physical acceptance |
| `camrod_sensor_kit/README.md` | Replaced the rear-axle frame tree and mount table with the axle-midpoint tree and converted coordinates |
| `camrod_localization/README.md`, `camrod_control/README.md` | Synchronized EKF TF ownership, GNSS caveat, footprint values, and parking distance preservation |
| `camrod_bringup/docs/field_test_runbook.md` | Updated pose and boundary evidence instructions to the center-frame contract |

No physical sensor mount, body dimension, planning margin, map, or safety
threshold changed. Only frame ownership and frame-dependent coordinates changed;
GNSS antenna lever-arm measurement remains pending on the real robot.

---

<!-- HH_260802 - Record the explicitly requested v2.1.2 tag refresh. -->
## [v2.1.2-rpp-boundary-evidence] - 2026-08-02 (HH_260802)

### Changed

| Doc | What changed |
|-----|--------------|
| `README.md`, `docs/V2_1_2_RELEASE_NOTES.md` | Promoted the refreshed v2.1.2 software/sim baseline and documented its tag history, scope, verification, and field limits |
| root `DONE.txt`, `TODOLIST.txt` | Recorded the rear JPEG worker, structured Ranger transition evidence, full-footprint regressions, RPP sweep, and remaining Jetson/real-wheel acceptance |
| `camrod_bringup/docs/pose_latency_diagnosis.md` | Distinguished the measured 15 Hz pose cadence from the unproven physical sine-wave cause and documented the 1.1 m simulation decision |
| `camrod_bringup/docs/field_test_runbook.md` | Replaced throttled rosout evidence with timestamped target/limited/actuator steering comparison |

The original `v2.1.2` tag pointed to `22f9b0cb`. At the operator's explicit
request, the tag is republished on the final release commit containing this
checkpoint. This remains a software/configuration/simulation baseline, not a
completed real-robot driving acceptance.

---

<!-- HH_260731 - Record physical stationary acceptance and keep drive work open. -->
## [post-v2.1.1-stationary-field-validation] - 2026-07-31 (HH_260731)

### Changed

| Doc | What changed |
|-----|--------------|
| `camrod_bringup/docs/v2_1_1_field_validation_20260731.md` | Added the physical no-motion radar, camera, GNSS/IMU, localization, voice, CPU, and two-boundary evidence with exact log paths |
| root `DONE.txt` | Moved radar-OFF 10-minute and front-camera/YOLO 5-minute acceptance to FIELD-PASS and recorded all partial/failing measurements separately |
| root `TODOLIST.txt` | Removed only passed TODO 1/2, retained original numbering, and narrowed the rear-camera, radar-ON, CPU, GNSS/IMU, voice, boundary, and localization work from measured evidence |
| `camrod_bringup/docs/field_test_runbook.md` | Added executable-probe preflight, 600-second radar-OFF evidence, physical-channel verification, repeated dual-GNSS flags, low-overhead rear-camera measurement, and production-only CPU rules |

No motion or engage was commanded. The physical body is
`1.49160 × 1.07000 m`; the active Nav2/safety footprint is the same asymmetric
body plus a 0.10 m margin on every side, or `1.69160 × 1.27000 m`.

---

<!-- HH_260731 - Record the indoor crab-yaw and projected lateral recovery audit. -->
## [post-v2.1.1-crab-yaw-audit] - 2026-07-31 (HH_260731)

### Changed

| Doc | What changed |
|-----|--------------|
| `camrod_platform/README.md` | Documented deterministic four-quadrant parallel motion, feedback-mode odometry, non-zero covariance, and per-wheel actuator evidence |
| `camrod_localization/README.md` | Assigned yaw rate to IMU rather than Ranger parallel odometry, documented the unmeasured GNSS lever arm, and corrected failure behavior |
| `camrod_control/README.md` | Distinguished projected reverse/crab candidate authorization from automatic recovery command generation |
| `camrod_bringup/docs/field_test_runbook.md` | Added header-matched GNSS-to-EKF/final XY/yaw and crab-yaw evidence, with covariance-valid yaw-pair filtering |
| root `DONE.txt`, `TODOLIST.txt` | Recorded completed indoor code/diagnosis and full-bringup rate evidence separately from remaining physical crab, lever-arm, and automatic-recovery acceptance |

No physical movement was performed for this checkpoint. Full-footprint cost,
dynamic-obstacle, engage, ESTOP, CAN, charging, and cancel protections remain
active.

---

<!-- HH_260730 - Separate resolved evidence from remaining field work. -->
## [post-v2.1.1-checklist-split] - 2026-07-30 (HH_260730)

### Changed

| Doc | What changed |
|-----|--------------|
| root `DONE.txt` | Added a status-qualified ledger for completed implementation, diagnosis, unit/sim verification, measured results, and their original TODO numbers |
| root `TODOLIST.txt` | Removed completed narratives and retained only unfinished physical execution, evidence, safety, and PASS criteria |
| `README.md`, `docs/V2_1_1_RELEASE_NOTES.md` | Linked the two-file contract and preserved the explicit absence of a post-change full real-robot FIELD-PASS |

An item moves out of `TODOLIST.txt` only when its recorded acceptance criteria
pass. Partial passes are recorded in `DONE.txt` while the smaller unresolved
condition remains in the live checklist.

---

<!-- HH_260730 - Record the v2.1.1 checkpoint and keep field PASS separate. -->
## [v2.1.1-field-readiness] - 2026-07-30 (HH_260730)

### Changed

| Doc | What changed |
|-----|--------------|
| `README.md` | Promoted v2.1.1 and summarized localization/controller, recovery, UI/voice, CPU, and unchanged-radar scope |
| `docs/V2_1_1_RELEASE_NOTES.md` | Added the exact release scope, configuration invariants, build/test/sim evidence, and remaining physical acceptance |
| root `TODOLIST.txt` | Added the ordered 2026-07-31 field session, corrected the real EKF rate to 15 Hz, and distinguished implemented manual projection from its pending runtime acceptance |
| `camrod_localization/README.md`, `camrod_bringup/docs/pose_latency_diagnosis.md` | Updated the production EKF/final-pose contract and screening rate from 10 Hz to 15 Hz while retaining the 1 Hz GNSS caveat |
| `camrod_bringup/docs/post_v2_1_0_indoor_validation.md` | Recorded that manual projection is implemented, startup/ready smoke evidence passed, and manual/UI field sequences remain pending |

Package and bringup configuration mirrors were byte-identical across 139
package-owned pairs at release preparation. Radar exclusions and stop geometry
were not changed for v2.1.1.

---

<!-- HH_260730 - Track the continued field-readiness work without rewriting v2.1.0. -->
## [post-v2.1.0-field-readiness] - 2026-07-30 (HH_260730)

### Changed

| Doc | What changed |
|-----|--------------|
| `camrod_bringup/docs/field_test_runbook.md` | Added independent front-JPEG payload decoding, five-minute RViz/WebKit/window-off CPU comparison, explicit gate verification, rosbag metadata names, and storage preflight |
| `camrod_bringup/README.md` | Documented the camera payload and concurrent runtime-profile acceptance commands |
| `camrod_bringup/docs/post_v2_1_0_indoor_validation.md` | Recorded fresh production-entry sim evidence for manual/UI goal policy, active v1.0.3 costmap/planner behavior, the implemented-but-field-pending far-manual endpoint correction, selected-pose latency, planning CPU, and UI/voice state sequencing |
| `camrod_bringup/docs/pose_latency_diagnosis.md` | Replaced the contaminated preliminary sample with the clean single-bringup before/after selector measurement and preserved the real dual-GNSS 1 Hz field caveat |
| `camrod_localization/README.md` | Corrected selected-pose rates to real 15 Hz/sim 20 Hz and documented freshest-header callback-order selection |
| `camrod_planning/README.md` | Documented the active stable-map LaneletRoute result, lethal clicked-cell diagnosis, retained manual yaw, implemented manual position projection with pending runtime acceptance, separate UI snap policy, and non-weakened footprint safety |
| `camrod_ui/README.md`, `camrod_voice/README.md` | Defined ERROR/not-ready versus degraded WARN/DUMMY startup readiness and the common manual/UI mission-phase and audio sequence |
| `camrod_map/README.md` | Documented the empty active profile, stable `lanelet2_maps.osm` runtime entrypoint, default semantic files, and current v1.0.3 audit snapshot |
| root `TODOLIST.txt` | Replaced stale moved-map planning evidence with the active v1.0.3 costmap result, retained the implemented manual-safe-snap acceptance run and all physical acceptance work, and recorded goal/path/cmd_vel, selected-pose delay, CPU coalescing, and UI/voice evidence |

This work preserves the user's corrected `robot_visualization.yaml`; no
visualization geometry or marker parameter is changed by this documentation
update.

---

<!-- HH_260729 - Track the post-tag route recovery and steering-lag remediation separately. -->
## [post-v2.1.0-route-recovery] - 2026-07-29 (HH_260729)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Identified the route recovery and Ranger steering-lag work as an unreleased delta after the immutable v2.1.0 baseline |
| `docs/V2_1_0_TODO_REMEDIATION.md` | Recorded baseline equality, root causes, requirement-to-code traceability, safety invariants, state transitions, parameters, changed ownership boundaries, and validation evidence |
| root `TODOLIST.txt` | Added plain-language explanations, the purpose of every field command/artifact, safety stop conditions, and code/config/test ownership for items 11-13; marked items 11-12 software-implemented but field-pending, and separated the item 13 driver mitigation from measurement-dependent controller tuning |
| `camrod_control/README.md` | Documented route hold, fail-closed clear proof, constrained opposite-direction escape, and rejected unsafe runtime ranges |
| `camrod_planning/README.md` | Documented retained-goal reissue prerequisites, delay, and retry bound |
| `camrod_platform/README.md` | Documented steering-error velocity scaling and runtime parameter ranges |
| `camrod_bringup/README.md` | Documented the three synchronized deployment mirrors, route recovery status topics, and dedicated TODO 11-13 evidence recorder |
| `camrod_bringup/docs/field_test_runbook.md` | Added repeatable lanelet exit, reverse escape, automatic replan, left/right steering-transition checks, rosbag capture, active-parameter evidence, and PASS/FAIL form |
| `camrod_bringup/scripts/field_test_tool.sh` | Added `record-recovery` to collect the complete real-robot TODO 11-13 timeline and evidence metadata |

The v2.1.0 tag itself was not rewritten. Real-robot acceptance remains open and
must be recorded against the eventual commit/tag in `TODOLIST.txt`. Native
policy/driver tests and an ordinary-simulation startup/status smoke test passed;
forced bringup shutdown still exposes pre-existing planning respawn exceptions.

---

<!-- HH_260729 - Track the v2.1.0 disabled-hardware, radar, and camera hardening release. -->
## [v2.1.0-docs] - 2026-07-29 (HH_260729)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.1.0 and summarized fail-visible disabled-hardware contracts, radar dummy cost isolation/evidence, camera ownership/crash containment, and retained footprint/latch/storage safety |
| `docs/V2_1_0_RELEASE_NOTES.md` | Recorded the actual source/config delta, reviewed pre-fix ROS-log evidence, synchronized configuration surfaces, automated coverage, and explicit lack of a post-fix real-robot acceptance run |
| `camrod_bringup/README.md` | Documented the scoped component-camera ownership handoff and common disabled-sensor dummy policy |
| `camrod_sensing/README.md` | Reorganized SEN0592 hardware/software parameters, named fixed-return intervals, disabled startup learning, dummy behavior, obstacle evidence, and disabled-sensor schemas |
| `camrod_platform/README.md` | Documented the mutually exclusive, ESTOP/non-drivable Ranger dummy contract |
| `camrod_localization/README.md` | Documented rejection of disabled-GNSS `NO_FIX` placeholders before geographic conversion and EKF input |
| `camrod_system/README.md` | Documented fresh-marker DUMMY/WARN semantics, physical-failure fallback, and global/per-channel radar dummy identity |
| `camrod_bringup/docs/field_test_runbook.md` | Added radar-off cost-barrier, front/rear camera/YOLO lifetime/rate, and raw lanelet-footprint evidence checks |
| root `TODOLIST.txt` | Moved all unperformed physical radar, camera, lanelet, off-road/reverse recovery, lateral-control latency, driving, CPU, planning, voice, OpenCV ABI, and port acceptance work into a prioritized field checklist |

No post-fix real-robot bringup, engage, or driving test was performed while
preparing these documents. The v2.1.0 tag records the source/configuration/test
baseline; physical acceptance remains explicitly open in `TODOLIST.txt`.

---

<!-- HH_260728 - Track the final v2.0.9 obstacle latch and operations follow-up. -->
## [v2.0.9-final-field-safety] - 2026-07-28 (HH_260728)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Recorded command-independent obstacle latch release, the final 0.60 m forward-side probe, and the 90/95% disk diagnostic policy |
| `camrod_bringup/README.md` | Synchronized the 0.60/1.20 m straight/maneuver mirror contract and radar-inflated side expectations |
| `camrod_control/README.md` | Documented retained trigger geometry, fresh-grid clear evidence, and base-centred 0.60 m forward-side semantics |
| `camrod_bringup/docs/field_test_runbook.md` | Added zero/direction-change latch checks and 1.0/0.8 m radar-inflated side regressions |
| `camrod_system/README.md` | Documented the 90% WARN and 95% ERROR storage thresholds |
| `camrod_platform/README.md` | Documented state-change-only WS2815 refresh to preserve MCU serial RX |
| `camrod_voice/README.md` | Added the opt-in, validated Bluetooth amplifier service helper |

---

<!-- HH_260728 - Track the bounded radar self-return and forward-side-guard release. -->
## [v2.0.9-docs] - 2026-07-28 (HH_260728)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.9 while retaining the complete v2.0.8 battery, UI, planning, and full-footprint baseline |
| `camrod_sensing/README.md` | Documented generated/ROS radar topics, narrow fixed notches, bounded authorization-aware startup learning, and farther-return preservation |
| `camrod_system/README.md` | Documented per-sensor component/location/frame/mount metadata, STALE preservation, simultaneous fault lines, and the global throttled detail cap |
| `camrod_sensor_kit/README.md` | Synchronized the documented IMU mount pose with the canonical `robot_params.yaml` value |
| `camrod_control/README.md` | Documented the initial 0.75 m normal-forward profile, superseded in the final tag by the 0.60 m raw probe, and the retained 1.20 m crab/reverse envelope |
| `camrod_bringup/README.md` | Added radar and command-gate deployment mirror contracts |
| `camrod_bringup/docs/field_test_runbook.md` | Added startup calibration, engage-cancel, close-obstacle, and direction-dependent stop checks |
| `docs/V2_0_9_RELEASE_NOTES.md` | Recorded implementation scope, live radar measurements, validation, unchanged port routing, and field-test limits |

---

<!-- HH_260724 - Track manual engage and operator-stop visibility after v2.0.7 tagging. -->
## [manual-engage-operator-stop-docs] - 2026-07-24 (HH_260724)

### Changed

| Doc | What changed |
|-----|--------------|
| `camrod_ui/README.md` | Documented Manual driving display and `/service/state=OPERATOR_STOPPED` cancel/stop behavior |
| `camrod_system/README.md` | Documented suppression of expected Nav2 ABORTED diagnostics during campsite maneuver ownership |
| `camrod_bringup/README.md` | Documented live terminal watch visibility for manual/mission engage and platform drive-enable |
| root `README.md` | Added the operator-visible manual driving and stop/cancel state summary |
| `docs/V2_0_7_RELEASE_NOTES.md` | Added operator stop, manual driving, Nav2 ABORTED suppression, and follow-up verification evidence |

---

<!-- HH_260724 - Track the low-battery campsite mission policy and v2.0.7 release docs. -->
## [v2.0.7-battery-policy-docs] - 2026-07-24 (HH_260724)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.7 and added the sim validation flag for finish-current-mission low-battery return |
| `camrod_bringup/README.md` | Documented the low-battery user-return simulation and 34% blocked recall probe |
| `camrod_control/README.md` | Documented the 20% hard stop, 35% charger departure gate, and parked-versus-charging state semantics |
| `camrod_ui/README.md` | Documented the 35% campsite dispatch gate and user-confirmed low-battery return latch parameters |
| `docs/V2_0_7_RELEASE_NOTES.md` | Recorded the battery policy scope, state semantics, synchronized config values, and validation evidence |

---

<!-- HH_260723 - Track the v2.0.7 localization, routing, perception, and occupancy release. -->
## [v2.0.7-docs] - 2026-07-23 (HH_260723)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.7 and summarized the explicit reverse-route, camera/YOLO, campsite-occupancy, and perception-only cost profile |
| package READMEs | Synchronized localization diagnostics, semantic detections, occupied-site blocking, and raw-LiDAR cost-switch contracts |
| `docs/V2_0_7_RELEASE_NOTES.md` | Recorded root causes, runtime interfaces, verification evidence, and remaining field limits |

---

<!-- HH_260723 - Track the v2.0.6 localization, routing, perception, and occupancy field release. -->
## [v2.0.6-field-docs] - 2026-07-23 (HH_260723)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.6 and summarized the explicit reverse-route, camera/YOLO, campsite-occupancy, and perception-only cost profile |
| package READMEs | Synchronized localization diagnostics, semantic detections, occupied-site blocking, and raw-LiDAR cost-switch contracts |
| `docs/V2_0_6_RELEASE_NOTES.md` | Recorded root causes, runtime interfaces, verification evidence, and remaining field limits |

---

<!-- HH_260722 - Track the hardware-verified dual-GNSS v2.0.6 production default. -->
## [v2.0.6-dual-gnss-docs] - 2026-07-22 (HH_260722)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.6 and added the two logical GNSS port roles used by default hardware bringup |
| `camrod_sensing/README.md` | Replaced the direct-rover diagram and stale mountpoint/topic guidance with the corrected moving-base cascade, A/B evidence, and no-argument default launch |
| `camrod_bringup/README.md` | Documented the five synchronized GNSS defaults, config-mirror contract, and live acceptance commands |
| `camrod_bringup/docs/field_test_runbook.md` | Added dual-port preflight, snapshot evidence, RTK/heading acceptance flags, and one-shot recovery guidance |
| `docs/V2_0_6_DUAL_GNSS_RELEASE_NOTES.md` | Recorded implementation scope, hardware measurements, verification evidence, and the two-logical-port limit |

---

<!-- HH_260722 - Track the hardware-verified dual-GNSS v2.0.6 production default. -->
## [v2.0.6-docs] - 2026-07-22 (HH_260722)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.6 and added the two logical GNSS port roles used by default hardware bringup |
| `camrod_sensing/README.md` | Replaced the direct-rover diagram and stale mountpoint/topic guidance with the corrected moving-base cascade, A/B evidence, and no-argument default launch |
| `camrod_bringup/README.md` | Documented the five synchronized GNSS defaults, config-mirror contract, and live acceptance commands |
| `camrod_bringup/docs/field_test_runbook.md` | Added dual-port preflight, snapshot evidence, RTK/heading acceptance flags, and one-shot recovery guidance |
| `docs/V2_0_6_RELEASE_NOTES.md` | Recorded implementation scope, hardware measurements, verification evidence, and the two-logical-port limit |

---

<!-- HH_260721 - Track active coordinate export and constrained roadside campsite behavior. -->
## [v2.0.5-roadside-docs] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Split normal campsite turnaround from B12/B13 roadside return behavior |
| package READMEs | Documented operational service poses across map, planning, control, UI, and bringup |
| `docs/V2_0_5_RELEASE_NOTES.md` | Added active coordinate values, full simulation evidence, and physical roadside validation limit |
| `camrod_sensing/README.md` | Documented the single ROS 2 ground-segmentation package and stale-tree build guard |

---

<!-- HH_260721 - Track the campsite retrace and parked departure validation release. -->
## [v2.0.5-docs] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.5 as the validated baseline and clarified same-lane campsite retrace |
| package READMEs | Updated control phase names, planning handoff guards, simulation charging feedback, UI departure ordering, and diagnostics behavior |
| `docs/V2_0_5_RELEASE_NOTES.md` | Recorded the complete reverse-parking simulation, build/test evidence, synchronized configuration, and hardware-only limits |

---

<!-- HH_260721 - Record the corrected non-hardware release-validation invocation. -->
## [2.0.4-validation] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Paired the simulated BMS publisher with `sim_platform_status_enable:=true` in bringup |
| `camrod_bringup/README.md` | Added the exact reverse-parking charging-recall launch and final report paths |
| `docs/V2_0_4_RELEASE_NOTES.md` | Updated final charging, obstacle, perception, and forced-cppcheck evidence |
| `camrod_platform/README.md` | Distinguished ordinary-PC simulation from real Ranger CAN startup |

Tracks changes to CAMROD documentation (READMEs, style guides, templates).
For code changes see git log and release tags.

---

## [v2.0.4-docs] - 2026-07-21 (HH_260721)

<!-- HH_260721 - Track the native control, EKF-only, and charging-recall release evidence. -->

### Changed - native control and full reverse-parking validation

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.4 as the validated baseline and updated charging/build commands |
| `camrod_control/README.md` | Replaced the stale Python tree with native C++ nodes and policy helpers |
| `camrod_bringup/README.md` | Added charging recall, directional gate, replan, and config-mirror commands |
| `camrod_localization/README.md` | Documented robot_localization EKF as the only runtime backend |
| `docs/V2_0_4_RELEASE_NOTES.md` | Recorded implementation scope, exact simulation evidence, package tests, and known limits |

---

## [v2.0.3-docs] - 2026-07-21 (HH_260721)

<!-- HH_260721 - Track the control consolidation and reverse-only release evidence. -->

### Changed - control ownership and reverse-parking validation baseline

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.3 as the current validated baseline and linked its release notes |
| `camrod_control/README.md` | Documented gate, maneuver, parking, charging-departure, and command-topic ownership |
| `camrod_bringup/README.md` | Documented the four byte-identical control configuration mirrors and reverse-only validation command |
| package READMEs | Replaced legacy docking, parking, platform-gate, and message-alias descriptions with current interfaces |
| `docs/V2_0_3_RELEASE_NOTES.md` | Added migration inventory, build/package evidence, complete reverse-only simulation results, and known limits |

---

## [v2.0.2-docs] — 2026-07-16 (HH_260716)

### Changed — field map, sensing, localization, and validation baseline

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.2 current release; summarized moved Park map, radar thresholds, JECH NTRIP, camera/YOLO gating, GNSS health gates, and full config/install sync |
| `camrod_bringup/README.md` | Updated active map path/profile and added the reusable camera/YOLO probe |
| `camrod_bringup/docs/field_test_runbook.md` | Added all-package install checks, radar minimum-range validation, GNSS covariance triage, YOLO subscriber-gating notes, and duplicate debug-process load guidance |
| `camrod_map/README.md` | Documented `copy_park_moved`, map version 13, and synchronized projection metadata |
| `camrod_sensing/README.md` | Updated JECH mountpoint, single/dual GNSS rates, 15-degree radar profile, and measured self-echo dead zones |
| `camrod_localization/README.md` | Updated monitor thresholds and clarified why live GNSS may still result in `DR_ONLY` |
| `camrod_perception/README.md` | Documented the component-container path, continuous detection health topic, and subscriber-gated YOLO image behavior |
| `docs/V2_0_2_RELEASE_NOTES.md` | Added operator-facing change inventory, validation checklist, and known runtime observations |

## [lights-docs] — 2026-07-08 (260708)

### Added — exterior light feature docs

| Doc | What changed |
|-----|--------------|
| `camrod_platform/docs/lights-design-doc.html` | New: implementation design for headlight relay + WS2815 turn indicators (decisions, architecture, serial protocol, failsafe, TODO) |
| `camrod_platform/docs/turn-signal-explainer.html` | New: visual comparison of curvature vs lanelet `turn_direction` direction sources |
| `camrod_platform/README.md` | Added light_controller/mcu_serial_bridge summary item, `lights.launch.py` row, lights topics in interface contract, launch args, `config/lights.yaml` row |
| root `README.md` | Added `/planning/route_turn_segments`, `/platform/headlight/command`, `/platform/lights/*` to the key-topics table |

---

## [field-test-docs] — 2026-07-08 (HH_260708)

### Added — outdoor test memory and evidence capture

| Doc | What changed |
|-----|--------------|
| `camrod_bringup/docs/field_test_runbook.md` | New: outdoor workflow for config sync, bringup logging, snapshots, topic Hz probes, software gates, radar/LiDAR/perception-cost checks, camping-site flow, and drop-zone return |
| `camrod_bringup/README.md` | Added `field_test_tool.sh` quick-start commands and field runbook reference |
| root `README.md` | Added field helper and runbook rows to runtime/operator reference |

---

## [1.11-docs] — 2026-05-28 (HH_260528)

### Changed — package READMEs

| Package | What changed |
|---------|-------------|
| `camrod_sensing` | Dual econ camera architecture (front: GPU/VPI, rear: CPU/GStreamer); `imu_mode` → `imu_model` rename; updated topic list and launch args |
| `camrod_docking` | Full Korean → English translation; Camera TF ownership moved to `camrod_sensor_kit`; new frame names `camera_front_link`/`camera_rear_link` |
| `camrod_sensor_kit` | TF diagram updated with `camera_front_link` and `camera_rear_link` frames |
| `camrod_platform` | Added `platform_type`, `ranger_bridge_enable`, `sensor_kit_bridge_enable` launch args |
| `camrod_bringup` | Updated sensing config filename; added per-camera enable flags; platform args |

### Changed — docs/

- `docs/archive/TODO_BRINGUP_SUMMARY.md` — updated topic paths and node names to match HH_260528 state
- `docs/archive/TODO_MODULE_RUNTIME_FLOW.md` — sensing section updated for dual camera and unified IMU

### Added — code (reflected in docs)

- `camrod_sensing/config/camera/camera_rear_calibration.yaml` — rear camera intrinsics (plumb_bob, 1920×1080)
- `camrod_bringup/config/sensing/camera/camera_params.yaml` — bringup-level deployment override for both cameras

### Deleted — code (reflected in docs)

- `camrod_sensing/launch/imu_cv7.launch.py` — replaced by unified `imu.launch.py`
- `camrod_sensing/launch/imu_gq7_ntrip.launch.py` — replaced by unified `imu.launch.py`
- `camrod_sensing/src/camera_publisher_node.cpp` — renamed to `camera_front_publisher_node.cpp`
- All `*copy_org*`, `*copy_750*`, `*copy_722*` backup files across sensing and bringup

---

## [1.10-docs] — 2026-05-21

### Added
- `docs/templates/README_STYLE_GUIDE.md` — canonical 12-section README structure, Mermaid legend, Interface Contract column rules, Key Behavior block format
- `docs/templates/PACKAGE_README_TEMPLATE.md` — copy-paste skeleton for new package READMEs
- `docs/DOCS_CHANGELOG.md` — this file
- `camrod_parking/README.md` → renamed to `camrod_docking/README.md` with architecture update
- Root `README.md` — Documentation Map table, Hardware & Software Requirements, Docker section, First Run Guide, Glossary, versioning table
- All package READMEs — stateDiagram-v2 and sequenceDiagram blocks for key behaviors

### Changed
- All package READMEs now follow the fixed 12-section structure from `README_STYLE_GUIDE.md`
- Topic tables expanded with Required / Rate / Meaning columns (Interface Contract format)
- Architecture diagrams split into Context (graph LR) + Runtime (graph TD) where applicable
- `PARAMETER_NAMING_STANDARD.md` — added §6 Quick Reference Cheatsheet, §7 Writing New Configs, §8 Per-Package Canonical Keys Index, §9 Related Docs

---

## [1.9-docs] — 2026-05-13

### Added
- Initial `PARAMETER_NAMING_STANDARD.md` with canonical naming rules and migration policy

### Changed
- `camrod_system` diagnostic checker params migrated to canonical `*_hz` / `*_s` suffixes

---

## [1.0-docs] — 2026-04-28

### Added
- Initial `README.md` at workspace root with architecture diagram and build/run instructions
- Package-level `README.md` files for all initial packages
