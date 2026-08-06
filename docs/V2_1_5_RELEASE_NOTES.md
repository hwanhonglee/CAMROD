# CAMROD v2.1.5 Release Notes

<!-- HH_260805 - Release the scoped runtime remediation and align the
unverified GNSS position contract with robot_center_link. -->
<!-- HH_260806 - Add the provisional boundary reduction and enforce an
unrecoverable physical-body contact independently of the planning margin. -->

Release date: 2026-08-06 (Asia/Seoul)

Previous baseline: `v2.1.4` / `f83975d5e`

## Release Summary

v2.1.5 closes the post-v2.1.4 runtime work as one reviewable baseline. It
eliminates the reproduced AMD64 checker/Nav2 component shutdown faults,
prevents full-graph DDS shared-memory exhaustion by scoping SHM to physical
LiDAR only, and makes the existing GNSS estimator assumption explicit at
`robot_center_link`. It also records a provisional 0.10 m reduction on every
horizontal body side and separates physical-body contact from recoverable
planning-margin contact in the final command gate.

This release does not claim a newly surveyed antenna, sensor layout, or body
envelope. GNSS remains unverified, sensor mount coordinates are unchanged, and
the reduced body dimensions are a simulation candidate only. Jetson/physical
mission acceptance remains in `TODOLIST.txt`.

## Active Contract

| Item | v2.1.5 value | Meaning |
|---|---:|---|
| Navigation base | `robot_center_link` | Axle midpoint for localization, planning, control, and platform |
| GNSS TF/position assumption | `(0, 0, 0)` from robot center | Matches the estimator's direct NavSatFix-to-center use; not a survey |
| GNSS verification | `pose_verified=false` | Requires both dual-GNSS mounts and receiver reference measurement |
| Provisional physical body | `1.29160 x 0.87000 m` | Previous front/rear/left/right extents minus `0.10 m`; field survey required |
| Planning boundary | `1.39160 x 0.97000 m` | Physical body plus `0.05 m` on every side |
| Physical-body map contact | `lanelet_physical_body_cost` | Hard stop; every automatic recovery candidate is rejected |
| Planning-margin contact | `lanelet_footprint_cost` | Ordinary output zero; projected bounded escape may be admitted |
| Checker topology | `24` checkers in `4` serialized containers | Standalone executables remain field fallbacks |
| System core | `4` nodes in one serialized container | Aggregate/status fault domain |
| Nav2 topology | planner/controller container | Smoother, behavior, BT, and lifecycle servers remain standalone |
| LiDAR cost grid | default `OFF` | Node/topic/checker requirement activate together |
| DDS shared memory | default `OFF`, physical LiDAR only | Never exported to simulation or the full ROS graph |
| Operator renderer | Chromium | WebKit remains the measured lighter fallback |
| Active map | version `15`, SHA `d7b730...213f` | User-authored active/copy OSM pair remains byte-identical |

## Provisional Boundary And Runtime Policy

The prior body was `1.49160 x 1.07000 m`; the prior planning rectangle was
`1.59160 x 1.17000 m`. v2.1.5 preserves those values and their bag hashes in
the test record, while deploying the user's provisional candidate below:

| Side | Body extent | Planning extent |
|---|---:|---:|
| Front | `0.65837 m` | `0.70837 m` |
| Rear | `0.63323 m` | `0.68323 m` |
| Left | `0.43505 m` | `0.48505 m` |
| Right | `0.43495 m` | `0.48495 m` |

An initial physical-contact simulation exposed that the gate sampled only the
larger planning polygon: it issued `0.05 m/s` recovery and moved `0.0652 m`
while the body was already on map cost 100. The corrected gate samples the
body first. Only margin-only contact can reach the projected crab/reverse/yaw
selector; body contact has no automatic escape path.

![Fresh boundary-policy simulation](assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)

| Fresh map-v15 scenario | Measured result |
|---|---|
| Normal route | PASS: `10.0403 m`, goal error `0.2932 m`, no route hold |
| Margin ordinary command | PASS: body max `70`, planning max `100`, output `0.0 m/s` |
| Margin recovery | PASS: `CRAB_RIGHT`, `0.133 m`, max `0.05 m/s` |
| Physical-body contact | PASS: no candidate, no owner motion, output `0.0 m/s` |

These are AMD64 simulation policy results, not evidence that the reduced
rectangle contains the real wheels, sensors, brackets, cables, or payload.

## GNSS Center Alignment

Before v2.1.5, `gnss_link` used `x=-0.443 m`. That value was only the old
rear-axle `0.0 m` placeholder converted to the axle-midpoint coordinate system;
it was not measured on the robot.

The localization input adapter converts NavSatFix directly into a map pose and
publishes the selected base as `robot_center_link`. It does not apply an
antenna lever-arm transform. Keeping `gnss_link=-0.443 m` therefore made TF and
diagnostic metadata disagree with the estimator's center-position assumption.

v2.1.5 changes only the GNSS assumption:

| Source | Before | v2.1.5 |
|---|---:|---:|
| Package `robot_params.yaml` | `(-0.443, 0, 0)` | `(0, 0, 0)` |
| Bringup mirror | `(-0.443, 0, 0)` | `(0, 0, 0)` |
| Xacro missing-config fallback | `(-0.443, 0, 0)` | `(0, 0, 0)` |
| Default/sim diagnostic metadata | `-0.44300,0,0` | `0.00000,0,0` |
| Diagnostic location | `unverified` | `robot_center_assumed` |
| Verification flag | `false` | `false` |

![Current sensor mount side view](assets/module-guides/sensor-kit/sensor-mount-side-view.png)

![GNSS center assumption and sensor X ledger](assets/module-guides/sensor-kit/sensor-x-before-after.png)

The GNSS alignment itself leaves IMU, LiDAR, camera, radar, axle, controller,
map, and parking coordinates unchanged. The body and planning rectangles are
changed separately by the provisional boundary work above. If field
measurement shows the GNSS position solution is not the robot center, the
correction must be implemented in localization together with TF; changing
only the visualization frame is not accepted.

## Runtime And Shutdown

- Added `camrod_runtime` single- and multithreaded scoped component containers.
  Manager, components, and executor share one explicit `rclcpp::Context`.
- Shutdown joins the deferred signal handler, detaches component callback
  groups, destroys node instances before plugin unload, and releases explicit
  executor Context ownership in a deterministic order.
- ROS 2 Humble can retain Context owners into DSO static destruction. After all
  owned resources are explicitly finalized, the container bypasses only that
  unsafe process-exit destructor phase to prevent CycloneDDS `tev`/`recv`
  workers from executing unmapped RMW code.
- Twenty-four checkers run in four process fault domains: hardware/sensing,
  localization, autonomy/topics, and planning/lifecycle. The system-core
  aggregate/status container remains separate.
- Nav2 composes planner and controller only. Vendor servers that own private
  default-context executors remain standalone, preserving lifecycle behavior.
- Robot and Guest UI treat both `KeyboardInterrupt` and
  `ExternalShutdownException` as normal launch shutdown.

Native tracing and the four final simulation runs are recorded in
[`amd64-scoped-container-shutdown-20260805.json`](evidence/v2.1.5/runtime-topology/amd64-scoped-container-shutdown-20260805.json).

## Transport And Sensor Containers

- Removed global CycloneDDS/iceoryx environment injection. Even when requested,
  DDS-SHM resolves active only for `sim:=false` with the physical LiDAR driver.
- LiDAR preprocessing and ground segmentation retain intra-process transport;
  the optional transient-local cost grid keeps DDS transport and defaults OFF.
- Rear physical capture, image rectification, and AprilTag detection can share
  one conditional component container. Standalone ownership remains available.
- Front camera/YOLO and the other existing component paths keep their established
  topic ownership; no duplicate publishers are introduced.

The measured AMD64 process/CPU/PSS comparisons remain in
[`amd64-container-ab-20260805.json`](evidence/v2.1.5/runtime-topology/amd64-container-ab-20260805.json).

## Measured AMD64 A/B

<!-- HH_260805 - Keep workstation measurements separate from Jetson acceptance
and report one logical CPU as 100 percent. -->
![Measured AMD64 runtime topology A/B](assets/module-guides/system/runtime-topology-amd64-ab-20260805.png)

The full-simulation samples used an Intel i5-12400F, `sim:=true`, and disabled
RViz, UI window, DDS-SHM, and LiDAR cost-grid. Values are process-tree means;
CPU `100%` means one logical CPU, not the whole workstation.

| System core, 3 runs | Standalone | Container | Change |
|---|---:|---:|---:|
| Processes | `69.0` | `66.1` | `-3.0` (`-4.3%`) |
| CPU | `87.8%` | `85.2%` | `-2.5 points` |
| RSS | `2538.9 MiB` | `2496.2 MiB` | `-42.7 MiB` |
| PSS | `1632.6 MiB` | `1612.9 MiB` | `-19.7 MiB` |
| Startup to `[SYSTEM] OK` | `26.51 s` | `26.91 s` | `+0.40 s` |
| Controlled stop | `3/3` | `3/3` | equal |

| Isolated LiDAR, 2 runs at 60k points/10 Hz | Standalone | Container | Change |
|---|---:|---:|---:|
| Processes | `4` | `3` | `-1` |
| CPU | `8.36%` | `6.90%` | `-17.5%` |
| RSS | `124.84 MiB` | `164.11 MiB` | `+39.27 MiB` |
| PSS | `84.75 MiB` | `128.76 MiB` | `+44.01 MiB` |
| Output | `10.005 Hz`, `324/324` | `10.002 Hz`, `324/324` | zero loss |

| Operator window, 3 runs | WebKit | Chromium | Chromium - WebKit |
|---|---:|---:|---:|
| Processes | `69.1` | `80.4` | `+11.3` |
| CPU | `89.3%` | `90.8%` | `+1.44 points` |
| RSS | `3005.2 MiB` | `3920.1 MiB` | `+915.0 MiB` |
| PSS | `1926.2 MiB` | `2253.5 MiB` | `+327.3 MiB` |
| Startup to `[SYSTEM] OK` | `26.77 s` | `25.44 s` | `-1.33 s` |

WebKit was materially lighter on this AMD64 host. The startup difference is
system readiness, not browser first-paint or frame pacing, so Chromium remains
the requested default and Jetson measurement remains required. The resource
A/B predates the scoped shutdown correction; the later four-run lifecycle
record supersedes only its checker/Nav2 shutdown verdict, not its resource
samples.

## Validation

- Three explicit-topology full-simulation runs and one default-argument run
  reached managed Nav2 active and `[SYSTEM] OK`.
- Each run cleanly stopped six component containers. There were zero `-11`,
  forced-kill, UI event-loop, iceoryx capacity, or residual-process failures.
- GNSS geometry, package/bringup mirrors, diagnostic metadata, and regenerated
  sensor-kit assets are protected by source contracts.
- The active and named-copy OSM files remain byte-identical at SHA
  `d7b7307eb66175f8963aa638af6b48cf6007169db6f35a89ac21a8c79bab213f`.
- Eleven selected packages built successfully. Their fresh xUnit reports total
  `487` tests, zero errors/failures, and `17` existing lint skips. Direct UI
  pytest passed `28/28`; focused source contracts passed `126/126`; all `388`
  package/bringup/install config comparisons matched.
- A final release-candidate run on ROS domain 207 reached managed Nav2 active
  and `[SYSTEM] OK`, returned top-level exit `0`, and stopped all six containers
  cleanly. Startup-only map/costmap WARN/ERROR states resolved to OK; they did
  not remain latched.
- A fresh full-bringup map-v15 run after the body-guard correction reached
  `[SYSTEM] OK`; normal route, margin stop, margin `CRAB_RIGHT`, and physical
  hard-stop scenarios all passed with final zero output.
- The repository-standard `./src/colcon_build.sh` rebuilt control, bringup,
  planning, sensor-kit, and platform. Fresh tests passed all `32/32` CTest
  targets; aggregate xUnit records were `334`, with zero errors/failures and
  eight planning cppcheck skips caused by the known disabled 2.7 checker.
- Exact machine-readable build and run results are recorded in
  [`amd64-scoped-container-shutdown-20260805.json`](evidence/v2.1.5/runtime-topology/amd64-scoped-container-shutdown-20260805.json)
  and summarized in `DONE.txt`.

## Field Work Still Required

1. Measure rover and moving-base antenna XYZ from `robot_center_link`, baseline
   length/direction, and the receiver's reported position reference.
2. If needed, implement and validate heading-aware antenna-to-center lever-arm
   correction before setting `pose_verified=true`.
3. Run Jetson production topology versus standalone resource A/B and ten clean
   mission/cancel/restart cycles.
4. Measure the complete body/wheel/sensor/bracket/cable/payload envelope before
   accepting the provisional boundary, then repeat the margin and body-contact
   tests on the physical robot.
5. Complete rear-camera rate, seven-channel radar, GNSS reacquisition, IMU
   startup, wheel response, parking/docking, charging, and full-route field tests.

Exact procedures and pass/fail thresholds remain in [`TODOLIST.txt`](../TODOLIST.txt).
