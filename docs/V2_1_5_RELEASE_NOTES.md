# CAMROD v2.1.5 Release Notes

<!-- HH_260805 - Release the scoped runtime remediation and align the
unverified GNSS position contract with robot_center_link. -->
<!-- HH_260806 - Add the provisional boundary reduction and enforce an
unrecoverable physical-body contact independently of the planning margin. -->
<!-- HH_260806 - Supersede the temporary GNSS center assumption with the
measured left-antenna lever arm and heading-aware center correction. -->
<!-- HH_260806 - Finalize map-v16 campsite sequencing and replace the reduced
body candidate with the fabrication-inclusive measured envelope. -->

Release date: 2026-08-06 (Asia/Seoul)

Previous baseline: `v2.1.4` / `f83975d5e`

## Release Summary

v2.1.5 closes the post-v2.1.4 runtime work as one reviewable baseline. It
eliminates the reproduced AMD64 checker/Nav2 component shutdown faults,
prevents full-graph DDS shared-memory exhaustion by scoping SHM to physical
LiDAR only, and converts the measured left-GNSS antenna solution to
`robot_center_link`. It separates physical-body contact from recoverable
planning-margin contact, restores the fabrication-inclusive measured body as
the active hard-stop envelope, and serializes campsite/Nav2 command ownership.

Only the GNSS lateral offset is measured. Rover X/Z, moving-base XYZ, baseline
direction, receiver position reference, moving residuals, sensor housings, and
four-side swept clearance remain unverified. Jetson/physical mission acceptance
remains in `TODOLIST.txt`.

## Active Contract

| Item | v2.1.5 value | Meaning |
|---|---:|---|
| Navigation base | `robot_center_link` | Axle midpoint for localization, planning, control, and platform |
| GNSS physical TF | left antenna `(0,+0.45,0) m` | ROS body `+Y` is left; X/Z retain current values |
| GNSS estimator output | heading-corrected robot center | `p_center = p_fix - R(yaw)[0,0.45]` |
| GNSS verification | `pose_verified=false` | Remaining dual-GNSS geometry and moving residuals require field acceptance |
| Physical body | `1.39160 x 1.07000 m` | Fabrication-inclusive measured total used by the hard-stop layer |
| Planning boundary | `1.49160 x 1.17000 m` | Physical body plus `0.05 m` on every side |
| Physical-body map contact | `lanelet_physical_body_cost` | Hard stop; every automatic recovery candidate is rejected |
| Planning-margin contact | `lanelet_footprint_cost` | Ordinary output zero; projected bounded escape may be admitted |
| Checker topology | `24` checkers in `4` serialized containers | Standalone executables remain field fallbacks |
| System core | `4` nodes in one serialized container | Aggregate/status fault domain |
| Nav2 topology | planner/controller container | Smoother, behavior, BT, and lifecycle servers remain standalone |
| LiDAR cost grid | default `OFF` | Node/topic/checker requirement activate together |
| DDS shared memory | default `OFF`, physical LiDAR only | Never exported to simulation or the full ROS graph |
| Operator renderer | Chromium | WebKit remains the measured lighter fallback |
| Campsite policy | B1-B10 `turnaround`; B11-B13 `roadside_stop` | Constrained sites stop without an on-site zero-turn |
| Command handoff | `0.5 s` zero hold | Campsite phases own final cmd_vel; Nav2 translation cannot overlap a maneuver/rotation handoff |
| Active map | version `16`, SHA `fd9c18...d0cf` | User-edited active/copy OSM pair remains byte-identical |

## Body Boundary History And Runtime Policy

The prior body was `1.49160 x 1.07000 m`; the prior planning rectangle was
`1.59160 x 1.17000 m`. v2.1.5 preserves those values and their bag hashes in
the test record. The initially tagged reduced candidate was:

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
rectangle contains the real wheels, sensors, brackets, cables, or payload. The
candidate is now historical. The active configuration uses:

| Side | Physical body | Planning boundary |
|---|---:|---:|
| Front | `0.70837 m` | `0.75837 m` |
| Rear | `0.68323 m` | `0.73323 m` |
| Left | `0.53505 m` | `0.58505 m` |
| Right | `0.53495 m` | `0.58495 m` |

The active totals are `1.39160 x 1.07000 m` body and
`1.49160 x 1.17000 m` planning. The body/outer-margin decision policy is
unchanged; current four-side and swept-clearance field acceptance remains open.

## Campsite Sequencing

![Map-v16 campsite validation](assets/test_result/camping-site-sequencing-20260806/campsite-policy-validation.png)

![Turnaround and roadside phase order](assets/test_result/camping-site-sequencing-20260806/campsite-phase-sequence.gif)

- B1-B10 are explicitly `turnaround`. All ten full site-maneuver runs observed
  `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN ->
  ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE`; map-derived lateral entry distances
  ranged from `1.79 m` to `5.31 m`.
- B11-B13 are explicitly `roadside_stop`. Their lateral move is capped at
  `0.60 m`; all three arrival-only runs observed
  `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN` with no zero-turn and no RETURN.
- While a campsite/drop-zone phase owns motion, Nav2 commands cannot overwrite
  its crab or rotation. A `0.5 s` stationary handoff precedes Nav2 resumption;
  pure Nav2 rotation also settles for `0.5 s` before translation.
- Static road-lanelet cost is bypassed only in explicit campsite motion phases.
  Dynamic LiDAR/radar stops remain active.
- The prior B11 on-lane return alignment encountered
  `lanelet_physical_body_cost` and remained stopped. Safe B11-B13 return
  geometry is therefore field-pending, not released as a completed scenario.

Structured reports and reproduction commands are in
[`camping-site-sequencing-20260806`](assets/test_result/camping-site-sequencing-20260806/README.md).

## GNSS Left-Antenna Correction

Before v2.1.5, `gnss_link` used `x=-0.443 m`. That value was only the old
rear-axle `0.0 m` placeholder converted to the axle-midpoint coordinate system;
it was not measured on the robot.

The initially tagged v2.1.5 contract temporarily placed GNSS at `(0,0,0)` to
remove that mismatch. The physical clarification now identifies NavSatFix as
the left antenna, `0.45 m` from center. TF alone is therefore insufficient:
the position itself must be corrected with vehicle heading.

| Source | Converted placeholder | Initial center assumption | Current correction |
|---|---:|---:|---:|
| Package/bringup TF | `(-0.443,0,0)` | `(0,0,0)` | `(0,+0.45,0)` |
| Xacro fallback | `(-0.443,0,0)` | `(0,0,0)` | `(0,+0.45,0)` |
| Default/sim diagnostic mount | `-0.44300,0,0` | `0,0,0` | `0,+0.45000,0` |
| Localization position | direct NavSatFix | direct NavSatFix | subtract heading-rotated lever arm |
| Verification flag | `false` | `false` | `false` |

The adapter requires a fresh usable dual-GNSS heading before applying the
correction. Jump rejection then operates on the corrected center position.
Simulation publishes a left-antenna NavSatFix and matching raw heading, so it
executes the same transform instead of disabling it.

The focused AMD64 ROS A/B matched 30 timestamps. The correction displacement
was `0.450000 m`; center residual was `0.449995 m` with correction disabled and
`0.000071 m` mean/max with production correction enabled. The test also fixed
an existing fake WGS84 northing inverse error. This remains simulation evidence,
not a field GNSS PASS. [Structured result](assets/test_result/gnss-lever-arm-20260806/README.md).

![Current sensor mount side view](assets/module-guides/sensor-kit/sensor-mount-side-view.png)

![GNSS left-antenna lever arm](assets/module-guides/sensor-kit/gnss-left-antenna-lever-arm.png)

![Sensor X ledger](assets/module-guides/sensor-kit/sensor-x-before-after.png)

The GNSS correction itself leaves IMU, LiDAR, camera, radar, axle, controller,
map, parking, body, and planning coordinates unchanged. A separate boundary
update uses the confirmed base-platform
`1.19160 x 0.87000 m` and fabrication-inclusive `1.39160 x 1.07000 m` totals
as chassis and active-body values respectively. Lanelet map revision 16 is a
separate user geometry change.

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
- The active and named-copy map-v16 OSM files remain byte-identical at SHA
  `fd9c1855573784e4e4e952f931c87e3b2c2858fa20f9c8ae5c2ad9adfc32d0cf`.
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
- A fresh isolated map-v16 full simulation reached `[SYSTEM] OK`. B1-B10
  turnaround reports passed 10/10 and B11/B12/B13 arrival-only campsite reports passed; the optional
  LiDAR cost-grid was explicitly OFF and no longer creates a false baseline-rate failure.
- The repository-standard `./src/colcon_build.sh` rebuilt control, sensor-kit,
  planning, platform, localization, system, and bringup. Fresh tests passed all
  `49/49` CTest targets; aggregate xUnit records were `361`, with zero
  errors/failures and 13 known planning/localization static-analysis skips.
- Exact machine-readable build and run results are recorded in
  [`amd64-scoped-container-shutdown-20260805.json`](evidence/v2.1.5/runtime-topology/amd64-scoped-container-shutdown-20260805.json)
  and summarized in `DONE.txt`.

## Field Work Still Required

1. Measure rover X/Z and moving-base antenna XYZ from `robot_center_link`,
   baseline length/direction, and the receiver's reported position reference.
2. Validate the implemented heading-aware lever-arm correction during straight,
   turn, and crab motion before setting `pose_verified=true`.
3. Run Jetson production topology versus standalone resource A/B and ten clean
   mission/cancel/restart cycles.
4. Verify the complete body/wheel/sensor/bracket/cable/payload envelope against
   the active fabrication-inclusive boundary, then repeat the margin and body-contact
   tests on the physical robot.
5. Select and validate a safe B11-B13 return pose/yaw/path; do not treat the
   arrival-only simulation as return approval.
6. Complete rear-camera rate, seven-channel radar, GNSS reacquisition, IMU
   startup, wheel response, parking/docking, charging, and full-route field tests.

Exact procedures and pass/fail thresholds remain in [`TODOLIST.txt`](../TODOLIST.txt).
