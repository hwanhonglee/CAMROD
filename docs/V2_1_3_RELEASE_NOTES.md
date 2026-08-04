# CAMROD v2.1.3 release notes

<!-- HH_260804 - Include the live retry-loop containment and operator-stop fix
     while keeping all physical driving and charger claims field-pending. -->

Release date: 2026-08-04 (Asia/Seoul)

## Outcome

v2.1.3 is the synchronized source, configuration, documentation, and
simulation-evidence checkpoint for axle-midpoint navigation, bounded automatic
route-boundary recovery, rapid-recontact containment, and Guest/Robot UI
mission-state parity. It retains the v2.1.2 RPP and physical safety profile.

No post-change full real-robot driving acceptance has been completed. Remaining
Jetson, wheel, GNSS lever-arm, charger-contact, radar, camera-rate, and narrow
route checks stay FIELD-PENDING in [`TODOLIST.txt`](../TODOLIST.txt).

## Canonical robot frame

- `robot_center_link`, at the midpoint of the 0.886 m axle spacing, is the
  canonical localization, Nav2, sensing, control, parking, diagnostics, UI,
  voice, and simulation reference.
- `robot_base_link` remains a fixed compatibility child 0.443 m behind the
  center. It is no longer an independent dynamic odometry base.
- Sensor X coordinates use `new_x = old_x - 0.443 m`. Physical mounts, Y/Z/RPY,
  measured body size, map coordinates, goals, and safety margins are unchanged.
- The physical body remains 1.49160 x 1.07000 m. The deployed planning boundary
  remains 1.69160 x 1.27000 m with 0.10 m margin per side.
- AprilTag longitudinal thresholds were shifted by 0.443 m to preserve the same
  physical charger stop positions.

The complete conversion table and A/B drive measurements are in
[`V2_1_3_ROBOT_CENTER_MIGRATION.md`](V2_1_3_ROBOT_CENTER_MIGRATION.md).

## Boundary recovery

- `cmd_vel_safety_gate` evaluates left crab, right crab, and reverse projections
  against the complete planning footprint and all ordinary interlocks.
- A unique clear lateral direction selects pure crab away from contact. If both
  lateral directions are blocked and reverse is clear, reverse is selected.
  Ambiguous, blocked, stale, canceled, or unauthorized cases remain stopped.
- `route_safety_recovery_controller` owns automatic recovery commands only
  during a navigation-triggered `ROUTE_SAFETY_HOLD`.
- Raw recovery is limited to 0.10 m/s, 0.40 m, and 10 seconds. Pose, candidate,
  and gate status must remain fresh. Angular velocity stays zero while touching
  the boundary.
- The retained Nav2 goal resumes only after 1.0 second of continuous fresh
  lanelet clearance. One automatic release is allowed. Recontact of the same
  route within 5.0 seconds latches `ROUTE_SAFETY_HOLD`, blocks further recovery
  candidates/releases, and requires operator stop/replan/re-engage.
- Full bringup with the user-provided map revision 14 observed recontact
  `0.276 s` after release; earlier revision-13 runs measured `0.372 s` and
  `0.267 s`. All cases latched, and `/control/cmd_vel_ros` remained all zero. The narrow
  754/2751/2720 route is still not declared mission-capable.

Manual pre-owner probes and current automatic-owner runs are clearly separated
in [`V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md`](V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md).

## Lanelet map revision 14

- The user-provided v14 edit keeps all 69 relations while reshaping ten
  boundary LineStrings used by lanelets 163, 2292, 2690, 2744, 754, and 2751.
- Relative to v13, 38 existing map nodes moved, 33 nodes were added, and three
  were removed. The active map and named Park copy are byte-identical.
- The full stack parsed 55 lanelets, 14 Areas, 1,031 points, and 179 lines from
  v14 and reached `[SYSTEM] OK`.
- A B6 run still contacted `lanelet_footprint_cost` near
  `(4.3688, 45.0583)`. This revision is therefore recorded as a valid loaded
  map input, not as proof that the campsite route is wide enough.

## UI and mission state

- Guest UI destination selection uses the same backend mission path as Robot
  UI, including `DEPARTING_DROP_ZONE`, `DEPARTING_CHARGER`, site entry, unload,
  return, parking, waiting-for-charge, and charging phases.
- Guest `usage_complete` publishes `MotionOperation.RETURN`; it no longer writes
  the return service state directly.
- Platform SOC ratio is normalized consistently. New campsite dispatch requires
  at least 35% SOC from either UI.
- `ROUTE_SAFETY_HOLD` is displayed as a control safety overlay while the service
  lifecycle remains visible. Generic diagnostics WARN does not replace normal
  MOVING, ENTERING, or RETURNING state.
- `OPERATOR_STOPPED` is represented consistently after cancel/stop.
- Robot backend Nav2 cancellation now uses the rclpy `Client.call_async()` API.
  The previous rclcpp-style method caused `POST /ui/stop` to return HTTP 500.
  Live simulation now returns HTTP 200, cancels campsite/drop-zone/parking
  owners plus Nav2, enters gate `STANDBY`, and publishes state 16.

## Planning and configuration invariants

- The selected RPP profile remains 0.4 m/s desired speed with 1.1 m fixed
  lookahead at a 15 Hz controller/pose cadence.
- Package-owned YAML, `camrod_bringup/config`, and installed deployment mirrors
  use the same center frame, footprint, parking, diagnostics, and recovery
  values.
- Jetson-specific drivers, CUDA/TensorRT, camera, CAN, and SDL runtime settings
  were not changed to fit the development PC.

## Evidence layout

v2.1.3 keeps numeric evidence versioned while visual results are owned by the
module that explains them:

```text
docs/assets/module-guides/
  sensor-kit/         # rear axle versus robot center GIF
  planning/           # route-footprint risk PNG
  control/            # boundary stop and pre/current recovery GIF/PNGs
  ui/                 # Guest UI state screenshots

docs/evidence/v2.1.3/
  reference-frame/    # A/B summaries and timelines
  boundary-geometry/  # physical, 5 cm, and deployed footprint sweeps
  boundary-recovery/  # pre-owner and automatic-owner timelines
  ui/                 # Guest ROS/WebSocket mission lifecycle

docs/evidence/module-guides/bringup/
  runtime-visual-capture-20260804.json
  raw/runtime-visual-capture-20260804.log
```

Fourteen `SIM RUNTIME CAPTURE` PNGs show actual RViz, ROS CLI, and browser
surfaces for every CAMROD-owned package. They are indexed in
[`MODULE_VISUAL_GUIDE.md`](MODULE_VISUAL_GUIDE.md) and are explicitly not field
evidence.

Filenames identify command ownership instead of relying on timestamps.
Duplicate root copies and ignored `src/build`, `src/log`, pytest, and Python
cache directories are not release artifacts.

## Verification

- The release baseline built eleven affected ROS packages with
  `--symlink-install`; its selected package run reported 454 tests, zero errors,
  zero failures, and 17 skips.
- The final map/retry/UI follow-up rebuilt `camrod_control`, `camrod_ui`, and
  `camrod_bringup`; CTest reported 141 tests, zero errors, zero failures, and
  zero skips. Ten focused UI tests passed.
- The frame/deployment contract now passes 28 frame checks plus one retry-config
  mirror check; control policy and parking tests pass 54 checks.
- Python compilation, 243 YAML parses, package/bringup/install configuration
  synchronization, Markdown links, image/GIF decoding, JSON parsing, secret
  scanning, and repository diff checks passed.
- The complete `camrod_voice` CMake build remains unavailable on this host only
  because the existing `SDL2_mixer` system package is absent. Its changed Python
  source compiles, and no production dependency was weakened.

## Required physical acceptance

Before declaring the narrow route or automatic recovery field-ready, run the
Jetson procedure in
[`field_test_runbook.md`](../camrod_bringup/docs/field_test_runbook.md). Record
both contact sides, rear-blocked recovery, operator cancel, distance/time
bounds, wheel directions, retained-goal behavior, and final zero command on one
timeline. Do not convert simulation evidence into FIELD-PASS.
