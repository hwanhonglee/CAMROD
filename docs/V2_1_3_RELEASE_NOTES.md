# CAMROD v2.1.3 release notes

<!-- HH_260804 - Release the center-frame migration, bounded automatic route
     recovery, UI parity, and organized simulation evidence without claiming
     real-robot driving acceptance. -->

Release date: 2026-08-04 (Asia/Seoul)

## Outcome

v2.1.3 is the synchronized source, configuration, documentation, and
simulation-evidence checkpoint for axle-midpoint navigation, bounded automatic
route-boundary recovery, and Guest/Robot UI mission-state parity. It retains
the v2.1.2 RPP and physical safety profile.

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
  lanelet clearance. The narrow 754/2751/2720 route still stops again and is
  not declared mission-capable.

Manual pre-owner probes and current automatic-owner runs are clearly separated
in [`V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md`](V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md).

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

## Planning and configuration invariants

- The selected RPP profile remains 0.4 m/s desired speed with 1.1 m fixed
  lookahead at a 15 Hz controller/pose cadence.
- Package-owned YAML, `camrod_bringup/config`, and installed deployment mirrors
  use the same center frame, footprint, parking, diagnostics, and recovery
  values.
- Jetson-specific drivers, CUDA/TensorRT, camera, CAN, and SDL runtime settings
  were not changed to fit the development PC.

## Evidence layout

v2.1.3 moves release evidence out of the workspace root into predictable paths:

```text
docs/assets/v2.1.3/
  reference-frame/    # rear axle versus robot center GIF
  boundary-geometry/  # stop location and narrow-route risk PNGs
  boundary-recovery/  # pre-owner and automatic-owner GIF/PNG results
  ui/                 # Guest UI state screenshots

docs/evidence/v2.1.3/
  reference-frame/    # A/B summaries and timelines
  boundary-geometry/  # physical, 5 cm, and deployed footprint sweeps
  boundary-recovery/  # pre-owner and automatic-owner timelines
  ui/                 # Guest ROS/WebSocket mission lifecycle
```

Filenames identify command ownership instead of relying on timestamps.
Duplicate root copies and ignored `src/build`, `src/log`, pytest, and Python
cache directories are not release artifacts.

## Verification

- Eleven affected ROS packages built successfully with `--symlink-install`.
- Selected package tests reported 454 tests, zero errors, zero failures, and
  17 skips. Nine focused UI policy tests also passed.
- The center-frame contract passed 28 checks; control policy tests passed 54.
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
