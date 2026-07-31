# CAMROD v2.1.1 release notes

<!-- HH_260730 - Record the exact software/configuration checkpoint separately
     from the physical acceptance work that remains in TODOLIST.txt. -->

Release date: 2026-07-30 (Asia/Seoul)

## Outcome

v2.1.1 is a field-readiness release for localization freshness, consistent
manual/UI path tracking, bounded route recovery, startup/mission state
visibility, camera robustness, diagnostics, and repeatable field evidence.
It does not claim a completed post-change real-robot driving acceptance.

## Localization and control

- The real `robot_localization` EKF output changes from 10 Hz to 15 Hz to match
  the Nav2 controller. GNSS receiver cadence remains 1 Hz.
- The selected-pose path now chooses the freshest header-stamped pose/odometry
  payload instead of republishing a previous-cycle pose when callbacks arrive
  in odometry-first order.
- Direct UI RPP and the RPP nested inside the manual `RotationShim` use the
  same desired velocity, lookahead, regulation, interpolation, and collision
  parameters. `RotationShim` remains only to honor an arbitrary manual final
  yaw.
- Minimum lookahead increases from 0.5 m to 0.9 m, nominal lookahead from
  0.8 m to 1.0 m, and lookahead time from 1.5 s to 1.8 s. This is a staged
  straight-line oscillation reduction; the left/right field run remains open.
- Ranger steering-transition speed limiting and route-safety recovery from
  the post-v2.1.0 remediation are included.

## Planning and safety

- Manual goal x/y is projected to the reachable LaneletRoute endpoint while
  preserving the clicked yaw. UI goals retain the configured lane-position
  and lane-heading rules.
- Route-safety holds retain their violating direction, allow only a bounded
  clear opposite-direction escape, and can reissue the retained goal after a
  continuously clear enabled interval.
- The full robot footprint, lethal/unknown cost handling, obstacle latch, and
  operator-cancel priority are not weakened.
- Path visualization and obstacle monitoring keep only the latest high-rate
  path sample and perform work at their configured output/check cadence.

## Radar

v2.1.1 does not widen fixed-return exclusions or change radar stop geometry.
The latest physical log showed actual gate stops from both LEFT2 and RIGHT2,
while the radar-disabled run showed all seven dummy channels and no radar
cost stop. Clear-area ON/OFF comparison and obstacle separation remain the
first radar field task in `TODOLIST.txt`.

## Camera, diagnostics, UI, and voice

- Front-camera/NvJPEG payload validation and YOLO compressed-input containment
  prevent malformed frames from terminating the shared component container.
- Disabled hardware remains fail-visible through typed dummy heartbeats and
  component/location detail in system diagnostics.
- UI and voice share the source-neutral sequence `INITIALIZING`, `READY`,
  `GOAL_RECEIVED`, `PATH_PREPARING`, `DRIVING`, `SAFETY_STOP`, and `ARRIVED`.
- Startup audio is `system.startup`; `system.ready` is emitted once only after
  module health, localization/TF, Nav2 action server, command gate, platform,
  and engage state prerequisites are present.
- The native WebKit window remains the default lightweight operator UI.

## Map and configuration

- The stable runtime map entry is `lanelet2_maps.osm`, byte-identical to the
  retained `copy_park_v1.0.3` snapshot. Older intermediate copy files are
  replaced by explicit versioned audit snapshots.
- Package-owned configuration and the `camrod_bringup/config` deployment
  mirrors were compared across 139 pairs with zero byte differences.
- GNSS and IMU physical port values have no semantic change in this release.

## Verification

- Six affected packages built successfully:
  `camrod_localization`, `camrod_planning`, `camrod_system`, `camrod_voice`,
  `camrod_ui`, and `camrod_bringup`.
- Their package test suites passed. The UI package also disables an unrelated
  user-site `anyio` pytest plugin that is incompatible with ROS Humble's
  pytest version, so ordinary `colcon test` is deterministic.
- A production-entry `bringup.launch.py sim:=true` smoke test reached
  `SYSTEM OK` and played `system.startup` followed by `system.ready`.
- The smoke test was shut down after readiness. It did not exercise physical
  CAN, cameras, radar, GNSS motion, manual/UI driving, or final arrival.

## Required field acceptance

<!-- HH_260730 - Keep completed software/sim evidence out of the live field
     checklist so an implemented item is not mistaken for an accepted drive. -->
Completed implementation, diagnosis, unit-test, and indoor/sim evidence is
maintained in [`DONE.txt`](../DONE.txt). The ordered 2026-07-31 checklist in
[`TODOLIST.txt`](../TODOLIST.txt) now contains only unfinished physical
acceptance. The release must not be treated as field accepted until radar
ON/OFF, dual-camera/YOLO, manual/UI four-goal, left/right lateral convergence,
voice/WebKit mission sequence, and production CPU comparisons have recorded
PASS evidence.

## Post-release stationary field evidence

<!-- HH_260731 - Record individual physical acceptance without rewriting the
     immutable release-time statement or claiming a driving acceptance. -->

On 2026-07-31, an actual no-motion bringup completed two individual field
acceptance items:

- radar disabled for 600.063 seconds: seven exact dummy ranges, evidence
  active 0, radar high-cost cell 0, radar cost-stop 0;
- physical front camera and YOLO for 300 seconds: 2,750/2,750 JPEG decodes and
  camera/YOLO restart 0.

Rear-camera FPS, seven-physical-channel radar separation, dual-GNSS Fixed
stability, production-only CPU, voice/manual/UI, boundary recovery, and
lateral/crab driving remain open. Exact results and logs are in
[`v2_1_1_field_validation_20260731.md`](../camrod_bringup/docs/v2_1_1_field_validation_20260731.md).
