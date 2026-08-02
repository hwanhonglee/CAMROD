# CAMROD v2.1.2 release notes

<!-- HH_260802 - Record the explicitly requested refreshed v2.1.2 checkpoint
     without converting simulation evidence into real-robot acceptance. -->

Release date: 2026-08-02 (Asia/Seoul)

## Outcome

The refreshed v2.1.2 tag is the synchronized software, configuration, and
simulation checkpoint for low-speed RPP tracking, complete-footprint lanelet
stopping, Ranger steering evidence, and rear-camera publication isolation. It
retains all v2.1.1 safety behavior and the July 31 v2.1.2 field artifacts.

No post-change full real-robot driving acceptance has been completed. The
remaining physical criteria stay in [`TODOLIST.txt`](../TODOLIST.txt).

## RPP tracking

- Direct UI RPP and the RPP nested in manual `RotationShim` use identical
  `lookahead_dist` and `min_lookahead_dist` values of `1.1 m`.
- The source speed remains `0.4 m/s`, or at most `0.20 m/s` after the final
  command-gate scale. Controller and selected-pose cadence remain 15 Hz.
- Isolated full-bringup trials compared 0.8, 1.0, 1.1, 1.2, 1.4, and 1.6 m on
  straight and S-curve FollowPath inputs. Valid 1.0-1.2 m straight trials had
  zero repeated centerline crossings; 1.1 m was the balance between 1.0 m
  tracking error and 1.2 m steering variation.
- The full simulator did not reproduce the reported physical sine-wave motion.
  Therefore 1.1 m is a simulation-selected candidate, not a real-robot
  stability claim.

## Lanelet boundary safety

- The published asymmetric planning boundary remains the measured body plus
  0.10 m on every side.
- Regression tests prove that off-lane cost 100 inside the planning margin
  stops before the measured body reaches that cell, and that cost 100 touching
  the measured body also stops at the deployed 0.25 m grid resolution.
- The complete-footprint threshold remains 100. Soft rasterized lane-edge cost
  98 is still allowed for narrow mapped lanes; dynamic-obstacle, unknown-cell,
  engage, ESTOP, CAN, charging, and operator-cancel protections are unchanged.
- A full-bringup 0.8 m stress trial raised `lanelet_footprint_cost`; boundary
  stop trials were excluded from RPP tracking scores.

## Ranger steering evidence

- `ranger_msgs/msg/SteeringTransitionState` records every incoming Twist,
  selected motion mode, target and rate-limited Ranger steering command,
  signed error, translation scale, and SDK speed.
- `/platform/steering_transition_state` is added to the recovery rosbag. It is
  compared with `/actuator_state` to separate controller reversals, driver slew
  limiting, and physical wheel/CAN lag.
- Ranger's 0.25 rad/s steering transition, 0.05 rad full-speed error, and
  0.35 rad translation-stop error remain unchanged.

## Rear camera

- Monitoring JPEG encoding runs in a latest-frame worker, so slow CPU encoding
  cannot block raw image and CameraInfo publication.
- Source timestamps and existing topic names are retained. The ARM/Jetson-only
  camera build, GStreamer/VIC path, calibration contract, and device settings
  are unchanged.

## Configuration and documentation

- Package-owned and bringup copies of `nav2_base.yaml` and
  `nav2_vehicle.yaml` are byte-identical.
- The field recorder, runbook, platform/sensing documentation, `DONE.txt`, and
  `TODOLIST.txt` describe the same topic and parameter contracts.
- The original annotated `v2.1.2` tag targeted `22f9b0cb`. At the operator's
  explicit request it is moved to the final release commit containing this
  refreshed checkpoint.

## Verification

- `ranger_msgs`, `ranger_base`, `camrod_sensing`, `camrod_control`,
  `camrod_planning`, `camrod_platform`, and `camrod_bringup` built successfully.
- The five CAMROD package suites reported 330 tests, zero errors, zero failures,
  and eight skips. Focused Ranger steering/parallel policy tests passed 5/5.
- The generated steering interface, rear-camera isolated host compile, 13
  bringup contract tests, field-tool shell syntax, config/install synchronization,
  and repository diff checks passed.
- The vendor Ranger all-linter target still reports pre-existing package.xml
  ordering, copyright-header, and repository-wide formatting failures. Those
  failures are outside this scoped message/telemetry change; the driver build
  and focused functional tests pass.
- The isolated x86 rear-camera link reports the known workspace `cv_bridge`
  OpenCV 4.5 versus `/usr/local` OpenCV 4.14 warning. Production remains the
  unchanged Jetson ARM build and requires its physical cadence test.
- The 21 isolated full-bringup RPP runs are recorded as simulation evidence.
  Intentionally cancelled 12-second FollowPath actions and boundary-stopped
  trials are not counted as successful navigation or field acceptance.

## Required physical acceptance

Use [`field_test_runbook.md`](../camrod_bringup/docs/field_test_runbook.md) and
record `/platform/steering_transition_state`, `/actuator_state`, localization,
raw/final command, boundary, and cost-grid topics on one clock. Both left and
right offsets must converge without repeated centerline crossing before RPP or
wheel-lag work moves from FIELD-PENDING to FIELD-PASS. Rear raw-camera cadence
must also be measured on the Jetson production profile.
