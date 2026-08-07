# CAMROD v2.1.6 Release Notes

<!-- HH_260807 - Record the field-driven cadence, motion, boundary-recovery,
radar visualization, and deployment-default corrections without relabeling
v2.1.5 simulation or physical evidence. -->

Release date: 2026-08-07 (Asia/Seoul)

Previous baseline: `v2.1.5`

## Release Summary

v2.1.6 synchronizes the deployed launch/configuration contract after the latest
physical bringup diagnosis. Receiver corrections are configured at `5 Hz` while
the EKF and selected localization pose remain `20 Hz`; those rates describe
different stages and must not be compared as if they were one publisher.
Straight cruise is reduced to final `2.0 km/h`, the planning envelope is the
measured body plus `0.10 m` on every side, and the LiDAR raw/filtered path is
restored to a `10 Hz` target without enabling the optional LiDAR cost raster.

The route-boundary policy now permits enough bounded release cycles for a long
route without reopening an unsafe same-direction loop. A release budget of 12
applies within the 5-second recontact window. At the budget, same-direction
Nav2 resume stays blocked, but a separately projected inward escape remains
eligible when overlap decreases monotonically and both the swept physical body
and endpoint planning footprint are clear. Dynamic obstacles and platform
interlocks remain fail-closed.

## Active Contract

| Item | v2.1.6 value | Meaning |
|---|---:|---|
| Navigation frame | `robot_center_link` | Axle midpoint used by localization, planning, control, and platform |
| GNSS position source | left antenna `(0,+0.45,0) m` | Heading-rotated lever arm is subtracted before EKF input |
| GNSS receiver/correction cadence | `5 Hz`, `200 ms` epoch | Rover override is written at launch; moving-base epoch match remains field acceptance |
| Localization output | `20 Hz` | EKF predicts between physical GNSS epochs; pose WARN/ERROR remain below `18/14 Hz` |
| GNSS health window | nominal `5 Hz`; rolling minimum `3 Hz` over `2 s` | Counts unique receiver epochs and tolerates short scheduler jitter without accepting a dead stream |
| GNSS heading gap | at most `3 s` | GNSS-anchored EKF yaw delta may rotate only the lever arm and never marks GNSS yaw valid |
| Straight cruise | final `2.0 km/h` (`0.555556 m/s`) | Raw RPP `1.111111 m/s` passes through the retained `0.5` command gate |
| Physical body | `1.39160 x 1.07000 m` | Stops ordinary motion and defines every swept-body recovery check |
| Planning boundary | `1.59160 x 1.27000 m` | Physical body plus `0.10 m` on all four sides |
| Route clear proof | `1.5 s` after observed recovery motion | Braking/coasting alone cannot release Nav2 |
| Recovery release budget | `12` in `5 s` | Budget blocks same-direction Nav2 resume, not a newly projected safe inward escape |
| Recovery motion envelope | raw `0.10 m/s`, `0.10 rad/s`, `12 deg`, `0.40 m`, `10 s` | Global budget is not reset by a crab/reverse/reverse-yaw stage transition |
| LiDAR raw/filtered cadence | target `10 Hz` | `max_process_hz: 0.0` processes every incoming cloud |
| LiDAR cost grid | default `OFF` | Optional node/topic and graph entry are omitted; the shared checker still monitors radar + inflation |
| Radar hardware profile | angle level `4`, range level `1` | Widest beam and approximately `0.50 m` hardware observation window |
| Radar visualization | seven physical `/range_ros` streams | `radar_status_gui.py` subscribes only; it never starts or publishes a dummy source |
| Operator renderer | WebKit | Fullscreen field default; Chromium and `auto` remain explicit alternatives |

## GNSS And Localization

- The dual-rover launch writes `CFG-RATE` in RAM even when the rest of startup
  configuration is disabled. The moving base is not configured by ROS, so both
  receivers must show matching 200 ms `iTOW` increments in the field.
- Raw GNSS and robot-center-corrected GNSS diagnostics expect `5 Hz`.
  `/localization/pose` independently expects `20 Hz` from EKF prediction.
- Duplicate or out-of-order receiver epochs do not inflate localization health.
  The monitor evaluates unique timestamps over the configured rolling window.
- A valid dual-GNSS heading remains the anchor for the `0.45 m` lever arm. The
  bounded EKF yaw-delta fallback bridges only a short heading gap; it does not
  synthesize a valid absolute yaw observation.

Physical RTK Fixed retention, base/rover epoch alignment, moving residuals,
antenna X/Z and moving-base XYZ remain field-pending.

## Motion, Boundary, And Recovery

The production RPP request is raw `1.111111 m/s`; after the final `0.5` gate the
straight reference is `0.555556 m/s` (`2.0 km/h`). Curve, approach, campsite,
parking, and AprilTag limits retain their documented ratios. Historical 3 km/h
AMD64 runs remain valid evidence for the exact configuration they measured and
are not relabeled as v2.1.6 physical acceptance.

The active physical and planning polygons are evaluated in
`robot_center_link`. A virtual lanelet cost-100 overlap always zeros ordinary
motion. Recovery is admitted only when the candidate moves inward with
monotonically decreasing overlap, its continuous swept body is clear, and its
endpoint planning polygon is clear. No safe candidate means zero output.

The older one-release logs remain evidence of the former policy. They explain
the reproduced deadlock but do not prove the new 12-release policy. A supervised
run must capture first contact, candidate selection, owner motion, 1.5-second
clear proof, release/recontact count, continued route motion, and final stop.

## LiDAR And Radar

- Physical LiDAR raw and ground-segmented filtered clouds target `10 Hz`.
  Disabling `enable_lidar_cost_grid` removes only the optional raster path; it
  does not make `/sensing/lidar/points_filtered` optional.
- The preprocessor has no artificial rate cap. Field acceptance still requires
  `10 Hz` under production Jetson load with point count and age recorded.
- Seven SEN0592 channels use real serial publishers. The radar status GUI reads
  their normalized range streams, exposes missing/stale channels, and does not
  make dummy data look like hardware.
- The fixed-return bands and per-channel stop windows account for the robot body
  before the additional near-field clearance. Their current numerical values
  remain in `camrod_sensing/README.md` and the synchronized radar YAML.

## Deployment And Config Synchronization

- Package-owned YAML is canonical and the bringup mirrors carry the same active
  values. Config-sync tests cover the mirrored GNSS, localization, LiDAR, radar,
  planning, control, platform, and sensor-kit files.
- `colcon_build.sh` is the canonical install-sync entrypoint. Single-config CMake
  packages default to `Release` unless the caller explicitly supplies another
  `CMAKE_BUILD_TYPE`; the UI bundle and package data are installed in the same
  build scope.
- `setup_camrod.sh` remains dependency-only. There is no separate
  `camrod_setting.sh`; setup does not launch runtime nodes or silently run field
  acceptance.
- WebKit is the full-bringup and standalone operator-window default on the
  current robot image because its Snap Chromium exits before opening a window.

## Evidence And Acceptance Boundary

Existing v2.1.5 map-v17 service, 3 km/h, one-release recovery, renderer A/B,
stationary sensor, and reduced-boundary artifacts retain their original labels,
rates, geometry, hashes, and verdicts. Configured values in this document are
not new measurements.

Before declaring physical `FIELD-PASS`, complete the current `TODOLIST.txt`:

1. verify dual-GNSS 5 Hz/200 ms epochs, RTK flags, heading and 20 Hz final pose
   during a supervised 1-to-2 km/h run;
2. verify raw and filtered LiDAR at 10 Hz under full production load;
3. exercise margin and virtual-body recovery, including the 12-release budget,
   while proving actual obstacles and interlocks remain stopped;
4. validate all seven real radar streams and their self-return separation;
5. repeat complete site-return-park-charge-next-site operation on the robot
   without a process restart.

Exact procedures and pass/fail thresholds are maintained in
[`TODOLIST.txt`](../TODOLIST.txt).
