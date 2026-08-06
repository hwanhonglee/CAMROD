# v2.1.5 Field Handoff - 2026-08-08

<!-- HH_260807 - Provide one cross-PC handoff for the active software baseline,
measured simulation evidence, and the next physical acceptance session. -->

This document is the starting point on another PC. It separates implemented
behavior, AMD64 simulation evidence, and work that still needs a physical
Jetson/robot PASS.

## What Changed

| Area | Active behavior |
|---|---|
| Reference frame | Localization, planning and control use `robot_center_link`; rear-axle compatibility remains at `robot_base_link`, X `-0.443 m` |
| GNSS lever arm | NavSatFix is the left antenna at body Y `+0.45 m`; the adapter subtracts the heading-rotated offset |
| Body safety | Physical `1.39160 x 1.07000 m` is a non-recoverable hard-stop envelope |
| Planning margin | `1.49160 x 1.17000 m`, 5 cm per side; margin-only contact can request bounded recovery |
| Safety raster | Independent robot-centered `600 x 600 @ 0.05 m`; Nav2 retains the global `0.25 m` grid |
| Command ownership | A 0.5 s zero handoff applies only between explicit maneuver and Nav2 owners, not between normal Nav2 rotation/translation commands |
| Curve tracking | Continuous RPP at `1.1 m` minimum lookahead; gross initial heading uses a separate `75/5 deg` zero-linear gate |
| Speed | Final cruise `3.0 km/h`; maneuver ratios are listed below; recovery remains `0.18 km/h` |
| Pose/control clock | Real EKF, local path/tracking and Nav2 controller use `20 Hz`; delayed-data history is `1.0 s` |
| GNSS correction clock | Hardware launch still writes `1 Hz`; 5/10 Hz is field A/B work, not a released claim |
| Campsite policy | B1-B10 turn around on site; B11-B13 stop roadside at a 0.60 m crab cap and have no approved return geometry |
| Planning diagnostics | Expected empty local paths in service-owned maneuver/parking/departure states remain `OK`; route travel still checks freshness and point count |
| Runtime | Checker/Nav2/system-core containers ON; LiDAR cost grid and DDS-SHM OFF by default; Chromium is the default operator window |
| Active map | User-authored map v17, SHA `8cd05c...5e021`; 55 lanelets, 14 areas, 1,652 nodes |

## Speed Profile

| Operation | Final limit |
|---|---:|
| RPP cruise | `3.000 km/h` |
| Curve floor / final approach | `1.500 / 0.750 km/h` |
| Campsite crab | `1.800 km/h` |
| Campsite reverse / drop-zone exit / reverse parking | `1.200 km/h` |
| AprilTag approach / insertion | `1.500 / 0.375 km/h` |
| Optional yaw-zone approach 1 / 2 | `1.875 / 1.500 km/h` |
| Boundary recovery | `0.180 km/h` |
| Zero-turn / gross heading alignment | `0 km/h` linear |

## AMD64 Evidence Already Completed

| Scenario | Result | Limit |
|---|---|---|
| Clear route | `10.0403 m`, goal error `0.2932 m`, no route hold | Simulation geometry only |
| Margin stop/recovery | ordinary output zero; `CRAB_RIGHT` `0.133 m` at max `0.05 m/s` | Does not validate real envelope |
| Physical-body contact | no candidate, owner motion or final output | Policy SIM-PASS |
| B7 stop-go | false handoff/stale `15/15 -> 0/0`; `224.92 s`, `27.1492 m` before real margin contact | Full route still depends on map width |
| B8 RPP `1.1 m` | switches `403 -> 0`, `59.931 m`, `GOAL_REACHED` | `1.2 m` comparison failed after 0.999 s recontact |
| 3 km/h smoke | `3.000001 km/h`, selected pose `20.024 Hz`, max step `6.485 cm`, no step over 20 cm | Fake GNSS is 10 Hz |
| GNSS lever arm | 30 matched samples; center residual `0.449995 -> 0.000071 m` | Synthetic WGS84/heading |
| Campsites | B1-B10 full seven-phase sequence; B11-B13 arrival-only | B11-B13 RETURN is not approved |
| Repeated service | B1 -> B2 -> B3, `3/3`, `677.237 s`, restart `0` | Simulated CAN/charging and kinematics |
| B2 margin recovery | `3/3`, `REVERSE_YAW_RIGHT`, mission complete, second hold `0` | Real boundary/contact remains pending |
| Persistent obstacle | 3.00 m lane: immediate stop, one no-path preflight, no ABORT loop, clear -> resume `0.242 m` | Successful bypass needs a wider surveyed lane |

Source reports live under `docs/assets/test_result/`. Do not reinterpret these
as Jetson or physical-robot acceptance.

The final release-candidate set is grouped at
[`docs/assets/test_result/v2-1-5-service-validation-20260807/`](../../docs/assets/test_result/v2-1-5-service-validation-20260807/README.md),
including raw JSON/log, SHA manifest, PNG, and GIF.

## Previous TODO Crosswalk

<!-- HH_260807 - Keep the old numbered field work traceable after removing
duplicated history from the active TODO. -->

| Previous TODO | Current active item | What remains |
|---|---|---|
| `robot_center_link` field check | `P0-A`, `P0-B` | Body/sensor measurements, TF ownership and moving GNSS residual |
| `3` rear camera rate | `P1-E` | Production-load 10 Hz acceptance and AprilTag latency |
| `4`, `11`, `12` footprint/recovery | `P0-D` | Real margin escape, body hard stop, recontact latch and mission continuation |
| `5` radar seven-channel test | `P1-E` | New-bracket TF, clear-area noise and real-obstacle separation |
| `6`, `7` goal/path/obstacle planning | `P0-C`, `P0-D` | Manual/UI round trip, 20 s wide-lane fallback and narrow-lane denial |
| `8`, `14` runtime resources | `P1-F` | Jetson container/standalone CPU, GPU, PSS and ten clean restarts |
| `9` state/voice/UI | `P0-C`, `P1-G` | Complete public state order, cancel behavior and Robot/Guest UI parity |
| `10`, `13` GNSS/IMU/control latency | `P0-B` | Fixed reacquisition, 1/5/10 Hz A/B and 3 km/h lateral convergence |
| `15` next-site map survey | Deployment note | Run only when the deployment site changes |

The removed paragraphs were completed implementation notes or simulation
history. They remain in `DONE.txt`, release notes, and versioned test-result
directories; they were not discarded as unfinished field work.

## Other-PC Start

```bash
cd <CAMROD_WS>/src
git fetch origin --tags
git checkout develop
git pull --ff-only origin develop
./colcon_build.sh
source ../install/setup.bash
ros2 run camrod_bringup field_test_tool.sh config
```

Record the exact source before every test:

```bash
git rev-parse HEAD
git describe --tags --always --dirty
sha256sum lanelet2_maps.osm
```

Launch and collect from separate terminals:

```bash
ros2 run camrod_bringup field_test_tool.sh launch rviz:=true
ros2 run camrod_bringup field_test_tool.sh watch
ros2 run camrod_bringup field_test_tool.sh pose-latency 60
ros2 run camrod_bringup field_test_tool.sh record-recovery <LOG_DIR>
```

## Physical Test Order

1. Measure road width and complete body/sensor geometry before driving.
2. Verify dual-GNSS Fixed, IMU startup, TF ownership and stationary state.
3. Drive straight at `1`, `2`, then `3 km/h` with one common localization/control bag.
4. Exercise one-side planning-margin recovery and physical-body hard stop.
5. Run B1-B10 site, user RETURN, drop-zone parking/charging and next-site dispatch at least three consecutive cycles without restart.
6. Inject a transient obstacle, then a 20 s persistent obstacle in both narrow and proven-wide lane sections.
7. Measure Radar after bracket installation, then compare Jetson container and standalone resource use.

The detailed PASS/FAIL criteria are intentionally kept only in the root
`TODOLIST.txt` so completed history does not remain duplicated as active work.

## GNSS 0.30 m Review

`pose0_rejection_threshold: 3.0` is a covariance-normalized three-sigma gate,
not a three-metre gate. Do not replace it with `0.3`.

The proposed absolute rule is an additional timestamp-aligned planar
innovation gate after initialization:

- reject a GNSS sample when GNSS centre versus EKF prediction exceeds `0.30 m`;
- continue bounded IMU/wheel prediction while reporting GNSS degraded;
- require `0.15..0.20 m` recovery evidence before clearing the degraded state;
- keep startup and deliberate re-attachment on a separate stationary policy;
- compare at the GNSS measurement stamp, never against the latest pose.

At `3 km/h`, the robot moves `0.833 m` between valid 1 Hz fixes and about
`0.30 m` in `0.36 s`. A previous-fix gate or an unaligned latest-pose comparison
would therefore reject normal motion.

## Result Placement

Use one directory per test session:

```text
<LOG_DIR>/
  FIELD_RESULT.txt
  bringup.log
  snapshot/
  pose_latency.json
  recovery/
  rosbag2_*/
  resource_profile/
```

When a field item passes, add the measured result to `DONE.txt` and remove or
shrink the corresponding root TODO in the same commit.
