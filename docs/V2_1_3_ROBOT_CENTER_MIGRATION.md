# CAMROD v2.1.3 robot_center_link Migration

<!-- HH_260803 - Record the complete rear-axle-to-axle-midpoint frame migration. -->

Release checkpoint: 2026-08-04 (Asia/Seoul)

## Purpose

CAMROD uses four-wheel steering with Dual-Ackermann, crab, and zero-turn
motion. The canonical navigation and control reference is now
`robot_center_link`, located halfway between the front and rear axle centers.
The name is robot-generic and does not couple shared software to the Ranger
product name.

This is a coordinate-reference migration. No sensor, chassis part, axle, map,
goal, or safety margin was physically moved or resized.

## Frame Definition

| Frame | Physical location | Runtime role |
|---|---|---|
| `robot_center_link` | Front/rear axle midpoint | Canonical odometry, EKF, Nav2, sensing cost-grid, control, parking, UI/voice readiness, and simulation base |
| `robot_base_link` | Rear axle center, 0.443 m behind `robot_center_link` | Legacy compatibility TF only |
| `sensor_kit_base_link` | Coincident with `robot_center_link` | Parent of all sensor mount frames |

The measured wheelbase is 0.886 m, so:

```text
center_offset_from_rear_axle = wheelbase / 2 = 0.443 m
new_center_x = old_rear_axle_x - 0.443 m
```

The TF tree has one dynamic base and no two-parent frame:

```text
map -> odom -> robot_center_link
                 |-> robot_base_link       x = -0.443 m
                 `-> sensor_kit_base_link  x =  0.000 m
                       `-> sensor frames
```

`robot_base_link` must not be published independently by Ranger odometry or
EKF. It is a fixed child supplied by `camrod_sensor_kit`.

## Rear-Axle Versus Center Reference

<!-- HH_260804 / v2.1.3 - Preserve the A/B drive comparison that selected the
     canonical center reference without presenting it as field acceptance. -->

The same simulated route and controller profile were replayed with the old
rear-axle reference and the new axle-midpoint reference. On the common route
segment, center reference reduced cross-track RMS from 0.0588 m to 0.0549 m
and yaw-error RMS from 2.901 deg to 2.713 deg. It progressed 0.8566 m farther
before the first route-boundary hold.

![Rear-axle versus robot-center drive](assets/module-guides/sensor-kit/test-results/reference-frame-20260803/rear-axle-vs-robot-center-drive.gif)

The [comparison summary](assets/module-guides/sensor-kit/evidence/v2.1.3-reference-frame/rear-axle-vs-robot-center-summary.json)
and [center-reference timeline](assets/module-guides/sensor-kit/evidence/v2.1.3-reference-frame/robot-center-drive-timeline.json)
retain the exact samples. Both references stopped again on the narrow curve;
the result supports a more consistent 4WS reference but does not prove that
the mapped route is wide enough or replace physical steering validation.

## Sensor Coordinate Conversion

All Y/Z and roll/pitch/yaw values are unchanged. Only X changes because the
origin moves 0.443 m forward. Values below are in metres and degrees.

| Sensor | Old XYZ from rear axle | New XYZ from axle midpoint | RPY, unchanged |
|---|---:|---:|---:|
| IMU | `(1.13100, 0.00000, 0.75600)` | `(0.68800, 0.00000, 0.75600)` | `(0, 0, 0)` |
| GNSS placeholder | `(0.00000, 0.00000, 0.00000)` | `(-0.44300, 0.00000, 0.00000)` | `(0, 0, 0)` |
| LiDAR | `(1.20636, 0.00000, 0.59538)` | `(0.76336, 0.00000, 0.59538)` | `(0, 0, 0)` |
| Front camera | `(1.20637, 0.00000, 0.49568)` | `(0.76337, 0.00000, 0.49568)` | `(0, 0, 0)` |
| Rear camera | `(-0.17633, 0.00000, 0.30013)` | `(-0.61933, 0.00000, 0.30013)` | `(0, 0, 180)` |
| Front radar 1 | `(1.07087, -0.11005, 0.33378)` | `(0.62787, -0.11005, 0.33378)` | `(0, 0, 0)` |
| Front radar 2 | `(1.07087, 0.11005, 0.33378)` | `(0.62787, 0.11005, 0.33378)` | `(0, 0, 0)` |
| Left radar 1 | `(0.73488, 0.41005, 0.29013)` | `(0.29188, 0.41005, 0.29013)` | `(0, 0, 90)` |
| Left radar 2 | `(0.15966, 0.41005, 0.29013)` | `(-0.28334, 0.41005, 0.29013)` | `(0, 0, 90)` |
| Right radar 1 | `(0.73488, -0.41005, 0.29013)` | `(0.29188, -0.41005, 0.29013)` | `(0, 0, -90)` |
| Right radar 2 | `(0.15966, -0.41005, 0.29013)` | `(-0.28334, -0.41005, 0.29013)` | `(0, 0, -90)` |
| Rear radar | `(-0.17433, 0.00000, 0.33978)` | `(-0.61733, 0.00000, 0.33978)` | `(0, 0, 180)` |

The GNSS row converts the previous `0/0/0` placeholder mathematically. It is
not a physical antenna measurement. Before claiming lever-arm accuracy on the
robot, measure antenna X/Y/Z from `robot_center_link`, update both
`robot_params.yaml` copies, and repeat GNSS-to-final-pose validation.

> **Operational correction, 2026-08-06:** the NavSatFix reference was confirmed
> as the left antenna, `0.45 m` left of center. `gnss_link` therefore uses
> `(0.00000,+0.45000,0.00000)` and localization publishes
> `p_center = p_fix - R(yaw)[0,0.45]`. The earlier `(0,0,0)` center assumption is
> superseded. `pose_verified=false` remains until X/Z, the moving-base antenna,
> baseline direction, receiver reference, and moving residuals are accepted.

## Body And Safety Boundary

<!-- HH_260806 - Supersede the temporary reduced candidate with the
fabrication-inclusive measured total while retaining release checkpoints. -->
> **Operational follow-up, 2026-08-06:** the active physical extents are front
> `0.70837`, rear `0.68323`, left `0.53505`, and right `0.53495 m`, producing
> the fabrication-inclusive `1.39160 x 1.07000 m` body. A `0.05 m` margin on
> every side produces planning extents
> `0.75837/0.73323/0.58505/0.58495 m`, or `1.49160 x 1.17000 m`.
> Physical-body cost 100 is an unrecoverable hard stop; only margin-only contact
> can request projected bounded recovery. The intermediate
> `1.29160 x 0.87000 m` candidate and complete v2.1.3 values below are
> historical. Sensor housing and swept-clearance FIELD-PASS remains pending.

At the v2.1.3 release checkpoint, the physical body was
1.49160 x 1.07000 m and the planning margin was 0.10 m on every side, so the
planning boundary was 1.69160 x 1.27000 m.

| Boundary | Rear-axle values, old | Axle-midpoint values, new | Invariant |
|---|---:|---:|---:|
| Measured body X | front `1.20137`, rear `0.29023` | front `0.75837`, rear `0.73323` | sum `1.49160` |
| Measured body Y | left `0.53505`, right `0.53495` | unchanged | sum `1.07000` |
| Planning X | front `1.30137`, rear `0.39023` | front `0.85837`, rear `0.83323` | sum `1.69160` |
| Planning Y | left `0.63505`, right `0.63495` | unchanged | sum `1.27000` |

Nav2 local/global costmaps, the platform visualization boundary, and the
command safety gate use the same new planning polygon. Boundary thresholds and
cost-100 stop behavior are unchanged.

The measured body-box center is 0.01257 m ahead and 0.00005 m left of the axle
midpoint. The URDF visual/collision origin includes this small offset; it does
not redefine `robot_center_link`.

### Margin Geometry Comparison

The stored geometry sweeps compare the same 48 route samples. They answer the
question of whether reducing the planning margin by 0.05 m would make the
narrow route valid, without changing the deployed runtime configuration.

| Evaluated envelope | Center-pose failures | No tested lateral/yaw adjustment found |
|---|---:|---:|
| physical body, 1.4916 x 1.0700 m | 11 | 0 |
| body plus 0.05 m per side, 1.5916 x 1.1700 m | 20 | 11 |
| deployed planning boundary, 1.6916 x 1.2700 m | 26 | 20 |

In that v2.1.3 sweep, the 0.05 m margin improved geometric feasibility but
still left 11 sampled poses unresolved. It was diagnostic evidence rather than
a field acceptance result. Source files:
[physical body](assets/module-guides/control/evidence/v2.1.3-boundary-geometry/physical-body-envelope-sweep.json),
[0.05 m margin](assets/module-guides/control/evidence/v2.1.3-boundary-geometry/five-centimeter-margin-sweep.json),
and [planning footprint](assets/module-guides/control/evidence/v2.1.3-boundary-geometry/planning-footprint-envelope-sweep.json).

## Parking Distance Preservation

AprilTag parking measures longitudinal tag distance from its base frame. Moving
that frame forward would otherwise stop the rear of the robot 0.443 m too far
from the charger. Therefore only frame-dependent longitudinal thresholds add
0.443 m:

| Parameter | Old, rear axle | New, axle midpoint |
|---|---:|---:|
| `slowdown_start_distance_m` | 1.000 | 1.443 |
| `final_insertion_start_distance_m` | 0.550 | 0.993 |
| `parked_distance_from_tag_m` | 0.500 | 0.943 |
| `axis_full_trust_distance_m` | 0.300 | 0.743 |
| `axis_minimum_trust_distance_m` | 4.000 | 4.443 |

Retry travel distance, speed, heading/lateral tolerance, charging completion,
and parking state logic are unchanged.

## Runtime Consumers Changed

| Area | Changed reference |
|---|---|
| Platform | Ranger odometry child frame, status frame, dummy odometry, visualization pose |
| Localization | Real/sim EKF `base_link_frame`, wheel adapter, pose selector |
| Planning | Nav2 local/global/behavior base frame, goal and planning launch defaults |
| Sensing | LiDAR/radar/inflation cost-grid base frame and LiDAR static-TF parent |
| Control | Command gate base/footprint, reverse parking, AprilTag parking |
| System/UI/voice | Diagnostic TF probes, sensor mount metadata, and readiness TF target |
| Simulation | Fake odometry/sensor base and validation runner |
| Bringup | Canonical center and legacy rear-axle launch arguments plus mirrored configs |

Package-owned and `camrod_bringup/config` deployment copies must remain byte
identical where the existing sync contract applies. The automated
`test_robot_center_frame_contract.py` test locks the conversion values and
cross-package frame contract.

## Software Verification - 2026-08-04

- Xacro expansion and `check_urdf` passed. The root was
  `robot_center_link`, with `robot_base_link` and `sensor_kit_base_link` as its
  two children.
- Eleven changed packages built successfully: sensor kit, platform,
  localization, map, planning, sensing, control, system, perception, UI, and
  bringup. Their complete colcon test run passed.
- The center-frame contract suite passed 28 checks, including all converted
  sensors, diagnostic metadata, footprint invariants, parking distances, and
  package/bringup mirrors.
- Live simulation TF reported `robot_center_link -> robot_base_link` as
  `(-0.443000, 0, 0)` and `robot_center_link -> lidar_link` as
  `(0.763360, 0, 0.595380)`.
- Live parameters reported `robot_center_link` for EKF, Nav2 local costmap,
  command gate, and fake sensors. Nav2 and the gate both reported the
  `0.85837/0.83323/0.63505/0.63495 m` planning extents.
- The quick simulation validation passed baseline rates, all four directional
  cost stops, and manual navigation. Manual navigation moved 3.76 m, produced
  a 21-point global path, reached success, and allowed a maximum 0.25 m/s
  command.
- Configuration synchronization passed for package, bringup, and install
  copies.

## Simulation Visual Evidence

The following tracked artifacts use the same 1.69160 x 1.27000 m planning
boundary and `robot_center_link` geometry as the v2.1.3 runtime configuration.

![Center-frame boundary contact sheet](assets/module-guides/control/test-results/pre-owner-boundary-recovery-20260803/pre-owner-robot-center-contact-sheet.png)

![Center-frame route risk map](assets/module-guides/planning/test-results/pre-owner-boundary-feasibility-20260803/robot-center-narrow-route-risk-map.png)

[Open the pre-owner center-frame boundary recovery GIF](assets/module-guides/control/test-results/pre-owner-boundary-recovery-20260803/pre-owner-robot-center-recovery.gif).
The same recorded contact is summarized with the current contour in the
[margin-contact analysis](assets/module-guides/control/test-results/route-boundary-recovery-20260806/01-margin-contact-analysis.png).

The source measurements are preserved as
[geometry JSON](assets/module-guides/control/evidence/v2.1.3-boundary-geometry/robot-center-route-samples.json)
and [recovery timeline JSON](assets/module-guides/control/evidence/v2.1.3-boundary-recovery/pre-owner-robot-center-timeline.json).
This recovery GIF predates the automatic command owner and uses manually
injected candidates. Current owner behavior is recorded separately in the
[v2.1.3 boundary recovery validation](V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md).

The local host could not rebuild the complete `camrod_voice` package because
its existing system dependency `SDL2_mixer` is not installed. The changed
voice adapter Python source passed syntax validation; no Jetson-specific
dependency or runtime setting was changed to hide this host limitation.

## Field Acceptance

Before moving the physical robot:

```bash
timeout 3 ros2 run tf2_ros tf2_echo robot_center_link robot_base_link -p 6
timeout 3 ros2 run tf2_ros tf2_echo robot_center_link lidar_link -p 6
ros2 param get /localization/ekf_filter_node base_link_frame
ros2 param get /control/cmd_vel_safety_gate robot_base_frame
```

Expected center-to-rear translation is X `-0.443`, Y/Z `0`. Verify the complete
TF tree has one parent per frame, then repeat straight, left/right offset,
crab, zero-turn, lane-boundary, campsite, return, and docking tests. Simulation
verifies frame/state/algorithm integration but cannot replace physical GNSS
lever-arm, steering, charger-contact, and sensor mount measurements.
