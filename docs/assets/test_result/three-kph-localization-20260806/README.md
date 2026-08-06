# 3 km/h Command And Localization Smoke Test

<!-- HH_260806 - Record the 3 km/h active speed profile and bound the AMD64 localization claim. -->

This record verifies the active production speed ratios and one isolated
AMD64 kinematic drive. It does not accept moving localization on the physical
Jetson, dual-GNSS antennas, IMU, or Ranger wheel encoders.

![3 km/h command and pose result](three-kph-command-pose.png)

## Active Linear-Speed Contract

The final command gate retains `speed_scale=0.5`. Raw controller and maneuver
limits are therefore twice the final platform command.

| Operation | Cruise ratio | Raw limit | Final limit |
|---|---:|---:|---:|
| RPP cruise | `100%` | `1.666667 m/s` | `0.833333 m/s` (`3.000 km/h`) |
| RPP curvature floor | `50%` | `0.833333 m/s` | `1.500 km/h` |
| RPP final approach | `25%` | `0.416667 m/s` | `0.750 km/h` |
| Campsite crab | `60%` | `1.000000 m/s` | `1.800 km/h` |
| Campsite reverse entry | `40%` | `0.666667 m/s` | `1.200 km/h` |
| Drop-zone exit | `40%` | `0.666667 m/s` | `1.200 km/h` |
| Reverse parking | `40%` | `0.666667 m/s` | `1.200 km/h` |
| AprilTag approach | `50%` | `0.833333 m/s` | `1.500 km/h` |
| AprilTag insertion | `12.5%` | `0.208333 m/s` | `0.375 km/h` |
| Optional yaw-zone approach 1 | `62.5%` | `1.041667 m/s` | `1.875 km/h` |
| Optional yaw-zone approach 2 | `50%` | `0.833333 m/s` | `1.500 km/h` |
| Zero-turn / gross yaw alignment | `0%` | `0 m/s` | `0 km/h` |

Route-boundary recovery is intentionally excluded from proportional scaling.
Its raw `0.10 m/s` limit becomes `0.05 m/s` (`0.18 km/h`, `6%` of cruise)
after the gate because it is a separately bounded safety primitive. Angular
limits are also unchanged.

## Localization Contract

The field EKF fuses all of the following sources:

| Source | Fused values |
|---|---|
| GNSS | Absolute `x/y/z` plus independent dual-GNSS yaw |
| IMU | Roll/pitch orientation and angular velocity `x/y/z`; absolute IMU yaw is disabled |
| Wheel odometry | Body velocity `vx/vy` and yaw rate |

The current physical profile configures the EKF and Nav2 controller at `20 Hz`
while keeping dual-GNSS correction at `1 Hz`. Its lag history changes from
`0.3 s` to `1.0 s`, covering the previously observed 15 Hz stationary
selected-pose header-age maximum of `747.6 ms` so delayed measurements can be
rewound and replayed. This does not by itself correct wheel scale, IMU bias,
GNSS multipath, timestamp errors, or CPU starvation, and the 20 Hz physical
profile remains pending Jetson moving validation.

## AMD64 Result

| Check | Result |
|---|---:|
| Manual route | `PASS`, `11.74 m` displacement, Nav2 success |
| Final command maximum | `0.833333 m/s` (`3.000001 km/h`) |
| Raw RPP command maximum | `1.666667 m/s` (`6.000001 km/h`) |
| Selected pose | `757` samples, `20.024 Hz` |
| Pose gap p95 / max | `50.903 / 51.752 ms` |
| Pose header age p95 / max | `1.531 / 45.982 ms` |
| Pose step p95 / max | `4.197 / 6.485 cm` |
| Pose jumps over `20 cm` | `0` |

The test used the simulation EKF at `20 Hz` and fake GNSS/IMU/wheel inputs at
`10 Hz`. A second validation-runner invocation confirmed that disabling the
optional LiDAR cost grid satisfies the baseline diagnostic contract; its
manual-goal phase reused the already-completed goal and is not counted as a
second driving result.

## Reproduction Boundary

- Source commit before these uncommitted changes: `27e895fa6325ccc67c5ed058d0f1c2f990d8a08d`.
- Current user map SHA-256: `559bdad88e2733c854024013e02c2f29e4ad4aaef5bc6678e3c593d6c4e49516`.
- Raw bag path: `/tmp/camrod_three_kph_smoke_bag_20260806`.
- Raw DB3 SHA-256: `2caa1add8d474b89fc3e74b69012053d1e43a587c96d704f40b65caf33a7c807`.
- Structured values: [`result.json`](result.json).

Physical acceptance still requires staged open-space runs at `1`, `2`, and
`3 km/h` while recording the real `1 Hz` GNSS, IMU, wheel odometry, EKF,
selected pose, controller command, final gate command, and CPU scheduling.
