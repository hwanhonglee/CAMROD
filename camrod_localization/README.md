# camrod_localization

<!-- HH_260804 - Replace repeated architecture/state diagrams with one
source-and-measurement visual plus compact profile and validation tables. -->

GNSS, dual-GNSS heading, IMU, and Ranger wheel-velocity fusion for the
canonical `robot_center_link` pose and localization-health state.

![Localization pose generation](../docs/assets/module-guides/localization/pose-generation-and-timing.png)

## At A Glance

| Uses | Function | Main outputs |
|---|---|---|
| `robot_localization` EKF | Predicts a smooth 2D pose between sensor updates | `/localization/odometry`, `/localization/pose` |
| GNSS position/heading, IMU, wheel odometry | Fuses absolute and relative motion observations | EKF ROS boundary topics |
| Monitor, selector, map helper | Publishes mode, confidence, degraded state, and lanelet/drop-zone references | `/localization/mode`, `/localization/state`, `/localization/centerline_pose` |

## Profiles

| Item | Field | Simulation |
|---|---:|---:|
| EKF frequency | `15 Hz` | `20 Hz` |
| Base frame | `robot_center_link` | `robot_center_link` |
| Mode | 2D | 2D |
| TF | EKF publishes `odom -> robot_center_link` | Same |
| Prediction | `predict_to_current_time: true` | Same |
| Lag handling | `smooth_lagged_data: true` | Same |
| IMU yaw source | Physical heading/IMU policy | Map-consistent fake yaw allowed |

## Inputs And Outputs

| Direction | Topic | Role |
|---|---|---|
| Input | `/sensing/gnss/pose_with_covariance_ros` | Absolute map position and optional valid heading covariance |
| Input | `/sensing/imu/data_ros` | Orientation components and angular velocity |
| Input | `/platform/status/odometry` | Ranger wheel motion source |
| Internal | `/localization/input/wheel_odometry_ros` | EKF-ready wheel odometry |
| Output | `/localization/pose_ros` | ROS pose consumed by Nav2 and probes |
| Output | `/localization/pose` | Generated CAMROD pose contract |
| Output | `/localization/mode` | `NORMAL`, degraded/dead-reckoning, or invalid state |
| Output | `/localization/centerline_pose` | Lanelet-aligned planning reference |

## Measured Simulation Timing

| Stream | Measured rate | Header-age p95 |
|---|---:|---:|
| GNSS pose | `10.000 Hz` | `1.32 ms` |
| IMU | `10.000 Hz` | `1.28 ms` |
| Wheel odometry | `10.000 Hz` | `2.32 ms` |
| EKF odometry | `20.000 Hz` | `1.02 ms` |
| Selected pose | `20.000 Hz` | `1.83 ms` |

The 30-second stationary run proves topic continuity and prediction cadence.
It does **not** prove field position accuracy, multipath rejection, vibration
performance, GNSS antenna lever arm, or reduced driving oscillation.

## Reported Physical Stationary Performance

![Physical stationary field report](../docs/assets/module-guides/bringup/field-stationary-report-20260731.png)

| 60-second field metric | Result |
|---|---:|
| Final selected pose | `14.99 Hz` (15 Hz target met) |
| Selected-pose header age p50/p95/max | `35.6 / 352.5 / 747.6 ms` |
| GNSS-to-final XY p95 | `0.041 m` |
| GNSS-to-final yaw p95 | `0.74 deg` |
| Stationary GNSS yaw span | `2.532 deg` |

This run occurred under CPU saturation and did not exercise driving. Raw files
are referenced by the field report but are not committed; the result does not
accept moving latency, crab yaw, lateral overshoot, or antenna lever arm.

## Nodes

| Node | Responsibility |
|---|---|
| `localization_input_adapter` | Converts GNSS/platform inputs to EKF ROS boundaries |
| `ekf_filter` | Runs the selected `robot_localization` profile |
| `localization_output_adapter` | Converts EKF output to generated CAMROD messages |
| `pose_selector` | Publishes the selected primary pose source |
| `localization_monitor` | Evaluates freshness, innovations, confidence, and mode |
| `localization_map_helper` | Matches drop zones and publishes lanelet-aligned pose |
| `gnss_reattach` | Handles bounded recovery when GNSS returns after degradation |

## Active Health Values

| Check | Value |
|---|---:|
| GNSS timeout in monitor | `4.0 s` |
| IMU timeout | `1.0 s` |
| Wheel timeout | `1.0 s` |
| GNSS innovation WARN/FAIL | `3.0 / 6.0` |
| GNSS reattach timeout | `2.0 s` |

## Run And Validate

```bash
ros2 launch camrod_localization localization.launch.py

ros2 topic hz /localization/pose_ros
ros2 topic echo /localization/mode
ros2 topic echo /localization/state
ros2 run tf2_ros tf2_echo odom robot_center_link
```

| Config path | Purpose |
|---|---|
| `config/filter/ekf.yaml` | Field EKF profile |
| `config/filter/ekf_sim.yaml` | Simulation EKF profile |
| `config/filter/monitor.yaml` | Mode, timeout, innovation, and confidence policy |
| `config/filter/pose_selector.yaml` | Public pose selection |
| `config/reference/map_helper.yaml` | Lanelet/drop-zone matching |

Evidence JSON: [`pose-chain-sim-20260804.json`](../docs/evidence/module-guides/localization/pose-chain-sim-20260804.json).
