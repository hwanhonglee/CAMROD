# Pose latency diagnosis

This note separates three different causes that can look like delayed vehicle
pose during driving:

1. the absolute GNSS correction cadence,
2. the EKF/selected localization output cadence and message age, and
3. system-wide CPU scheduling delay.

It is a measurement guide, not authorization to change the GNSS rate. The
production dual-antenna rate must not be changed without a stationary hardware
acceptance test followed by an open-sky field test.

## Active production rate configuration

The shared receiver YAML contains `rate: 10.0` in
`camrod_sensing/config/gnss/zed_f9p_rover.yaml`. That value applies to the
single-antenna profile only.

The normal bringup profile sets `ublox_dual_antenna: true` in
`camrod_bringup/config/bringup/launch_defaults.yaml`. Bringup forwards that
argument through `camrod_bringup/launch/_bringup_impl.py` and
`camrod_sensing/launch/sensing.launch.py`.

When dual-antenna mode is true,
`camrod_sensing/launch/gnss.launch.py::_dual_antenna_runtime_params()` creates
an inline parameter overlay containing:

```python
"rate": 1.0,
"nav_rate": 1,
```

The ublox node receives parameters in this order:

```python
parameters=[ublox_param_file, ublox_inline_params]
```

The inline overlay therefore wins over the YAML value. The active field
receiver cadence is 1 Hz, despite the 10 Hz value visible in the shared YAML.
The reason recorded in the launch file is that the field moving base emits
navigation epochs at 1 Hz and its rate must match the rover for RELPOSNED
moving-baseline heading.

Do not raise this value on the rover alone. A rate mismatch can make
moving-baseline epochs unavailable or unstable, lose fixed heading, or create
intermittent position/heading timing. A higher rate also increases serial,
DDS, adapter, and EKF input load. Any rate change must configure and verify
both GNSS roles together and must confirm NAV-PVT RTK state plus RELPOSNED
moving/fixed/heading-valid flags.

## Pose data flow and expected cadence

The production data flow is:

```text
/sensing/gnss/ublox_gps_node/fix                    (dual mode: 1 Hz)
  -> localization_input_adapter
  -> /sensing/gnss/pose_with_covariance_ros         (follows accepted GNSS)
  -> robot_localization EKF pose0 position + pose1 heading

/sensing/imu/data_ros                               (configured: 100 Hz)
/platform/status/odometry
  -> localization_input_adapter
  -> /localization/input/wheel_odometry_ros
  -> robot_localization EKF

robot_localization EKF                              (real: 15 Hz, sim: 20 Hz)
  -> /localization/primary/odometry_ros
  -> localization_input_adapter
  -> /localization/primary/pose_with_covariance
  -> localization_pose_selector
  -> /localization/pose and /localization/pose_ros
```

The adapter preserves the input headers while converting messages, and the
selector republishes the selected primary pose with that source header.
`predict_to_current_time: true` makes the EKF predict its output to the current
filter cycle.

Consequently, a healthy real robot should still publish the final localization
pose near the 15 Hz EKF rate. A 1 Hz GNSS fix is not by itself a reason for a
1 Hz `/localization/pose`. It does, however, mean that absolute lateral
position and GNSS heading corrections arrive only once per second; wheel and
IMU data predict between those updates. That correction cadence can contribute
to a visible staircase correction or lateral overshoot even while the final
pose topic remains healthy at 15 Hz.

## Clean sim measurement and selector correction on 2026-07-30

The final comparison used one isolated `bringup.launch.py sim:=true` process on
ROS domain 194. RViz, WebKit/UI, voice, physical cameras, YOLO, and physical
LiDAR acquisition were disabled; command gates stayed closed during the
latency probe. No second bringup or build ran in the measurement window.

The first clean run exposed one extra selector cycle: the input adapter
publishes the matching odometry before pose covariance. The selector reacted to
the new odometry stamp but copied the previous pose-covariance payload, then
marked that new stamp as published. When the matching pose covariance arrived,
the same stamp was already consumed. At the 20 Hz sim rate this added about
50 ms; at the former 10 Hz real filter rate it could add about 100 ms.
The v2.1.1 real profile now runs at 15 Hz, matching the controller, so the
remaining publication-period bound is approximately 67 ms before transport
and scheduling delay.

`localization_pose_selector_node` now chooses the freshest cached payload by
header stamp and reconstructs the matching pose/odometry pair before publishing.
It does not increase any sensor or filter rate. A regression test covers
odometry-first, pose-first, equal-stamp, and single-input callback orders.

The fresh post-fix 10-second probe measured:

| Stage | Configured | Measured | Header age p50 / p95 |
|---|---:|---:|---:|
| fake GNSS fix | 10 Hz | 9.998 Hz | 4.9 / 9.9 ms |
| generated GNSS pose | 10 Hz | 10.033 Hz | 8.6 / 13.3 ms |
| fake IMU | 10 Hz | 10.033 Hz | 9.0 / 14.0 ms |
| platform odometry | 10 Hz | 9.999 Hz | 10.4 / 15.0 ms |
| normalized wheel odometry | 10 Hz | 9.999 Hz | 11.5 / 16.4 ms |
| sim EKF output | 20 Hz | 20.016 Hz | 4.3 / 8.5 ms |
| adapter primary pose | 20 Hz | 20.015 Hz | 5.9 / 10.0 ms |
| final selected pose | 20 Hz | 20.000 Hz | 6.3 / 10.9 ms |

In the immediately preceding clean run, the adapter pose was already fresh
(6.1 / 13.7 ms), but the selected pose was 55.9 / 63.9 ms old. The correction
therefore removed roughly one complete 20 Hz cycle while preserving the final
20 Hz output rate. This proves the indoor selector delay was not caused by the
10 Hz simulated GNSS input.

The real dual-GNSS profile still requires the stationary/open-sky test below.
Its 1 Hz absolute correction cadence can affect moving lateral correction, but
it should no longer be compounded by a selector callback-order delay.

## Real non-motion diagnosis

Run the production-equivalent `bringup.launch.py sim:=false` with command gates
disabled, the platform unable to move, no active build, and only one bringup.
Wait for startup to settle, then measure all topics in the same 60-second
window:

```bash
ros2 run camrod_bringup field_test_tool.sh pose-latency 60
```

- `/sensing/gnss/ublox_gps_node/fix`
- `/sensing/gnss/pose_with_covariance_ros`
- `/sensing/imu/data_ros`
- `/platform/status/odometry`
- `/localization/input/wheel_odometry_ros`
- `/localization/primary/odometry_ros`
- `/localization/primary/pose_with_covariance`
- `/localization/pose_ros`

For each topic record receive rate, inter-arrival p50/p95/max, and
`now - header.stamp` p50/p95/max. Record total CPU and per-process CPU over the
same exact window. Keep RViz, WebKit, Brave, and development builds off for the
first baseline; repeat with each UI enabled separately only after the baseline.

Interpret the result as follows:

| Observation | Primary diagnosis |
|---|---|
| GNSS is near 1 Hz, while EKF and final pose are near 15 Hz with fresh stamps | Expected dual-GNSS cadence; no localization publication-rate failure |
| GNSS alone is stale or substantially below 1 Hz | Receiver/serial/timestamp path |
| GNSS, IMU, and wheel all lose rate together while total CPU is high | System scheduling load |
| Inputs are healthy, but EKF output is below 14 Hz or has growing stamp age | EKF scheduling/configuration |
| EKF output is healthy, but adapter or selected pose loses rate/adds age | Adapter/selector path |
| Final pose is healthy when stationary, but lateral correction overshoots only while moving | Dynamic fusion/controller-delay test still required |

For an initial pass, treat a real EKF/final-pose sustained rate below 14 Hz,
header-age p95 above 200 ms, or repeated age above 500 ms as evidence requiring
investigation. These are diagnostic screening limits, not final safety
acceptance thresholds. A stationary test cannot validate the phase delay
between lateral vehicle motion, GNSS correction, steering actuation, and
lanelet-boundary contact; that requires a supervised low-speed field run with
the same timestamps recorded in a rosbag.

## Controller decision after the 2026-07-31 field sample

The stationary field sample measured the real EKF, adapter, and final selected
pose at `14.99 Hz`, so increasing localization publication rate is not the
first correction. The selected-pose header-age p95 was `352.5 ms` under a
CPU-saturated session, which remains a scheduling concern and requires a
production-only profile, but it does not prove that pose cadence causes the
lateral sine-wave motion.

At the final field speed limit of `0.20 m/s`, velocity-scaled RPP computes only
`0.20 * 1.8 = 0.36 m` before applying its minimum lookahead. The minimum is
therefore the effective low-speed control parameter. HH_260801 synchronizes
the UI mission RPP and the manual-engage RotationShim internal RPP at a
`1.1 m` minimum preview. A full-bringup sweep used straight and S-curve
FollowPath inputs with lateral offsets. `1.0 m` corrected fastest but used more
steering variation, while `1.2 m` was smoother but retained more path error;
`1.1 m` was selected as the balance. Boundary-stop trials are excluded from
tracking scores. The `2.0 m` cap, controller frequency, pose rate, Ranger
`0.25 rad/s` steering slew, gain, footprint, and cost thresholds remain
unchanged.

Record `/platform/steering_transition_state` with `/actuator_state` during both
left- and right-offset runs. Repeated target-angle sign changes identify the
path tracker; a stable target with delayed limited angle identifies the Ranger
slew limiter; a stable limited angle with delayed actuator angles identifies
the physical steering/CAN layer. Motion-mode changes reveal a separate
Ackermann/spinning or parallel transition. Tune only the layer supported by
that evidence.
