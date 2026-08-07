# Pose latency diagnosis

<!-- HH_260807 - Dual GNSS is configured at 5 Hz; preserve old 1 Hz measurements as history, not current acceptance. -->

This note separates three different causes that can look like delayed vehicle
pose during driving:

1. the absolute GNSS correction cadence,
2. the EKF/selected localization output cadence and message age, and
3. system-wide CPU scheduling delay.

It is a measurement guide. HH_260807 changes the configured production
dual-antenna rate to 5 Hz to match the moving base; that configuration is not
accepted until a stationary hardware test and an open-sky field test pass.

## Active production rate configuration

The shared receiver YAML contains `rate: 5.0` in
`camrod_sensing/config/gnss/zed_f9p_rover.yaml`. Both single-antenna operation
and the dual-mode inline overlay now use this rate.

The normal bringup profile sets `ublox_dual_antenna: true` in
`camrod_bringup/config/bringup/launch_defaults.yaml`. Bringup forwards that
argument through `camrod_bringup/launch/_bringup_impl.py` and
`camrod_sensing/launch/sensing.launch.py`.

When dual-antenna mode is true,
`camrod_sensing/launch/gnss.launch.py::_dual_antenna_runtime_params()` creates
an inline parameter overlay containing:

```python
"rate": 5.0,
"nav_rate": 1,
```

The ublox node receives parameters in this order:

```python
parameters=[ublox_param_file, ublox_inline_params]
```

The inline overlay therefore remains authoritative. It now writes a 200 ms
measurement period to rover RAM, matching the configured moving-base epochs.
`nav_rate: 1` means one navigation solution per measurement cycle, not 1 Hz.

Do not change this value on the rover alone. A rate mismatch can make
moving-baseline epochs unavailable or unstable, lose fixed heading, or create
intermittent position/heading timing. A higher rate also increases serial,
DDS, adapter, and EKF input load. Any rate change must configure and verify
both GNSS roles together and must confirm NAV-PVT RTK state plus RELPOSNED
moving/fixed/heading-valid flags.

## Pose data flow and expected cadence

The production data flow is:

```text
/sensing/gnss/ublox_gps_node/fix                    (dual mode target: 5 Hz)
  -> localization_input_adapter
  -> /sensing/gnss/pose_with_covariance_ros         (follows accepted GNSS)
  -> robot_localization EKF pose0 position + pose1 heading

/sensing/imu/data_ros                               (configured: 100 Hz)
/platform/status/odometry
  -> localization_input_adapter
  -> /localization/input/wheel_odometry_ros
  -> robot_localization EKF

robot_localization EKF                              (real: 20 Hz, sim: 20 Hz)
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

Consequently, a healthy real robot should publish accepted GNSS poses near
5 Hz and the final localization pose near the 20 Hz EKF rate. Wheel and IMU
data continue to predict between the 200 ms absolute position/heading updates.
A visible correction staircase at this cadence requires checking epoch loss,
header age, lever-arm timing, and fusion behavior rather than accepting 1 Hz.

With `two_d_mode=true`, the real EKF fuses absolute GNSS `x/y` and dual-GNSS
yaw, IMU yaw rate, and wheel-odometry body `vx/vy` plus yaw rate. The filter
clamps `z/roll/pitch`, `vz`, roll/pitch rates, and `az`. Absolute IMU yaw is disabled.
The adapter prefers
`/platform/status/odometry` and falls back to `/rmp401/odom` after `0.7 s`.

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
The v2.1.1 real profile ran at 15 Hz, matching its controller, so its remaining
publication-period bound was approximately 67 ms before transport and
scheduling delay. HH_260806 synchronizes both real loops at 20 Hz, reducing
the configured period to 50 ms; this remains pending moving Jetson acceptance.

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
Its configured 5 Hz cadence is not accepted until the physical link shows
aligned 200 ms epochs without loss or invalid heading flags.

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
| GNSS is near 5 Hz, while EKF and final pose are near 20 Hz with fresh stamps | Expected configured cadence; continue RTK/heading and epoch-alignment checks |
| GNSS alone is stale or substantially below 4 Hz | Receiver/serial/timestamp/link-bandwidth path |
| GNSS, IMU, and wheel all lose rate together while total CPU is high | System scheduling load |
| Inputs are healthy, but EKF output is below 18 Hz or has growing stamp age | EKF scheduling/configuration |
| EKF output is healthy, but adapter or selected pose loses rate/adds age | Adapter/selector path |
| Final pose is healthy when stationary, but lateral correction overshoots only while moving | Dynamic fusion/controller-delay test still required |

For an initial pass, treat a real EKF/final-pose sustained rate below 18 Hz,
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

The HH_260801 controller sweep used the former final `0.20 m/s` field baseline.
At that speed, velocity-scaled RPP computed only `0.20 * 1.8 = 0.36 m` before
the minimum lookahead, so the minimum was the effective parameter. The UI
mission RPP and manual-engage RotationShim internal RPP remain synchronized at
the selected `1.1 m` minimum preview: `1.0 m` corrected fastest but used more
steering variation, while `1.2 m` was smoother but retained more path error.

The active final cruise reference is now `2.0 km/h` (`0.555556 m/s`) while
preserving operational linear-speed ratios. At this speed, the vehicle travels
approximately `2.78 cm` per `20 Hz` EKF cycle and `0.1111 m` between configured
`5 Hz` GNSS corrections. The prior stationary header-age p95/max of
`352.5/747.6 ms` corresponds to `0.196/0.415 m` of travel,
so delayed absolute correction can become visually significant. HH_260806
synchronizes the real EKF and controller at `20 Hz`; `smooth_lagged_data`
history increases from `0.3 s` to `1.0 s` so the filter can rewind/replay the
measured maximum delay. The prior 15 Hz stationary result does not accept this
new cadence under moving Jetson load.

The local-path extractor and tracking-error fallback heartbeats now also use
`20 Hz`. Both nodes publish immediately on pose/path callbacks, so the change
mainly removes a 15 Hz fallback/default mismatch. The active process-noise
matrix is not rescaled: the bundled `robot_localization` EKF already multiplies
it by elapsed time, so the per-second model noise remains consistent at 20 Hz.

The earlier AMD64 12 m kinematic smoke run reached a final command of
`3.000001 km/h`, maintained selected pose at `20.024 Hz`, measured a maximum
pose step of `6.485 cm`, and found zero steps over `20 cm`. Its fake GNSS is
`10 Hz` and sim EKF is `20 Hz`, so its input cadence does not reproduce or
accept the physical 5 Hz moving-base link/20 Hz EKF/Jetson case. Boundary-stop trials remain excluded from
tracking scores. The `2.0 m` lookahead cap, Ranger `0.25 rad/s` steering slew,
gains, footprint, costs, and angular-speed limits remain unchanged.

Record `/platform/steering_transition_state` with `/actuator_state` during both
left- and right-offset runs. Repeated target-angle sign changes identify the
path tracker; a stable target with delayed limited angle identifies the Ranger
slew limiter; a stable limited angle with delayed actuator angles identifies
the physical steering/CAN layer. Motion-mode changes reveal a separate
Ackermann/spinning or parallel transition. Tune only the layer supported by
that evidence.
