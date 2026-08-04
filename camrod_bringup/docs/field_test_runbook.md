# CAMROD Field Test Runbook

HH_260708 - Use this page before outdoor testing when the robot is available
but the exact failure point is not yet known. The goal is to collect the same
evidence every time, then change one thing at a time.

## 1. Preflight

From `/home/nvidia/camrod_ws/src`:

```bash
git status --short --branch
./colcon_build.sh
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
ros2 run camrod_bringup field_test_tool.sh config
test -x /home/nvidia/camrod_ws/src/camrod_bringup/scripts/camera_payload_probe.py
# HH_260727 - Confirm both role-specific GNSS ports before real bringup.
ls -l /dev/ttyACM0 \
  /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03DF8V-if00-port0
df -h /home/nvidia
```

Expected:

- `field_test_tool.sh config` prints `config sync OK`.
- The camera payload probe is executable. A missing mode bit makes
  `ros2 run ... camera_payload_probe.py` report `No executable found` even when
  the source file exists.
- Every paired bringup/package config must match, and the full config trees must
  match their `install/<package>/share/<package>/config` copies. A source-only
  `OK` is not sufficient after changing deployment YAML.
- `/dev/ttyACM0` is the POWER+GPS heading rover used for NAV-PVT and
  NAV-RELPOSNED. The FTDI DN03DF8V by-id path is POWER+XBEE into the Lite
  moving base; its `/dev/ttyUSB*` assignment may change between boots.
- Both values come from the node-specific sections of
  `config/sensing/gnss/zed_f9p_rover.yaml`; launch device/baud arguments default
  to `__config__`.
- Keep enough free storage for the selected rosbag duration. The 2026-07-30
  preflight had about 16 GB free and 12 GB of old `.ros` data; do not delete old
  evidence implicitly, but clear or archive it with operator approval before a
  long recording if free space is marginal.

## 2. Start Bringup With Logging

```bash
ros2 run camrod_bringup field_test_tool.sh launch rviz:=true
```

Equivalent manual launch:

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=false rviz:=true
```

The helper writes a timestamped bringup log under:

```text
$HOME/camrod_field_logs/YYYYMMDD_HHMMSS/bringup.log
```

## 3. Collect Baseline Snapshot

In a second terminal after bringup settles:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
ros2 run camrod_bringup field_test_tool.sh snapshot
```

This stores:

- git head and dirty state
- config sync result
- serial ports and `can0`
- ROS node/topic/service lists
- `/system/status` and diagnostics samples
- localization/planning/gate samples
- dual-GNSS NTRIP ownership, NAV-PVT, NAV-RELPOSNED, RXM-RTCM, and both
  serial-device roles
- CPU and memory snapshots
- short Hz samples for LiDAR, radar, cost grids, perception, paths, and cmd_vel

## 3A. Record TODO 11-13 On One Clock

<!-- HH_260729 / TODOLIST 11-13 - A normal snapshot does not preserve the
     complete route hold, action result, footprint/cost, and wheel timeline. -->

Start this in a separate terminal before creating a supervised lanelet-boundary
contact or centerline-alignment run:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
LOG_DIR="$HOME/camrod_field_logs/$(date +%Y%m%d_%H%M%S)_todo11_13"
ros2 run camrod_bringup field_test_tool.sh record-recovery "$LOG_DIR"
```

Keep the command running through the boundary trigger, blocked commands,
opposite escape, route clear, optional Nav2 reissue, operator-cancel check, and
both left/right steering runs. Press Ctrl+C only after all runs.

The directory contains:

- `route_recovery_bag`: route/action/goal, gate, full footprint, lanelet and
  obstacle grids, raw/final commands, platform/wheel/actuator feedback, TF, and
  structured `/platform/steering_transition_state` target/limited/scale data;
- `meta/recovery_parameters.txt`: the actual gate, goal-recovery, and Ranger
  values used by the running nodes;
- `meta/config_sync.txt`, `meta/git.txt`, requested, initially available,
  initially missing, and actually recorded topic lists plus `bag_info.txt`;
- `FIELD_RESULT.txt`: TODO 11, 12, and 13 PASS/FAIL fields.

The bag is necessary because each artifact answers a different question:
`bringup.log` explains decisions, gate status proves authorization, cost grids
and footprint prove geometry, raw/final commands prove where motion was
blocked, action/goal topics prove mission continuity, and actuator/wheel
feedback measures the physical steering delay. A commit and synchronized
configuration make the run reproducible, but do not by themselves constitute a
real-robot PASS.

The pre-fix Orin logs referenced by the v2.1.0 validation document explain why
the changes were needed, but they cannot pass these post-fix acceptance tests.
Likewise, a nominal simulation startup log proves node/config loading only; it
does not replace the real footprint, obstacle, CAN wheel, and latency evidence.

## 4. Live Watch

```bash
ros2 run camrod_bringup field_test_tool.sh watch
```

Use this while UI/RViz tests are running. It shows:

- system status
- localization mode
- planning state machine state
- planning engaged state
- platform e-stop state
- top CPU processes

## 5. Topic-Rate Probe

```bash
ros2 run camrod_bringup field_test_tool.sh hz 5
```

### 5.1 Localization rate and header-age probe

<!-- HH_260730 - One synchronized probe distinguishes a slow physical GNSS
input from EKF/selector delay and system-wide scheduling stalls. -->

With production bringup already running, all software engage gates closed, and
the platform physically unable to move, capture the complete real localization
chain in one 60-second window:

```bash
ros2 run camrod_bringup field_test_tool.sh pose-latency 60
```

The command prints a table and writes a JSON report under
`$HOME/camrod_field_logs/<timestamp>/pose_latency.json`. To select the report
path explicitly:

```bash
ros2 run camrod_bringup field_test_tool.sh pose-latency \
  60 /tmp/camrod_pose_latency.json
```

The eight rows cover physical GNSS, generated GNSS pose, IMU, platform wheel
odometry, normalized wheel input, EKF output, adapter output, and final selected
pose. Each row reports receive rate, inter-arrival p50/p95/max, and
`now - header.stamp` p50/p95/max. The JSON also stores unwrapped yaw change,
yaw span, XY displacement, velocity percentiles, and crab sample count for each
applicable stream. A second table pairs GNSS pose with EKF/adapter/selected pose
by header stamp and reports XY and yaw error p50/p95/max. XY pairing remains
available whenever the poses are valid. Yaw pairing is counted separately and
uses only orientations whose covariance is finite and no greater than the
production GNSS heading-validity ceiling (`100 rad²`). A GNSS placeholder such
as `yaw=0`, covariance `1e6` is shown as zero yaw pairs (`-/-/-`), not as a
localization yaw error.

<!-- HH_260803 - GNSS NavSatFix represents the antenna point while the final
pose represents robot_center_link. The current -0.443/0/0 mount is only the
converted old rear-axle 0/0/0 placeholder. Until the antenna mount is measured,
treat a stable XY difference as a possible lever-arm issue, not an EKF-rate issue. -->
A missing publisher, graph type mismatch, invalid/zero/future header, no valid
message, or only one valid message returns a non-zero exit code and is retained
in the JSON `errors` list.

Run this probe once with RViz/WebKit/Brave and development builds stopped.
Measure CPU over the same window with `field_test_tool.sh profile 60
pose_latency_baseline`. Interpret the chain using
`pose_latency_diagnosis.md`; do not raise the dual-GNSS 1 Hz runtime rate based
on this stationary test alone.

For the front camera and YOLO path specifically:

```bash
ros2 run camrod_bringup field_test_tool.sh camera-yolo 300
```

`/perception/camera/yolo_image` is generated only while it has a subscriber.
With no RViz/CLI image subscriber it can be silent even though TensorRT inference
and `/perception/camera/detections_2d` are healthy.

<!-- HH_260730 / TODOLIST 2 - Rate alone did not expose zero-length NvJPEG output. -->
The command now runs an independent payload probe for the same interval. It
requires exactly one compressed-image publisher and a non-dummy CameraInfo
shape, decodes every received JPEG with OpenCV, and reports minimum/maximum
payload bytes, decoded count, failures, shape mismatches, and dummy activity.
Any invalid payload makes the command fail even when `ros2 topic hz` looks
normal. The default duration is 300 seconds when the argument is omitted.

### CPU/UI comparison

Use one settled bringup per configuration and keep the same goal/path/cmd_vel
state. Do not compare a planning-idle run with an active-planning run.

```bash
# A: RViz + managed Chromium kiosk
ros2 run camrod_bringup field_test_tool.sh launch \
  rviz:=true enable_operator_ui_window:=true
ros2 run camrod_bringup field_test_tool.sh profile 300 rviz_chromium

# B: Chromium kiosk only
ros2 run camrod_bringup field_test_tool.sh launch \
  rviz:=false enable_operator_ui_window:=true
ros2 run camrod_bringup field_test_tool.sh profile 300 chromium_only

# C: no local visualization window; HTTP UI backend remains available
ros2 run camrod_bringup field_test_tool.sh launch \
  rviz:=false enable_operator_ui_window:=false
ros2 run camrod_bringup field_test_tool.sh profile 300 windows_off
```

Stop the previous bringup completely before starting the next configuration.
Each profile captures `tegrastats`, one-second `top` samples, before/after
process lists, memory/disk state, and the critical path/cost/camera rates over
the same five-minute interval. Close unrelated build jobs and duplicate debug
subscribers first; their CPU is not part of the robot runtime.

<!-- HH_260731 - The profile itself is a deliberate diagnostic stress load. -->
The profile starts multiple topic-rate subscribers, including large image
streams. Label its result as a diagnostic stress profile. For the
production-only comparison, close VS Code, Brave, Claude/Codex, duplicate
`ros2 topic echo`/`hz`, and other debug subscribers before each run. Acceptance
also requires zero SIGABRT/restart for required nodes; a lifecycle respawn does
not turn a `controller_server` crash into a PASS.

If a behavior is wrong, collect this immediately before changing parameters.
The important questions are:

- Is `/planning/cost_grid/inflation` fresh?
- Is `/sensing/cost_grid/lidar` fresh when LiDAR sees the obstacle?
- Is `/sensing/cost_grid/radar` fresh when each radar direction is blocked?
- Is `/perception/obstacles/fused_obstacles` publishing when the camera/LiDAR
  detector sees the object?
- Are `/planning/global_path` and `/planning/local_path` changing in the
  expected way?

<!-- HH_260722 - Add repeatable acceptance checks for the default two-port GNSS route. -->
### Dual-GNSS acceptance

The ordinary hardware bringup command must start the corrected moving-base
route without GNSS overrides. In a second terminal, check:

```bash
ros2 node list | grep /sensing/gnss/moving_base_rtcm_writer
ros2 topic info /sensing/gnss/ntrip_client/rtcm -v
ros2 topic echo /sensing/gnss/ublox_gps_node/navpvt --once
ros2 topic echo /sensing/gnss/navrelposned --once
ros2 topic echo /sensing/gnss/rxmrtcm --once
```

Acceptance requires:

- exactly one NTRIP publisher and one subscriber, owned by
  `moving_base_rtcm_writer`;
- NAV-PVT carrier solution fixed (`(flags & 0xC0) == 0x80`) with
  centimeter-scale `hAcc`;
- NAV-RELPOSNED moving baseline, relative position valid, heading valid, and
  carrier solution fixed (normally decimal flags `311`), with a stable physical
  antenna baseline.

Do not enable `ublox_dual_warm_start_on_startup` during normal operation. Use
it for one launch only after an explicit direct-rover/one-port diagnostic leaves
the rover tracking the wrong reference, then restart with the default `false`.

<!-- HH_260731 - A single good accuracy field does not prove carrier Fixed. -->
Do not accept a single `--once` sample or centimeter-scale `hAcc` alone.
Capture the flags distribution for at least 120 seconds immediately after a
bringup restart. Final field acceptance requires NAV-PVT flags 131 and
NAV-RELPOSNED flags 311 to remain fixed for 10 minutes without flags 11 or
float fallback.

## 6. Gate Commands

Close software gates:

```bash
ros2 run camrod_bringup field_test_tool.sh stop-gates
```

`stop-gates` uses the stack-native `avg_msgs/msg/AvgBool` type, attempts all
three manual/mission/platform close commands even if one owner is missing, and
then requires both `/control/planning_engaged=false` and
`/control/command_enabled=false`. A failure means the physical e-stop must
remain engaged; it is not reported as a successful software stop.

Open software gates only with explicit supervision:

```bash
ros2 run camrod_bringup field_test_tool.sh enable-gates --allow-motion
```

The helper does not replace the physical e-stop.

## 7. Scenario Checklist

Run these in order and take a `snapshot` after any failure.

1. Manual lane goal
   - Send a reachable goal in RViz or UI.
   - Confirm `/planning/global_path` is lanelet-centered.
   - Confirm `/planning/local_path` updates near the robot.
   <!-- HH_260720 - Check the single control gate and Ranger boundary outputs. -->
   - Confirm `/control/cmd_vel_raw`, `/control/cmd_vel`, and `/control/cmd_vel_ros`
     are consistent with gate state.

2. Front obstacle stop
   - Place an obstacle on the active path.
   - Confirm cost appears in `/sensing/cost_grid/lidar` and/or
     `/sensing/cost_grid/radar`.
   - Confirm `/planning/cost_grid/inflation` updates.
   - Confirm `cmd_vel_safety_gate` reports the stop source.

3. Side and rear radar stop
   - Test left, right, and rear separately.
   - Confirm the correct radar topic updates.
   - Confirm the cost-grid side matches the physical side.
   <!-- HH_260728 - Verify narrow self-return notches without recreating the
        old one-sided LEFT2 blind zone. -->
   - With the area clear, confirm stationary readings inside the configured
     fixed-return bands do not create radar cost.
   - Place an obstacle at approximately 0.50 m from LEFT2. It is below the old
     0.75 m floor but outside the new notches, so it must create cost and stop.
   - Move the obstacle through several distances on each channel. Values outside
     the configured narrow bands must remain obstacles; never widen a band from
     a single startup sample containing a wall or person.
   - HH_260729 - The active field profile has
     `startup_return_learning_enable: false`; normal bringup must not report a
     newly learned exclusion. For a supervised calibration only, clear every
     sensor area, keep the robot stationary/disengaged, enable the parameter,
     and restart. Each healthy channel then collects for 8 s from its own first
     valid sample; a channel that first appears at/after the 15 s deadline is
     rejected. Restore the parameter to false before driving.
   - During that supervised calibration, confirm
     `/control/planning_engaged=false` is received before collection. If either
     manual or mission engage becomes true, calibration must report
     cancellation and retain fail-safe obstacle costs.
   - Force or observe two simultaneous sensor warnings and confirm `[SYSTEM]`
     prints both lines independently with `component`, logical `location`, TF
     `frame`, mount pose, and live range/rate values. A STALE transition must
     retain the same identity fields.
   - During normal forward travel, the raw side-near probe extends 0.60 m from
     the canonical robot base frame; it is not an extra clearance measured from the body
     edge. Crab/reverse maneuver checking remains 1.20 m.
   <!-- HH_260728 - Include radar cost inflation in the straight-side regression. -->
   - With `obstacle_radius_m: 0.30` and the 0.10 m radar grid, confirm a
     base-centred side hit near `|y|=1.0 m` does not stop straight travel, while
     a closer hit near `|y|=0.8 m` does stop it. The farther hit must still stop
     a crab command toward that side.
   <!-- HH_260728 - Reproduce and prevent stop-induced command-direction latch release. -->
   - With a radar obstacle on the commanded path, confirm the gate remains
     latched after the upstream command becomes zero or changes direction.
     Remove the obstacle and confirm release only after 2 s of continuously
     fresh clear radar/merged-grid evidence, followed by the 1 s stop hold.
   - Restart with `enable_radar:=false`. Confirm there is no
     `sen0592_radar_node`, all seven checkers say `DUMMY DATA`, and every dummy
     range is `max_range + 0.001 m`.
   - Confirm the radar cost-grid log names all seven channels under the
     dummy-state barrier, `/sensing/radar/obstacle_evidence` remains exactly
     `clear`, and no radar source appears in a cmd_vel stop reason. A visible
     LEFT2 Range marker or DUMMY diagnostic is transport status, not an
     obstacle.
   - Save the effective `sensor_enabled` array and startup log before calling a
     radar-ON run a seven-channel test. One DUMMY channel means it is not a
     seven-channel physical acceptance run.
   - A return from a channel with a known nearby object is not a self echo merely
     because it falls inside a fixed-return exclusion. Remove the object and
     repeat, then place a test obstacle at the same distance to prove the
     exclusion does not hide it.
   - Radar-OFF acceptance requires one 600-second summary containing every
     range count/value, channel/global dummy state, evidence clear/active count,
     radar-grid high-cost cells, gate status, and `/rosout` radar cost-stop
     count. Repeated DUMMY diagnostic text alone is insufficient.

4. Perception-to-cost path
   - Put a vehicle/person in camera view.
   - Confirm perception publishes first.
   - Confirm compact object cost appears in LiDAR/perception cost grid.
   - Confirm the inflation grid sees it before the robot is close.
   - With the component path enabled, confirm exactly one front image publisher
     and no front-camera dummy publisher. The front camera and YOLO must remain
     alive for at least 5 minutes while
     `/perception/camera/detections_2d` publishes.
   - Confirm `/dev/video1` rear raw image is near 10 Hz and the compressed
     monitoring stream is near 2 Hz. A low-rate warning must say `FPS low`;
     it must not be mislabeled as an encoding mismatch.
   - Measure rear raw/compressed/camera_info together with a low-overhead
     counter before decoding full payloads. If raw is already slow, separate
     camera capture FPS, CPU scheduling, and JPEG compression cost before
     changing the diagnostic threshold.

5. Raw lanelet footprint boundary
   - Run once with `enable_radar:=false` in a verified clear route corridor.
   - If `lanelet_footprint_cost` appears, capture the pose, planning-boundary
     polygon, and exact raw lanelet grid cell before changing any threshold.
   - Confirm every point of `/platform/robot/planning_boundary` stays inside
     the allowed map corridor. Do not disable the whole-footprint guard merely
     to make the test move.
   <!-- HH_260729 - Validate bounded route recovery and retained-goal reissue. -->
   - Start `field_test_tool.sh record-recovery <log_dir>` first and keep the
     generated `FIELD_RESULT.txt` beside the bag.
   - During an active UI mission, create a supervised boundary contact and
     confirm `/control/command_enabled=false` plus
     `operating_state=ROUTE_SAFETY_HOLD`. Save the trigger reason/vector.
   - Keep the pose outside or the lanelet grid stale. The hold must not clear,
     and zero/rotation/same-direction commands must remain zero at
     `/control/cmd_vel_ros`.
   - Command a slow opposite translation. It may pass only when the projected
     full footprint 0.25 m ahead is clear. Place a rear obstacle in that escape
     corridor and confirm the command returns to zero with the dynamic source
     in the stop reason.
   - Re-enter the safe corridor and keep pose/grid fresh. Require 1.0 s
     continuous clear, then `ENABLED`. If Nav2 had reported `ABORTED`, require
     the same goal/source to be reissued after 0.5 s without site reselection.
     Verify no more than two reissues and no automatic restart after operator
     cancel.

6. Camping site mission
   - Select a camping site in UI.
   - Confirm lanelet route to snapped entry.
   - Confirm crab entry from snapped lane position.
   - Confirm 180-degree dwell after entering the site.

7. Return mission
   - Press return in UI.
   - Confirm crab exits without replaying the 180-degree turn.
   - Confirm route returns to the drop-zone lanelet.
   - Confirm drop-zone reverse parking uses drop-zone yaw, not campsite crab yaw.

8. Steering transition and lateral overshoot
   - Start on both left and right of the centerline with small yaw error.
   - Record localization stamp, centerline error, raw/final cmd_vel, Ranger
     `/platform/steering_transition_state`, `/actuator_state`, wheel feedback,
     and boundary distance on one clock. Use target sign changes to identify
     controller oscillation, limited angle for driver slew, and actuator angle
     for physical wheel lag.
   - During longitudinal-to-lateral and lateral-to-longitudinal changes, require
     translation scale 0 above 0.35 rad steering lag, linear recovery through
     the interval, and full speed only at or below 0.05 rad.
   - Confirm the robot converges from both sides without crossing repeatedly or
     touching lanelet cost 100. Do not tune prediction, gain, hysteresis, or
     footprint thresholds until the measured delay for each pipeline segment is
     attached to the field report.

## 8. What To Change First

Change only one layer at a time:

- Wrong path shape: check route planner, goal snapper, replan monitor, planner selector.
- Path is good but robot stops: check inflation grid freshness and cmd_vel gate source.
- Perception sees object but cost does not: check perception marker/object input age,
  radius limits, and LiDAR cost-grid subscriptions.
- Radar side mismatch: check radar launch order, TF link, and radar cost-grid inputs.
- GNSS recovery holds: check `/localization/mode`, GNSS topic freshness, covariance,
  and internet/NTRIP status before changing offsets.
- GNSS publishes but mode is `DR_ONLY`: check XY covariance trace first. The
  current monitor requires `covariance[0] + covariance[7] <= 1.0`, GNSS rate
  >= 0.8 Hz, jump <= 1.0 m, and age <= 4.0 s. A live `/fix` alone is not enough.
- High CPU: take `snapshot` first, then compare after changing component container,
  DDS/QoS, marker rate, or debug image settings.
- High CPU with many debug terminals: close duplicate `ros2 topic echo`/`hz`
  processes before evaluating YOLO, cost-grid, or planning rates.

## 9. End Of Test

Before leaving the field:

```bash
ros2 run camrod_bringup field_test_tool.sh stop-gates
ros2 run camrod_bringup field_test_tool.sh snapshot
```

Keep the latest `$HOME/camrod_field_logs/...` directory with the issue report.
