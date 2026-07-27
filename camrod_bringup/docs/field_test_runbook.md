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
# HH_260727 - Confirm both role-specific GNSS ports before real bringup.
ls -l /dev/ttyACM0 \
  /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03DF8V-if00-port0
```

Expected:

- `field_test_tool.sh config` prints `config sync OK`.
- Every paired bringup/package config must match, and the full config trees must
  match their `install/<package>/share/<package>/config` copies. A source-only
  `OK` is not sufficient after changing deployment YAML.
- `/dev/ttyACM0` is the POWER+GPS heading rover used for NAV-PVT and
  NAV-RELPOSNED. The FTDI DN03DF8V by-id path is POWER+XBEE into the Lite
  moving base; its `/dev/ttyUSB*` assignment may change between boots.
- Both values come from the node-specific sections of
  `config/sensing/gnss/zed_f9p_rover.yaml`; launch device/baud arguments default
  to `__config__`.

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

For the front camera and YOLO path specifically:

```bash
ros2 run camrod_bringup field_test_tool.sh camera-yolo 12
```

`/perception/camera/yolo_image` is generated only while it has a subscriber.
With no RViz/CLI image subscriber it can be silent even though TensorRT inference
and `/perception/camera/detections_2d` are healthy.

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

## 6. Gate Commands

Close software gates:

```bash
ros2 run camrod_bringup field_test_tool.sh stop-gates
```

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
   - Stationary body returns below 0.30 m are filtered; LEFT2 uses 0.75 m for
     its measured 0.70-0.72 m multipath return. Test a real obstacle beyond the
     active threshold and confirm that it still stops the robot.

4. Perception-to-cost path
   - Put a vehicle/person in camera view.
   - Confirm perception publishes first.
   - Confirm compact object cost appears in LiDAR/perception cost grid.
   - Confirm the inflation grid sees it before the robot is close.

5. Camping site mission
   - Select a camping site in UI.
   - Confirm lanelet route to snapped entry.
   - Confirm crab entry from snapped lane position.
   - Confirm 180-degree dwell after entering the site.

6. Return mission
   - Press return in UI.
   - Confirm crab exits without replaying the 180-degree turn.
   - Confirm route returns to the drop-zone lanelet.
   - Confirm drop-zone reverse parking uses drop-zone yaw, not campsite crab yaw.

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
