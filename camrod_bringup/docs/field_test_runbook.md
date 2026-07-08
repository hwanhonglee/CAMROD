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
```

Expected:

- `field_test_tool.sh config` prints `config sync OK`.
- Platform config must match in all four places:
  - `camrod_bringup/config/platform`
  - `camrod_platform/config`
  - `install/camrod_bringup/.../config/platform`
  - `install/camrod_platform/.../config`

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

If a behavior is wrong, collect this immediately before changing parameters.
The important questions are:

- Is `/planning/cost_grid/inflation` fresh?
- Is `/sensing/cost_grid/lidar` fresh when LiDAR sees the obstacle?
- Is `/sensing/cost_grid/radar` fresh when each radar direction is blocked?
- Is `/perception/obstacles/fused_obstacles` publishing when the camera/LiDAR
  detector sees the object?
- Are `/planning/global_path` and `/planning/local_path` changing in the
  expected way?

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
   - Confirm `/planning/cmd_vel_raw`, `/planning/cmd_vel`, and `/platform/cmd_vel`
     are consistent with gate state.

2. Front obstacle stop
   - Place an obstacle on the active path.
   - Confirm cost appears in `/sensing/cost_grid/lidar` and/or
     `/sensing/cost_grid/radar`.
   - Confirm `/planning/cost_grid/inflation` updates.
   - Confirm `planning_cmd_vel_gate` reports the stop source.

3. Side and rear radar stop
   - Test left, right, and rear separately.
   - Confirm the correct radar topic updates.
   - Confirm the cost-grid side matches the physical side.
   - Right self-echo WARN can remain visible, but true external cost must still
     stop the robot.

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
- High CPU: take `snapshot` first, then compare after changing component container,
  DDS/QoS, marker rate, or debug image settings.

## 9. End Of Test

Before leaving the field:

```bash
ros2 run camrod_bringup field_test_tool.sh stop-gates
ros2 run camrod_bringup field_test_tool.sh snapshot
```

Keep the latest `$HOME/camrod_field_logs/...` directory with the issue report.
