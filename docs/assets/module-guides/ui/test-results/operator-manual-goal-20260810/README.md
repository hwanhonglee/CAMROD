# Operator-map manual Goal Pose - 2026-08-10

<!-- HH_260810 - Preserve the browser and ROS integration result separately
from physical touchscreen, steering, and clearance acceptance. -->

Evidence class: `MEASURED AMD64 ROS SIM`. Field claim: `false`.

![Confirmed operator-map goal](../../evidence/ui-captures/operator-manual-goal-20260810.png)

The capture records a full-map draft at `(124.535, -23.426, 12.3 deg)` so the
remote selection range and confirmation controls are visible together. The ROS
measurements below are from the subsequent reachable browser-selected goal.

## Scope

This test checks that normal bringup starts without RViz and that the existing
Robot UI can replace the routine RViz `2D Goal Pose` interaction. It does not
validate physical steering, map survey accuracy, obstacle clearance, GNSS
alignment, touchscreen calibration, or Jetson resource use.

## Launch

```bash
ROS_DOMAIN_ID=87 ros2 launch camrod_bringup bringup.launch.py \
  sim:=true \
  clean_before_launch:=false \
  clean_on_shutdown:=false \
  enable_operator_ui_window:=false \
  enable_guest_ui:=false \
  api_ui_port:=18110
```

The `rviz` argument was intentionally omitted. The resolved default was false,
and no RViz process or node appeared in the launch graph.

## Result

| Check | Observed result |
|---|---|
| Precondition | UI ready, System OK, mission READY, battery 80% |
| Browser interaction | Real pointer events selected x/y/yaw; real departure-button event confirmed dispatch |
| Browser layout | 1600x857; all 357 map polylines visible in goal mode; horizontal/vertical overflow 0 |
| Raw goal | `/goal_pose`, frame `map`, `(-18.275650, 43.558984, -15.039 deg)` |
| Authorization | `/platform/drive_enable=true`, `/planning/engage=true` |
| Planning mode | source `manual`, mission `DRIVING`, goal snapper source `manual_ros` |
| Paths | Global 500 bounded/1024 raw points; local 154 bounded/raw points |
| Motion profile | UI reported 2.0 km/h active profile |
| Completion | Goal snapper pose-distance hold, controller `Reached the goal`, BT `Goal succeeded`, System OK |
| Stop | `POST /ui/stop` succeeded and released Nav2/local owners |
| Shutdown | Parent SIGINT ended the isolated launch cleanly |

The backend unit contract additionally covers finite-value validation,
normalized yaw, readiness rejection, active-service rejection, battery
rejection, and goal-before-engage publication order. The React source contract
covers pointer selection, confirmation, API dispatch, and stable goal-control
dimensions.

Machine-readable values are in
[`integration-summary.json`](integration-summary.json).

## Remaining Acceptance

- Repeat on the ARM64 8-core/16-GB Jetson with the production WebKit window.
- Measure touchscreen map-coordinate error against a surveyed pose.
- Verify active campsite, return, parking, charging, and SOC-below-35% rejects.
- Verify physical final stop, steering response, and operator-stop latency.
