# CAMROD v2.0.7 Release Notes

<!-- HH_260723 - Record the localization, routing, perception, and occupied-campsite field release. -->
<!-- HH_260724 - Record the battery-aware campsite mission and charging-release policy. -->

Release date: 2026-07-24
Initial field baseline documented: 2026-07-23
Target branch: `develop`
Target remote: `hwanhonglee/CAMROD`

## Scope

- Restored contiguous robot_localization pose inputs so GNSS position and
  moving-baseline heading both reach the EKF.
- Reduced Lanelet centerline lookup work, kept throttled outputs continuous,
  and made the diagnostic compare snapped output with its live localization
  input instead of blaming system-wide CPU slowdown on snapping.
- Restricted reverse Lanelet routing to an explicit campsite-return request;
  ordinary operator goals always retain the normal one-way shortest path.
- Isolated child-launch configurations so the sensing camera ownership flag
  cannot disable the following perception include.
- Verified the front camera and TensorRT YOLO path and preserved semantic class
  labels through 2D and 3D detections.
- Added tent-based campsite occupancy, control-side entry rejection, and UI
  selection blocking for occupied sites.
- Changed the compatibility LiDAR cost-grid rasterizer to consume
  `/perception/obstacles` by default. Raw filtered LiDAR inputs remain available
  through `raw_lidar_cost_enabled`, while radar cost remains unchanged.
- Synchronized the active dual-GNSS field ports (`/dev/ttyACM0` rover output,
  `/dev/ttyUSB4` moving-base correction input), selected `JECH-RTCM32` in both
  NTRIP configuration mirrors, and provided a live GNSS status GUI for NAV-PVT,
  NAV-COV, NAV-RELPOSNED, and NavSatFix inspection.

## Localization And Diagnostics

<!-- HH_260723 - Explain why the old Lanelet warning did not prove bad GNSS coordinates. -->

`/localization/centerline_pose` is a map-helper output derived from the selected
EKF pose; it is not the source used to create `/localization/pose`. The old
Lanelet checker compared this helper output with a fixed 10 Hz target. During
the recorded warning, the system diagnostic also reported 100% CPU, so an
upstream scheduling slowdown could be mislabeled as “intermittent snapping
failure.”

The map helper now republishes its last valid snap when a nearest-lanelet search
is intentionally throttled. The checker also subscribes to
`/localization/pose`, evaluates output against `min(10 Hz, live input rate)`,
and reports a rate warning only when the helper drops input poses. EKF rate and
system CPU pressure remain visible through their owning diagnostics.

GNSS position and heading are fused as contiguous `pose0` and `pose1` inputs.
This matters because robot_localization stops discovering numbered pose inputs
at the first missing index.

## Route Direction

<!-- HH_260723 - Preserve the explicit authorization boundary for reverse campsite return. -->

Vehicle yaw alone no longer reverses an ordinary route. A reverse route is
authorized only by the typed campsite return request, consumed once for the
matching goal and latched only for replans of that same goal. This preserves the
same-lane return behavior without making unrelated destinations appear
backward.

## Camera, YOLO, And Occupancy

<!-- HH_260723 - Record the fixed launch ownership and semantic occupancy chain. -->

Each child launch now runs in a scoped launch-configuration group. The sensing
include may therefore receive `enable_front_camera=false` when the composable
camera+YOLO container owns `/dev/video0` without leaking that value into
perception and disabling `obstacle_fusion_node`.

The verified pipeline is:

```text
/dev/video0
  -> /sensing/camera/econ_front/image_rect/compressed
  -> yolov9mit
  -> /perception/camera/detections_2d
  -> obstacle_fusion_node
  -> /perception/obstacles
  -> /perception/camera_lidar/detections_3d
  -> campsite_occupancy_node
  -> /perception/camping_sites/occupancy
```

YOLO publishes the configured semantic label, including `tent`, instead of only
the TensorRT class-array index. A campsite becomes occupied after three
qualifying tent hits within two seconds and remains occupied for the configured
field-session hold. Control refuses entry or aborts an approach that becomes
occupied, and the backend/frontend reject or disable duplicate site selection.

## Dynamic Cost Profile

<!-- HH_260723 - Document the reversible raw-LiDAR cost switch. -->

The compatibility grid on `/sensing/cost_grid/lidar` now rasterizes
`/perception/obstacles` and perception markers. Direct
`/sensing/lidar/points_filtered` and legacy filtered-cloud inputs are excluded
while `raw_lidar_cost_enabled: false`. Set the switch to `true` and restart the
node to restore those raw inputs. Radar continues to publish and merge through
its independent grid.

## Localization, Perception, And Routing Verification

<!-- HH_260723 - Record the release checks performed before tagging. -->

- Camera input measured approximately 10.0 Hz in the integrated bringup path.
- `/perception/camera/detections_2d` measured approximately 3.9 Hz with the
  configured 5 Hz inference throttle.
- `obstacle_fusion_node`, `campsite_occupancy_node`, and the
  `/perception/obstacles` publisher were present in the integrated graph.
- A synthetic tent Detection3D inside a configured campsite produced the
  corresponding occupied mission key.
- Package builds, unit/lint tests, configuration mirror checks, Python syntax,
  frontend production build, and `git diff --check` are required to pass in the
  final release commit.

## Localization, Perception, And Routing Known Limits

<!-- HH_260723 - Separate verified software behavior from remaining field observations. -->

- A published YOLO detection array may legitimately contain no objects when the
  scene has no class above `min_confidence: 0.5`; topic rate and publisher
  presence are the health checks.
- `/perception/obstacles` requires live filtered LiDAR input as well as the
  camera/detection pipeline.
- CPU saturation remains a separate system fault. The Lanelet checker no longer
  misattributes a matching upstream slowdown, but sustained 100% CPU can still
  delay sensing, planning, and localization callbacks.
- The raw-LiDAR cost switch is read at node startup and requires a restart after
  changing its value.

## Battery And Operator Policy Scope

<!-- HH_260724 - Preserve the battery, manual-driving, and operator-stop follow-up scope. -->

- Kept the v2.0.6 localization, routing, perception, and campsite-occupancy
  baseline as the starting point.
- Raised the hard critical-SOC command stop to 20% in `cmd_vel_safety_gate`.
- Added a 35% SOC gate for new campsite dispatch from the UI backend.
- Added the same 35% SOC gate to charging departure, so a parked/charging robot
  cannot leave the station for a new `camping_site_*` mission below the margin.
- Added a finish-current-mission low-battery latch. If SOC falls below 35%
  during an active campsite mission, the robot finishes the current site phase,
  waits at `WAITING_FOR_RETURN_REQUEST`, and moves only after the ordinary user
  return request.
- Added persistent UI and terminal visibility for the new operational states:
  the UI header shows service state plus battery policy, and the field-test
  watch/snapshot topics include `/service/state`, manual/mission engage,
  platform drive-enable, and the cmd_vel gate status.
- Added explicit manual driving and operator-stop visibility. Manual ENGAGE
  without a campsite selection displays `Manual driving`; operator stop/cancel
  publishes `/service/state=OPERATOR_STOPPED` after cancelling Nav2, campsite,
  drop-zone, and parking motion owners.
- Updated `planning_nav_status_checker` so expected Nav2 cancel/ABORTED samples
  during campsite maneuver ownership do not raise `System warning`.
- Documented final parking state semantics: the parking controller can be
  internally `PARKED` while externally `/service/state` is `CHARGING` when CAN
  charging feedback is present.

## Battery Policy

<!-- HH_260724 - Keep percent thresholds explicit for field review. -->

| Condition | Policy |
|---|---|
| `SOC <= 20%` | Hard safety stop through `cmd_vel_safety_gate`; command output is blocked regardless of mission state |
| `20% < SOC < 35%` and no active mission | Reject new campsite dispatch and keep charging/standby |
| `20% < SOC < 35%` during a campsite mission | Finish the current site phase, warn the UI, and wait for user return confirmation |
| `SOC >= 35%` | Accept new campsite missions, including charger departure from `CHARGING`/`DROP_ZONE_WAIT` |

`DEPARTING_CHARGER` means the robot has accepted a new mission and is physically
leaving the charger. It is not a full-charge target. When no mission is
selected, the robot remains parked/charging.

## State Semantics

<!-- HH_260724 - Separate parking-controller phase from operator-visible service state. -->

- Parking controller internal final phase: `PARKED`.
- `/service/state=WAITING_FOR_CHARGING`: final parking reached the expected
  contact pose, but CAN has not confirmed charging yet.
- `/service/state=CHARGING`: CAN reports charging. The robot is both parked and
  charging.
- `/service/state=DEPARTING_CHARGER`: a new accepted campsite mission is
  releasing the robot from the charger.
- `/service/state=OPERATOR_STOPPED`: operator stop/cancel closed the engage
  gates, cleared the active UI site, requested Nav2 action cancel, and cancelled
  local maneuver controllers.

During `SITE_ENTRY`, `UNLOAD_WAIT`, `WAITING_FOR_RETURN_REQUEST`, return
maneuver, parking handoff, and `OPERATOR_STOPPED`, Nav2 may report terminal
ABORTED/CANCELED samples because motion ownership has moved to a local control
phase. These samples are not treated as system-health warnings. A planning abort
while still in `MOVING_TO_SITE` remains a warning/error condition.

## Configuration Sync

<!-- HH_260724 - List the synchronized config surfaces changed for this release. -->

- `camrod_control/config/cmd_vel_safety_gate.yaml`
- `camrod_bringup/config/control/cmd_vel_safety_gate.yaml`
- `camrod_bringup/config/bringup/launch_defaults.yaml`
- `camrod_control/src/cmd_vel_safety_gate_node.cpp`
- `camrod_control/include/camrod_control/cmd_vel_gate_policy.hpp`
- `camrod_control/include/camrod_control/charging_mission_override.hpp`
- `camrod_common/avg_msgs/msg/AvgServiceState.msg`
- `camrod_system/src/diagnostics/planning_nav_status_checker_node.cpp`
- `camrod_system/config/diagnostics/default/planning/planning_nav_status_checker.yaml`
- `camrod_system/config/diagnostics/sim/planning/planning_nav_status_checker.yaml`
- `camrod_bringup/config/system/diagnostics/default/planning/planning_nav_status_checker.yaml`
- `camrod_bringup/config/system/diagnostics/sim/planning/planning_nav_status_checker.yaml`
- `camrod_ui/camrod_ui_robot/launch/ui.launch.py`
- `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`
- `camrod_ui/camrod_ui_robot/assets/frontend/src/App.js`
- `camrod_bringup/scripts/field_test_tool.sh`

The default critical stop is `0.20`, charger departure requires `0.35`, and UI
mission dispatch requires `35.0`.

## Battery And Operator Workflow Verification

<!-- HH_260724 - Record the checks used before tagging v2.0.7. -->

- `python3 -m py_compile camrod_bringup/scripts/sim_validation_runner.py camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`
- `npm --prefix camrod_ui/camrod_ui_robot/assets/frontend run build`
- `./src/colcon_build.sh --packages-up-to camrod_control camrod_bringup camrod_ui --allow-overriding avg_msgs camrod_control camrod_bringup camrod_ui`
- `colcon test --packages-select camrod_control --event-handlers console_direct+ --ctest-args -R test_control_policies`
- `ros2 run camrod_bringup sim_validation_runner.py` with
  `run_low_battery_finish_then_return:=true`,
  `charging_recall_via_ui:=true`, and
  `run_charging_recall_battery_gate:=true`
- `./src/colcon_build.sh --packages-up-to avg_msgs camrod_system camrod_ui camrod_bringup --allow-overriding avg_msgs camrod_system camrod_ui camrod_bringup`
- `colcon test --packages-select camrod_system camrod_ui --event-handlers console_direct+`
- `./camrod_bringup/scripts/field_test_tool.sh config`
- `python3 -m py_compile camrod_ui/runtime/python/camrod_ui/ui_backend_node.py camrod_bringup/scripts/sim_validation_runner.py`
- `git diff --check`

The final simulation report passed and confirmed:

```text
low_battery_finish_no_auto_return = True
low_battery_finish_manual_return_sent = True
low_battery_finish_return_started = True
low_battery_finish_return_ok = True
charging_recall_low_battery_block = True
charging_recall_low_battery_departure_seen = False
charging_recall_low_battery_cmd_released = False
charging_recall_ok = True
```

Observed service-state sequence:

```text
SITE_ENTRY -> UNLOAD_WAIT -> WAITING_FOR_RETURN_REQUEST
-> RETURN_WITH_CARGO -> DROP_ZONE_PARKING
-> WAITING_FOR_CHARGING -> CHARGING
-> DEPARTING_CHARGER -> MOVING_TO_SITE -> SITE_ENTRY
```

## Battery And Operator Policy Known Limits

<!-- HH_260724 - Keep the safety boundary clear for field operation. -->

- The low-battery latch waits for the same return request used by ordinary
  operation. It intentionally does not auto-send `MotionOperation.RETURN`.
- The 35% threshold is an admission threshold, not a full-charge target.
- Wireless-charging success still depends on CAN charging feedback. Without
  that feedback the service state remains `WAITING_FOR_CHARGING`.
- `System warning` is health severity from diagnostics, not the service phase
  itself. Site entry and operator-stopped states remain normal service states.
