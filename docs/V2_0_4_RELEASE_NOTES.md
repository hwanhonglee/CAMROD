# CAMROD v2.0.4 Release Notes

<!-- HH_260721 - Record the native control baseline and its interface/configuration corrections. -->

Release date: 2026-07-21
Target branch: `develop`
Target remotes: `hwanhonglee/CAMROD`, `tele-genius/CAMROD`

## Scope

- Replaced the Python command gate and campsite, drop-zone, and reverse-parking
  runtimes with native C++ nodes under `camrod_control`.
- Split gate behavior into command authorization, charging-mission override,
  and motion cost-stop classes with native GoogleTest coverage.
- Added and connected the AprilTag detector/controller implementation. Reverse
  parking is the only method exercised in this release validation; real
  camera/TF/tag/contact validation remains and `image_proc` was not changed.
- Changed reverse parking to require charging feedback before `PARKED`:
  `complete_without_charging: false`, `charging_wait_timeout_s: 20.0`.
- Removed the custom ESKF executable, configs, and launch selection. The
  compatibility message field remains reserved while localization uses the
  `robot_localization` EKF backend.
- Removed ROS message/service/action type aliases from maintained code. Internal
  CAMROD topics use generated `avg_msgs`; standard ROS messages remain only at
  explicit Nav2, Ranger, sensor-driver, RViz, and diagnostics boundaries.
- Removed the unused `lateral_cmd_dynamic_obstacle_threshold` parameter and made
  `side_cost_threshold` the single lateral obstacle threshold.
- Implemented `lanelet_safety_allow_rotation_in_place` instead of leaving it as
  an ignored parameter.

## Reverse Parking And Charging Recall

<!-- HH_260721 - Preserve the final installed-tree charging recall evidence. -->

Launch used `parking_method:=reverse` and a simulated normalized Ranger/BMS
heartbeat. Result file: `/tmp/camrod_v204_charging_recall_final.json`.

Result: `OVERALL=PASS`.

- `camping_site_13`: lanelet arrival, `CRAB_IN`, `ROTATE_180`, unload wait,
  return request, yaw restore, and `CRAB_OUT` passed.
- Return navigation reached the drop-zone snap pose and completed
  `ALIGN_PARKING_YAW`.
- Reverse parking passed `REVERSE_APPROACH -> WAIT_FOR_CHARGING -> PARKED`.
- While charging, a `camping_site_1` request produced `DEPARTING_CHARGER`,
  released nonzero command output, cleared charging feedback after physical
  departure, generated new global/local paths, and reached the site entry.
- UI backend received the maneuver, drop-zone, reverse parking, charging, and
  recall state transitions during the same run.

## Obstacle And Route-Cost Validation

<!-- HH_260721 - Record directional stops, replanning, and route-lanelet filtering. -->

Result file: `/tmp/camrod_v204_obstacle_final.json`.

Result: `OVERALL=PASS`.

- All 12 direction/source cases passed: front, left, right, and rear multiplied
  by LiDAR, radar, and combined sources. Maximum `/control/cmd_vel` was 0.0 in
  every blocked case.
- Obstacle replan observed `BLOCKED`, hold, `CLEAR`, and `IDLE` states; one
  global path and 206 local-path updates were produced without an unexpected
  planner-selector fallback.
- Three sensing unit tests passed for inside/outside route filtering, the exact
  metric margin boundary, and malformed/unknown masks.
- LiDAR and radar use the active route lanelets plus
  `route_lanelet_margin_m: 0.35`. When a deliberate site/parking maneuver is
  outside the active route, the filter fails open while live gate obstacle
  sources continue to stop the commanded travel direction.

## Build, Lint, And Configuration

- `./colcon_build.sh`: UI production bundle compiled and 63 ROS packages built.
  `camrod_voice` was skipped because SDL2_mixer is absent on this machine.
- `camrod_control`: 17 native cases passed (15 policy and 2 reverse-parking axis).
- `camrod_sensing`: 3 route-lanelet cost-filter cases passed.
- `camrod_platform`: 17 light-decision cases passed.
- `camrod_localization`: 7 owned-code lint checks passed.
- `camrod_planning`: 6 owned-code lint checks passed.
- `camrod_bringup`: 4 owned-code lint checks passed.
- `field_test_tool.sh config`: all package/bringup mirrors and installed config
  copies passed. The implemented AprilTag detector now has a bringup mirror.
- The final parameter audit matched gate 132/132, campsite 66/66, drop-zone
  33/33, reverse parking 33/33, and AprilTag parking 34/34 declarations.
- No active message/service/action alias declarations or active-package empty
  directories remain. Vendored sources are excluded from package-owned lint.

## Known Limits

<!-- HH_260721 - Separate simulation evidence from physical hardware validation. -->

- Physical Ranger CAN, charging contacts, charger disengagement force, and real
  battery transitions were not exercised. The available Ranger DBC exposes BMS
  charging feedback but no separate charger-disconnect command; departure uses
  a bounded motion authorization until charging feedback clears.
- AprilTag camera, TF, detector, controller, and charging-contact behavior were
  intentionally not tested. `image_proc` was not installed or modified.
- The frontend has no unit-test script. Its production bundle compiled and its
  backend state transitions were exercised in simulation. `npm audit` reports
  36 existing dependency advisories; no breaking automatic upgrade was applied.
