# CAMROD v2.0.5 Release Notes

<!-- HH_260721 - Record the campsite-retrace, charging simulation, and parked-site departure release. -->

Release date: 2026-07-21
Target branch: `develop`
Target remotes: `hwanhonglee/CAMROD`, `tele-genius/CAMROD`

## Scope

- Changed campsite return behavior to preserve the post-180-degree heading and
  crab-retrace the same local lane instead of performing another physical turn.
- Delayed return-route activation until the vehicle pose and lanelet snap pose
  are both fresh and spatially consistent.
- Prevented the centerline snapper from selecting a distant lanelet solely due
  to heading agreement after a campsite maneuver.
- Made ordinary `sim:=true` charging feedback pass through the Ranger platform
  bridge from raw battery/system messages, preserving one authoritative
  `/platform/status` publisher.
- Delayed a parked or charging campsite goal until drop-zone straight exit and
  yaw alignment have completed.
- Added explicit English service progress for return-request wait, charger
  connection wait, charging, charger departure, and uncharged drop-zone departure.
- Corrected event-driven cost-grid and dynamically discovered parking
  diagnostics so healthy simulation reaches an overall `OK` state.
- Regenerated all active `copy_park_moved` drop-zone and campsite coordinates
  with the runtime LocalCartesian projection instead of retaining stale YAML.
- Added map-authored roadside service behavior for inaccessible B12/B13 while
  preserving their semantic mission keys and physical area polygons.
- Removed the duplicate standalone `ground_segmentation` source package. The
  ROS 2 wrapper remains the single owner of its integrated header-only core.

## Active Map And Roadside Sites

<!-- HH_260721 - Record the final active-map coordinate export and B12/B13 operating policy. -->

- `area_exporter` now carries optional `service_mode`, `service_x/y/z`, and
  `service_yaw_deg` OSM tags into campsite YAML.
- The active drop-zone is `(-13.5777, 40.7413)` with yaw `-82.2127 deg`.
- B12's reprojected area centroid is `(8.57397, 21.3498)` and B13's is
  `(1.02362, 28.2433)`.
- Both constrained sites use the B11-side roadside service pose
  `(12.7921, 22.52, -74.495 deg)`.
- UI, planning, control, and the validator all prefer that operational pose,
  while physical `corners` remain available for area occupancy/adoption logic.
- Their phase contract is `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN -> CRAB_OUT ->
  ALIGN_RETURN_ROUTE_YAW`. `ROTATE_180` and `ALIGN_RETRACE_YAW` are forbidden
  for `roadside_stop`; heading reversal occurs at the lanelet snap pose.

## Campsite Retrace And Return

<!-- HH_260721 - Record the validated same-lane campsite return sequence and snap guard. -->

The reverse-parking simulation exercised campsite 2 through:

`CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT`.

The route handoff occurred near `(23.37, -6.15)`, with a centerline snap near
`(23.49, -6.09)`. The resulting return route contained 20 lanelets and 624 path
points, and navigation reached the drop-zone approach successfully.

For a reproduced post-turn input near `(23.36, -6.13)` at approximately 118
degrees, the previous heading-only selection snapped about 15.8 m away. The new
`heading_filter_max_extra_distance_m: 2.0` guard selected geometry about 0.13 m
away.

## Reverse Parking And Charging

<!-- HH_260721 - Record the ordinary-simulation charging and parked departure behavior. -->

- Reverse parking completed `ALIGN_PARKING_YAW -> REVERSE_APPROACH ->
  WAIT_FOR_CHARGING -> PARKED` with simulated charging feedback.
- The fake sensor node publishes raw `BatteryState` and Ranger `SystemState`;
  `ranger_platform_bridge` remains the sole normalized platform-status writer.
- Selecting another campsite while parked or charging first requests
  parking `CANCEL`, then `EXIT_STRAIGHT` and `ALIGN_EXIT_YAW`. Releasing final
  parking ownership prevents charger-disconnect feedback from restoring
  `DROP_ZONE_WAIT` and closing the command gate during departure. The UI
  publishes the route goal only after `/control/drop_zone/exit_complete=true`.
- Charger feedback clears after campsite departure, allowing the safety gate to
  leave its charging hold and pass the active mission command.

## Diagnostics And Configuration

<!-- HH_260721 - Record event-driven map diagnostics and synchronized package/bringup parameters. -->

- The lanelet cost grid is treated as event-driven with `expected_hz: 0.0`;
  stale and unknown-cell checks remain active.
- Dynamically discovered diagnostic categories start neutral instead of
  producing a permanent warning. A short full-stack simulation reported all
  modules healthy.
- Modified control, planning, system, and bringup configuration mirrors are
  byte-identical after synchronization.
- Renamed campsite phase and parameter values are synchronized across the
  controller, command gate, validator, package configs, and bringup configs.
<!-- HH_260721 - Record removal of the obsolete UI build tree and path documentation. -->
- Removed the ignored `camrod_ui/runtime/assets/frontend` build/install residue;
  frontend resolution documentation now matches the active
  `camrod_ui_robot/assets/frontend` source and install layout.

## Verification

<!-- HH_260721 - Separate automated evidence from hardware-only validation. -->

- Normal campsite turnaround simulation: `OVERALL=PASS`, with both
  `ROTATE_180` and `ALIGN_RETRACE_YAW` observed.
- B12 roadside simulation: `OVERALL=PASS`, with both rotation phases absent
  and `CRAB_OUT/DONE` observed.
- Full B12 to drop-zone reverse-parking and charging-recall simulation:
  `OVERALL=PASS`; charging, departure override, command release, disconnect,
  new route, and B12 re-arrival were all observed.
<!-- HH_260721 - Record the externally visible English service-state sequence. -->
- The validated public service sequence was `SITE_ENTRY -> UNLOAD_WAIT ->
  WAITING_FOR_RETURN_REQUEST -> RETURN_WITH_CARGO -> DROP_ZONE_PARKING ->
  WAITING_FOR_CHARGING -> CHARGING -> DEPARTING_CHARGER -> MOVING_TO_SITE ->
  SITE_ENTRY`.
- Directional gate and obstacle-replan simulation: `OVERALL=PASS`; all 12
  LiDAR/radar/combined direction cases produced zero final command.
- Maintained-package test selection: 171 tests, 0 errors, 0 failures, 16 skipped.
- Follow-up planning tests: 42 tests, 0 errors, 0 failures, 8 skipped.
- Follow-up bringup tests: 21 tests, 0 errors, 0 failures, 0 skipped.
- `./colcon_build.sh`: UI production bundle and 62 ROS packages built;
  `camrod_voice` remained skipped because SDL2_mixer is unavailable.
- The dependency-layout audit confirmed that runtime launches reference only
  `ground_segmentation_ros2`; the build wrapper now rejects both copies existing.
- The retained ground-segmentation package passed all four registered checks,
  then started with the expected PointCloud2 subscriber and obstacle publisher.
- Python byte-compilation, YAML parsing, configuration-mirror comparison, and
  `git diff --check` passed.

## Known Limits

<!-- HH_260721 - Keep physical CAN, charger, and AprilTag evidence outside the simulation claim. -->

- Physical Ranger CAN, charging contacts, charger-release force, and real BMS
  transitions require robot testing.
- AprilTag parking was not exercised in this release; `image_proc` was not
  installed or modified.
- The frontend has no unit-test script. Its production bundle compiled, and the
  parked/charging departure state sequence was exercised through the ROS UI
  backend in simulation.
- The B12/B13 roadside pose is derived from map geometry near B11. Rock,
  terrain, vehicle footprint, and passenger-clearance margins must be measured
  on the real site before unattended use.
