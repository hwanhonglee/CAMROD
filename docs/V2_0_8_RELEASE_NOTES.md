# CAMROD v2.0.8 Release Notes

<!-- HH_260727 - Release record for steering, footprint safety, source-aware goals, and load work. -->

Release date: 2026-07-27

## Operator changes

- The Ranger longitudinal/lateral wheel transition is rate limited to
  `0.5 rad/s` by default and can be adjusted dynamically from the operator UI
  or the ROS parameter service.
- `ui.launch.py` and full `bringup.launch.py` start the lightweight GTK/WebKit
  operator window by default. Use
  `enable_operator_ui_window:=false` for a headless robot or when rendering the
  UI on another device at `http://<robot-ip>:8010`.
- The WebKit window retries automatically if it starts before the HTTP backend
  is ready, including HTTP error responses, and stops retrying after success.
- RViz `/goal_pose` now preserves the clicked position and direction. UI
  campsite goals remain lanelet-regulated on
  `/planning/site_goal_pose_ros`.

## Safety and planning

- Raw map-boundary cost is evaluated against the complete published planning
  footprint, including polygon edges and interior grid cells, for translation,
  crab, reverse, and rotation commands.
- The full-footprint hard stop uses cost 100 (off-lane/undrivable), while the
  rasterized lane-edge penalty 98 remains traversable so narrow mapped lanes do
  not immobilize the robot. Maneuver-phase static bypasses cannot bypass this
  footprint check.
- Manual goals use `LaneletRoute`, `RotationShim`, and a yaw-aware goal
  checker by default. This keeps long narrow-map routes smooth, publishes the
  active lanelet IDs needed by sensor filtering, and preserves the clicked
  final arrow direction. `Smac2D` remains an explicit launch-time diagnostic
  override. Manual goals clear stale UI mission ownership and cannot start a
  campsite/drop-zone maneuver after arrival.
- Campsite auto-entry now requires an explicit regulated `camping_site_*`
  mission key, so a stale site/route pose pair cannot arm a maneuver.
- Regulated UI goals are limited to vehicle-routable lanelets and continue to
  use the selected lanelet planner/controller policy.
- Planner fallbacks were removed from the regulated production BT so a failed
  lanelet rule cannot silently become an unrestricted grid route.
- Goal source and Nav2 selector delivery have an explicit 0.12 s ordering
  interval.
- State-machine startup, return, recall, scenario, and recovery goals now use
  that same regulated snapper path; there is no unsourced direct Nav2 fallback.

## Runtime load

- Stable `/platform/status` heartbeat publication is reduced from 50 Hz to
  10 Hz while safety-relevant state transitions still publish immediately.
- UI websocket work is suppressed when no client is connected and stable
  battery/occupancy values are no longer rebroadcast.
- Active-route ID changes rebuild only the dependent 600×600 sensor mask;
  they no longer rebuild the independent 960×960 all-lane planning grid.
  Field logs showed the unnecessary rebuild taking 7–9 seconds at full load.
- The navigation-status diagnostic counts each aborted action UUID once,
  instead of counting every repeated `GoalStatusArray` publication as another
  failure.
- In the measured full hardware stack, `ui_backend` CPU fell from 19.1% to
  8.0% of one core. See
  [v2.0.8 runtime profile](../camrod_bringup/docs/v2_0_8_runtime_profile.md)
  for the test conditions, complete process sample, and caveats.

## Validation summary

- Full real `bringup.launch.py` was used with the command gate disarmed.
- Lanelet `ComputePathToPose` produced a 31-pose, approximately 6 m path.
- RPP `FollowPath` produced nonzero raw Nav2 commands while every final Ranger
  command remained zero as required by the closed gate.
- Native control policy tests cover footprint edge, interior, reverse, lateral,
  and rotation boundary cases.
- Simulation verified repeated manual/regulated source switching, path
  orientation, selector IDs, raw command generation, final gate behavior, and
  actual armed motion. A 4 m free-space test correctly stopped after about
  3.15 m when the footprint edge entered cost 100.
- The final manual-goal mission-isolation adjustment was built and
  static-checked. The narrow-lane footprint threshold is covered by native
  policy tests. Per operator request, final post-reboot driving acceptance is
  left to the field test.
- Lanelet planner tests publish route IDs, turn segments, and reverse requests
  only on test-specific topics, preventing a test run from changing a live
  robot's transient active-route mask.
- HH_260727 - A headless `bringup.launch.py sim:=true` smoke test published a
  manual 48.44 m goal route and observed 247 path poses, the requested -77 deg
  final yaw, and a live Nav2 command (`linear.x=0.500`,
  `angular.z=0.369`). The final drive command remained blocked as designed
  because operator engage was false.

## Known field prerequisite

The hardware pose observed during release validation was approximately
31.75 m from the nearest drivable map lane. A single end-to-end hardware
`NavigateToPose` cannot be accepted until the localization/map alignment is
corrected. Planner and controller stages were therefore exercised
independently under the full real bringup, without enabling vehicle motion.
