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
- Corrected event-driven cost-grid and dynamically discovered parking
  diagnostics so healthy simulation reaches an overall `OK` state.

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
  `EXIT_STRAIGHT`, then `ALIGN_EXIT_YAW`. The UI publishes the route goal only
  after `/control/drop_zone/exit_complete=true`.
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

## Verification

<!-- HH_260721 - Separate automated evidence from hardware-only validation. -->

- Full reverse-parking simulation validation: `OVERALL=PASS`.
- Maintained-package test selection: 171 tests, 0 errors, 0 failures, 8 skipped.
- Follow-up planning tests: 42 tests, 0 errors, 0 failures, 8 skipped.
- Follow-up bringup tests: 21 tests, 0 errors, 0 failures, 0 skipped.
- `./colcon_build.sh`: UI production bundle and 63 ROS packages built;
  `camrod_voice` remained skipped because SDL2_mixer is unavailable.
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
