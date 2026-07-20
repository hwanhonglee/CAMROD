# CAMROD v2.0.3 Release Notes

<!-- HH_260721 - Record the validated control-package consolidation baseline. -->

Release date: 2026-07-21
Target branch: `develop`
Target remotes: `hwanhonglee/CAMROD`, `tele-genius/CAMROD`

## Scope

- Consolidated the command safety gate, campsite/drop-zone maneuvers, and final
  parking controllers under `camrod_control`.
- Removed the obsolete `camrod_docking` and `camrod_parking` packages, duplicate
  platform command gate, topic aliases, copied configuration backups, and
  committed Python cache files.
- Made `/control/cmd_vel` the generated CAMROD command contract and
  `/control/cmd_vel_ros` the single standard ROS boundary consumed by Ranger.
- Replaced compatibility message aliases with generated `avg_msgs` message,
  service, and action definitions used by CAMROD-owned interfaces.
- Kept route planning in `camrod_planning`; crab, zero-turn, drop-zone alignment,
  reverse parking, and AprilTag parking controller ownership now belongs to
  `camrod_control`.
- Added active-route lanelet filtering for LiDAR and radar costs. Obstacle costs
  outside the active lanelets plus the configured margin are removed.
- Selected EKF as the default localization filter. ESKF remains an explicit
  optional profile and is not started by the default bringup.

## Reverse-Only Validation

<!-- HH_260721 - Preserve reproducible evidence for the requested reverse-only test. -->

The v2.0.3 simulation used `parking_method:=reverse`; neither the AprilTag
detector nor the AprilTag parking controller was launched.

```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false clean_before_launch:=false \
  parking_method:=reverse control_cmd_vel_gate_speed_scale:=1.0

ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=false \
  -p skip_manual_goal:=true \
  -p run_camping:=true \
  -p camping_mission_key:=camping_site_13 \
  -p camping_wait_drop_zone:=true \
  -p camping_timeout_s:=600.0 \
  -p report_file:=/tmp/camrod_v203_reverse_parking.json
```

Result: `OVERALL=PASS`.

- Sensor baseline and all seven radar direction rates passed.
- Campsite phases passed: crab entry, 180-degree rotation, unload wait, return
  yaw alignment, crab exit, and maneuver completion.
- Return navigation reached the drop-zone snap pose.
- Drop-zone parking-yaw alignment completed.
- `reverse_parking_controller` entered `REVERSE_APPROACH` and reported `PARKED`.

## Build And Package Tests

- `./colcon_build.sh`: 63 packages built successfully; the UI production bundle
  was generated and installed. `camrod_voice` remained skipped because
  SDL2_mixer is not installed on this machine.
- `camrod_control`: 83 checks passed with zero failures, including 81 gate-policy
  cases and reverse-parking geometry tests.
- `camrod_sensing`: three active-route lanelet cost-filter tests passed with zero
  failures.
- `field_test_tool.sh config`: package, bringup, and install configuration mirrors
  passed synchronization checks.

## Known Limits

<!-- HH_260721 - Separate v2.0.3 evidence from the charging sequence still awaiting validation. -->

- `complete_without_charging` remains `true` for this baseline, so reverse
  parking may report `PARKED` before charging feedback becomes true.
- Charging feedback, charger departure after a campsite request, obstacle stop
  matrices, replanning, and UI mission transitions require the broader v2.0.4
  validation pass.
- AprilTag hardware, rear-camera TF, tag detection, and charging-contact tests
  were intentionally excluded from this reverse-only validation. `image_proc`
  configuration was not changed.
- The UI production build reports existing npm dependency advisories; no
  automatic dependency upgrade was applied in this release.
