# CAMROD v2.2.1 Release Notes

<!-- HH_260825 - Record current-pose campsite return, guarded charger
departure, front-radar near-field policy, external CARLA plant integration,
canonical build boundaries, measured results, and remaining field acceptance. -->

## Scope

`v2.2.1` advances the `develop` baseline at
`0a81ccdace53fcf94aab9a1482dbae6cdbc07fd6`. It removes the normal campsite
return dependency on the historical entry XY, delays movement after a charging
destination request, extends only the two front radar stop windows, and adds an
external CARLA plant adapter. The user-authored lanelet map geometry is not
modified by this release.

## Runtime Changes

| Area | v2.2.1 behavior |
|---|---|
| Campsite return | B1-B10 finish `CRAB_OUT` within `0.15 m` of a fresh live lanelet projection, command zero for `1.20 s`, then plan from current XY; longitudinal distance from the entry snap is not a completion condition |
| Fail-closed fallback | Stale, non-finite, or wrong-frame live projection cannot complete return; the historical exact-anchor sequencer remains available only as fallback |
| Roadside sites | B11-B13 keep the constrained roadside exit and legal forward loop; this release does not add a zero-turn there |
| Charging departure | A campsite selection in `CHARGING` or `WAITING_FOR_CHARGING` is queued for `7.0 s` with manual, mission, and platform authorization false |
| Timer ownership | One ROS timer exists only while departure is pending; Stop/shutdown destroys it, duplicate requests coalesce, and expiry emits one parking cancel and one station `EXIT` |
| Front radar | FRONT1 uses `(0.220, 0.520] m` and FRONT2 `(0.117, 0.417] m`, providing `0.30 m` beyond each measured body-return envelope |
| Radar spatial authority | The longer front range remains clipped to the active lanelet and `1.27 m` local-path corridor; side/rear usable range stays `0.10 m`, and REAR remains quarantined |
| Validator | Charging recall is keyed to public `CHARGING`, parking `PARKED`, and `DEPARTING_CHARGER` states instead of presentation-specific command-gate text |
| External simulation | `camrod_carla_adapter` maps 4WS commands/feedback and allows fake sensors to follow fresh external odometry with a `0.5 s` stale timeout |
| Build outputs | The canonical wrapper pins `build/install/log` to `~/camrod_ws`; active package READMEs no longer instruct direct colcon builds from the source checkout |

## Operator Parameters

These are the supported edit points. Package-owned safety files and their
bringup mirrors must remain byte-identical.

| Purpose | Parameter | Active value | Files |
|---|---|---:|---|
| Use current lane projection for campsite return | `enable_live_lanelet_return_handoff` | `true` | `camrod_control/config/control.yaml`, `camrod_bringup/config/control/control.yaml` |
| Projection acceptance distance | `return_lanelet_handoff_distance_m` | `0.15 m` | same control pair |
| Stopped steering-settle hold | `return_lanelet_handoff_hold_s` | `1.20 s` | same control pair |
| Charging departure delay | `system/api_ui_charging_departure_delay_s` | `7.0 s` | `camrod_bringup/config/bringup/launch_defaults.yaml` |
| Front hardware range level | `hardware_range_levels[FRONT1/2]` | `2` | sensing/bringup `sen0592_radar.yaml` pair |
| Front software range ceiling | `software_max_ranges_m[FRONT1/2]` | `0.55 m` | same radar driver pair |
| Front stop candidate ceiling | `stop_candidate_max_ranges_m[FRONT1/2]` | `0.520/0.417 m` | sensing/bringup `cost_grid.yaml` pair |
| Final front corridor | `front_dynamic_path_width_m` | `1.27 m` | control/bringup `cmd_vel_safety_gate.yaml` pair |
| External plant ownership | `external_simulator` | `false` physical default | `camrod_bringup/config/bringup/launch_defaults.yaml` |
| External odometry freshness | `external_simulator_odometry_timeout_s` | `0.5 s` | same launch-default file |

Changing a radar body exclusion changes the sensor-face coordinate at which the
usable `0.30 m` window starts. It must be based on a physical empty-scene/raw
measurement, not adjusted solely to make simulation pass.

## Measured AMD64 Results

| Check | Result |
|---|---|
| B8 live lanelet handoff | PASS: current-to-projection `0.140 m`, old entry-anchor error `0.231 m`, zero hold `1.20 s`, source `done_live_lanelet`, current-start planner mode `reverse_shortest` |
| Charging recall | PASS: configured `7.0 s`, observed `6.996327111 s`; all three authorization gates false during dwell; parking cancel `1`, EXIT `1`, no early site goal |
| FRONT1 range injection | PASS: continuous `0.300 m` `AvgRange` at `20 Hz` produced active obstacle evidence and cost `95` |
| Focused departure/radar/assets contracts | PASS: `53/53` |
| CARLA-free isolated build | PASS: control/planning/sensing/UI/bringup `5/5` |
| Focused CARLA-free contracts | PASS: `65/65` |
| Full isolated test inventory | 6 bringup targets remain failing: inherited parking mirror/stale assertion debt plus worktree-root and Pillow environment limits; this release does not claim zero failures |
| UI production bundle | PASS; current npm audit reports `40` inherited findings (`11` low, `7` moderate, `20` high, `2` critical) |
| CARLA source contracts | PASS: `21/21` |
| CARLA adapter Release build | PASS |
| Prior complete external-overlay report | `786` collected, `778` pass, `8` skip, failure `0`; adapter `86/86` |

The measured campsite, charging, and radar record is under
[`v2-2-1-safety-handoff-20260825`](assets/module-guides/bringup/test-results/v2-2-1-safety-handoff-20260825/README.md).
The CARLA source/evidence boundary is documented in
[`virtual_carla.md`](virtual_carla.md).
## Verification Commands

```bash
cd ~/camrod_ws/src
./setup_camrod.sh
./colcon_build.sh --packages-select \
  camrod_control camrod_planning camrod_sensing camrod_ui camrod_bringup \
  camrod_carla_adapter

diff -u camrod_control/config/control.yaml \
  camrod_bringup/config/control/control.yaml
diff -u camrod_sensing/config/radar/cost_grid.yaml \
  camrod_bringup/config/sensing/radar/cost_grid.yaml
diff -u camrod_sensing/config/radar/sen0592_radar.yaml \
  camrod_bringup/config/sensing/radar/sen0592_radar.yaml

cd ~/camrod_ws
colcon test --packages-select \
  camrod_control camrod_planning camrod_sensing camrod_ui camrod_bringup
colcon test-result --verbose

cd ~/camrod_ws/src
scripts/virtual_carla/test.sh --source-only
```

## Field Acceptance Still Required

- Repeat B1-B10 current-projection exit with physical steering feedback and
  payload, including stale-projection fault injection.
- Verify the 7-second departure dwell, audible/visual warning, Stop cancellation,
  charger clearance, and obstacle stop before and after station exit.
- Measure FRONT1/FRONT2 empty-scene multipath and real-object stop/release at ten
  repetitions per channel before accepting the `0.30 m` physical window.
- Run resource and topic-rate acceptance on the ARM64 8-core/16-GB target. AMD64
  timing and CPU data are integration evidence, not Jetson acceptance.
- Review and upgrade the React dependency tree in a dedicated compatibility
  change. This release does not use `npm audit fix --force` to trade known API
  behavior for an unreviewed major-version migration.
- Run CARLA with a rendered RGB-capable environment; NullRHI evidence does not
  establish camera image quality or camera perception behavior.
