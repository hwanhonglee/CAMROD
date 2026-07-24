# CAMROD v2.0.7 Release Notes

<!-- HH_260724 - Record the battery-aware campsite mission and charging-release policy. -->

Release date: 2026-07-24
Target branch: `develop`
Target remote: `hwanhonglee/CAMROD`

## Scope

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
  watch/snapshot topics include `/service/state` and the cmd_vel gate status.
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

## Configuration Sync

<!-- HH_260724 - List the synchronized config surfaces changed for this release. -->

- `camrod_control/config/cmd_vel_safety_gate.yaml`
- `camrod_bringup/config/control/cmd_vel_safety_gate.yaml`
- `camrod_bringup/config/bringup/launch_defaults.yaml`
- `camrod_control/src/cmd_vel_safety_gate_node.cpp`
- `camrod_control/include/camrod_control/cmd_vel_gate_policy.hpp`
- `camrod_control/include/camrod_control/charging_mission_override.hpp`
- `camrod_ui/camrod_ui_robot/launch/ui.launch.py`
- `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`
- `camrod_ui/camrod_ui_robot/assets/frontend/src/App.js`
- `camrod_bringup/scripts/field_test_tool.sh`

The default critical stop is `0.20`, charger departure requires `0.35`, and UI
mission dispatch requires `35.0`.

## Verification

<!-- HH_260724 - Record the checks used before tagging v2.0.7. -->

- `python3 -m py_compile camrod_bringup/scripts/sim_validation_runner.py camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`
- `npm --prefix camrod_ui/camrod_ui_robot/assets/frontend run build`
- `./src/colcon_build.sh --packages-up-to camrod_control camrod_bringup camrod_ui --allow-overriding avg_msgs camrod_control camrod_bringup camrod_ui`
- `colcon test --packages-select camrod_control --event-handlers console_direct+ --ctest-args -R test_control_policies`
- `ros2 run camrod_bringup sim_validation_runner.py` with
  `run_low_battery_finish_then_return:=true`,
  `charging_recall_via_ui:=true`, and
  `run_charging_recall_battery_gate:=true`
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

## Known Limits

<!-- HH_260724 - Keep the safety boundary clear for field operation. -->

- The low-battery latch waits for the same return request used by ordinary
  operation. It intentionally does not auto-send `MotionOperation.RETURN`.
- The 35% threshold is an admission threshold, not a full-charge target.
- Wireless-charging success still depends on CAN charging feedback. Without
  that feedback the service state remains `WAITING_FOR_CHARGING`.
