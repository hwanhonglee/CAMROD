# camrod_bringup

<!-- HH_260720 - Describe the current control and parking launch topology. -->

`camrod_bringup` starts the CAMROD stack in dependency order and provides the
field/simulation configuration tree.

<!-- HH_260723 - Document scoped module launch arguments and camera ownership. -->
Every included module runs in its own launch-configuration scope. When
`camera_yolo_container.launch.py` owns the front camera, sensing receives a
local `enable_front_camera=false` only to avoid a duplicate device open; that
value cannot leak into perception and disable camera-LiDAR fusion.

## Launch Order

```text
platform -> sensor_kit -> map -> sensing -> perception -> localization
         -> planning -> control gate/maneuvers -> selected control parking node -> system -> UI
```

Control starts after planning because it consumes Nav2 commands and mission
state. The selected parking controller is also owned by `camrod_control` and
starts after the maneuver controllers publish the parking operation handoff.

## Common Launches

Simulation:

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=false parking_method:=reverse
```

Real hardware:

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=false parking_method:=reverse
```

`parking_method` accepts only `reverse` or `apriltag`. Exactly one parking node
is launched and checked by `camrod_system`.

<!-- HH_260722 - Document the hardware-verified dual-GNSS defaults used by full bringup. -->
## Dual-GNSS Field Default

Real-hardware bringup now uses the same correction route as standalone sensing:

```text
NTRIP -> moving_base_rtcm_writer -> /dev/ttyUSB4 Lite moving base
      -> UART/XBee corrected moving-base RTCM -> /dev/ttyACM0 heading rover
      -> NAV-PVT + NAV-RELPOSNED
```

`/dev/ttyACM0` is the POWER+GPS heading-rover output. `/dev/ttyUSB4` is the
POWER+XBEE FTDI input to the Lite moving base. With both present, the ordinary
real-hardware command above needs no GNSS overrides.

| Launch default | Value |
|---|---|
| `ublox_dual_antenna` | `true` |
| `ublox_dual_forward_ntrip_to_rover` | `false` |
| `ublox_dual_warm_start_on_startup` | `false` |
| `ublox_dual_base_rtcm_device` | `/dev/ttyUSB4` |
| `ublox_dual_base_rtcm_baud` | `115200` |

Verify the live ownership and solutions without printing NTRIP credentials:

```bash
ros2 node list | grep moving_base_rtcm_writer
ros2 topic info /sensing/gnss/ntrip_client/rtcm -v
ros2 topic echo /sensing/gnss/ublox_gps_node/navpvt --once
ros2 topic echo /sensing/gnss/navrelposned --once
```

The NTRIP topic must have one publisher and one subscriber (the base writer).
Absolute RTK requires NAV-PVT `(flags & 0xC0) == 0x80`; heading additionally
requires RELPOSNED moving-baseline, valid-position, valid-heading, and fixed
flags (the verified receiver normally reports decimal `311`).
Detailed wiring, A/B results, and recovery behavior are documented in
[camrod_sensing/README.md](../camrod_sensing/README.md); use the
[field test runbook](docs/field_test_runbook.md) for the operator checklist.

## Relevant Configuration

| Path | Purpose |
|---|---|
| `config/bringup/launch_defaults.yaml` | Module enable flags and launch defaults |
| `config/sensing/gnss/zed_f9p_rover.yaml` | Deployment mirror of rover device, RTCM isolation, rate, and publish settings |
| `config/sensing/gnss/ntrip_client.yaml` | Deployment mirror of the active NTRIP caster and retry settings |
| `config/control/cmd_vel_safety_gate.yaml` | Bringup mirror of command authorization and motion-safety policy |
| `config/control/control.yaml` | Bringup mirror of campsite/drop-zone maneuver tuning |
| `config/control/parking.yaml` | Bringup mirror of reverse and AprilTag parking tuning |
| `config/perception/apriltag_parking_detector.yaml` | Bringup mirror for the implemented AprilTag parking detector; inactive when reverse parking is selected |
| `config/perception/perception_params.yaml` | Bringup mirror for camera-LiDAR fusion, YOLO throttling, and campsite occupancy |
| `config/sensing/lidar/cost_grid.yaml` | Bringup mirror for perception-only dynamic cost and the `raw_lidar_cost_enabled` recovery switch |
| `config/system/diagnostics/*/localization/localization_lanelet_checker.yaml` | Lanelet output diagnostics evaluated against the live upstream localization pose rate |
| `config/control/yaw_alignment_zones.yaml` | Optional command-gate yaw zones |
| `config/planning/` | Nav2 and mission-state configuration |
| `config/map/drop_zones.yaml` | Drop-zone station position and reverse-axis yaw |
| `config/platform/ranger_driver.yaml` | Ranger CAN/SDK bridge |
| `config/system/system_checker*.yaml` | Runtime graph manifests |

<!-- HH_260721 - Define the package-to-bringup control configuration mirror contract. -->
Package-owned defaults remain canonical under `camrod_control/config/`. The
four files under `camrod_bringup/config/control/` are byte-identical deployment
mirrors; bringup additionally supplies resolved map/config paths.

<!-- HH_260722 - Define the sensing-to-bringup GNSS configuration mirror contract. -->
GNSS parameter defaults remain canonical under `camrod_sensing/config/gnss/`.
The two files under `camrod_bringup/config/sensing/gnss/` are byte-identical
deployment mirrors; `field_test_tool.sh config` and the bringup regression test
reject drift between them.

<!-- HH_260721 - Record the active profile's semantic mirror contract. -->
For `copy_park_moved`, the generic and explicit profile drop-zone/campsite YAML
files are byte-identical across package and bringup trees. B12/B13 carry the
shared `roadside_stop` service pose; all other campsite entries retain the
normal turnaround default.

## Simulation Validation

The validation runner checks sensor rates, directional gate stops, Nav2
replanning, and the complete campsite/charging round trip. The validated release
uses `parking_method:=reverse`; AprilTag nodes are not exercised.

<!-- HH_260721 - Explain ordinary simulation charging without the dedicated validator. -->
For normal `sim:=true` runs, `fake_sensor_publisher` publishes deterministic
`/battery_state` and `/system_state` feedback. `ranger_platform_bridge` remains
the only `/platform/status` publisher, so reverse parking reaches
`WAIT_FOR_CHARGING` and then `PARKED` without a CAN device.

<!-- HH_260721 - Document both sides of the simulated platform-status contract. -->
Start bringup with the gate subscribed to the runner's simulated Ranger/BMS
status before running the charging-recall scenario:

```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false parking_method:=reverse \
  sim_platform_status_enable:=true
```

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=false \
  -p skip_manual_goal:=true \
  -p run_camping:=true \
  -p camping_mission_key:=camping_site_12 \
  -p camping_wait_drop_zone:=true \
  -p camping_timeout_s:=600.0 \
  -p simulate_platform_status:=true \
  -p run_low_battery_finish_then_return:=true \
  -p run_charging_recall:=true \
  -p charging_recall_via_ui:=true \
  -p run_charging_recall_battery_gate:=true \
  -p charging_recall_mission_key:=camping_site_12 \
  -p report_file:=/tmp/camrod_v207_b12_battery_policy.json
```

<!-- HH_260721 - Validate either normal turnaround or constrained roadside phase contracts. -->
The full camping check requires the service-mode-specific campsite phases, return navigation,
drop-zone yaw alignment, `REVERSE_APPROACH`, `WAIT_FOR_CHARGING`, `PARKED`, and
the charging recall transition through `DEPARTING_CHARGER` to a new site route.
<!-- HH_260721 - Keep the documented recall check on the same UI path used in operation. -->
With `charging_recall_via_ui:=true`, the validator publishes
`UiDestinationCommand` and requires `EXIT_STRAIGHT`, `ALIGN_EXIT_YAW`, and the
public `DEPARTING_CHARGER` service state before the new route is released.
With `run_low_battery_finish_then_return:=true`, it drops the simulated SOC
during the active campsite mission and requires the UI backend to finish the
site phase, wait at `WAITING_FOR_RETURN_REQUEST` without moving, then continue
to drop-zone charging only after the validator sends the same `RETURN`
operation as the user return button.
With `run_charging_recall_battery_gate:=true`, it first drops the simulated
platform SOC to `charging_recall_low_battery_percentage` (default `0.34`) and
requires that the UI/gate do not emit charger departure or released motion. It
then restores the normal fake SOC and validates the ordinary charging recall.

<!-- HH_260724 - Include operational service and gate states in terminal field status. -->
For terminal-side operation, `camrod_bringup/scripts/field_test_tool.sh watch`
prints `/service/state` and `/control/cmd_vel_safety_gate/status` in addition
to system health, localization, planning state, manual/mission engage,
platform drive-enable, command enable, and platform status. `snapshot` and `hz`
include the same topics for post-run evidence.

<!-- HH_260721 - Record the operator/UI departure sequence validated in ordinary simulation. -->
Selecting another campsite from `DROP_ZONE_WAIT` or charging state does not
publish `/planning/site_goal_pose_ros` immediately. The UI sends a drop-zone `EXIT` operation and
releases the pending goal only after `EXIT_STRAIGHT`, `ALIGN_EXIT_YAW`, and
`/control/drop_zone/exit_complete=true`.

Directional gate and replan validation:

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=true \
  -p skip_manual_goal:=true \
  -p run_obstacle_replan:=true \
  -p run_camping:=false \
  -p simulate_platform_status:=true \
  -p report_file:=/tmp/camrod_v207_obstacle_gate.json
```
