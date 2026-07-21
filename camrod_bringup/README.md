# camrod_bringup

<!-- HH_260720 - Describe the current control and parking launch topology. -->

`camrod_bringup` starts the CAMROD stack in dependency order and provides the
field/simulation configuration tree.

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

## Relevant Configuration

| Path | Purpose |
|---|---|
| `config/bringup/launch_defaults.yaml` | Module enable flags and launch defaults |
| `config/control/cmd_vel_safety_gate.yaml` | Bringup mirror of command authorization and motion-safety policy |
| `config/control/control.yaml` | Bringup mirror of campsite/drop-zone maneuver tuning |
| `config/control/parking.yaml` | Bringup mirror of reverse and AprilTag parking tuning |
| `config/perception/apriltag_parking_detector.yaml` | Bringup mirror for the implemented AprilTag parking detector; inactive when reverse parking is selected |
| `config/control/yaw_alignment_zones.yaml` | Optional command-gate yaw zones |
| `config/planning/` | Nav2 and mission-state configuration |
| `config/map/drop_zones.yaml` | Drop-zone station position and reverse-axis yaw |
| `config/platform/ranger_driver.yaml` | Ranger CAN/SDK bridge |
| `config/system/system_checker*.yaml` | Runtime graph manifests |

<!-- HH_260721 - Define the package-to-bringup control configuration mirror contract. -->
Package-owned defaults remain canonical under `camrod_control/config/`. The
four files under `camrod_bringup/config/control/` are byte-identical deployment
mirrors; bringup additionally supplies resolved map/config paths.

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
  -p run_charging_recall:=true \
  -p charging_recall_mission_key:=camping_site_12 \
  -p report_file:=/tmp/camrod_v205_b12_charging_recall.json
```

<!-- HH_260721 - Validate either normal turnaround or constrained roadside phase contracts. -->
The full camping check requires the service-mode-specific campsite phases, return navigation,
drop-zone yaw alignment, `REVERSE_APPROACH`, `WAIT_FOR_CHARGING`, `PARKED`, and
the charging recall transition through `DEPARTING_CHARGER` to a new site route.

<!-- HH_260721 - Record the operator/UI departure sequence validated in ordinary simulation. -->
Selecting another campsite from `DROP_ZONE_WAIT` or charging state does not
publish `/goal_pose` immediately. The UI sends a drop-zone `EXIT` operation and
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
  -p report_file:=/tmp/camrod_v205_obstacle_gate.json
```
