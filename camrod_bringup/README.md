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
| `config/perception/apriltag_parking_detector.yaml` | Inactive-by-default AprilTag detector placeholder mirror |
| `config/control/yaw_alignment_zones.yaml` | Optional command-gate yaw zones |
| `config/planning/` | Nav2 and mission-state configuration |
| `config/map/drop_zones.yaml` | Drop-zone station position and reverse-axis yaw |
| `config/platform/ranger_driver.yaml` | Ranger CAN/SDK bridge |
| `config/system/system_checker*.yaml` | Runtime graph manifests |

<!-- HH_260721 - Define the package-to-bringup control configuration mirror contract. -->
Package-owned defaults remain canonical under `camrod_control/config/`. The
four files under `camrod_bringup/config/control/` are byte-identical deployment
mirrors; bringup additionally supplies resolved map/config paths.

## Simulation Validation

The validation runner checks sensor rates, directional gate stops, Nav2
replanning, and the complete campsite/charging round trip. The validated release
uses `parking_method:=reverse`; AprilTag nodes are not exercised.

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=false \
  -p skip_manual_goal:=true \
  -p run_camping:=true \
  -p camping_wait_drop_zone:=true \
  -p camping_timeout_s:=600.0 \
  -p simulate_platform_status:=true \
  -p run_charging_recall:=true \
  -p charging_recall_mission_key:=camping_site_1 \
  -p report_file:=/tmp/camrod_v204_charging_recall_final.json
```

The full camping check requires campsite crab/zero-turn/exit, return navigation,
drop-zone yaw alignment, `REVERSE_APPROACH`, `WAIT_FOR_CHARGING`, `PARKED`, and
the charging recall transition through `DEPARTING_CHARGER` to a new site route.

Directional gate and replan validation:

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=true \
  -p skip_manual_goal:=true \
  -p run_obstacle_replan:=true \
  -p run_camping:=false \
  -p report_file:=/tmp/camrod_v204_obstacle_final.json
```
