# CAMROD

<!-- HH_260720 - Document the consolidated parking and command-gate package boundaries. -->

Current validated baseline: `v2.0.4` ([release notes](docs/V2_0_4_RELEASE_NOTES.md)).

<!-- HH_260721 - Link the reverse-only validation baseline from the workspace entry point. -->

CAMROD is a ROS 2 Humble autonomous mobile robot stack. Route planning, local
vehicle maneuvers, reverse parking, and hardware command authorization are
separate runtime responsibilities.

## Package Responsibilities

| Package | Responsibility |
|---|---|
| `camrod_planning` | Lanelet/Nav2 route planning, mission state, goal snapping, path progress |
| `camrod_control` | Command safety gate, campsite/drop-zone maneuvers, reverse and AprilTag parking controllers |
| `camrod_platform` | Ranger CAN/status bridge, driver integration, lights and visualization |
| `camrod_localization` | Sensor adapters, robot_localization EKF, pose selection and monitoring |
| `camrod_map` | Lanelet map loading, semantic areas, cost grids and RViz configuration |
| `camrod_sensing` | GNSS, IMU, LiDAR, radar and camera pipelines |
| `camrod_perception` | General obstacle detection and fused obstacle outputs |
| `camrod_system` | Runtime graph checks and diagnostics aggregation |
| `camrod_ui` | Operator UI and mission requests |
| `camrod_voice` | Voice-event adaptation and playback |
| `camrod_bringup` | Full-stack launch/config orchestration and simulation validation |

## Velocity Command Path

```text
Nav2 / camping_site_maneuver_controller / drop_zone_maneuver_controller / parking controller
                              |
                              v
                    /control/cmd_vel_raw
                              |
                              v
             /control/cmd_vel_safety_gate
                              |
                              v
                       /control/cmd_vel
                              |
                              v
         explicit standard ROS conversion boundary
                              |
                              v
                    /control/cmd_vel_ros
                              |
                              v
                     Ranger /cmd_vel input
```

The control gate owns engage, operator arm, e-stop, command timeout,
localization recovery, cost-grid, platform CAN state, charging, and critical
battery checks. Ranger consumes the gate's explicit standard ROS output
directly; no second platform gate or `/platform/cmd_vel` alias exists.

## Mission Sequence

1. Planning drives to the selected campsite lanelet snap pose.
2. `camping_site_maneuver_controller` performs crab entry and a 180-degree zero-turn.
3. A return request triggers yaw restoration, crab exit, and route ownership
   handoff to planning.
4. Planning returns to the drop-zone lanelet snap pose.
5. `drop_zone_maneuver_controller` aligns the body for the configured reverse axis.
6. The selected controller under `camrod_control/parking` performs only final parking.

With `parking_method:=reverse`, `reverse_parking_controller` commands the low-speed reverse
approach and reports `PARKED`. With `parking_method:=apriltag`, the current node
uses the AprilTag detector/controller interface under perception and control.

## Charging Departure

Charging state comes from Ranger/BMS feedback inside the generated
`AvgPlatformStatus` message on `/platform/status`. While charging, the safety gate normally blocks
motion and reports `CHARGING`. A concrete `camping_site_*` mission request opens
a bounded departure window, allowing the active control/planning command path to move away
from the charger. If charging feedback does not clear before the
window expires, command output closes again.

## Build And Run

```bash
cd /home/hong/camrod_ws
./src/colcon_build.sh
source install/setup.bash
ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=false parking_method:=reverse
```

Full simulation validation:

```bash
ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p quick:=true \
  -p run_gate_matrix:=false \
  -p skip_manual_goal:=true \
  -p run_camping:=true \
  -p camping_mission_key:=camping_site_13 \
  -p camping_wait_drop_zone:=true \
  -p camping_timeout_s:=600.0 \
  -p simulate_platform_status:=true \
  -p run_charging_recall:=true \
  -p charging_recall_mission_key:=camping_site_1
```

Package details are documented in each package README.
