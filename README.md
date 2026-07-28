# CAMROD

<!-- HH_260720 - Document the consolidated parking and command-gate package boundaries. -->

<!-- HH_260723 - Promote the localization, routing, perception, and campsite-occupancy field release. -->

<!-- HH_260724 - Promote the battery-aware campsite mission and charging-release policy. -->

<!-- HH_260727 - Promote the source-aware goal, full-footprint, and runtime-tuning release. -->

<!-- HH_260728 - Promote the bounded radar self-return and forward-side-guard release. -->

Current validated baseline: `v2.0.9` ([release notes](docs/V2_0_9_RELEASE_NOTES.md)).

CAMROD is a ROS 2 Humble autonomous mobile robot stack. Route planning, local
vehicle maneuvers, reverse parking, and hardware command authorization are
separate runtime responsibilities.

The v2.0.8 field profile retains the v2.0.7 battery-aware campsite admission
and charging-departure policy: critical SOC at or below 20% hard-stops command
output, and new campsite missions require at least 35% SOC. If SOC falls below
35% during an active campsite mission, the robot finishes the current site
phase and waits for the normal user return request before moving back to the
drop zone.

It also adds dynamic Ranger steering-transition tuning, complete-footprint
map-boundary enforcement, source-specific RViz/UI goal policies, a lightweight
operator WebKit window, and measured UI/status load reductions. Existing
one-way Lanelet routing, campsite return, camera-LiDAR fusion, and
occupied-campsite protections remain in force.

The v2.0.9 delta replaces broad one-sided radar ignore floors with measured
narrow notches and a bounded, disengaged startup calibration. Each channel now
gets its full calibration interval from its own first valid sample, so a
late-starting port is not silently finalized with zero samples. A
transient-local manual-or-mission authorization state also prevents a restarted
radar node from learning while driving or while a cost stop is latched. Farther
side returns remain obstacles. Normal forward travel uses a 0.75 m body-near
side guard while crab and reverse retain the 1.20 m maneuver envelope. Sensor
diagnostics also retain the logical location, TF frame, mount pose, and live
measurement through the final `[SYSTEM]` summary. The v2.0.8 battery, UI,
planning, full-footprint boundary, and hardware-port policies are otherwise
unchanged.

<!-- HH_260724 - Clarify operator-visible manual driving and stop/cancel state. -->
Manual ENGAGE without a campsite selection is shown as `Manual driving` in the
robot UI. Operator stop/cancel clears active site buttons, cancels Nav2 and local
maneuver owners, closes engage gates, and publishes
`/service/state=OPERATOR_STOPPED`. During campsite entry, Nav2 cancel/ABORTED
events are expected because the local campsite maneuver owns motion; they no
longer raise a system warning while `/service/state` is in the site maneuver
handoff.

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
2. `camping_site_maneuver_controller` performs crab entry. Normal campsites
   then perform a 180-degree zero-turn.
3. A return request preserves the post-turn heading, crab-retraces the campsite
   entry lane, and hands route ownership back to planning after the exit pose is
   reached.
<!-- HH_260721 - Describe the same-lane campsite retrace without implying a second physical 180-degree turn. -->
<!-- HH_260721 - Keep inaccessible B12/B13 on the map-authored roadside service sequence. -->
   B12 and B13 are exceptions: both use the B11-side roadside service pose,
   skip `ROTATE_180` and `ALIGN_RETRACE_YAW`, then crab out with the opposite
   body-frame direction because their heading never changed.
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

<!-- HH_260721 - Document the drop-zone departure handoff required before a new route. -->
When a campsite is selected while the robot is parked or charging at the drop
zone, the UI publishes the campsite mission key first, then waits for
`EXIT_STRAIGHT -> ALIGN_EXIT_YAW` from the drop-zone controller. The campsite
`/planning/site_goal_pose_ros` is published only after
`/control/drop_zone/exit_complete=true`.

## Build And Run

```bash
cd /home/hong/camrod_ws
./src/colcon_build.sh
source install/setup.bash
# HH_260721 - Ordinary simulation feeds raw Ranger/BMS feedback through the platform bridge.
ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false parking_method:=reverse

# HH_260721 - Use the dedicated status publisher only for the charging-recall validator below.
ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false parking_method:=reverse \
  sim_platform_status_enable:=true
```

<!-- HH_260721 - Keep real-hardware startup from depending on an interactive launch-time sudo prompt. -->
For the default real-hardware launch, `can0` must already exist. Install the
boot-time setup once with
`sudo /home/hong/camrod_ws/src/camrod_platform/scripts/install_can0_service.sh`;
see `camrod_platform/README.md` for verification steps.

<!-- HH_260727 - Record the current real-hardware dual-GNSS port and correction defaults. -->
The default hardware GNSS route requires two logical ports. The stable FTDI
path `/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN03DF8V-if00-port0`
feeds NTRIP to the Lite moving base independently of its `/dev/ttyUSB*`
assignment, while
`/dev/ttyACM0` reads NAV-PVT and NAV-RELPOSNED from the heading rover. Full
bringup applies this route without extra GNSS arguments. Both ports are owned
by node-specific sections in the selected `zed_f9p_rover.yaml`; see
[camrod_sensing/README.md](camrod_sensing/README.md) and
[camrod_bringup/README.md](camrod_bringup/README.md) for wiring, acceptance
flags, and recovery steps.

Full simulation validation:

<!-- HH_260721 - Keep the validation runner's fake BMS publisher paired with the gate subscription above. -->
`simulate_platform_status:=true` requires the bringup launch option
`sim_platform_status_enable:=true` shown above.

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

Package details are documented in each package README.
