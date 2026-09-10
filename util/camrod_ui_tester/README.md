# camrod_ui_tester

`camrod_ui_tester` runs the production `camrod_ui` against a lightweight,
interactive ROS 2 runtime. It is intended for UI development when platform,
planning, localization, control, parking, and system packages cannot all be
started.

The simulator publishes real `avg_msgs` contracts, `map -> robot_center_link`
TF, Nav2 action endpoints, vehicle pose, battery, safety, planning, and service
state. It also observes commands emitted by `camrod_ui` and advances a simple
closed-loop mission model. It never publishes a velocity command.

The launch also starts a `/ranger_base_node` parameter stub, so the platform
tuning controls can read and update `steering_transition_rate_radps` without a
real chassis driver.

## Safety boundary

Use an isolated ROS domain. The simulator refuses to start when
`ROS_DOMAIN_ID` is unset or `0` unless `allow_default_ros_domain:=true` is
explicitly supplied. Do not run it on the vehicle network.

## Build

The package lives under `src/util/camrod_ui_tester`. `src/util` carries a
`COLCON_IGNORE` for its non-package tooling, so use the workspace wrapper —
it re-admits this package through an explicit base path:

```bash
cd ~/camrod_ws/src
./colcon_build.sh --packages-select camrod_ui_tester
source ../install/setup.bash
```

With plain `colcon build`, pass the base path yourself:

```bash
cd ~/camrod_ws
colcon build --base-paths src src/util/camrod_ui_tester --packages-up-to camrod_ui_tester
source install/setup.bash
```

## Run

```bash
export ROS_DOMAIN_ID=91
ros2 launch camrod_ui_tester live_ui_sim.launch.py
```

Open:

- Robot UI: <http://127.0.0.1:8010>
- Guest UI: <http://127.0.0.1:8012>
- Simulator control: <http://127.0.0.1:8020>

The launch defaults to `closed_loop` mode. Selecting a campsite in the real
Robot UI causes the simulator to acknowledge drop-zone exit, interpolate the
robot pose toward the selected site, update speed and battery, and publish an
arrival state. The control page can inject battery limits, charging, E-stop,
safety hold, system faults, and localization loss while the UI is open.

The simulator control page also provides separate **Delivery** and **Recall**
workflow panels. Select B1-B13, then inject or animate each lifecycle phase:

1. move to site
2. site arrived
3. return to drop zone
4. drop-zone return complete
5. docking start
6. docking complete

Recall uses the production guest lifecycle states (`RECALL_TO_SITE_ROAD`,
`GUEST_LOADING_WAIT`, and `RETURN_WITH_CARGO`); delivery uses the corresponding
robot lifecycle states. Docking publishes `DROP_ZONE_PARKING` while active and
`WAITING_FOR_CHARGING` when complete.

Useful launch arguments:

```bash
ros2 launch camrod_ui_tester live_ui_sim.launch.py \
  mode:=manual \
  speed_scale:=2.0 \
  route_duration_s:=8.0 \
  enable_guest_ui:=false \
  enable_operator_ui_window:=false
```

Modes:

- `manual`: only explicit control-panel state changes advance the simulator.
- `closed_loop`: UI ROS commands automatically advance departures and routes.
- `scripted`: reserved for YAML-driven timed scenarios; manual injection still works.

## HTTP control API

```bash
curl http://127.0.0.1:8020/api/sim/state
curl -X POST http://127.0.0.1:8020/api/sim/scenario/low_battery
curl -X POST http://127.0.0.1:8020/api/sim/scenario/clear_faults
curl -X POST http://127.0.0.1:8020/api/sim/signal/arrive
curl -X POST http://127.0.0.1:8020/api/sim/workflow/delivery/move_to_site \
  -H 'Content-Type: application/json' -d '{"site":"B6"}'
curl -X POST http://127.0.0.1:8020/api/sim/workflow/recall/site_arrived \
  -H 'Content-Type: application/json' -d '{"site":"B6"}'
curl -X PATCH http://127.0.0.1:8020/api/sim/state \
  -H 'Content-Type: application/json' \
  -d '{"battery_percent":24,"safety_hold":true}'
```

## Current simulation scope

Included:

- UI readiness inputs, TF, and NavigateToPose/FollowPath action endpoints
- Robot and Guest battery/service/safety state
- campsite destination, drop-zone exit, route motion, arrival, and return
- manual battery, charging, E-stop, safety, system, and localization injection
- global/local path preview and command event log

Not modeled as physical evidence:

- chassis dynamics and braking distance
- collision and obstacle physics
- camera, LiDAR, radar, GNSS, or AprilTag payload fidelity
- charger-contact and parking-controller performance

Those remain full-simulation or vehicle validation responsibilities.
