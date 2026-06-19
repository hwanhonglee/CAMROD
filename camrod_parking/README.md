# camrod_parking

`camrod_parking` contains rule-based controllers for motions that are outside
normal lanelet/Nav2 driving:

- campsite crab entry/exit using `Twist.linear.y`
- 180-degree in-place site rotation
- drop-zone rear alignment and reverse parking

The controllers publish to `/planning/cmd_vel_raw` only while their parking
sequence is active, so the existing planning/platform gates remain in the
command path without racing the Nav2 controller during normal lanelet driving.

## Interfaces

### Site Maneuver

- Start: `/parking/site_maneuver/start` (`std_msgs/Bool`) or `/parking/site_maneuver/start_service`
- Return from site: `/parking/site_maneuver/return` (`std_msgs/Bool`) or `/parking/site_maneuver/return_service`
- Cancel: `/parking/site_maneuver/cancel` (`std_msgs/Bool`) or `/parking/site_maneuver/cancel_service`
- Status: `/parking/site_maneuver/status` (`avg_msgs/ModuleState`)

When `use_goal_pair_for_lateral_offset` is enabled, the node computes the crab
offset from `/goal_pose` (raw site center) and `/planning/goal_pose_snapped_ros`
(lanelet snap pose). HH_260618: auto-start requires this pair by default; if the
raw site center is missing, the node reports ERROR instead of pretending the
default offset is a valid campsite parking motion. HH_260618: Auto-start also requires the current pose to be within `route_goal_reached_distance_m` of the latest lanelet route goal, keeps the goal pair valid until a new goal replaces it (`goal_pair_max_age_s: 0.0`), and the default `max_lateral_offset_m` is 7.0 m so campsite centers several meters from the road are reachable.

HH_260618: `crab_timeout_speed_scale` accounts for downstream command scaling
in the planning cmd_vel gate. With `cmd_vel_gate_speed_scale: 0.5`, the timeout
duration is computed from `crab_speed_mps * 0.5` so long off-lane campsite
entries do not time out while the robot is still moving correctly.

HH_260618: Crab-out completion uses signed lateral progress back toward the
original lanelet snap pose, not only Euclidean distance. This prevents one
control tick of overshoot from missing the return completion condition.

### Drop-Zone Parking

- Start: `/parking/drop_zone/start` (`std_msgs/Bool`) or `/parking/drop_zone/start_service`
- Cancel: `/parking/drop_zone/cancel` (`std_msgs/Bool`) or `/parking/drop_zone/cancel_service`
- Status: `/parking/drop_zone/status` (`avg_msgs/ModuleState`)

The reverse controller treats the configured station yaw as the desired parked
robot rear/charger yaw, aligns the robot front 180 degrees away, then reverses
with yaw/lateral feedback until charging is detected or the configured reverse
distance is reached. HH_260618: `rear_matches_station_yaw: true` is the default
for maps where the station yaw points along the robot rear/charger direction.
`auto_select_reverse_yaw_to_station: true` chooses the 180-degree-equivalent
robot yaw that puts the station in front of the reverse axis, so service starts
and recovery starts do not fail when the robot begins on the opposite side of
the station.
HH_260618: during reverse, crossing the station reverse axis is treated as a
successful no-charging simulation completion instead of a yaw-selection error.

## 2026-06-17 Runtime Update

> HH_260617: `camrod_parking` is now a first-class source package and is built by `colcon_build.sh`.
> HH_260618: In full bringup this package launches only when `parking_method:=rule_based` or `parking`; `parking_method:=docking` launches `camrod_docking` instead.

### Runtime Nodes

| Node | Trigger | Command output | Stop/completion condition |
|---|---|---|---|
| `/parking/site_maneuver` | `PlanningState.GOAL_REACHED` for `camping_site_*`, or `/parking/site_maneuver/start_service` | `/planning/cmd_vel_raw` with `linear.y` crab and `angular.z` rotate | HH_260618: Entry pose reached, 180-degree rotation complete, then wait inside the site until a return request triggers crab-out |
| `/parking/drop_zone_parking` | `PlanningState.GOAL_REACHED` for `RETURN_TO_DROP_ZONE/drop_zone`, or `/parking/drop_zone/start_service` | `/planning/cmd_vel_raw` with yaw alignment and reverse command | `/platform/status/is_charging`, reverse distance limit, or timeout |

### Controller Design

- Site entry uses Ranger parallel/side-slip command (`Twist.linear.y`) so the robot body does not rotate during crab motion.
- Site handling rotates the body 180 degrees after lateral entry, then waits inside the site until `/parking/site_maneuver/return` or `return_service` unless `auto_return_after_unload_wait` is explicitly enabled.
- Drop-zone parking uses a rule-based reverse pose controller: align parked rear yaw to station yaw within `align_yaw_tolerance_deg`, then reverse with `reverse_yaw_kp` and `reverse_lateral_kp` feedback.
- The default station pose is loaded from bringup `map/drop_zones.yaml` via `drop_zones_yaml` and `drop_zone_id`.
- HH_260618: Idle parking nodes do not publish zero Twist. They wake at `idle_tick_rate_hz` (default 1 Hz), switch to `control_rate_hz` (default 10 Hz) only while active, and throttle status/diagnostics with `status_publish_rate_hz`.

### Smoke Test

```bash
ros2 launch camrod_parking parking.launch.py
ros2 service call /parking/site_maneuver/start_service std_srvs/srv/Trigger '{}'
ros2 service call /parking/drop_zone/start_service std_srvs/srv/Trigger '{}'
ros2 topic echo /parking/site_maneuver/status --once
ros2 topic echo /parking/drop_zone/status --once
```
