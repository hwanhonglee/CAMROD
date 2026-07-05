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
- Site maneuver path visualization: `/parking/site_maneuver/reverse_path` (`nav_msgs/Path`)

When `use_goal_pair_for_lateral_offset` is enabled, the node computes the crab
offset from `/goal_pose` (raw site center) and `/planning/goal_pose_snapped_ros`
(lanelet snap pose). HH_260618: auto-start requires this pair by default; if the
raw site center is missing, the node reports ERROR instead of pretending the
default offset is a valid campsite parking motion. HH_260623: Auto-start also requires the current pose to be within `route_goal_reached_distance_m` (default 0.3 m) of the latest lanelet route goal, keeps the goal pair valid until a new goal replaces it (`goal_pair_max_age_s: 0.0`), and the default `max_lateral_offset_m` is 7.0 m so campsite centers several meters from the road are reachable.

HH_260706: campsite crab uses `crab_speed_mps: 0.24` by default after field
testing showed the previous 0.18 m/s setting made site entry take too long.
HH_260618: `crab_timeout_speed_scale` accounts for downstream command scaling
in the planning cmd_vel gate. With the current parking default
`crab_timeout_speed_scale: 0.4`, the timeout duration is computed from
`crab_speed_mps * 0.4` so long off-lane campsite entries do not time out while
the robot is still moving correctly.

HH_260618: Crab-out completion uses signed lateral progress back toward the
original lanelet snap pose, not only Euclidean distance. This prevents one
control tick of overshoot from missing the return completion condition.

HH_260622 - Default campsite entry is `site_entry_mode: crab`: the robot keeps
the lanelet-snap body yaw, uses `Twist.linear.y` for wheel-crab lateral entry,
then rotates the body 180 degrees only after it is inside the selected campsite.
Reverse campsite entry remains selectable with `site_entry_mode: reverse` for
fallback testing.

HH_260624 - Campsite 180-degree rotation direction can follow the lanelet-side
site index policy: B1/B3/B5/B7/B9/B11 rotate clockwise, while
B2/B4/B6/B8/B10/B12/B13 rotate counter-clockwise by default.

HH_260622 - New campsite goals are ignored while `site_maneuver` is already
inside or exiting a campsite (`CRAB_IN`, `ROTATE_180`, `UNLOAD_WAIT`,
`WAIT_RETURN`, `CRAB_OUT`, or reverse-mode equivalents). A blocked new goal
latches/starts the return request instead of overwriting the original
site/route pair, so the robot must crab/reverse back to the lanelet snap pose
before any new Nav2 route can start.

HH_260622 - The rule-based site maneuver path is published on
`/parking/site_maneuver/reverse_path` so RViz can display the final off-lane
campsite trajectory separately from Nav2 global/local paths.

HH_260619 - `reverse_return_timeout_margin_s` gives reverse-out an additional
timeout margin after return is requested. Reverse-out follows a curved
nonholonomic return path and can take longer than the straight reverse-in
estimate, so it uses a separate timeout guard instead of reusing only the
entry/crab estimate.

HH_260619 - Reverse-out completion uses signed progress along the lanelet-snap
return axis plus lateral tolerance. This prevents the robot from timing out or
overshooting when it crosses the snap pose between control ticks instead of
hitting the exact XY point.

### Drop-Zone Parking

- Start: `/parking/drop_zone/start` (`std_msgs/Bool`) or `/parking/drop_zone/start_service`
- Cancel: `/parking/drop_zone/cancel` (`std_msgs/Bool`) or `/parking/drop_zone/cancel_service`
- Status: `/parking/drop_zone/status` (`avg_msgs/ModuleState`)
- Reverse path visualization: `/parking/drop_zone/reverse_path` (`nav_msgs/Path`)

The reverse controller treats the configured station yaw as the desired parked
robot front yaw, aligns to that yaw, then reverses with yaw/lateral feedback
until charging is detected or the configured reverse distance is reached.
HH_260619 - C-track `drop_zones.yaml` stores the parked robot front yaw, so
`rear_matches_station_yaw` is `false`; set it `true` only for maps where the
station yaw points along the robot rear/charger direction. `auto_select_reverse_yaw_to_station`
defaults to `false`; drop-zone parking no longer chooses a
180-degree-equivalent body yaw automatically. The only 180-degree body rotation
phase is campsite handling.
HH_260618: during reverse, crossing the station reverse axis is treated as a
successful no-charging simulation completion instead of a yaw-selection error.
HH_260619 - The intended drop-zone reverse approach is published on
`/parking/drop_zone/reverse_path` for RViz validation.

## 2026-06-17 Runtime Update

> HH_260617: `camrod_parking` is now a first-class source package and is built by `colcon_build.sh`.
> HH_260618: In full bringup this package launches only when `parking_method:=rule_based` or `parking`; `parking_method:=docking` launches `camrod_docking` instead.

### Runtime Nodes

| Node | Trigger | Command output | Stop/completion condition |
|---|---|---|---|
| `/parking/site_maneuver` | `PlanningState.GOAL_REACHED` for `camping_site_*`, or `/parking/site_maneuver/start_service` | `/planning/cmd_vel_raw` with default `linear.y` crab entry/exit plus campsite-only 180-degree rotation; reverse entry is fallback-selectable | HH_260701: Crab entry reaches the raw site center, rotates 180 degrees inside the site, then restores lanelet yaw before crab-out by default |
| `/parking/drop_zone_parking` | `PlanningState.GOAL_REACHED` for `RETURN_TO_DROP_ZONE/drop_zone`, or `/parking/drop_zone/start_service` | `/planning/cmd_vel_raw` with yaw alignment and reverse command | `/platform/status/is_charging`, reverse distance limit, or timeout |

### Controller Design

- Site entry uses Ranger parallel/side-slip command (`Twist.linear.y`) so the robot body does not rotate during crab motion.
- Site handling rotates the body 180 degrees after lateral entry, then waits inside the site until `/parking/site_maneuver/return` or `return_service` unless `auto_return_after_unload_wait` is explicitly enabled.
- HH_260701 - With `align_return_yaw_before_crab_out=true`, return restores the original lanelet-snap yaw before crab-out and uses the opposite crab direction so the following Nav2 return route does not start with another 180-degree spin.
- HH_260622 - UI usage-complete must publish `/parking/site_maneuver/return` first. `/planning/state_machine/return_to_drop_zone` is published by `site_maneuver` only after `CRAB_OUT`/`REVERSE_OUT` reaches the lanelet snap pose.
- Drop-zone parking uses a rule-based reverse pose controller: align vehicle body yaw to station/goal yaw within `align_yaw_tolerance_deg`, then reverse with `reverse_yaw_kp` and `reverse_lateral_kp` feedback.
- HH_260619 - Drop-zone parking does not run the campsite 180-degree body rotation phase; it only aligns yaw and reverses to the station pose.
- HH_260622 - RViz displays the site maneuver path and drop-zone reverse path from `/parking/site_maneuver/reverse_path` and `/parking/drop_zone/reverse_path`.
- HH_260624 - The default station pose is loaded from map-exported `drop_zones.yaml` via `drop_zones_yaml` and `drop_zone_id`, matching the planning return goal selector.
- HH_260618: Idle parking nodes do not publish zero Twist. They wake at `idle_tick_rate_hz` (default 1 Hz), switch to `control_rate_hz` (default 10 Hz) only while active, and throttle status/diagnostics with `status_publish_rate_hz`.
- HH_260622 - Site/drop-zone phases are mirrored to `/AMR_service_state` so `camrod_planning` and `camrod_ui` can show mission-level progress while Nav2 is no longer controlling the robot.

### Phase Contract

| Sequence | Phase | External state |
|---|---|---|
| Campsite entry | `CRAB_IN → ROTATE_180` by default; `ALIGN_ENTRY_YAW → REVERSE_IN → ROTATE_180` only when `site_entry_mode=reverse` | `/AMR_service_state.state=SITE_ENTRY` |
| Campsite unload | `UNLOAD_WAIT → WAIT_RETURN` | `/AMR_service_state.state=UNLOAD_WAIT` |
| Campsite return | `ALIGN_RETURN_YAW → CRAB_OUT → DONE` by default; `CRAB_OUT → DONE` only when `align_return_yaw_before_crab_out=false`; `REVERSE_OUT → DONE` only when `site_entry_mode=reverse` | `/AMR_service_state.state=RETURN_WITH_CARGO`, then `/planning/state_machine/return_to_drop_zone=true` |
| Drop-zone parking | `ALIGN_REAR_YAW → REVERSE_APPROACH → PARKED` | `/AMR_service_state.state=DROP_ZONE_PARKING`, then `DROP_ZONE_WAIT` |

HH_260701 - Return flow is designed to rotate back to lanelet yaw before crab-out, then request return-to-drop-zone only after reaching the original lanelet snap pose.

HH_260701 - State naming notes for operators:

| Name | Scope | Meaning |
|---|---|---|
| `UNLOAD_WAIT` | campsite-internal | The robot is already inside the selected campsite after the 180-degree turn. UI should show the arrival/return prompt here. |
| `WAIT_RETURN` | campsite-internal | The campsite maneuver is holding zero velocity and waiting for `/parking/site_maneuver/return`. |
| `RETURN_WITH_CARGO` | campsite-internal exit | The robot is leaving the campsite back to the lanelet snap pose through `ALIGN_RETURN_YAW` and `CRAB_OUT` or `REVERSE_OUT`. This is not the Nav2 road return yet. |
| `RETURN_TO_DROP_ZONE` | planning scenario | Nav2 is driving on the lanelet route from the campsite road/snap pose back to the configured drop-zone route goal. |

HH_260701 - Manual campsite adoption uses `/parking/site_maneuver/adopt`.
This path is for the case where the operator manually drives or manually goals
into B1/B2/etc. before using the UI. The node validates the current pose is near
the requested campsite, reconstructs a route/site pair from the latest
`/planning/lanelet_pose_ros` when available, and enters `WAIT_RETURN` so the
normal return button can start the exit sequence.

## 2026-07-02 Runtime Update

> HH_260702: Parking remains rule-based and must not re-plan the road route while it owns `/planning/cmd_vel_raw`.

The expected state separation is:

| Flow | Owner | Route behavior |
|---|---|---|
| Lanelet road drive to campsite road/snap pose | `camrod_planning` / Nav2 | LaneletRoute global path remains map-fixed; local path slices the current segment |
| Campsite crab-in, 180-degree site rotation, wait, crab-out | `camrod_parking/site_maneuver` | Parking publishes its own maneuver path and commands `/planning/cmd_vel_raw` |
| Road return from campsite snap pose to drop-zone route goal | `camrod_planning` / Nav2 | Starts only after site maneuver publishes return-to-drop-zone request |
| Drop-zone reverse parking | `camrod_parking/drop_zone_parking` | Aligns to drop-zone yaw and reverses; it does not run campsite 180-degree behavior |

If a vehicle is already manually positioned inside a campsite, use the adoption path instead of replaying the full campsite entry:

```bash
ros2 service call /parking/site_maneuver/adopt std_srvs/srv/Trigger '{}'
ros2 topic echo /parking/site_maneuver/status --once
```

### Smoke Test

```bash
ros2 launch camrod_parking parking.launch.py
ros2 service call /parking/site_maneuver/start_service std_srvs/srv/Trigger '{}'
ros2 service call /parking/drop_zone/start_service std_srvs/srv/Trigger '{}'
ros2 topic echo /parking/site_maneuver/status --once
ros2 topic echo /parking/drop_zone/status --once
ros2 topic echo /parking/site_maneuver/reverse_path --once
ros2 topic echo /parking/drop_zone/reverse_path --once
```
