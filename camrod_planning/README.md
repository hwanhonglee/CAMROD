# camrod_planning

<!-- HH_260720 - Document planning as route and mission coordination only. -->

`camrod_planning` owns lanelet/Nav2 route generation and semantic mission state.
It does not own the final velocity safety gate, campsite local maneuvers, or
reverse parking.

## Main Nodes

| Node | Responsibility |
|---|---|
| `planning_state_machine` | Mission state, scenario state and maneuver handoffs |
| `goal_snapper` | Source-aware manual passthrough or regulated lanelet goal conversion |
| `centerline_snapper` | Localization-to-lanelet reference pose |
| Nav2 servers | Global route planning, local control, behaviors and navigation actions |
| `local_path_extractor` | Local segment extraction from the current global path |
| `planning_progress` | Remaining route distance/time |

## Key Interfaces

| Direction | Topic | Purpose |
|---|---|---|
<!-- HH_260727 - Keep manual RViz and regulated UI policy boundaries explicit. -->
| Input | `/goal_pose` | Manual RViz destination; preserves the requested pose and final yaw |
| Input | `/planning/site_goal_pose_ros` | UI/mission destination; snaps to a routable vehicle lanelet |
| Input | `/planning/mission_key` | Semantic destination such as `camping_site_1` |
| Input | `/service/state` | Control/parking phase handoff into mission state |
| Output | `/planning/global_path` | Lanelet/Nav2 global path |
| Output | `/planning/goal_source` | `manual` or `regulated` selector policy latch |
<!-- HH_260720 - Document the generated global-path mirror used by CAMROD-owned consumers. -->
| Output | `/planning/global_path_avg` | Generated CAMROD mirror of the Nav2 global path |
| Output | `/planning/local_path` | Current local path |
| Output | `/planning/state_machine/state` | Semantic mission state |
| Output | `/planning/mission_engage` | Mission command authorization request |
| Output | `/control/cmd_vel_raw` | Nav2 controller velocity candidate |
| Output | `/planning/drop_zone_goal_raw` | Exact semantic drop-zone pose for control/parking |

## Ownership Handoffs

At a campsite lanelet snap pose, planning yields local motion to
`/control/camping_site_maneuver_controller`. After crab exit completes, the maneuver requests the
return route and planning resumes Nav2 ownership.

<!-- HH_260721 - Route semantic campsite keys through optional operational service poses. -->
`camping_sites.yaml` keeps each physical area centroid and polygon, but planning
uses `service_x/service_y/service_yaw_deg` when present. This routes B12 and B13
to their shared roadside stop while preserving `camping_site_12` and
`camping_site_13` as distinct mission keys. Entries without `service_*` fields
continue to use `x/y/yaw_deg`.

<!-- HH_260721 - Explain why return requests may remain pending briefly. -->
The return request remains pending until the lanelet snap pose is fresh and is
within the configured distance of `/localization/pose`. This keeps the Nav2
return path anchored to the completed campsite exit instead of an older route
position.

<!-- HH_260721 - Document campsite return routing without changing map-wide one-way semantics. -->
After an explicit campsite request arrives on
`reverse_lanelet_request_topic`, and the route start yaw differs from the
matched lanelet direction by at least
`reverse_lanelet_start_heading_threshold_deg`, `LaneletRoutePlanner` reverses
the legal goal-to-start shortest path. A campsite return therefore retraces the
approximately 53 m drop-zone arrival path instead of taking the approximately
149 m one-way loop. The request is consumed by that goal, and replanning the
same snapped destination keeps its selected reverse direction. Ordinary goals
cannot enter reverse routing from vehicle yaw alone and retain the OSM
`one_way` routing graph.

<!-- HH_260721 - Document bounded heading disambiguation after a 180-degree maneuver. -->
`centerline_snapper` uses vehicle heading only among spatially nearby lanelet
candidates. If a heading-aligned centerline is substantially farther than the
nearest geometry, the nearest geometry wins so an in-place turn cannot move the
route start to another road.

At the drop-zone lanelet snap pose, planning yields to
`/control/drop_zone_maneuver_controller`. After body-yaw alignment, control starts the
selected parking implementation.

<!-- HH_260721 - Keep normal charging progress distinct from planning failures. -->
`DROP_ZONE_WAIT` and `CHARGING` move the mission to `WAIT_DZ`.
`WAITING_FOR_CHARGING` keeps the parking scenario active until charger feedback
arrives, while `WAITING_FOR_RETURN_REQUEST` keeps the campsite unload scenario
stationary until an explicit return request.

Raw commands always pass through `camrod_control` and `camrod_platform` before
they can reach the Ranger base.

## Manual and regulated goals

<!-- HH_260727 - Document why RViz direction and UI lanelet rules no longer conflict. -->

The two operator paths intentionally use different Nav2 policies:

| Source | Position/orientation policy | Planner | Controller | Goal checker |
|---|---|---|---|---|
| RViz `/goal_pose` | Preserve clicked pose; project drive geometry to the legal lanelet and retain clicked final yaw | `LaneletRoute` | `RotationShim` | `manual_goal_checker` (0.25 m, 10 deg) |
| UI `/planning/site_goal_pose_ros` | Snap to a vehicle-routable lanelet centerline and lane heading | configured lanelet planner | configured controller | `goal_checker` |

The source is published before the resolved goal, with a 0.12 s selector
settle interval. This prevents the first goal after a manual/regulated switch
from using the previous policy. The default manual route follows the connected
lanelet graph so long narrow-map goals remain smooth and publish the active
route IDs used by sensor filtering. The route endpoint keeps the clicked yaw,
so either arrow direction is valid at arrival. `nav2_manual_planner:=Smac2D`
remains available for an explicit free-space diagnostic, but no manual mode
bypasses costmaps, the robot footprint, or the final command safety gate.
Manual goals also clear stale UI mission ownership and never start a
campsite/drop-zone service maneuver on arrival.
Camping-site auto-entry additionally requires an explicit regulated
`camping_site_*` mission key; a retained site/route pose pair alone is never
treated as mission authority.

<!-- HH_260727 - Keep internally generated missions on the same single-writer source contract. -->
Startup, return, recall, scenario, and recovery goals all enter through
`/planning/auto_goal_raw` and the regulated goal snapper. The state machine
does not publish directly to Nav2, so an earlier manual selector cannot leak
into a later automatic mission.

## Launch

```bash
ros2 launch camrod_planning planning.launch.py
```

The installed Smac lattice primitive is resolved by `nav2_lanelet.launch.py`;
no host-specific lattice path is required.
