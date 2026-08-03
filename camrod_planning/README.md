# camrod_planning

<!-- HH_260720 - Document planning as route and mission coordination only. -->

`camrod_planning` owns lanelet/Nav2 route generation and semantic mission state.
It does not own the final velocity safety gate, campsite local maneuvers, or
reverse parking.

<!-- HH_260804 - Make loaded Nav2 plugins and cross-package state ownership
reviewable without presenting the expected lifecycle as runtime evidence. -->
## Visual Runtime Guide

![Nav2 servers and mission states](../docs/assets/module-guides/planning/nav2-servers-and-mission-states.png)

The image is generated from the active Nav2 and bringup configuration. Full
bringup selects `LaneletRoute + RPP`; the other listed planner/controller
plugins remain loaded for selector and fallback use. `/planning/state` reports
planner lifecycle, `/service/state` reports the operator-visible mission phase,
and `/system/status` reports health severity.

![Expected service-state progression](../docs/assets/module-guides/bringup/mission-lifecycle-contract.gif)

This state GIF is a source-derived contract, not a successful drive recording.
The current B6/B12 simulation reaches campsite entry but is then correctly held
by the full-footprint lanelet gate; see the
[measured bringup result](../docs/assets/module-guides/bringup/simulation-evidence-20260804.png)
and [module visual guide](../docs/MODULE_VISUAL_GUIDE.md).

```bash
python3 camrod_bringup/scripts/render_module_readme_assets.py --module planning
```

## Main Nodes

| Node | Responsibility |
|---|---|
| `planning_state_machine` | Mission state, scenario state and maneuver handoffs |
| `goal_snapper` | Source-aware lane-center projection with manual/UI heading policies |
| `centerline_snapper` | Localization-to-lanelet reference pose |
| Nav2 servers | Global route planning, local control, behaviors and navigation actions |
| `local_path_extractor` | Local segment extraction from the current global path |
| `planning_progress` | Remaining route distance/time |

## Key Interfaces

| Direction | Topic | Purpose |
|---|---|---|
<!-- HH_260727 - Keep manual RViz and regulated UI policy boundaries explicit. -->
| Input | `/goal_pose` | Manual RViz destination; projects position to a routable lane center and preserves final yaw |
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
| RViz `/goal_pose` | Project clicked x/y to a reachable vehicle-lane centerline; preserve clicked final yaw | `LaneletRoute` | `RotationShim` | `manual_goal_checker` (0.25 m, 10 deg) |
| UI `/planning/site_goal_pose_ros` | Snap to a vehicle-routable lanelet centerline and lane heading | configured lanelet planner | configured controller | `goal_checker` |

The source is published before the resolved goal, with a 0.12 s selector
settle interval. This prevents the first goal after a manual/regulated switch
from using the previous policy. The default manual route follows the connected
lanelet graph so long narrow-map goals remain smooth and publish the active
route IDs used by sensor filtering. Manual x/y is projected before the action
goal is released, so the retained CAMROD goal, recovery logic, and LaneletRoute
endpoint share one reachable position instead of retaining an unsafe clicked
costmap cell. The manual checker retains its 0.25 m tolerance. The endpoint
keeps the clicked yaw, so either arrow direction remains valid at arrival; UI
goals additionally replace that yaw with lane heading.

<!-- HH_260730 - Replace the superseded v1.0.1 corridor result with the active
     v1.0.3 stable-map cost evidence. -->
The active empty-profile `lanelet2_maps.osm` is the v1.0.3 snapshot. For start
`(-13.958, 43.540, 7.73 deg)` and clicked goal
`(12.173, 20.458, 107.8 deg)`, the exact clicked costmap cell was lethal
(cost 254). NavFn, ThetaStar, SmacHybrid, and SmacLattice therefore returned
zero poses. LaneletRoute instead generated 249 safe poses over 49.047 m in
approximately 19.6 ms and ended at the reachable cost-186 position
`(11.1604526307, 20.1120538289)`, retaining yaw 107.8 deg. Its maximum path
cost was 218 and it used no cells at or above the lethal threshold of 252.

The pre-safe-snap four-case baseline exposed a 1.07 m difference between the
released far manual action goal and that reachable route endpoint. A fresh
build and four-case run must confirm the position-projection contract above
before manual arrival is considered validated. No manual mode bypasses
costmaps, the robot footprint, or the final command safety gate.
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

<!-- HH_260729 - Bound automatic reissue to a route-safety ABORT of the retained goal. -->
`goal_snapper` retains the currently released snapped goal and source across a
control-side `ROUTE_SAFETY_HOLD`. It reissues that exact goal only after all of
the following have occurred: the hold was observed, Nav2 reported `ABORTED`,
the command gate returned to `ENABLED`, and that enabled state remained clear
for 0.5 s. Reissue uses a 2.0 s minimum interval and is limited to two attempts
per goal. A fresh operator/UI goal resets the counter. Success or a new
accepted/executing status clears the pending abort; an unrelated planning
failure, a canceled action without route hold, or an operator stop cannot start
this recovery by itself.

<!-- HH_260804 - Record the center-frame RPP and bounded recovery handoff. -->
Nav2, goal snapping, and replanning now use `robot_center_link`, the midpoint
of the 0.886 m axle spacing. `robot_base_link` remains only a fixed rear-axle
compatibility child and is not a second navigation origin. The synchronized
RPP profile runs at 15 Hz with `desired_linear_vel: 0.4`, lookahead
min/default/max `1.1/1.1/2.0 m`, `max_angular_accel: 0.8`, and no Nav2 reverse
or rotate-to-heading behavior.

Planning does not choose the boundary escape direction. During
`ROUTE_SAFETY_HOLD`, control owns bounded crab/reverse motion. After 1.0 s of
continuous clear evidence, planning resumes the exact retained goal and RPP
again controls yaw. Simulation confirmed that handoff, but lanelets
754/2751/2720 contain a corridor narrower than the active
1.69160 x 1.27000 m planning rectangle; that route needs map or operational
route correction rather than extra crab or a reduced footprint.
See [the recorded simulation](../docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md)
for route IDs, displacement, yaw, and retry evidence.

## Launch

```bash
ros2 launch camrod_planning planning.launch.py
```

The installed Smac lattice primitive is resolved by `nav2_lanelet.launch.py`;
no host-specific lattice path is required.
