# camrod_planning

<!-- HH_260720 - Document planning as route and mission coordination only. -->

`camrod_planning` owns lanelet/Nav2 route generation and semantic mission state.
It does not own the final velocity safety gate, campsite local maneuvers, or
reverse parking.

## Main Nodes

| Node | Responsibility |
|---|---|
| `planning_state_machine` | Mission state, scenario state and maneuver handoffs |
| `goal_snapper` | Semantic/raw goal to lanelet route goal conversion |
| `centerline_snapper` | Localization-to-lanelet reference pose |
| Nav2 servers | Global route planning, local control, behaviors and navigation actions |
| `local_path_extractor` | Local segment extraction from the current global path |
| `planning_progress` | Remaining route distance/time |

## Key Interfaces

| Direction | Topic | Purpose |
|---|---|---|
| Input | `/goal_pose` | Manual or UI destination request |
| Input | `/planning/mission_key` | Semantic destination such as `camping_site_1` |
| Input | `/AMR_service_state` | Control/parking phase handoff into mission state |
| Output | `/planning/global_path` | Lanelet/Nav2 global path |
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

<!-- HH_260721 - Document bounded heading disambiguation after a 180-degree maneuver. -->
`centerline_snapper` uses vehicle heading only among spatially nearby lanelet
candidates. If a heading-aligned centerline is substantially farther than the
nearest geometry, the nearest geometry wins so an in-place turn cannot move the
route start to another road.

At the drop-zone lanelet snap pose, planning yields to
`/control/drop_zone_maneuver_controller`. After body-yaw alignment, control starts the
selected parking implementation. `reverse_parking_controller:PARKED` moves the mission to
`WAIT_DZ`.

Raw commands always pass through `camrod_control` and `camrod_platform` before
they can reach the Ranger base.

## Launch

```bash
ros2 launch camrod_planning planning.launch.py
```

The installed Smac lattice primitive is resolved by `nav2_lanelet.launch.py`;
no host-specific lattice path is required.
