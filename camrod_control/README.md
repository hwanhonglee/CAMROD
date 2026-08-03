# camrod_control

`camrod_control` owns vehicle-motion maneuvers, final parking controllers, and
the supervised velocity command path. All active runtime nodes are native C++.

<!-- HH_260720 - Document the new package boundary and canonical velocity topics. -->

Runtime responsibilities:

- `cmd_vel_safety_gate`: engage, platform status, timeout, localization recovery, and cost-grid command gating
- `camping_site_maneuver_controller`: campsite service-pose crab entry/exit and policy-selected rotation
- `drop_zone_maneuver_controller`: drop-zone departure and yaw alignment before reverse parking
- `reverse_parking_controller`: final yaw-aware reverse motion only
- `apriltag_parking_controller`: final AprilTag-based parking controller

<!-- HH_260721 - State the same-lane campsite return contract without implying a second turn. -->
The configured campsite flow is `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT ->
WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT`. `ALIGN_RETRACE_YAW` verifies the
180-degree retrace heading already reached inside the site; it does not command
a second 180-degree turn. An opposite-side through-exit is not inferred from a
site center. It requires an explicit opposite lanelet snap pose in the semantic
map before that mode can be enabled safely.

<!-- HH_260721 - Document the operator-visible site wait state. -->
The controller publishes `UNLOAD_WAIT` while settling at the site and then
`WAITING_FOR_RETURN_REQUEST` while it is stationary and accepting the return
button. Neither state is a diagnostic warning.

<!-- HH_260723 - Block duplicate use of a campsite confirmed occupied by perception. -->
`camping_site_maneuver_controller` subscribes to
`/perception/camping_sites/occupancy`. It rejects automatic or manual entry into
an occupied mission key and stops an in-progress approach if that site becomes
occupied before entry completes.

<!-- HH_260721 - Document the constrained roadside exit and on-lane return alignment. -->
B12 and B13 use `service_mode: roadside_stop` with a shared B11-side service
pose. Their flow is `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN -> CRAB_OUT ->
ALIGN_RETURN_ROUTE_YAW`. The controller first retraces the lateral entry
segment without rotating near the B12/B13 obstacles, then turns 180 degrees at
the lanelet snap pose so planning can select the reversed shortest route.
Campsites without that field continue to use the normal turnaround flow above.

<!-- HH_260721 - Document the post-maneuver planning synchronization guard. -->
After `CRAB_OUT`, planning waits until `/planning/lanelet_pose` has converged
to `/localization/pose` before publishing the drop-zone route. This prevents
Nav2 from reusing a stale pre-maneuver start pose.

<!-- HH_260721 - Clarify charging mission override and method-independent parking handoff. -->

`cmd_vel_safety_gate` normally blocks commands while
the `is_charging` field of `/platform/status` is true. A new `camping_site_*` mission key opens
a bounded `DEPARTING_CHARGER` window only when battery feedback is present and
at or above `minimum_mission_departure_battery_percentage` (default `0.35`), so
`drop_zone_maneuver_controller` can leave the station. The gate returns to
normal `ENABLED` after charging feedback clears, or closes again when the
departure window expires.
The same gate keeps the hard critical-SOC stop at
`critical_battery_percentage` (default `0.20`) regardless of mission state.

`drop_zone_maneuver_controller` reads semantic drop-zone yaw as the reverse travel axis,
aligns the robot body 180 degrees away from that axis, and starts the selected
parking method only after yaw convergence.

<!-- HH_260721 - Document the bounded departure order used for a new campsite call. -->
After parking, a new campsite call starts `EXIT_STRAIGHT`, then
`ALIGN_EXIT_YAW` toward the current lanelet pose. The route goal must remain
pending until `/control/drop_zone/exit_complete` reports success.

<!-- HH_260721 - Document final parking and charger departure service states. -->
Final parking publishes `DROP_ZONE_PARKING`. The parking controller may
transiently report `WAITING_FOR_CHARGING` after it reaches the expected contact
pose but before CAN confirms charger power. Once `/platform/status.is_charging`
is true, the externally visible service state is `CHARGING` even though the
parking controller's internal phase is still `PARKED`. A new campsite call publishes
`DEPARTING_CHARGER` when that feedback was active, or `DEPARTING_DROP_ZONE`
when the robot was parked without an active charger.

<!-- HH_260720 - Keep launch topology separate from safety policy tuning. -->

`launch/cmd_vel_safety_gate.launch.py` only selects topics, namespace, and the parameter file.
All field-tuned gate policy is grouped in `config/cmd_vel_safety_gate.yaml`; full bringup may
still override those values from `launch_defaults.yaml` without duplicating them in the launch file.
Authorization, charging mission override, and motion cost-stop logic are split
into testable classes instead of being embedded in the ROS callback surface.

Canonical command path:

```text
/control/cmd_vel_raw -> cmd_vel_safety_gate -> /control/cmd_vel
Nav2 /control/nav2_cmd_vel_ros -----------^             |
                                                       +-> /control/cmd_vel_ros -> Ranger
```

`/control/cmd_vel` is the generated CAMROD command contract. The gate publishes
`/control/cmd_vel_ros` only as the explicit `geometry_msgs/Twist` boundary required by Ranger.
It also publishes transient-local `/control/planning_engaged`, the raw
manual-or-mission authorization state. This remains distinct from
`/control/command_enabled`, which can be false during a cost, CAN, or e-stop
hold, and lets restarted sensing nodes refuse unsafe runtime calibration.

<!-- HH_260720 - Flatten runtime sources and remove the redundant Python package wrapper. -->

```text
camrod_control/
  include/camrod_control/
    charging_mission_override.hpp
    cmd_vel_gate_policy.hpp
    control_diagnostics.hpp
    drop_zone_station_pose.hpp
    motion_geometry.hpp
    motion_cost_stop.hpp
    reverse_parking_axis.hpp
    ros_message_conversion.hpp
  src/
    cmd_vel_safety_gate_node.cpp
    motion_cost_stop.cpp
    camping_site_maneuver_controller_node.cpp
    drop_zone_maneuver_controller_node.cpp
    reverse_parking_controller_node.cpp
    apriltag_parking_controller_node.cpp
  test/
    test_control_policies.cpp
    test_reverse_parking_axis.cpp
```

<!-- HH_260721 - Describe each maintained control file by its concrete runtime responsibility. -->

## File Responsibilities

| File | Responsibility |
|---|---|
| `src/cmd_vel_safety_gate_node.cpp` | Merges navigation and maneuver commands, applies engage/CAN/charging/localization/cost-stop conditions, and publishes the final CAMROD and Ranger boundary commands |
| `src/route_safety_recovery_controller_node.cpp` | Owns bounded automatic reverse/crab commands during a route-safety hold and fails closed on stale evidence or limits |
| `src/camping_site_maneuver_controller_node.cpp` | Loads each operational service pose/mode and runs reverse or crab entry, optional 180-degree turn, unload wait, same-trace exit, and return-request phases |
| `src/drop_zone_maneuver_controller_node.cpp` | Exits the drop-zone station, aligns the body for the configured reverse axis, and starts the selected final-parking controller |
| `src/reverse_parking_controller_node.cpp` | Performs yaw-corrected reverse parking and waits for normalized CAN charging confirmation |
| `src/apriltag_parking_controller_node.cpp` | Performs tag-guided reverse approach, final insertion, retry, and charging-confirmed completion |
| `src/motion_cost_stop.cpp` | Evaluates forward, reverse, crab, and zero-turn obstacle/lanelet costs and maintains the clear-time stop latch |
| `include/camrod_control/cmd_vel_gate_policy.hpp` | Produces the authoritative command-block reason list from engage, e-stop, CAN, charging, battery, and hold state |
| `include/camrod_control/charging_mission_override.hpp` | Opens a deduplicated, time-bounded charging stop override for an allowed campsite mission |
| `include/camrod_control/motion_cost_stop.hpp` | Declares all-direction cost-stop configuration, decisions, grid inputs, corridor checks, and latch state |
| `include/camrod_control/route_recovery_candidate.hpp` | Selects and continues the one unambiguous recovery direction without changing sides during a hold |
| `include/camrod_control/reverse_parking_axis.hpp` | Converts between station reverse-axis yaw and robot body yaw and measures signed axis distance |
| `include/camrod_control/drop_zone_station_pose.hpp` | Loads the selected drop-zone station position and reverse-axis yaw from semantic map YAML |
| `include/camrod_control/motion_geometry.hpp` | Provides planar angle, relative-position, clamp, and yaw/quaternion operations |
| `include/camrod_control/ros_message_conversion.hpp` | Converts only explicit ROS boundary pose/twist messages to and from generated CAMROD messages |
| `include/camrod_control/control_diagnostics.hpp` | Constructs control `ModuleState` and standard ROS diagnostic messages |
| `launch/cmd_vel_safety_gate.launch.py` | Launches the final command gate and maps bringup override names to node parameters |
| `launch/maneuvers.launch.py` | Launches campsite and drop-zone maneuver controllers with shared command and semantic-map paths |
| `launch/parking.launch.py` | Selects exactly one reverse or AprilTag parking method and starts AprilTag perception only when selected |
| `config/cmd_vel_safety_gate.yaml` | Defines gate topology, CAN/charging interlocks, localization holds, cost-stop behavior, and output shaping |
| `config/control.yaml` | Defines campsite and drop-zone maneuver topics, geometry, speeds, tolerances, timeouts, and phase behavior |
| `config/parking.yaml` | Defines separate reverse and AprilTag final-parking controller parameters |
| `config/yaw_alignment_zones.yaml` | Defines optional planar map zones for command-gate heading locks; disabled by default |
| `test/test_control_policies.cpp` | Regresses gate authorization, charging mission override, all-direction obstacle/lanelet stops, and latch clearing |
| `test/test_reverse_parking_axis.cpp` | Regresses station yaw, body yaw, and signed reverse-axis conventions |

<!-- HH_260721 - Record the charging-complete parking contract and motion cost-stop behavior. -->

`reverse_parking_controller` enters `WAIT_FOR_CHARGING` after its configured
reverse distance and reports `PARKED` only after `/platform/status.is_charging`
becomes true. `complete_without_charging` is `false` and the default wait timeout
is 20 seconds.

<!-- HH_260721 - Keep normalized platform status single-writer in simulation and hardware. -->
On hardware, Ranger CAN provides `/battery_state` and `/system_state`. Ordinary
simulation emulates those same raw boundaries. In both modes only
`ranger_platform_bridge` publishes the normalized `/platform/status`, avoiding
conflicting charging values from multiple publishers.

`motion_cost_stop` checks live LiDAR/radar costs for forward, reverse, crab,
and zero-turn commands, then holds a stop latch until the clear interval passes.
<!-- HH_260728 - Document command-independent dynamic-obstacle latch release. -->
The latch preserves the source and exact corridor/path/rotation probe that
triggered the stop. Stop-induced zero commands, changed directions, replanning,
or stale trigger-sensor data cannot count as clear; the configured clear window
starts only on fresh clear grids for the saved hazard.
Static lanelet checks are direction-configurable for
site maneuvers. LiDAR/radar cost nodes first remove costs outside the active
route lanelets plus `route_lanelet_margin_m` (0.35 m); live obstacle checks stay
active during configured static-cost maneuver exceptions.

<!-- HH_260728 - Document the field-tuned straight-travel side guard separately
     from the deliberately wider maneuver envelope. -->
Normal forward travel uses a 0.60 m raw side probe measured from
the canonical robot base frame, not an additional 0.60 m beyond the body edge. Radar then
paints each return with `obstacle_radius_m: 0.30`; at the 0.10 m grid
resolution, a base-centred side hit near `|y|=1.0 m` remains clear for forward
travel while a closer hit near `|y|=0.8 m` overlaps the probe and blocks.
Crab and reverse commands retain the 1.20 m side envelope, and a command toward
either side remains blocked by radar/LiDAR cost in that maneuver envelope.

<!-- HH_260727 - State that raw lanelet boundaries apply to the complete robot polygon. -->
For raw lanelet/map-boundary cost, the gate checks the complete polygon from
`/platform/robot/planning_boundary`, including its edges and covered grid-cell
centers. A rectangle matching the configured front/rear/left/right extents is
used until that polygon arrives. Translation, crab motion, reverse motion, and
in-place rotation are blocked if any part of the body reaches cost 100
(`lanelet_safety_footprint_threshold`) or an unknown/out-of-grid cell;
the map's soft/rasterized boundary penalty 98 remains traversable on narrow
lanes. Maneuver/static-cost bypass phases skip only legacy center/corridor
checks and never this full-footprint check. `robot_center_link` alone is no
longer the boundary decision point.

<!-- HH_260803 - Synchronize control geometry with the axle-midpoint base. -->
Control now consumes `robot_center_link` at the front/rear axle midpoint. The
same physical planning boundary changed only in X coordinates: front/rear
`1.30137/0.39023 m` from the old rear-axle frame became
`0.85837/0.83323 m` from the center. Left/right `0.63505/0.63495 m`, all
boundary costs, margins, and stop behavior are unchanged. AprilTag
longitudinal thresholds add 0.443 m to preserve the same physical charger stop
points; see `docs/ROBOT_CENTER_LINK_MIGRATION.md`.

<!-- HH_260731 - Document projected reverse/crab recovery candidates without
     weakening ordinary footprint checks or implying automatic motion. -->
When this full-footprint or directional lanelet check stops an active command,
the gate enters `ROUTE_SAFETY_HOLD` and saves the original translation vector.
The same direction remains blocked until fresh lanelet grid and pose evidence
proves the saved full-footprint route clear for 1.0 s. Missing, stale, or
frame-mismatched evidence fails closed; pose evidence older than 0.5 s is stale.
A reverse candidate is considered when its translation cosine against the
trigger is at most `-0.5`. A pure orthogonal crab candidate is also admitted so
a side footprint contact caused during forward travel can move sideways rather
than farther along the blocked route. A rotation-only trigger may likewise
consider a translational candidate. Zero/rotation commands still cannot clear
or redefine the hold.

The escape exception is bounded to present map-boundary contact. At the default
0.25 m probe distance, the projected complete footprint must be clear of cost
100 and unknown/out-of-grid cells. The actual current escape corridor must still
pass LiDAR, radar, merged-grid, retained dynamic-latch, engage, ESTOP, CAN,
charging, battery, and command-timeout checks. Ordinary motion never uses this
exception, and no lanelet threshold or footprint extent is reduced.

<!-- HH_260803 - Document the production-owned automatic recovery policy. -->
The gate publishes a single validated candidate on
`/control/route_safety_recovery/candidate`; the separate
`route_safety_recovery_controller` is the only automatic raw-command owner.
Exactly one clear lateral candidate selects pure crab away from the contacted
side. If both lateral candidates are blocked and reverse alone is clear, it
selects reverse. Both lateral candidates clear is ambiguous and both blocked is
unsafe, so either case remains stopped. The first unambiguous direction is
latched for that hold and can never switch sides while the robot is moving.

Recovery raw speed is limited to 0.10 m/s, accumulated travel to 0.40 m, and
elapsed time to 10 s. The normal 0.5 gate output scale produces at most 0.05
m/s at the platform command. Missing or stale gate, candidate, or pose input;
operator cancel; any ordinary authorization failure; and the distance/time
limit all force zero. Angular velocity is always zero during contact recovery:
rotating the 1.69160 x 1.27000 m rectangle would require a swept-footprint
collision proof that is not available. Once 1.0 s of clear evidence releases
the hold, the retained RPP goal resumes ordinary yaw control.

The measured crab/reverse runs and production-owner animation are in
[the automatic boundary recovery report](../docs/AUTOMATIC_BOUNDARY_RECOVERY_SIM_20260803.md)
and [recovery GIF](../docs/assets/20260803/automatic_route_recovery_20260803.gif).

<!-- HH_260729 - Record the runtime bounds that prevent recovery tuning from
     disabling freshness or extending the lanelet-contact exception. -->
Runtime updates are rejected unless clear time is in `[0.1, 30.0] s`, pose
freshness is in `[0.05, 5.0] s`, opposite projection is in `[0.05, 0.5] m`,
direction cosine is in `[-1.0, 0.0]`, and log interval is in `[0.1, 60.0] s`.
Rejection is atomic: the prior active safety configuration remains unchanged.

<!-- HH_260721 - Distinguish loaded configuration from method-conditional and disabled files. -->

Configuration activation:

- `cmd_vel_safety_gate.yaml`: loaded by the active gate; every YAML key maps to a declared parameter.
- `control.yaml`: loaded by both campsite and drop-zone controllers, which are enabled by default.
- `parking.yaml`: loaded for the selected controller only. `reverse` is the default; the AprilTag section becomes active only with `parking_method:=apriltag`.
- `yaw_alignment_zones.yaml`: optional and byte-mirrored into bringup, but not read while `enable_yaw_alignment_zone: false` remains the default.
