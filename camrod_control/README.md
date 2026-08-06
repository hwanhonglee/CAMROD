# camrod_control

<!-- HH_260805 - Document adaptive reverse/yaw/crab recovery while preserving
the measured-body stop and historical translation-only evidence labels. -->
<!-- HH_260806 - Separate the provisional physical-body hard stop from the
recoverable 5 cm planning margin and publish fresh map-v15 simulation results. -->

Native C++ motion owners for the final command gate, campsite/drop-zone local
maneuvers, parking, and bounded map-boundary recovery.

![Control command safety and recovery](../docs/assets/module-guides/control/command-safety-and-recovery.png)

## Actual Simulation Runtime

| Boundary and route in RViz | Gate state, zero output, and event times |
|---|---|
| ![Live boundary retry latch](../docs/assets/module-guides/control/runtime-boundary-retry-latch-20260804.png) | ![Live gate retry latch](../docs/assets/module-guides/control/runtime-retry-latch-terminal-20260804.png) |

`SIM RUNTIME CAPTURE`: with map v14, after one automatic release the same B6
boundary was recontacted in `0.276 s`; the gate latched `ROUTE_SAFETY_HOLD`, stopped issuing
recovery candidates, and published an all-zero final command.

## At A Glance

| Uses | Function | Main output |
|---|---|---|
| Nav2 command, local maneuver commands, parking, and recovery | Selects one raw command owner | `/control/cmd_vel_raw` |
| SOC, CAN, localization, LiDAR/radar/map costs | Applies the final motion authorization policy | `/control/cmd_vel`, `/control/cmd_vel_ros` |
| Physical body, planning margin, and projected candidates | Hard-stops body contact; permits only bounded margin escape | Gate/candidate/controller status |

## Active Safety Values

| Policy | Value | Result |
|---|---:|---|
| New campsite mission | `SOC >= 35%` | Departure admitted |
| Critical battery | `SOC <= 20%` | Hard command stop |
| Command timeout | `0.35 s` | Stale command becomes zero |
| Physical body | front/rear `0.65837/0.63323 m` | Cost 100/unknown is `lanelet_physical_body_cost`; no automatic motion |
| Physical body | left/right `0.43505/0.43495 m` | Independent hard-stop rectangle |
| Planning footprint | front/rear `0.70837/0.68323 m` | Body plus `0.05 m` X margin |
| Planning footprint | left/right `0.48505/0.48495 m` | Body plus `0.05 m` Y margin |
| Planning-margin stop | cost `100` or unknown | `lanelet_footprint_cost`; ordinary command remains zero |
| Soft lane edge | cost `98` | Traversable planning bias, not the hard body stop |
| Dynamic cost stop | threshold `85` | LiDAR/radar/merged hazard hold |
| Route-clear proof | `1.0 s` | Releases retained route hold |
| Automatic release budget | `1` | Allows one bounded Nav2 resume attempt |
| Rapid recontact window | `5.0 s` | Same-route recontact latches until operator stop/replan |
| Recovery proof probe | `0.25 m` | Projected complete footprint must be clear |
| Recovery owner | `0.10 m/s`, `0.40 m`, `10 s` | Maximum raw speed, total travel, duration |
| Contact recovery yaw | `0.10 rad/s`, `12 deg` | Bounded reverse-yaw only after projected full-footprint proof |

## Runtime Owners

| Node | Responsibility |
|---|---|
| `cmd_vel_safety_gate` | Final engage, platform, battery, localization, timeout, obstacle, and lanelet authorization |
| `route_safety_recovery_controller` | Executes projected crab/reverse/reverse-yaw stages within one shared speed/distance/yaw/time budget |
| `camping_site_maneuver_controller` | Site entry, arrival turnaround, unload wait, explicit return, and exit |
| `drop_zone_maneuver_controller` | Charger/drop-zone exit, route yaw alignment, and parking handoff |
| `reverse_parking_controller` | Yaw-aware reverse travel and charging wait |
| `apriltag_parking_controller` | Rear-camera tag approach, insertion, retry, and charging completion |

## Command Path

```text
/control/nav2_cmd_vel_ros -----------+
/control/cmd_vel_raw ----------------+--> cmd_vel_safety_gate
                                          |--> /control/cmd_vel
                                          +--> /control/cmd_vel_ros --> Ranger
```

`/control/planning_engaged` reports manual-or-mission authorization.
`/control/command_enabled` can still be false during a safety, CAN, charging,
localization, or battery hold.

## Mission Maneuvers

| Site mode | Phase order |
|---|---|
| Active B1-B13 turnaround | `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT` |
| Optional roadside policy | Supported by the controller but not selected by the active campsite configuration |
| Drop-zone departure | `EXIT_STRAIGHT -> ALIGN_EXIT_YAW -> route release` |
| Return parking | route arrival -> body alignment -> selected parking controller |

The low-battery latch does not auto-return while people may be unloading. If
SOC falls below 35% during a campsite mission, the current site phase finishes
and the normal user return request remains required.

## Parking And Charging State

| Condition | Controller phase | Public `/service/state` |
|---|---|---|
| Final pose not reached | active parking phase | `DROP_ZONE_PARKING` |
| Contact pose reached; CAN charge absent | `WAIT_FOR_CHARGING` | `WAITING_FOR_CHARGING` |
| Parking complete; CAN charge true | `PARKED` | `CHARGING` |
| Parked; no active charge feedback | `PARKED` | `DROP_ZONE_WAIT` |
| New allowed mission while charging | departure override | `DEPARTING_CHARGER` |

The reverse controller defaults to `complete_without_charging: false` and a
`20 s` charging wait. `CHARGING` therefore takes precedence on the public state
surface while the internal controller remains `PARKED`.

## Boundary Evidence

| First complete-footprint stop | Narrow-route risk map |
|---|---|
| ![First boundary stop](../docs/assets/module-guides/control/first-route-boundary-stop-location.png) | ![Narrow-route risk](../docs/assets/module-guides/planning/robot-center-narrow-route-risk-map.png) |

| Earlier manual probe | Current automatic owner |
|---|---|
| ![Manual recovery probe](../docs/assets/module-guides/control/pre-owner-robot-center-contact-sheet.png) | ![Automatic recovery milestones](../docs/assets/module-guides/control/automatic-owner-route-retry-contact-sheet.png) |

![Automatic boundary recovery](../docs/assets/module-guides/control/automatic-owner-route-retry.gif)

| v2.1.4 release-map staged recovery | Current decision policy |
|---|---|
| ![Map v15 recovery contact sheet](../docs/assets/module-guides/control/map-v15-boundary-recovery-contact-sheet.png) | ![Map v15 recovery policy](../docs/assets/module-guides/control/map-v15-boundary-recovery-policy.png) |

![Map-v15 reverse-yaw, retry latch, and crab recovery](../docs/assets/module-guides/control/map-v15-boundary-recovery.gif)

| Historical map-v14 measured rerun | Historical translation-only decision policy |
|---|---|
| ![Map v14 recovery contact sheet](../docs/assets/module-guides/control/map-v14-boundary-recovery-contact-sheet.png) | ![Recovery policy](../docs/assets/module-guides/control/map-v14-boundary-recovery-policy.png) |

![Map-v14 reverse, retry latch, and crab recovery](../docs/assets/module-guides/control/map-v14-boundary-recovery.gif)

<!-- HH_260806 - Link the current-map live retry and physical-clearance diagnosis. -->
### Current-map live diagnosis (2026-08-06)

| Margin-only first stop | Ten-release behavior | Body-only diagnostic |
|---|---|---|
| ![Margin contact](../docs/assets/test_result/route-boundary-recovery-20260806/01-margin-contact-analysis.png) | ![Ten-release timeline](../docs/assets/test_result/route-boundary-recovery-20260806/02-ten-release-retry-timeline.png) | ![Body contact segment](../docs/assets/test_result/route-boundary-recovery-20260806/03-body-only-drive-trajectory.png) |

The [full test record](../docs/assets/test_result/route-boundary-recovery-20260806/README.md)
uses the previous `1.49160 x 1.07000 m` body and separates a margin-only first
stop from a downstream physical-body map contact. Raising the release limit
from `1` to `10` repeated the same reverse/re-entry behavior and selected no
crab, so retry count alone is not a valid fix. The
[boundary adjustment replay](../docs/assets/test_result/robot-boundary-adjustment-20260806/README.md)
preserves that replay and adds fresh full-bringup tests of the final two-layer
policy. These controlled route tests do not replace the separate campsite
`GOAL_REACHED -> CRAB_IN` mission validation.

![Fresh physical-body and planning-margin policy](../docs/assets/test_result/robot-boundary-adjustment-20260806/02-runtime-boundary-policy.png)

| Simulation case | Measured result | Verdict |
|---|---:|---|
| One lateral side clear | Crab displacement `0.3375 m`; output `<= 0.05 m/s` | Hold released; mission not complete |
| Both sides blocked, reverse selected | Release displacement `0.0327 m` in static case | Bounded reverse works |
| Same-goal RPP retry | `0.4726 m`, yaw `-2.0008 deg` | Normal yaw resumed after release |
| Later map boundary | Second hold observed | Map corridor remains limiting |
| Live B6 retry containment | map v14 `0.276 s` (v13 `0.267/0.372 s`); final Twist all zero | Retry loop stopped; operator action required |
| Map-v14 route probe | reverse `0.0703 m`, retry `0.0661 m` / yaw `-0.1235 deg`, recontact `0.366 s` | Retry latched; final Twist all zero |
| Map-v14 static/crab probes | reverse `0.0721 m`; crab-left `0.3321 m` | Both bounded motions released the first hold |
| Map-v15 route probe | `REVERSE_YAW_RIGHT`, recovery `0.0582 m`, max yaw `0.05 rad/s`, recontact `0.335 s` | Retry latched; final Twist all zero |
| Map-v15 static probe | `REVERSE_YAW_RIGHT`, recovery `0.0405 m`, max yaw `0.05 rad/s`, recontact `0.400 s` | Retry latched; final Twist all zero |
| Map-v15 one-side probe | `CRAB_LEFT`, `0.3378 m`, max lateral `0.05 m/s` | First hold released without recontact; mission incomplete |
| Current-map normal route after boundary change | `10.0403 m`, goal error `0.2932 m`, no route hold | Controlled route completed; final Twist zero |
| Current-map margin-only contact | body max `70`, planning max `100`; ordinary output `0.0 m/s` | Hold enforced without body contact |
| Current-map margin recovery | `CRAB_RIGHT`, `0.133 m`, max lateral `0.05 m/s` | Both envelopes clear at release; final Twist zero |
| Current-map physical-body contact | body max `100`; owner motion false; recovery output `0.0 m/s` | Hold retained; no automatic escape |

The map-v14 PNG/GIF remains historical translation-only evidence. The map-v15
PNG/GIF is a v2.1.4 release-map full-simulation run of the active selector; its
OSM SHA is `e0b50f...e36d`, not the current-site map SHA `d7b730...213f`. It
reevaluates all five projected commands on every hold update: left/right crab, straight
reverse, and left/right reverse-yaw. A unique safe crab moves away from the
contact. Otherwise straight reverse creates room; it may transition to the
unique safe yaw arc, or to the original RPP turn sign when both arcs are clear.
Each stage must keep the complete projected footprint, fresh lanelet grid/pose,
and dynamic obstacle checks clear. A blocked active crab can use straight
reverse to reposition, then be reevaluated.

After a maximum `12 deg` heading correction, the owner removes angular velocity
and continues only a separately checked reverse translation. The total
`0.40 m`/`10 s` budget is not reset during a stage transition. Only one route
release is permitted; same-route recontact within 5 seconds latches zero output
until stop/replan/re-engage. The staged selector and swept projection are unit
validated and were exercised dynamically on the release map-v15 geometry. Both
route retry cases remained fail-closed rather than completing a mission;
physical-robot recovery remains pending. The current AMD64 result additionally
proves that a physical-body cost-100 contact is not a recoverable state.

## Configuration

| File | Owns |
|---|---|
| `config/cmd_vel_safety_gate.yaml` | Authorization, battery, timeout, dynamic cost, footprint, and recovery-candidate policy |
| `config/control.yaml` | Campsite, drop-zone, and recovery-owner motion values |
| `config/parking.yaml` | Reverse and AprilTag parking values |
| `config/yaw_alignment_zones.yaml` | Optional map yaw-lock zones; disabled by default |

## Run And Validate

```bash
ros2 launch camrod_control cmd_vel_safety_gate.launch.py
ros2 launch camrod_control maneuvers.launch.py
ros2 launch camrod_control parking.launch.py parking_method:=reverse

ros2 topic echo /control/cmd_vel_safety_gate/status
ros2 topic echo /control/route_safety_recovery/candidate
ros2 topic echo /control/route_safety_recovery_controller/status
```

Detailed timelines and reproduction commands are in the
[boundary recovery validation](../docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md)
and the [v2.1.4 release-map evidence directory](../docs/evidence/v2.1.4/map-v15-boundary-recovery/).
