# camrod_control

<!-- HH_260805 - Document adaptive reverse/yaw/crab recovery while preserving
the measured-body stop and historical translation-only evidence labels. -->
<!-- HH_260806 - Activate the fabrication-inclusive measured body, preserve the
10 cm planning margin, and serialize Nav2/campsite command ownership. -->
<!-- HH_260806 - Remove artificial Nav2 rotation/translation handoffs and use a
high-resolution local lanelet raster for final command safety. -->
<!-- HH_260806 - Preserve maneuver ratios under the 3 km/h production cruise profile. -->
<!-- HH_260807 - Require observed recovery motion before the 1.5 s clear release. -->
<!-- HH_260807 - Evaluate route safety with the final scaled command and record
the fixed-preview service A/B. -->
<!-- HH_260807 - Reduce the production cruise to 2 km/h while preserving maneuver ratios. -->

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
| Physical body, planning margin, and projected candidates | Hard-stops ordinary contact; permits only bounded monotonic inward escape | Gate/candidate/controller status |

## Active Safety Values

<!-- HH_260809 - Tie hard-stop and planning-margin checks to the same
tapered-front rounded geometry published by camrod_platform. -->

| Policy | Value | Result |
|---|---:|---|
| New campsite mission | `SOC >= 35%` | Departure admitted |
| Critical battery | `SOC <= 20%` | Hard command stop |
| Command timeout | `0.35 s` | Stale command becomes zero |
| Physical body | front/rear `0.70837/0.68323 m` | Fabrication-inclusive `1.39160 m` hard-stop length; only projected monotonic inward escape on cost 100 |
| Physical body | left/right `0.53505/0.53495 m` | Fabrication-inclusive `1.07000 m` hard-stop width |
| Boundary contour | taper/depth `0.12/0.12 m`, physical `R0.05 m` | Short front face, sloped shoulders, rounded six-corner hard-stop polygon |
| Planning footprint | front/rear `0.80837/0.78323 m` | Body plus `0.10 m` X margin |
| Planning footprint | left/right `0.63505/0.63495 m` | Body plus `0.10 m` Y margin |
| Planning contour | exact `0.10 m` offset, `R0.15 m` | Published polygon and pre-topic fallback use identical geometry |
| Planning polygon frame | `robot_center_link` | Gate consumes local points directly; map-pose callback timing cannot shift the collision envelope |
| Planning-margin stop | cost `100` or unknown | `lanelet_footprint_cost`; ordinary command remains zero |
| Soft lane edge | cost `98` | Traversable planning bias, not the hard body stop |
| Lanelet safety raster | `600 x 600 @ 0.05 m` | Independent `30 m` local grid; avoids the Nav2 grid's `0.125 m` half-cell dilation |
| Dynamic cost stop | threshold `85` | LiDAR/radar/merged hazard hold |
| Route-clear proof | `1.5 s` after admitted recovery motion | Releases retained route hold |
| Automatic release budget | `12` | Allows twelve bounded Nav2 resume attempts |
| Rapid recontact window | `5.0 s` | Exhaustion blocks same-direction Nav2 resume; projected-safe inward escape remains available |
| Recovery proof probe | `0.25 m` | Projected complete footprint must be clear |
| Recovery owner | `0.10 m/s`, `0.40 m`, `10 s` | Maximum raw speed, total travel, duration |
| Contact recovery yaw | `0.10 rad/s`, `12 deg` | Bounded reverse-yaw only after projected full-footprint proof |
| Maneuver release hold | `0.5 s` | Final output remains zero before Nav2 may resume |
| Normal Nav2 rotation/translation | Continuous owner | RotationShim/RPP mode changes do not create an artificial zero handoff |

![Physical hard-stop and planning recovery contours](../docs/assets/test_result/tapered-rounded-boundary-20260810/tapered-rounded-boundary-geometry.png)

![Forward curve crab and zero-turn contour motion](../docs/assets/test_result/tapered-rounded-boundary-20260810/tapered-rounded-boundary-motion.gif)

<!-- HH_260810 - Keep contour transform documentation separate from the gate's
measured boundary-contact and automatic-recovery evidence below. -->
Both envelopes follow `robot_center_link` rigidly during forward, curved, crab,
and zero-turn motion. Candidate collision checks and actual release decisions
remain runtime control behavior; see the [source-derived geometry record](../docs/assets/test_result/tapered-rounded-boundary-20260810/README.md).

## Active Linear-Speed Profile

The final command gate applies `speed_scale=0.5`. The table reports limits at
the platform output after that gate; ordinary operational linear speeds retain
their previous ratio to the current `2.0 km/h` cruise reference.

| Operation | Cruise ratio | Final limit |
|---|---:|---:|
| RPP cruise | `100%` | `2.000 km/h` |
| RPP curvature floor | `50%` | `1.000 km/h` |
| RPP final approach | `25%` | `0.500 km/h` |
| Campsite crab | `60%` | `1.200 km/h` |
| Campsite reverse / drop-zone exit / reverse parking | `40%` | `0.800 km/h` |
| AprilTag approach | `50%` | `1.000 km/h` |
| AprilTag insertion | `12.5%` | `0.250 km/h` |
| Optional yaw-zone approach 1 / 2 | `62.5% / 50%` | `1.250 / 1.000 km/h` |
| Zero-turn / gross yaw alignment | `0%` | `0 km/h` |
| Route-boundary recovery safety exception | `9%` | `0.180 km/h` |

Boundary recovery is intentionally not proportionally increased: its raw
`0.10 m/s`, `0.40 m`, `10 s`, and `12 deg` limits remain a separate safety
budget. Angular-speed limits are unchanged. The previous 3 km/h raw/final
values and AMD64 command trace remain as historical evidence in the
[3 km/h test record](../docs/assets/test_result/three-kph-localization-20260806/README.md).

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
| Active B1-B10 turnaround | `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT` |
| Active B11-B13 roadside arrival | capped `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN`; no on-site zero-turn |
| B11-B13 return | Field geometry decision pending; arrival-only tests do not publish RETURN |
| Drop-zone departure | `EXIT_STRAIGHT -> ALIGN_EXIT_YAW -> route release` |
| Return parking | route arrival -> body alignment -> selected parking controller |

<!-- HH_260807 - Explain the route-snap return anchor that prevents a narrow
lane handoff from inheriting Nav2's early goal-reached pose. -->
For automatic B1-B10 service, the actual Nav2 arrival pose starts and measures
site entry, while `/planning/goal_pose_snapped` is retained as a separate return
anchor. `CRAB_OUT` corrects both body axes toward that anchor, slows
proportionally near it, and hands motion back only within `0.04 m`. Manual/adopt
operation keeps the actual arrival/adopted pose as its anchor. This distinction
matters on the current B4 lane: the observed Nav2 arrival was `0.27 m` from the
snap, while the measured planning-margin clearance was only about `0.136 m` per
side. A focused B1 -> B4 service run returned at `0.03/0.04 m`, completed both
parking/charging cycles, and produced no post-return body or margin hold.

The final B1-B10 endurance kept every route-snap handoff within `0.03-0.04 m`.
B2-B10 also completed nine planning-margin recoveries with observed recovery
motion, release, continued service, and zero retry latch.

![B1-B10 route-snap return and recovery endurance](../docs/assets/test_result/b1-b10-service-endurance-20260807/b1-b10-service-endurance.png)

The [control/service evidence](../docs/assets/test_result/b1-b10-service-endurance-20260807/README.md)
separates measured planning-margin recovery from the then-active no-motion
physical-body policy. The current monotonic-overlap/swept-clearance rule for a
virtual body-boundary contact remains physical field work.

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

## Campsite Sequencing Evidence

![Map-v16 campsite policy validation](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-policy-validation.png)

![Turnaround and roadside phase order](../docs/assets/test_result/camping-site-sequencing-20260806/campsite-phase-sequence.gif)

All B1-B10 sites completed the full turnaround/return sequence with map-derived
lateral entries from `1.79 m` to `5.31 m`. B11-B13 each stopped at
`WAIT_RETURN` after a `0.60 m`-capped roadside crab and never entered
`ROTATE_180`. The final gate ignores Nav2 commands while a campsite phase owns
motion, then enforces a stationary handoff. Static road-lanelet cost is bypassed
only for explicit campsite motion; live dynamic obstacle checks remain active.
The [structured result](../docs/assets/test_result/camping-site-sequencing-20260806/README.md)
keeps the unresolved B11-B13 return geometry explicitly field-pending.

## Boundary Evidence

<!-- HH_260810 - Put the current tapered/rounded contour on its measured ROS
gate/recovery timeline instead of relying on older rectangular illustrations. -->

![Current tapered rounded boundary road result](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/tapered-rounded-boundary-road-sim.png)

![Current gate hold reverse-yaw release and route completion](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/tapered-rounded-boundary-road-sim.gif)

In this [current measured ROS simulation](../docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/README.md),
the physical body sampled 648 clear cells with maximum cost `0`, while the
873-cell planning contour reached cost `100`. The gate held ordinary output,
issued bounded `REVERSE_YAW_RIGHT` at no more than `0.05 m/s` and `0.05 rad/s`,
released after `0.0972 m` of observed motion, and completed the retained route.
There was no second hold or retry latch; physical-road behavior is unverified.

![Current B2 recovery repeatability](../docs/assets/test_result/v2-1-5-service-validation-20260807/b2-boundary-recovery.png)

On map v17, three identical B2 recontact trials selected
`REVERSE_YAW_RIGHT`, completed the original mission, and produced no second
hold or rapid-recontact latch. Recovery displacement was `0.0202`, `0.0742`,
and `0.0715 m`; maximum recovery output was `0.05 m/s`. The clear timer starts
only after `recovery_motion_observed=true`, and every continuous yaw arc is
checked against the physical body before its endpoint planning margin.

![3 km/h RPP service A/B](../docs/assets/test_result/rpp-lookahead-service-ab-20260807/rpp-lookahead-service-ab.png)

The later [source-profile A/B](../docs/assets/test_result/rpp-lookahead-service-ab-20260807/README.md)
showed why increasing the retry count is not a control fix. A velocity-scaled
preview of about `1.5 m` re-entered the same boundary after `0.850 s`; fixed
`1.1 m` completed B1/B2 without a retry latch. The gate now projects, proves
recovery motion, logs, and publishes the same final speed-scaled command. At
the active cruise this means it evaluates `0.555556 m/s`, not the unscaled
`1.111111 m/s` controller request.

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
uses the earlier `1.49160 x 1.07000 m` body and separates a margin-only first
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
| Reduced-boundary normal route (historical) | `10.0403 m`, goal error `0.2932 m`, no route hold | Controlled route completed; final Twist zero |
| Reduced-boundary margin-only contact (historical) | body max `70`, planning max `100`; ordinary output `0.0 m/s` | Hold enforced without body contact |
| Reduced-boundary margin recovery (historical) | `CRAB_RIGHT`, `0.133 m`, max lateral `0.05 m/s` | Both envelopes clear at release; final Twist zero |
| Reduced-boundary physical-body contact (historical) | body max `100`; owner motion false; recovery output `0.0 m/s` | Hold retained; no automatic escape |
| B7 clear-road stop-go regression | `224.92 s`, `27.1492 m`; handoff `0`, stale `0` | Artificial rotation/translation stop cycle removed |
| RPP right-oversteer regression | raw rotation/translation switches `403 -> 0`; B8 `59.931 m` and `GOAL_REACHED` | Continuous curve tracking selected at `1.1 m` |
| 3 km/h preview A/B | scaled preview about `1.5 m`: recontact `0.850 s`; fixed `1.1 m`: B1/B2 `2/2`, restart `0` | Fixed profile selected; retry-count increase rejected |
| B1-B10 service endurance | route-snap error `0.03-0.04 m`; B2-B10 recovery hold/motion/release `9/9/9`; retry latch `0` | 10/10 service cycles completed without restart |

The map-v14 PNG/GIF remains historical translation-only evidence. The map-v15
PNG/GIF is a v2.1.4 release-map full-simulation run of the active selector; its
OSM SHA is `e0b50f...e36d`, not the current map-v17 SHA `8cd05c...5e021`. It
reevaluates all five projected commands on every hold update: left/right crab, straight
reverse, and left/right reverse-yaw. A unique safe crab moves away from the
contact. Otherwise straight reverse creates room; it may transition to the
unique safe yaw arc, or to the original RPP turn sign when both arcs are clear.
Each stage must keep the complete projected footprint, fresh lanelet grid/pose,
and dynamic obstacle checks clear. A blocked active crab can use straight
reverse to reposition, then be reevaluated.

The [B7 stop-go regression](../docs/assets/test_result/cmd-vel-stop-go-20260806/README.md)
separates the corrected clear-road command stream from a later real 5 cm
planning-margin contact. It does not claim that the complete B7 route is clear.

The [RPP curve-tracking comparison](../docs/assets/test_result/rpp-curve-tracking-20260806/README.md)
separates gross start alignment from ordinary steering. The gate performs one
zero-linear alignment for a large initial yaw error; it no longer receives
repeated pure-rotation requests at every 2-degree path bend.

After a maximum `12 deg` heading correction, the owner removes angular velocity
and continues only a separately checked reverse translation. The total
`0.40 m`/`10 s` budget is not reset during a stage transition. Up to 12 route
releases are permitted in the 5-second recontact window. Reaching that budget
blocks another same-direction Nav2 resume, but does not suppress an inward
escape candidate whose contact overlap decreases monotonically and whose swept
physical body plus endpoint planning footprint are clear. Dynamic obstacles
and platform interlocks remain fail-closed. Older map-v15 retry/body cases in
the evidence table retain their measured zero-output outcomes; the current
physical-robot recovery policy remains field-pending.

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
