# CAMROD v2.1.3 UI and Boundary Recovery Validation

Release checkpoint: 2026-08-04 (Asia/Seoul)

<!-- HH_260803 - Keep simulation evidence separate from real-robot acceptance. -->
This report records software and simulation behavior after the
`robot_center_link` migration. It is not a real-robot driving acceptance.

## Guest UI Contract

The Guest UI and Robot UI now use one mission owner:

1. Guest campsite selection publishes `/ui/selected_destination`.
2. The Robot backend validates the stationary state and battery policy, then
   runs `DEPARTING_DROP_ZONE` or `DEPARTING_CHARGER` before `MOVING_TO_SITE`.
3. Site maneuver states remain control-owned: `SITE_ENTRY`, `UNLOAD_WAIT`, and
   `WAITING_FOR_RETURN_REQUEST`.
4. Guest `usage_complete` publishes `MotionOperation.RETURN` to the UI request
   topic. The Robot backend changes state to `RETURNING_TO_DROP_ZONE`, clears
   manual engage, enables mission engage, and commands the campsite controller.
5. Parking and charger feedback continue through `DROP_ZONE_PARKING`,
   `WAITING_FOR_CHARGING`, and `CHARGING`.

The ROS/WebSocket integration run observed all of the following: 80% normalized
battery, 13 sites, `camping_site_6`, departure, moving, return state, controller
RETURN operation, `ROUTE_SAFETY_HOLD` overlay, and `OPERATOR_STOPPED`. New site
dispatch is rejected below 35% SOC. Generic diagnostic WARN remains health
information and does not overwrite a normal service phase.

Result JSON:
[guest UI integration evidence](evidence/v2.1.3/ui/guest-mission-lifecycle.json)

![Guest UI ready for campsite dispatch](assets/module-guides/ui/guest-mission-dispatch-ready.png)

![Guest UI displaying a route safety hold](assets/module-guides/ui/guest-route-safety-hold.png)

## Recovery Policy

The final command chain is:

```text
lanelet contact -> cmd_vel_safety_gate candidate
                -> route_safety_recovery_controller raw command
                -> cmd_vel_safety_gate full interlock check
                -> Ranger command
```

- One lateral direction clear: pure crab away from the contacted side.
- Both lateral directions blocked and reverse clear: reverse.
- Both lateral directions clear: stop because the side is ambiguous.
- All candidates blocked or evidence stale: stop.
- Raw speed: at most 0.10 m/s; final configured output: at most 0.05 m/s.
- Bounds: 0.40 m travel, 10 s, 0.5 s pose age, 12 s static lanelet-grid age,
  and 1.0 s continuous clear evidence.
- Recovery angular velocity is zero. After release, the retained RPP path
  resumes and controls yaw normally.

The no-rotation rule is deliberate. A rotating 1.69160 x 1.27000 m planning
boundary sweeps corners through cells that are not covered by the current
translation-only projection. Allowing yaw at contact without swept-footprint
proof would weaken the boundary guarantee.

## Evidence Progression

<!-- HH_260804 / v2.1.3 - Keep historical manual-candidate runs because they
     explain what the gate proved before an automatic command owner existed. -->

The release evidence is intentionally split by command ownership:

| Stage | Reference and yaw | First hold | Retry distance | Meaning |
|---|---|---:|---:|---|
| pre-owner manual candidate | rear axle, fixed-yaw display | `(3.0825, 45.3403)` | 0.4719 m | gate admitted clear reverse/right-crab probes, but generated no recovery command |
| pre-owner manual candidate | rear axle, route yaw | `(3.0937, 45.3399)` | 0.4520 m | normal drive yaw followed the curve; manual reverse released the hold |
| pre-owner manual candidate | `robot_center_link` | `(3.9277, 45.2451)` | 0.5008 m | center reference progressed farther, then the same narrow curve stopped the retry |
| current automatic owner | `robot_center_link` | scenario-dependent | bounded by 0.40 m | controller publishes only the gate-selected crab/reverse command |

Historical visuals:
[fixed-yaw manual probe](assets/module-guides/control/pre-owner-manual-no-yaw.gif),
[yaw-aware manual probe](assets/module-guides/control/pre-owner-manual-yaw-aware.gif),
and [center-reference manual probe](assets/module-guides/control/pre-owner-robot-center-recovery.gif).
Their source timelines are stored beside the automatic results under
`docs/evidence/v2.1.3/boundary-recovery/`.

## Measured Runs

| Scenario | Automatic motion | Recovery result | Same-goal result |
|---|---|---|---|
| one-sided right contact | `CRAB_LEFT` | 0.3375 m, final lateral 0.05 m/s, yaw held | hold released after 1.0 s clear evidence |
| stationary route contact | `REVERSE` | final linear.x -0.05 m/s, 0.0327 m before release | 0.0592 m and +0.4065 deg yaw, then next hold |
| moving route contact | `REVERSE` selected | residual deceleration cleared the first hold before reverse output | 0.4726 m and -2.0008 deg yaw, then next hold |

![Automatic recovery policy](assets/module-guides/control/automatic-owner-policy.png)

![Automatic recovery contact sheet](assets/module-guides/control/automatic-owner-route-retry-contact-sheet.png)

[Open the automatic recovery GIF](assets/module-guides/control/automatic-owner-route-retry.gif).

The route cases intentionally prove bounded retry and repeat stop. They do not
prove mission completion: lanelets 754/2751/2720 include a corridor narrower
than the active planning rectangle. More crab or reducing the measured
footprint would not make that map valid. The lanelet geometry or operational
route must be corrected before that route can pass end to end.

## Reproduction

Start ordinary simulation without UI windows, then run each probe with the
installed semantic map path:

```bash
ros2 run camrod_bringup automatic_route_recovery_probe.py \
  --map <lanelet2_maps.osm> \
  --scenario one_sided_crab \
  --output /tmp/automatic_crab.json

ros2 run camrod_bringup automatic_route_recovery_probe.py \
  --map <lanelet2_maps.osm> \
  --scenario static_reverse_retry \
  --output /tmp/automatic_reverse.json

ros2 run camrod_bringup automatic_route_recovery_probe.py \
  --map <lanelet2_maps.osm> \
  --scenario route_retry \
  --output /tmp/automatic_route.json

# Rebuild the automatic GIF/contact sheet/policy PNG from all three timelines.
ros2 run camrod_bringup render_automatic_recovery_results.py \
  --map <lanelet2_maps.osm> \
  --route /tmp/automatic_route.json \
  --reverse /tmp/automatic_reverse.json \
  --crab /tmp/automatic_crab.json \
  --output-dir docs/assets/module-guides/control

# Rebuild only the earlier manual-candidate center-frame comparison.
ros2 run camrod_bringup render_boundary_recovery_results.py \
  --map <lanelet2_maps.osm> \
  --run docs/evidence/v2.1.3/boundary-recovery/pre-owner-robot-center-timeline.json \
  --output-dir docs/assets/module-guides/control \
  --planning-output-dir docs/assets/module-guides/planning \
  --analysis-output docs/evidence/v2.1.3/boundary-geometry/robot-center-route-samples.json
```

Current automatic-owner source evidence:

- [One-sided crab JSON](evidence/v2.1.3/boundary-recovery/automatic-owner-one-sided-crab.json)
- [Stationary reverse JSON](evidence/v2.1.3/boundary-recovery/automatic-owner-static-reverse-retry.json)
- [Moving route retry JSON](evidence/v2.1.3/boundary-recovery/automatic-owner-route-retry.json)

## Remaining Field Work

Run `field_test_tool.sh record-recovery <log_dir>` on the Jetson before moving.
Verify left and right contact separately, rear obstacle blocking, operator
cancel, distance/time bounds, actual wheel directions, and final command zero.
Do not mark TODO 11 or 12 FIELD-PASS from these simulation files alone.
