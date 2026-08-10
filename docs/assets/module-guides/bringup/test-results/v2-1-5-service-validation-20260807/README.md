# v2.1.5 service validation (2026-08-07)

This directory records the final amd64 ROS 2 simulation acceptance used for
the `v2.1.5` release update. It is **simulation evidence**, not Jetson or field
evidence. The active input was map v17 with SHA-256
`8cd05c66f846cae8718b5af148d123718f403a086f2e7d16165da89fb625e021`.

![Three-cycle service result](repeated-service-summary.png)

![Repeated service timeline](repeated-service-timeline.gif)

## Acceptance result

| Scenario | Result | Observed contract |
|---|---|---|
| Repeated service, B1 -> B2 -> B3 | PASS, 3/3 | Site route, crab-in, zero-turn, explicit RETURN, return route, drop alignment, reverse parking, charging, and next departure |
| Dynamic obstacle in cycle 2 | PASS | Immediate stop, 3 s clear, same mission resumed |
| Boundary recovery in all cycles | PASS | Recovery motion observed, 1.5 s clear release, no retry latch |
| Charger departure for B2/B3 | PASS | `DEPARTING_CHARGER` remained authorized while charger contact was still asserted |
| Bringup continuity | PASS | 677.237 s, zero restart |

The complete state and controller sequences are in
[`repeated-service-soak.json`](repeated-service-soak.json). The concise runner
output is in [`repeated-service-soak.log`](repeated-service-soak.log).

## Persistent obstacle result

![Persistent obstacle safe hold](obstacle-safe-hold.png)

The widest mapped road lanelet measured about `3.00 m`. A centered obstacle,
the `1.17 m` planning footprint, and the configured Nav2 inflation left no safe
SmacLattice path. This is therefore a safe no-path acceptance, not an avoidance
claim:

1. The command gate stopped immediately.
2. The monitor waited for 20 s of continuous blockage.
3. `ComputePathToPose(SmacLattice)` ran once without replacing LaneletRoute.
4. The failed result latched; no fallback selector or repeated Nav2 ABORT was produced.
5. Removing the obstacle resumed the original goal and moved `0.242 m` without a new command.

The machine-readable result is
[`obstacle-safe-hold.json`](obstacle-safe-hold.json). The exact one-shot monitor
and planner events are in [`obstacle-replan-monitor.log`](obstacle-replan-monitor.log)
and [`obstacle-planner-server.log`](obstacle-planner-server.log). A successful free-space
avoidance runtime claim remains pending a surveyed/wider lane where the active
footprint and inflation can fit on one side of the obstacle.

## B2 boundary recovery

![B2 boundary recovery repeatability](b2-boundary-recovery.png)

| Trial | Selected stage | Recovery displacement | Second hold | Mission |
|---:|---|---:|---|---|
| 1 | `REVERSE_YAW_RIGHT` | 0.0202 m | none | PASS |
| 2 | `REVERSE_YAW_RIGHT` | 0.0742 m | none | PASS |
| 3 | `REVERSE_YAW_RIGHT` | 0.0715 m | none | PASS |

The recovery sweep rejected any trajectory whose physical body touched the
lanelet boundary. The planning margin remained recoverable, and the 1.5 s clear
timer began only after a recovery command was admitted by the final gate.

## Reproduction

```bash
ROS_DOMAIN_ID=126 ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false enable_operator_ui_window:=false \
  enable_api_ui:=true enable_guest_ui:=false enable_lidar_cost_grid:=false \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm

ROS_DOMAIN_ID=126 ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p run_gate_matrix:=false -p skip_manual_goal:=true \
  -p run_service_soak:=true -p simulate_platform_status:=true \
  -p charging_recall_via_ui:=true -p service_soak_cycle_timeout_s:=300.0 \
  -p report_file:=/tmp/v2_1_5-repeated-service-soak.json
```

The obstacle run used `ROS_DOMAIN_ID=132`, enabled the optional LiDAR cost-grid,
seeded `(78.9961, -11.2035, 5 deg)`, placed a map-fixed obstacle `3.0 m` ahead,
and selected `obstacle_replan_expect_safe_hold:=true`.

Regenerate the visuals with:

```bash
python3 camrod_bringup/scripts/visualization/render_v2_1_5_service_results.py \
  --evidence-dir docs/assets/module-guides/bringup/test-results/v2-1-5-service-validation-20260807
```

## Remaining field acceptance

- Repeat at least ten service cycles on Jetson with physical CAN charging feedback.
- Measure a genuinely avoidable wide lane before claiming SmacLattice obstacle bypass.
- Exercise physical planning-margin contact and physical-body hard stop separately.
- Record Jetson CPU/GPU/RSS and component-container stability for the same scenario.
