# B1-B10 service endurance (2026-08-07)

<!-- HH_260807 - Preserve the final no-restart service acceptance, its exact
scope, and the field limitations beside the raw report and rendered results. -->

This folder records the final AMD64 deterministic-simulation acceptance for
the v2.1.5 service controller. It is not a Jetson or physical-road result.

## Result

| Check | Result | Measured value |
|---|---|---:|
| Continuous service | PASS | B1-B10 `10/10`, `2210.611 s`, bringup restart `0` |
| Full outbound scope | PASS | Cycle 1 seeded at the B1 route endpoint; cycles 2-10 completed charger departure and the full outbound route |
| Site service | PASS | Every cycle reached crab-in, completed 180-degree turn, unload wait, explicit RETURN, crab-out and route-snap return |
| Drop-zone lifecycle | PASS | Every cycle reached alignment, reverse parking, waiting for charging and charging |
| Charger recall | PASS | Cycles 2-10 reached `DEPARTING_CHARGER`, disconnected charge feedback and accepted the next site |
| Transient obstacle | PASS | B5 final command stopped, obstacle cleared, and the same mission resumed |
| Planning margin | PASS | B2-B10: hold `9`, recovery motion `9`, release `9`, retry latch `0` |
| Return handoff | PASS | Route-snap completion error `0.03-0.04 m`; B4 reduced an observed `0.27 m` Nav2 arrival offset to `0.04 m` |
| Diagnostics | PASS | Post-service-start system/path faults `0`; process faults and run-owned residual processes `0` |

The transient obstacle is intentionally shorter than the separate 20-second
fallback threshold. It proves immediate stop and same-mission resume. The
current map has no validated lane wide enough to claim a successful obstacle
bypass; the existing narrow-lane result remains a safe no-path hold, and a
surveyed wide-lane bypass remains field-pending.

## Files

| File | Purpose |
|---|---|
| `b1-b10-service-endurance.json` | Raw validation-runner report |
| `run-scope.json` | Exact route scope, acceptance counts and claim limits |
| `service-events.log` | Filtered public states, anchors, boundary holds and charging events |
| `b1-b4-return-anchor.json` | Focused B1/B4 service report |
| `b4-return-anchor.log` | B4 arrival offset and final anchor-error source lines |
| `b4-return-anchor-geometry.json` | Map width, clearance and selected return contract |
| `path-grace-smoke.log` | Same-goal `empty_route` handoff while `/system` remains OK |
| `ui-clean-shutdown.log` | Robot and Guest backends ready, SIGINT, clean exit |
| `b1-b10-service-endurance.png` | Ten-cycle summary generated from the raw report |
| `b1-b10-service-endurance.gif` | One measured frame per completed cycle |
| `evidence-manifest.json` | SHA-256 inventory and acceptance statement |

## Reproduce

```bash
ROS_DOMAIN_ID=153 ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false enable_operator_ui_window:=false \
  enable_api_ui:=true api_ui_port:=18117 enable_guest_ui:=false \
  enable_lidar_cost_grid:=false \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm

ROS_DOMAIN_ID=153 ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p run_gate_matrix:=false -p skip_manual_goal:=true \
  -p run_service_soak:=true -p simulate_platform_status:=true \
  -p charging_recall_via_ui:=true \
  -p service_soak_mission_keys:="['camping_site_1','camping_site_2','camping_site_3','camping_site_4','camping_site_5','camping_site_6','camping_site_7','camping_site_8','camping_site_9','camping_site_10']" \
  -p service_soak_obstacle_cycle:=5 \
  -p service_soak_cycle_timeout_s:=300.0

python3 camrod_bringup/scripts/visualization/render_v2_1_5_service_results.py \
  --endurance-report \
  docs/assets/module-guides/bringup/test-results/b1-b10-service-endurance-20260807/b1-b10-service-endurance.json
```

The first cycle is prepared near B1 by the validation harness. It must not be
reported as a charger-to-B1 outbound run. The remaining nine cycles are full
charger-departure services in one uninterrupted bringup process.
