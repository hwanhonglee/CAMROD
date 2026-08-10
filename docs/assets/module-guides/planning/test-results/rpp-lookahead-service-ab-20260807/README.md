# RPP lookahead service A/B (2026-08-07)

<!-- HH_260807 - Preserve the selected fixed-preview A/B inputs, raw event
excerpts, renderer output and checksum inventory as one auditable result. -->

This folder records the AMD64 deterministic-simulation decision for the
3 km/h RPP preview. It is not a Jetson or physical-road measurement.

## Result

| Profile | Effective preview | B1/B2 service | B2 boundary result |
|---|---:|---:|---|
| Velocity-scaled | about 1.5 m | FAIL | released, then same-boundary recontact after 0.850 s; retry latch |
| Fixed | 1.1 m | 2/2 PASS | `REVERSE_YAW_RIGHT`, released, no retry latch; return/park/charge complete |

The rejected run used the same map-v17 footprint, 3 km/h command profile and
recovery policy. Increasing the automatic retry count was rejected because it
repeats the approach that immediately recreated the same contact. The selected
run also proved obstacle stop, obstacle clear/resume, explicit RETURN after
`WAITING_FOR_RETURN_REQUEST`, drop-zone parking and charging.

## Files

- `comparison.json`: compact A/B inputs used by the renderer.
- `velocity-scaled-gate.log`: rejected-run gate events.
- `fixed-lookahead-gate.log`: selected source-config gate events.
- `fixed-lookahead-service.json`: complete selected B1/B2 runner report.
- `rpp-lookahead-service-ab.png`: human-readable comparison generated from
  `comparison.json`.
- `evidence-manifest.json`: SHA-256 inventory and measured claim boundary.

## Reproduce

```bash
ROS_DOMAIN_ID=141 ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false enable_operator_ui_window:=false \
  enable_api_ui:=true enable_guest_ui:=false \
  enable_lidar_cost_grid:=false \
  map_path:=/home/hong/camrod_ws/src/lanelet2_maps.osm

ROS_DOMAIN_ID=141 ros2 run camrod_bringup sim_validation_runner.py --ros-args \
  -p run_gate_matrix:=false -p skip_manual_goal:=true \
  -p run_service_soak:=true -p simulate_platform_status:=true \
  -p charging_recall_via_ui:=true \
  -p service_soak_mission_keys:="['camping_site_1','camping_site_2']" \
  -p service_soak_obstacle_cycle:=2 \
  -p service_soak_cycle_timeout_s:=300.0
```

`/platform/status` must have exactly one publisher,
`ranger_platform_bridge`; the runner only observes that contract.
