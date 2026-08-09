# Tapered Rounded Boundary Road Simulation

<!-- HH_260810 - Preserve the current map-v17 ROS run, exact contour overlay,
and recovery result without presenting simulation evidence as a field result. -->

`MEASURED ROS SIM`: this record comes from a running `sim:=true` ROS graph on
the active map-v17 road. It is not a geometric mock-up. It is also not physical
road evidence; field clearance and vehicle response remain pending.

## Result

![Measured road simulation summary](tapered-rounded-boundary-road-sim.png)

![Measured road simulation timeline](tapered-rounded-boundary-road-sim.gif)

| Check | Measured result |
|---|---:|
| Route lanelets | `2751 -> 2720 -> 2744 -> 2690` |
| Runtime / pose samples | `29.700 s` / `511` |
| Physical body at first hold | cost 100 contact `NO`, maximum cost `0` |
| Planning contour at first hold | cost 100 contact `YES`, maximum cost `100` |
| Automatic recovery | `REVERSE_YAW_RIGHT` |
| Recovery displacement / yaw | `0.0972 m` / `-4.545 deg` |
| Peak recovery command | `0.05 m/s`, `0.05 rad/s` |
| Final result | route complete, no second hold, final Twist zero |

The hold was caused by the outer `0.10 m` planning contour while the physical
body remained clear. The gate then issued bounded reverse-yaw motion, released
the hold, resumed the same route, and reached the goal. The displayed body and
planning boundaries are the exact current 30-point contours in
`robot_center_link`, transformed by all recorded ROS poses.

## Reproduce

Build and start the current stack from the workspace:

```bash
./colcon_build.sh --packages-select camrod_sensor_kit camrod_platform camrod_control camrod_planning camrod_bringup
ROS_DOMAIN_ID=47 ros2 launch camrod_bringup bringup.launch.py sim:=true rviz:=false clean_before_launch:=false
```

In another sourced shell, capture and render the scenario:

```bash
ROS_DOMAIN_ID=47 python3 camrod_bringup/scripts/automatic_route_recovery_probe.py \
  --map lanelet2_maps.osm \
  --output docs/assets/test_result/tapered-rounded-boundary-road-sim-20260810/runtime-b2-recovery.json \
  --scenario b2_recontact
python3 camrod_bringup/scripts/visualization/render_tapered_rounded_road_sim.py
pytest -q camrod_bringup/test/test_tapered_rounded_road_sim_assets.py
```

The renderer rejects a runtime record whose map version, map SHA, contour
dimensions, taper, corner radii, point counts, or reference frame differ from
the current checked-in sources.

## Files

| File | Purpose |
|---|---|
| `runtime-b2-recovery.json` | Raw pose, command, gate, milestone, and boundary-classification timeline |
| `road-sim-summary.json` | Stable aggregate metrics and source identities |
| `tapered-rounded-boundary-road-sim.png` | Route, first contact, recovery, and completion summary |
| `tapered-rounded-boundary-road-sim.gif` | 80-frame drive, contact, reverse-yaw, release, and completion replay |
| `SHA256SUMS` | Integrity hashes for the raw and derived records |
