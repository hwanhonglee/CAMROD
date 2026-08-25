# CAMROD Runtime Parameter Reference

<!-- HH_260818 - Provide one operator-facing index for motion, safety,
parking, sensing, localization, feature toggles, and synchronized mirrors. -->
<!-- HH_260819 - Add source-aware roadside Return, serialized operator
preemption, and event-driven telemetry scheduling values. -->
<!-- HH_260824 - Set the active AprilTag translation stop to the same 0.40 m
camera-frame range requested by the operator and document trigger provenance. -->
<!-- HH_260825 - Add live campsite lanelet handoff, charging departure dwell,
front-only radar stop windows, and external-simulator ownership parameters. -->

This guide lists the parameters normally changed for vehicle behavior. It does
not duplicate every ROS topic string or diagnostic checker threshold in the
repository.

## Precedence And Editing Rule

Full bringup resolves values in this order, highest priority first:

1. explicit `ros2 launch ... name:=value` argument;
2. `camrod_bringup/config/bringup/launch_defaults.yaml` override;
3. a parameter file under `camrod_bringup/config/<module>/`;
4. the package-owned file under `camrod_<module>/config/`;
5. the node's source-code default.

For synchronized contracts, edit the package-owned file and its bringup mirror
in the same change. Tests compare exact mirrors or the explicitly shared keys.
The deployed RPP preview and Ranger steering-rate profiles are intentional A/B
pairs and must not be made byte-identical without a new driving test.

## File Index

| What to tune | Package source | Full-bringup deployment file |
|---|---|---|
| RPP speed, lookahead, curvature and goal approach | `camrod_planning/config/nav2_vehicle.yaml` | `camrod_bringup/config/planning/nav2_vehicle.yaml` |
| Final command gate and obstacle/boundary policy | `camrod_control/config/cmd_vel_safety_gate.yaml` | `camrod_bringup/config/control/cmd_vel_safety_gate.yaml` plus `bringup/launch_defaults.yaml` overrides |
| Campsite, drop-zone and route recovery | `camrod_control/config/control.yaml` | `camrod_bringup/config/control/control.yaml` |
| Reverse and AprilTag parking | `camrod_control/config/parking.yaml` | `camrod_bringup/config/control/parking.yaml` |
| Ranger steering, crab selection, CAN/BMS | `camrod_platform/config/ranger_driver.yaml` | `camrod_bringup/config/platform/ranger_driver.yaml` |
| Body/planning contour and sensor TF | `camrod_sensor_kit/config/robot_params.yaml` | `camrod_bringup/config/sensor_kit/robot_params.yaml` |
| EKF | `camrod_localization/config/filter/ekf.yaml` | `camrod_bringup/config/localization/filter/ekf.yaml` |
| GNSS input filtering | `camrod_localization/config/source/input_adapter.yaml` | `camrod_bringup/config/localization/source/input_adapter.yaml` |
| GNSS receiver | `camrod_sensing/config/gnss/zed_f9p_rover.yaml` | `camrod_bringup/config/sensing/gnss/zed_f9p_rover.yaml` |
| Front/rear cameras | `camrod_sensing/config/camera/camera_params.yaml` | `camrod_bringup/config/sensing/camera/camera_params.yaml` |
| Radar channels and raster | `camrod_sensing/config/radar/*.yaml` | `camrod_bringup/config/sensing/radar/*.yaml` |
| Classified camera-LiDAR raster | `camrod_sensing/config/lidar/cost_grid.yaml` | `camrod_bringup/config/sensing/lidar/cost_grid.yaml` |
| Campsite/drop-zone coordinates | planning/map package YAML | `camrod_bringup/config/{planning,map}/*.yaml` |
| Module/container/UI feature switches | launch files | `camrod_bringup/config/bringup/launch_defaults.yaml` |
| Charging campsite departure dwell | UI backend default | `launch_defaults.yaml` key `system/api_ui_charging_departure_delay_s` |

## Vehicle Speed

The final gate applies `cmd_vel_gate_speed_scale: 0.5`. Values below show both
the raw owner limit and the resulting platform limit.

| Motion | Parameter | Raw | Final / meaning |
|---|---|---:|---:|
| Normal RPP cruise | `RPP.desired_linear_vel` | `1.111111 m/s` | `0.555556 m/s` = `2.0 km/h` |
| RPP curvature floor | `regulated_linear_scaling_min_speed` | `0.333333 m/s` | `0.166667 m/s` = `0.6 km/h` |
| RPP final goal approach | `min_approach_linear_velocity` | `0.277778 m/s` | `0.138889 m/s` = `0.5 km/h` |
| Campsite crab | `crab_speed_mps` | `0.666667 m/s` | `0.333334 m/s` = `1.2 km/h` |
| B11-B13 roadside crab | `roadside_crab_speed_mps` | `0.20 m/s` | `0.10 m/s` = `0.36 km/h` |
| Campsite reverse | `reverse_entry_speed_mps` | `0.444444 m/s` | `0.222222 m/s` = `0.8 km/h` |
| Drop-zone exit | `exit_speed_mps` | `0.444444 m/s` | `0.222222 m/s` = `0.8 km/h` |
| Reverse parking cruise | `reverse_speed_mps` | `0.444444 m/s` | `0.222222 m/s` = `0.8 km/h` |
| Reverse parking final | `final_approach_speed_mps` | `0.138889 m/s` | `0.069445 m/s` = `0.25 km/h` |
| AprilTag approach | `reverse_approach_speed_mps` | `0.555556 m/s` | `0.277778 m/s` = `1.0 km/h` |
| AprilTag low-speed approach | `final_insertion_speed_mps` | `0.138889 m/s` | `0.069445 m/s` = `0.25 km/h` immediately before the stop |
| Route recovery | `maximum_speed_mps` | `0.10 m/s` | `0.05 m/s` = `0.18 km/h` |

Do not raise only one raw maneuver speed when the ratio to normal cruise is
important. Change the profile as one set, rerun B1-B10 service simulation, and
measure physical steering settling.

## Normal Steering Versus Crab

| Parameter | Active value | Effect |
|---|---:|---|
| `parallel_command_lateral_deadband_mps` | `0.02` | `|linear.y|` at or below this remains Dual-Ackermann; above it selects parallel motion |
| `steering_mode_transition_stationary_enabled` | `true` | Holds translation during longitudinal/parallel mode changes |
| `steering_mode_transition_ready_error_rad` | `0.05` | Wheel-angle error allowed before traction |
| `steering_transition_rate_radps` | package `1.0`, bringup `1.5` | Maximum steering target slew; deliberate bench/deployment A/B |
| `steering_transition_stop_error_rad` | `0.70` | Ordinary path tracking stops only for a genuinely large steering lag |

Normal Nav2 publishes `linear.y=0`. Camping and recovery intentionally publish
lateral commands at least `0.10 m/s`, well outside the deadband.

## RPP Tracking

| Parameter | Package / bringup | Meaning |
|---|---:|---|
| `lookahead_dist` | `1.2 / 3.5 m` | Fixed package study value / deployed fallback value |
| `use_velocity_scaled_lookahead_dist` | `false / true` | Intentional A/B difference |
| `min_lookahead_dist` | `1.1 / 1.5 m` | Minimum preview |
| `max_lookahead_dist` | `2.0 / 3.5 m` | Maximum preview |
| `rotate_to_heading_angular_vel` | `0.35 rad/s` | Manual RotationShim heading speed |
| `max_angular_accel` | `0.8 rad/s^2` | Controller angular acceleration cap |

Change RPP values first in the full-bringup deployment file when testing the
robot. Preserve the package A/B profile until the result is accepted and
documented.

## Campsite In And Out

File pair: `camrod_control/config/control.yaml` and its bringup mirror.

| Parameter | Active value | Effect |
|---|---:|---|
| `site_entry_mode` | `crab` | B1-B10 use lateral entry |
| `use_goal_pair_for_lateral_offset` | `true` | Computes site offset from snap/site map coordinates |
| `site_rotate_direction_policy` | `entry_crab_side` | Turn direction follows the signed snap-frame crab side; restart/adopt retains live-yaw fallback |
| `site_pose_reached_distance_m` | `0.60 m` | B1-B10 reboot near the authored target restores `WAIT_RETURN` |
| `route_goal_reached_distance_m` | `0.60 m` | Bounds axial handoff and B11-B13 operational-target adoption independently of lateral tolerance |
| `entry_yaw_alignment_timeout_s` | `15.0 s` | Automatic crab entry fails closed if lanelet-snap yaw alignment does not finish |
| `entry_position_tolerance_m` | `0.15 m` | Completes crab-in position |
| `return_position_tolerance_m` | `0.04 m` | Required distance to the shared lanelet snap anchor on crab-out |
| `return_lateral_transition_tolerance_m` | `0.02 m` | Latches lateral correction complete before steering settle |
| `return_lateral_hysteresis_m` | `0.10 m` | Fails closed if lateral error escapes the latched band; never switches back to crab |
| `return_steering_settle_s` | `1.20 s` | Zero-command hold for the deployed `+/-90 -> 0 deg` steering transition |
| `enable_live_lanelet_return_handoff` | `true` | Normal exit completes at a fresh live lanelet projection instead of requiring historical entry XY |
| `return_lanelet_handoff_distance_m` | `0.15 m` | Maximum current-pose to live-projection separation for route handoff |
| `return_lanelet_handoff_hold_s` | `1.20 s` | Continuous zero-command steering-settle hold before publishing Return planning authorization |
| `crab_return_timeout_s` | `90 s` | Time available for exit plus repeated boundary recovery |
| `rotate_yaw_tolerance_deg` | `4 deg` | 180-degree target tolerance |
| `rotate_settle_hold_s` | `0.8 s` | Continuous settled-yaw proof before translation |
| `rotate_settle_max_rate_degps` | `3 deg/s` | Maximum residual yaw rate during settle proof |
| `unload_wait_s` | `5 s` | Delay before external Return becomes valid |
| `auto_return_after_unload_wait` | `false` | Prevents automatic motion while people unload |
| `roadside_max_lateral_offset_m` | `0.30 m` | B11-B13 roadside-only cap |

Entry still uses `/planning/goal_pose_snapped` for its map target, authored
tangent, signed crab side, and restart correlation. During `CRAB_OUT`, normal
completion uses the fresh `/planning/lanelet_pose_ros` projection and ignores
longitudinal distance from the historical entry snap. After a `1.20 s` stopped
hold within `0.15 m`, LaneletRoute plans from current XY. A stale, non-finite,
or wrong-frame live projection falls back to the exact-anchor sequencer and its
`0.04 m` tolerance. Restart/adopt still requires a fresh finite lanelet anchor
or a route goal correlated to it; current pose is not accepted without a live
projection.
B11-B13 match their signed `0.30 m` requested target using the existing
`0.15 m` lateral completion and `0.60 m` axial handoff bounds, not the distant
raw semantic site center or the global B1-B10 center radius.
`CRAB_IN` and `CRAB_OUT` publish `/control/camping_site_maneuver_controller/path_ros`.
B1-B10 may retrace the reverse shortest Return after their on-site turn.
B11-B13 publish a `roadside_forward` source only after `CRAB_OUT -> DONE`, so
LaneletRoute follows the legal forward one-way loop without rotating in-lane.

## Boundary Recovery

| Parameter | Active value | Effect |
|---|---:|---|
| `enable_route_safety_recovery_controller` | `true` | Enables projected recovery owner |
| `maximum_attempts` | `50` | Fresh candidates per episode |
| `maximum_total_distance_m` | `1.50 m` | Whole-episode travel cap |
| `maximum_total_duration_s` | `90 s` | Whole-episode time cap |
| `maximum_distance_m` | `0.40 m` | Per-attempt travel cap |
| `maximum_duration_s` | `10 s` | Per-attempt time cap |
| `retry_pause_s` | `0.5 s` | Zero command between attempts |
| `cmd_vel_gate_route_safety_recovery_max_auto_releases` | `50` | Gate release budget per contact region |
| `cmd_vel_gate_route_safety_recovery_progress_reset_distance_m` | `0.75 m` | Forward progress needed to reset region budget |

Candidates include crab left/right, straight reverse, and reverse-yaw left/right.
Physical-body collision, dynamic-obstacle, stale-data, episode distance, and
episode time limits remain fail-closed.

## Obstacle Stop And Replan

| Parameter | Active value | Effect |
|---|---:|---|
| `cmd_vel_gate_cost_threshold` | `85` | Dynamic cost stop threshold |
| `cmd_vel_gate_cost_stop_dynamic_source_labels` | `radar,fusion` | Sources allowed to stop motion |
| `cmd_vel_gate_cost_stop_classified_source_labels` | `fusion` | Fusion must have a current semantic class |
| `cmd_vel_gate_cost_stop_classified_front_lookahead_m` | `2.0 m` | Class-confirmed path-front stop distance |
| `cmd_vel_gate_cost_stop_clear_required_s` | `2.0 s` | Continuous clear proof before release |
| FRONT1 stop candidate | `(0.220, 0.520] m` from sensor face | Measured body envelope followed by `0.30 m` usable window |
| FRONT2 stop candidate | `(0.117, 0.417] m` from sensor face | Measured body envelope followed by `0.30 m` usable window |
| Side/rear usable window | `0.10 m` | Unchanged near-field policy; REAR remains quarantined |
| Front radar spatial gate | active lanelet + `1.27 m` local-path corridor | Prevents the longer scalar window from widening lateral stop authority |
| `obstacle_replan_monitor.block_hold_s` | `20.0 s` | Delay before fallback planner preemption, not stop delay |
| `enable_obstacle_replan_monitor` | `true` | Width-gated fallback replan monitor |
| fallback minimum corridor | `2.50 m` | Prevents replanning in an infeasible narrow lane |

The stop is immediate. The `20 s` value controls only replan escalation. Keep
the class-confirmed fusion `2.0 m` contract unchanged unless camera-LiDAR field
calibration is repeated.

## Parking And Charging

File pair: `camrod_control/config/parking.yaml` and its bringup mirror.

| Parameter | Reverse | AprilTag docking |
|---|---:|---:|
| Slowdown window | `0.30 m` remaining | UI camera range `0.80 -> 0.40 m` |
| Cruise raw speed | `0.444444 m/s` | `0.555556 m/s` |
| Final raw speed | `0.138889 m/s` | `0.138889 m/s` |
| Translation stop | station axis | `translation_stop_tag_distance_m: 0.40 m` |
| Approach steering | heading/lateral correction | combined reverse + steering, minimum radius `0.85 m` |
| Final yaw | map/station policy | heading-only, raw maximum `0.20 rad/s` |
| Yaw settling | controller-specific | `0.8 s`, maximum residual rate `3 deg/s` |
| Final-yaw odometry freshness | controller-specific | `0.5 s`; stale input holds zero |
| `stop_when_charging` | `true` | `true` |
| Charging required | `complete_without_charging: false` | `require_charging_for_completion: true` |
| Charging wait | `45 s` | explicit stopped `WAITING_FOR_CHARGING` phase |
| Charging mission dwell | `7.0 s` | Destination queued with all drive authorization false before one station `EXIT` |

`bringup.parking.method` is `apriltag` on the physical profile. Simulation is
forced to `reverse` because fake sensors do not publish a rear-camera tag.
The AprilTag thresholds consume the unmodified camera pose 3D norm, exactly as
the UI `Tag distance` does; deprecated robot-center longitudinal parameters stay
loadable but no longer own motion. After the `0.40 m` crossing, tag loss cannot
restart insertion or retry translation. Controller status preserves the active
`configured_stop_tag_m`, `stop_reason`, and exact `stop_trigger_tag_m` sample.
Charging CAN feedback immediately publishes zero
in any active final-parking phase, then the internal phase becomes
`PARKED` and public state is `CHARGING`. Radar-backed dynamic rotation protection
remains active during `FINAL_YAW_ALIGNMENT`.

## Manual Return And Docking UI

| Item | Topic/API | Effect |
|---|---|---|
| Manual Return in a site | `POST /ui/manual_return` | Latches site RETURN; planning recall is deferred until `CRAB_OUT` reaches a fresh live lanelet projection or the exact-anchor fallback |
| Manual Return while driving | same API | Cancels Nav2, closes motion authorization for `manual_return_preempt_hold_s=0.50 s`, then publishes one drop-zone route |
| Already at drop zone | same API | Starts drop-zone yaw alignment before selected parking method |
| Already charging | same API | Reports `already_charging`; does not move |
| `CHARGING` state but CAN contact lost | same API | Restarts drop-zone alignment instead of creating a Nav2 loop |
| Docking debug image | `/perception/apriltag_parking_detector/debug_image/compressed` | Lazy UI camera stream |
| Tag data | tag pose and detected topics | Exact x/y/z/distance/yaw and presence |
| Controller paths | reverse/AprilTag `path_ros` | UI parking trajectory |
| Charging | `/platform/status.is_charging` | UI boolean and immediate controller stop |

Both visible Return controls call the same API. Duplicate presses during the
preemption hold or an active return are idempotent. The obsolete manual Parking
ON/OFF endpoint is removed; final parking remains owned by the drop-zone state
handoff and configured parking method.

A campsite destination selected while charging is also idempotent. The UI
backend creates one ROS timer only while the departure is pending, closes
manual/mission/platform authorization, and reports `departure_delay_active` in
snapshots. Timer expiry opens authorization and publishes one `EXIT`; Stop or
shutdown destroys the timer first. This is event-driven and adds no permanent
poll loop on the ARM64 target.

The docking UI uses seven dynamic subscriptions only while its administrator
tab is open. Lease changes wake ROS through a GuardCondition; a `1 Hz` timer is
retained only for abandoned-lease expiry, while visible telemetry remains
`10 Hz`.

## Battery Policy

| Parameter | Active value | Effect |
|---|---:|---|
| `api_ui_minimum_mission_dispatch_battery_percent` | `35%` | New campsite admission |
| `api_ui_low_battery_return_after_current_mission` | `true` | Finish current site service and wait for the user Return |
| `cmd_vel_gate_critical_battery_percentage` | `0.20` | Hard stop at or below 20% |
| `cmd_vel_gate_allow_mission_departure_while_charging` | `true` | Allowed charger departure when SOC gate passes |
| `cmd_vel_gate_charging_departure_grace_s` | `15 s` | Stale charging contact grace after accepted departure |
| `cmd_vel_gate_allow_drop_zone_departure_while_charging` | `true` | Separately allows a controller-owned station exit without prepublishing the next mission key |
| `cmd_vel_gate_drop_zone_departure_status_timeout_s` | `2.0 s` | Revokes controller authorization and publishes zero after a missed heartbeat |
| `cmd_vel_gate_drop_zone_departure_require_source_stamp` | `true` | Rejects unstamped/replayed production controller status |
| Drop-zone captured exit | lanelet required, fixed fallback disabled, `0.20 m` tolerance | Captures fresh same-frame XY/yaw at EXIT start; campsite key/goal release only follows `exit_complete=true` |

## Localization And GNSS

| Parameter | Active value | File |
|---|---:|---|
| EKF `frequency` | `20 Hz` | `localization/filter/ekf.yaml` |
| EKF `sensor_timeout` | `0.2 s` | same |
| GNSS Mahalanobis `pose0_rejection_threshold` | `3.0 sigma` | same; not a direct metre threshold |
| Adapter `max_position_jump_m` | `8.0 m` | `localization/source/input_adapter.yaml` |
| Adapter reset | `2.0 s` | same |
| GNSS receiver `rate` / `nav_rate` | `10 Hz / 1 solution per epoch` | `sensing/gnss/zed_f9p_rover.yaml` |
| Wheel input | `10 Hz` measured in current sim | Ranger/platform chain |
| IMU input | `20 Hz` measured in current sim | IMU converter/profile |

The EKF rejection value is a normalized innovation gate, not “reject any
position differing by 3 m.” Do not set it to `0.3` to obtain a 0.3 m distance
gate; covariance changes the physical distance represented by sigma.

## Sensors And Perception

| Parameter/toggle | Active value | Main file |
|---|---:|---|
| Front/rear raw camera `fps` | `10 Hz` | `sensing/camera/camera_params.yaml` |
| Rear monitoring JPEG | `2 Hz` | same |
| LiDAR filtered/raster target | `10 Hz` | LiDAR config |
| Radar raster/channels | `10 Hz` | radar config |
| Front radar usable stop distance | `0.30 m` after each measured body echo | radar sensor/cost-grid YAML pair; sides remain `0.10 m` |
| `enable_lidar_cost_grid` | `true` | launch defaults; this is classified fusion compatibility raster |
| Direct raw LiDAR cost paint | `false` | LiDAR cost-grid file |
| `enable_radar_cost_grid` | `true` | launch defaults |
| `enable_yolo` | `true` | launch defaults |
| `enable_campsite_occupancy_guard` | `false` | launch defaults and control YAML |
| AprilTag detector | enabled only for selected physical docking path | perception/launch defaults |

## Runtime And Resource Toggles

| Toggle | Active value | Meaning |
|---|---:|---|
| `rviz` | `false` | Managed UI is the normal operator surface |
| `use_nav2_container` | `true` | Scoped planner/controller component process |
| `use_lidar_processing_container` | `true` | LiDAR preprocessing/ground segmentation composition |
| `use_camera_yolo_container` | `true` | Front camera and YOLO hot path |
| `use_rear_camera_apriltag_container` | `true` | Physical rear camera rectify/tag path |
| `use_system_tools_container` | `true` | Core diagnostic tools composition |
| `use_checker_components` | `true` | Four serialized checker fault-domain containers |
| `enable_dds_shared_memory` | `false` | Full-graph DDS SHM remains off on Humble |
| `enable_operator_telemetry` | `true` | Administrator live views available |
| `operator_telemetry_stream_rate_hz` | `10 Hz` | Selected-view stream ceiling |
| `external_simulator` | `false` | When true under `sim`, fake sensor pose follows fresh external odometry instead of integrating `cmd_vel` |
| `external_simulator_odometry_topic` | `/odom` | Map-aligned external plant pose/twist input |
| `external_simulator_odometry_timeout_s` | `0.5 s` | Pauses simulated sensor output when the external plant is stale |

## Validation-Only Runner Parameters

These values affect `sim_validation_runner.py`; they do not tune the physical
vehicle.

| Parameter | Release value | Effect |
|---|---:|---|
| `camping_return_via_ui` | `true` for the release run | Calls the production `/ui/manual_return` API instead of publishing a controller operation directly |
| `ui_backend_base_url` | `http://127.0.0.1:18122` in the recorded run | Backend selected for the Return API check |
| `normal_drive_lateral_limit_mps` | `0.02` | Fails a normal Nav2 run if maximum `|linear.y|` could select crab |
| `expect_lidar_cost_grid` | `false` in the recorded optional-grid run | Keeps the disabled compatibility raster out of baseline requirements |
| charging recall trigger | public `CHARGING` + parking `PARKED` | Validator no longer depends on presentation-specific cmd-gate text |

## Geometry And Coordinates

| Change | File |
|---|---|
| Body dimensions, taper, corner radius, sensor mounts | `camrod_sensor_kit/config/robot_params.yaml` and bringup mirror |
| Nav2 footprints | `camrod_planning/config/nav2_vehicle.yaml` and bringup deployment profile |
| Gate body/planning extents | `camrod_control/config/cmd_vel_safety_gate.yaml` and launch-default overrides |
| Campsite coordinates/mode | `camrod_planning/config/camping_sites.yaml` and bringup mirror |
| Drop zone / parking station | `camrod_map/config/drop_zones.yaml`, localization/control consumers, and bringup mirrors |
| Lanelet map | user-owned `lanelet2_maps.osm` selected by launch; never auto-rewritten by config synchronization |

Changing geometry requires all three body/footprint/gate contracts to agree.
Changing map coordinates requires re-exporting campsite and drop-zone YAML from
the selected map, not hand-editing only one consumer.

## Verification After A Change

```bash
cd ~/camrod_ws/src
./setup_camrod.sh
./colcon_build.sh --packages-select camrod_control camrod_planning camrod_sensing camrod_ui camrod_bringup

diff -u camrod_control/config/control.yaml camrod_bringup/config/control/control.yaml
diff -u camrod_control/config/parking.yaml camrod_bringup/config/control/parking.yaml
# Ranger files intentionally retain steering-profile A/B differences. Verify
# parallel_command_lateral_deadband_mps is 0.02 in both files.

cd ~/camrod_ws
colcon test --packages-select camrod_control camrod_planning camrod_sensing camrod_ui camrod_bringup
colcon test-result --verbose
```

For motion changes, also run B1-B10 service, a reversed-heading restart case,
boundary contact recovery, class-confirmed obstacle stop/clear, return to the
drop zone, selected parking, and charger-contact stop. AMD64 simulation is a
logic check; the 8-core/16-GB ARM64 robot still needs actuator, camera, tag,
radar, charging-current, and resource acceptance.
