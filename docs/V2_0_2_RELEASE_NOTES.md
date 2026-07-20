# CAMROD v2.0.2 Release Notes

Release date: 2026-07-16
Target branch: `develop`
Target remote: `hwanhong`

## Scope

- Active Park Lanelet2 geometry moved to `lanelet2_maps_(copy_park_moved).osm`
  and the map profile, WGS84 origin, UTM metadata, semantic drop zones, camping
  <!-- HH_260720 - Use the current parking terminology. -->
  sites, fake start pose, parking pose, and coordinate utilities were aligned.
- Bringup forwards one `map_info_file` to both map and localization so the
  Lanelet2 geometry and GNSS projection cannot silently use different origins.
- Radar detection angles are 15 degrees on all seven channels. The cost grid
  ignores body/self echoes below 0.30 m, with a 0.75 m override on LEFT2 for its
  repeated 0.70-0.72 m stationary body/multipath return.
- NTRIP uses the field `JECH-RTCM32` mountpoint.
- The camera/YOLO component path honors `enable_camera`, `enable_front_camera`,
  and `perception_enable_yolo`. Annotated YOLO images remain subscriber-gated;
  detections are the continuous inference-health output.
- The field config checker compares bringup/package source pairs, the entire
  installed bringup config, and every paired package install config.

## Runtime Findings

- A live GNSS fix does not by itself imply localization `NORMAL`. The monitor
  also requires GNSS age <= 4.0 s, XY covariance trace <= 1.0, rate >= 0.8 Hz,
  and per-sample jump <= 1.0 m while IMU and wheel streams remain healthy.
- On the 2026-07-16 full-stack run, front compressed camera input measured
  about 9.8-12.2 Hz. YOLO detections measured about 3.5-9.9 Hz depending on
  concurrent probe load, and the subscriber-enabled annotated image measured
  about 3.7-6.3 Hz. TensorRT loaded successfully and the YOLO component
  remained alive throughout the probes.
- Multiple simultaneous `ros2 topic echo` processes are material load on the
  Jetson. Close duplicate debug subscribers before judging drive, cost-grid,
  or inference stability.

## Operator Checks

```bash
./colcon_build.sh
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
ros2 run camrod_bringup field_test_tool.sh config
ros2 run camrod_bringup field_test_tool.sh camera-yolo 12
ros2 run camrod_bringup field_test_tool.sh snapshot
```

For `DR_ONLY` with live GNSS, inspect
`/sensing/gnss/pose_with_covariance` before changing the map origin. For radar
stops, confirm an external target beyond the configured per-channel dead zone
appears in `/sensing/cost_grid/radar` and closes `/planning/cmd_vel`.
