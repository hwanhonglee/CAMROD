# Package Work Log

<!-- HH_260109 Initialize bringup work log. -->

## 2026-02-06 21:26
- Path cost grid no longer forces outside_value (set to -1) so planners can compute the first path before any path is available.

## 2026-02-20 14:20
- HH_260220: Pass `sensor_kit_base_frame_id` (`sensor_kit_base_link`) from bringup to platform/sensor_kit launch.
- HH_260220: Remove sensing calibration broadcaster usage; sensor extrinsic TF now comes from sensor kit URDF only.
- HH_260220: Align perception camera frame to `camera_front_link`.

## 2026-02-09 16:30
- Enable localization pose selector defaults (ESKF primary, Kimera-VIO fallback) via bringup launch passthrough args.
- Add bringup selector config `config/localization/pose_selector.yaml`.
- Tighten goal snapping to lanelet-contained goals (`require_lanelet_containment=true`) to keep planned path on lanelet map.

## 2026-02-04 19:45
- Prefer Smac2D planner by default to ensure /planning/unsmoothed_plan is published.
- Fix global costmap width/height type mismatch by keeping width/height as integers (meters).
- Align global costmap origin/size to lanelet map bounds for consistent planning coverage.

## 2026-02-05 14:37
- Fake sensor now excludes crosswalk lanelets and supports optional start snapping to nearest path point.

## 2026-02-04 16:05
- Disable visualizer TF publishing by default to avoid duplicate map->base_link transforms.

## 2026-02-03 18:10
- Disable wheel_odometry_bridge in sim to avoid duplicate /platform/wheel/odometry (fake sensor already publishes).

## 2026-02-03 10:25
- Add ESKF GNSS reinit params in bringup config (reinit_on_gnss_reject / reinit_distance_threshold).

## 2026-02-02 15:40
- Stitch all lanelet centerlines for full-lap fake sensor motion; add loop connection controls.
- Add proximity (RangeSensorLayer) to Nav2 costmap plugins for near-range sensors.
- Increase lanelet/path grid resolution for finer visualization (0.1m cells).

## 2026-02-02 15:10
- Add fake sensor loop-closure parameters for continuous centerline laps (close_loop, close_loop_max_gap).

## 2026-02-02 14:30
- Prevent cleanup pkill from self-terminating; avoids stale duplicate nodes (e.g., /map/lanelet_cost_grid).

## 2026-02-02 11:55
- Base map cost grid now rasterizes only lanelet bounds; path grids mark non-path cells as high cost.
- Fake IMU now publishes yaw rate so ESKF heading follows the path.
- Add BT plugin lib for ComputePathThroughPoses to prevent bt_navigator activation failure.

## 2026-02-02 11:10
- Switch cost field markers to visualize combined inflation costmap output.
- Reduce lanelet grid resolutions and fit base grid to map bounds.

## 2026-02-02 11:05
- Synthesize centerline from left/right lanelet bounds when explicit centerline is absent.

## 2026-02-02 10:36
- Enforce fake sensor path to follow lanelet centerline only (no fallback to first way).

## 2026-02-02 10:35
- 2026-02-02: Add drop_zone_matcher config for initial localization OK gate.

## 2026-02-02 10:32
- Centralize all package configs under `camping_cart_bringup/config/*` and sync from package configs.
- Wire bringup launch defaults to use bringup config copies (sensing/perception/planning/visualizer).
- Add drop_zone_matcher param arg to bringup localization include.

## 2026-01-30 16:05
- 2026-01-30: Add RemovePassedGoals BT plugin library to bringup nav2 config for navigate_through_poses BT.
- 2026-01-30: Update bringup map/sim configs to new_lanelet2_maps.osm origin.

## 2026-01-29 22:13
- HH_260129: Fix nav2 costmap plugin class names to use '::' (Obstacle/Inflation) for pluginlib lookup.

## 2026-01-29 19:29
- HH_260129: Align bt_navigator plugin library names with Humble action BT libraries in bringup nav2 config.
- HH_260129: Remove missing BT plugin names (follow_waypoints) and use assisted_teleop action variant.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and suppress bringup/fake sensor startup logs by default.

## 2026-01-27 14:05
- HH_260127: Switch bringup Nav2 params to camping_cart_bringup/config/planning/nav2_lanelet.yaml (planning config now mirrored here).

## 2026-01-26 16:15
- HH_260126: Remove OpenVINS note (no longer used); Kimera-VIO will be vendor-added under localization/external when available.

## 2026-01-23 13:07
- HH_260123: Wire ESKF/supervisor launch args (use_eskf, eskf/supervisor params, wheel bridge) into bringup.
- HH_260123: Add ESKF and supervisor parameter files for localization stack defaults.

## 2026-01-19
- HH_260119: Review README ordering/architecture and confirm launch arg defaults; documentation-only update.

## 2026-01-14 14:20
- HH_260114: Declare/forward all fake sensor launch arguments (map path, origin, lanelet, speed) to avoid missing LaunchConfiguration errors in sim.
- HH_260114: Add sim static TF publishers for map->base_link to stabilize Nav2 in simulation.
- HH_260114: Update bringup license to Apache-2.0 and convert remaining bringup comments to English.

## 2026-01-12 15:41
- HH_260112: Add sim/RViz toggles and wire fake GNSS output for simulation bringup.
- HH_260112: Align EKF parameters and map paths with namespaced bringup configs.

## 2026-01-12 12:45
- HH_260112: Refresh cleanup list to cover renamed nodes and new goal snapper.

## 2026-01-12 10:40
- HH_260112: Namespace camping_cart nodes (map/sensing/perception/localization/visualizer/bringup) with short names.
- HH_260112: Add planning goal snapper node and wire /planning/goal_pose input.

## 2026-01-11 20:10
- HH_260109: Normalize EKF process_noise_covariance floats for ROS2 YAML parsing.
- HH_260109: Apply EKF parameters under /localization namespace in bringup config.
- HH_260109: Make EKF parameters node-agnostic to ensure namespaced launch loads all inputs.
- HH_260109: Add optional clean_before_launch to stop stale nodes before launching.
- HH_260109: Add explicit fake sensor launch defaults for map and publishing parameters.
- HH_260109: Delay bringup/fake launch by 1s after cleanup to avoid killing new nodes.

## 2026-01-09 20:30
- HH_260109: Add perception launch wiring and route obstacle input to /perception/obstacles.

## 2026-01-09 20:20
- HH_260109: Remove unused robot_info launch entry from bringup package.

## 2026-01-09 18:37
- HH_260109: Rename sensor kit package and wire visualizer/platform packages into bringup.
- HH_260109: Update GNSS/IMU/VIO/Wheel topic prefixes and map cost grid topic names.

## 2026-01-09 15:52
- HH_260109: Add fake sensor publisher + launch/config for lanelet-guided simulation.

## 2026-01-09 15:46
- HH_260109: Integrate EKF-based localization and sensing launch wiring in bringup.
- HH_260109: Add obstacle layer and 5 kph tuning in Nav2 parameters.
