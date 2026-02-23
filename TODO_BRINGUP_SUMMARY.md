# TODO Bringup Summary

<!-- HH_260109: Add bringup summary so current system usage is not forgotten. -->

## Launch Entry
- `camping_cart_bringup/launch/bringup.launch.py`

## Launches Included
- `camping_cart_sensor_kit/launch/sensor_kit.launch.py`
- `camping_cart_map/launch/lanelet2_map.launch.py`
- `camping_cart_visualizer/launch/visualizer.launch.py`
- `camping_cart_sensing/launch/sensing.launch.py`
- `camping_cart_perception/launch/perception.launch.py`
- `camping_cart_planning/launch/nav2_lanelet.launch.py`
- RViz2 (from bringup)

## Nodes Started Directly (inside bringup.launch.py)
- `camping_cart_map/lanelet_cost_grid_node`
- `camping_cart_localization/navsat_to_pose_node`
- `camping_cart_localization/pose_cov_bridge_node`
- `camping_cart_localization/centerline_snapper_node`
- `robot_localization/ekf_node`
- `camping_cart_localization/odometry_to_pose_node`
- `camping_cart_localization/localization_health_monitor_node`
- `ov_msckf/run_subscribe_msckf` (OpenVINS VIO)

## Optional (Separate) Launches
- `camping_cart_bringup/launch/fake_sensors.launch.py`
  - `camping_cart_bringup/scripts/fake_sensor_publisher.py`

## Current Runtime Inputs (Expected)
- `/sensing/gnss/navsatfix`
- `/sensing/imu/data`
- `/platform/wheel/odometry`
- `/localization/vio/odometry`
- `/sensing/camera/image_raw`
- `/sensing/camera/camera_info`
- `/perception/camera/detections_2d`

## Key Outputs
- `/localization/odometry/filtered` (EKF)
- `/localization/pose`
- `/localization/lanelet_pose`
- `/map/lanelet_cost_grid`
- `/perception/obstacles`
- `/visualizer/*`

## Not Used by Bringup (As of Now)
- `camping_cart_localization/random_centerline_pose_node` (test-only)
- `camping_cart_platform` package (CAN integration pending)

## Not Implemented Yet
- VIO producer node (real `/localization/vio/odometry`)
- Camera detector producing `/perception/camera/detections_2d`
- Platform CAN publisher for `/platform/status/velocity`
