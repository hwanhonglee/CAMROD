# TODO Bringup Summary

<!-- HH_260109: Add bringup summary so current system usage is not forgotten. -->

## Launch Entry
- `camping_cart_bringup/launch/bringup.launch.py`

## Launches Included
- `camping_cart_platform/launch/platform.launch.py`
- `camping_cart_map/launch/map.launch.py`
- `camping_cart_sensing/launch/sensing.launch.py`
- `camping_cart_perception/launch/perception.launch.py`
- `camping_cart_localization/launch/localization.launch.py`
- `camping_cart_planning/launch/planning.launch.py`
- RViz2 (from bringup)

## Nodes Started Directly (inside bringup.launch.py)
- None (bringup delegates to module launch files)

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
- `/map/cost_grid/lanelet`
- `/perception/obstacles`
- `/platform/robot/*`
- `/map/cost_grid/*`

## Not Used by Bringup (As of Now)
- `camping_cart_localization/random_centerline_pose_node` (test-only)
- `camping_cart_platform` package (CAN integration pending)

## Not Implemented Yet
- VIO producer node (real `/localization/vio/odometry`)
- Camera detector producing `/perception/camera/detections_2d`
- Platform CAN publisher for `/platform/status/velocity`
