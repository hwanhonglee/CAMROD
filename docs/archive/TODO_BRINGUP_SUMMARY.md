# TODO Bringup Summary

<!-- HH_260109: Add bringup summary so current system usage is not forgotten. -->

## Launch Entry
- `camrod_bringup/launch/bringup.launch.py`

## Launches Included
- `camrod_platform/launch/platform.launch.py`
- `camrod_map/launch/map.launch.py`
- `camrod_sensing/launch/sensing.launch.py`
- `camrod_perception/launch/perception.launch.py`
- `camrod_localization/launch/localization.launch.py`
- `camrod_planning/launch/planning.launch.py`
- RViz2 (from bringup)

## Nodes Started Directly (inside bringup.launch.py)
- None (bringup delegates to module launch files)

## Optional (Separate) Launches
- `camrod_bringup/launch/fake_sensors.launch.py`
  - `camrod_bringup/scripts/fake_sensor_publisher.py`

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
- `camrod_localization/random_centerline_pose_node` (test-only)
- `camrod_platform` package (CAN integration pending)

## Not Implemented Yet
- VIO producer node (real `/localization/vio/odometry`)
- Camera detector producing `/perception/camera/detections_2d`
- Platform CAN publisher for `/platform/status/velocity`
