# TODO Architecture Summary

<!-- HH_260109: Add architecture summary so system intent is preserved. -->

## Topic Prefixes
- `/sensing/*`: raw sensors + preprocessing
- `/perception/*`: obstacle/object inference
- `/platform/*`: vehicle CAN data (wheel/velocity)
- `/localization/*`: EKF + lanelet constraint outputs
- `/map/*`: lanelet map + cost grid
- `/visualizer/*`: RViz-only outputs

## Core Data Flow
1) GNSS + IMU + wheel + VIO -> EKF -> `/localization/odometry/filtered`
2) EKF odometry -> `/localization/pose`
3) Pose + Lanelet2 -> `/localization/lanelet_pose` + `/map/lanelet_cost_grid`
4) Sensing preprocess -> Perception obstacles -> Nav2 costmaps

## Sensing vs Perception
- Sensing: raw + preprocessing only (LiDAR filtering, camera passthrough, velocity conversion)
- Perception: obstacle extraction and object labeling

## TF Ownership
- Sensor kit owns static sensor extrinsics: `robot_base_link -> sensor_kit_base_link -> {imu_link, gnss_link, lidar_link, camera_front_link}`
- Sensing package does not publish calibration TF.

## Obstacles vs Objects
- Obstacles: geometry-only (PointCloud2/Polygon) for costmap
- Objects: class/ID/velocity (for behavior or UI)
- Current: `/perception/obstacles` feeds Nav2; `/perception/objects` is future work

## Planning Inputs
- `/localization/pose`
- `/map/lanelet_cost_grid`
- `/perception/obstacles`
