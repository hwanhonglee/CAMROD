# camrod_perception

Perception module for obstacle generation from LiDAR and camera-assisted fusion.

## Purpose
- Build obstacle outputs for planning/cost layers
- Fuse camera detections with LiDAR point clouds
- Provide LiDAR clustering fallback stream

## Entry Point
```bash
ros2 launch camrod_perception perception.launch.py
```

## Runtime Structure
```text
/sensing/lidar/points_filtered
    + /perception/camera/detections_2d
    + /sensing/camera/processed/camera_info
        -> obstacle_fusion -> /perception/obstacles

/sensing/lidar/points_filtered
        -> obstacle_lidar  -> /perception/lidar/bboxes
```

## Main Launch Arguments
- `perception_param_file`
- `enable_lidar_obstacle`
- `enable_module_checker`
- `module_namespace` (default `perception`)
- `system_namespace` (default `system`)

## Key Topics
- Inputs:
  - `/sensing/lidar/points_filtered`
  - `/perception/camera/detections_2d`
  - `/sensing/camera/processed/camera_info`
- Outputs:
  - `/perception/obstacles`
  - `/perception/lidar/bboxes`

## Configuration
- `camrod_bringup/config/perception/perception_params.yaml`

## Diagnostics
- Module-local topic: `/perception/diagnostic`
- Aggregated topic: `/diagnostics`

