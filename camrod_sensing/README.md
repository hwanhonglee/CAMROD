# camrod_sensing

Sensing module for LiDAR, camera, IMU, GNSS, and radar pipelines.

## Purpose
- Sensor input normalization into module namespaces
- LiDAR/camera preprocessing
- GNSS and optional NTRIP integration
- Near-range cost grid generation (`lidar`, `radar`)

## Launch Entry Points
- Main module:
  - `ros2 launch camrod_sensing sensing.launch.py`
- Sensor-specific:
  - `ros2 launch camrod_sensing lidar.launch.py`
  - `ros2 launch camrod_sensing gnss.launch.py`
  - `ros2 launch camrod_sensing camera.launch.py`
  - `ros2 launch camrod_sensing imu.launch.py`
  - `ros2 launch camrod_sensing radar.launch.py`

## Namespace Behavior
- Standalone sensor launch uses sensor-centric defaults:
  - LiDAR: `/lidar/...`
  - GNSS: `/gnss/...`
  - Camera: `/camera/...`
  - IMU: `/imu/...`
  - Radar: `/radar/...`
- `sensing.launch.py` normalizes to module namespace:
  - `/sensing/lidar/...`
  - `/sensing/gnss/...`
  - `/sensing/camera/...`
  - `/sensing/imu/...` (or configured equivalent)
  - `/sensing/radar/...`

## Core Topic Layout (Default via `sensing.launch.py`)
- LiDAR:
  - raw: `/sensing/lidar/vanjee/points_raw`
  - filtered: `/sensing/lidar/points_filtered`
  - diagnostic: `/sensing/lidar/diagnostic` (or module diagnostic stream)
- GNSS:
  - fix: `/sensing/gnss/navsatfix`
  - correction: `/sensing/gnss/rtcm`
- Camera:
  - input: `/sensing/camera/image_raw`, `/sensing/camera/camera_info`
  - processed: `/sensing/camera/processed/image`, `/sensing/camera/processed/camera_info`
- IMU conversion:
  - output twist: `/sensing/platform_velocity_converter/twist_with_covariance`
- Near-range grids:
  - `/sensing/lidar/near_cost_grid`
  - `/sensing/radar/near_cost_grid`

## Important Launch Arguments (`sensing.launch.py`)
- `enable_lidar_driver`, `enable_lidar_cost_grid`
- `enable_gnss`, `enable_ntrip`
- `enable_radar`, `enable_radar_cost_grid`
- `module_namespace`, `system_namespace`, `gnss_namespace`
- `lidar_raw_topic`, `lidar_filtered_topic`
- `gnss_navsatfix_topic`, `gnss_rtcm_topic`

## Configuration Files
- `config/sensing_params.yaml` (module-level compatibility overlay)
- `config/lidar/*` (preprocess, cost grid, Vanjee config)
- `config/camera/*` (preprocess)
- `config/imu/*` (MicroStrain and converter)
- `config/gnss/*` (u-blox F9P, NTRIP)
- `config/radar/*` (driver and cost grid)

## Diagnostics
- Module-local topic: `/sensing/diagnostic`
- Aggregated topic: `/diagnostics`

