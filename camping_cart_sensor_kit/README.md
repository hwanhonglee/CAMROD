# Package Work Log

<!-- HH_260109 Initialize sensor kit package work log. -->

## 2026-02-02 10:32
- `sensor_kit.launch.py` defaults now use bringup config (`config/sensor_kit/robot_params.yaml`).

## 2026-02-20 14:20
- HH_260220: TF chain updated to `robot_base_link -> sensor_kit_base_link -> {imu_link, gnss_link, lidar_link, camera_front_link}`.
- HH_260220: Removed rear sensor frames from the sensor kit URDF and robot params (`lidar_rear`, `camera_rear`).
- HH_260220: Added `sensor_kit_base_frame_id` launch argument to keep frame wiring explicit from bringup.

## 2026-01-19
- HH_260119: Review README ordering/architecture; no sensor kit functional changes (doc only).

## 2026-01-14 14:45
- HH_260114: Set maintainer email to hwanhong57@gmail.com and license to Apache-2.0.

## 2026-01-12 10:40
- HH_260112: Namespace robot_state_publisher under /sensor_kit.

## 2026-01-09 21:35
- HH_260109: Drop ament_export_libraries to avoid non-namespaced link flags.

## 2026-01-09 21:25
- HH_260109: Export sensor kit target with namespace for downstream linking.

## 2026-01-09 21:15
- HH_260109: Install sensor kit library to lib/ so ament export resolves correctly.

## 2026-01-09 21:05
- HH_260109: Install sensor kit export set so downstream CMake can locate the library.

## 2026-01-09 20:55
- HH_260109: Export sensor kit library target to satisfy downstream find_package.

## 2026-01-09 20:20
- HH_260109: Replace robot_info_node with shared robot params library for visualizer.

## 2026-01-09 18:37
- HH_260109: Export sensor kit headers for downstream visualizer use.

## 2026-01-09 17:55
- HH_260109: Rename package to camping_cart_sensor_kit and move visualization node out.

## 2026-01-09 15:46
- HH_260109: Initialized work log; no code changes in this package today.
