# TODO VIO References

<!-- HH_260109 Track VIO references, install notes, and topic wiring. -->

## OpenVINS (Recommended)
- Paper: "OpenVINS: A Research Platform for Visual-Inertial Estimation"
  - Venue: ICRA 2020
  - Link: https://github.com/rpng/open_vins
  - Paper PDF: https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf
- Branch used (ROS2): develop_v2.7
- Workspace location: `camping_cart_localization/external/open_vins`
- Local config: `camping_cart_localization/config/open_vins/camping_cart/estimator_config.yaml`
- Install (ROS2):
  1) Clone `open_vins` into workspace src.
  2) Build with `colcon build --packages-select open_vins` (or package list inside repo).
  3) Provide camera + IMU topics and calibration YAML.
- Topic wiring (target):
  - Input: `/sensing/camera/processed/image`, `/sensing/imu/data`
  - Output (desired): `/localization/vio/odometry`

## VINS-Fusion
- Paper: "VINS-Fusion: A Robust Multi-Sensor Visual-Inertial State Estimator"
  - Venue: IROS 2019
  - Link: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
  - Paper PDF: https://arxiv.org/abs/1901.04413
- Install (ROS2): needs ROS2 port; verify fork and compatibility.
- Topic wiring (target):
  - Input: `/sensing/camera/processed/image`, `/sensing/imu/data`
  - Output (desired): `/localization/vio/odometry`

## ORB-SLAM3
- Paper: "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multi-Map SLAM"
  - Venue: IEEE T-RO 2021
  - Link: https://github.com/UZ-SLAMLab/ORB_SLAM3
  - Paper PDF: https://arxiv.org/abs/2007.11898
- Install (ROS2): requires wrapper/bridge; heavier runtime.
- Topic wiring (target):
  - Input: `/sensing/camera/processed/image`, `/sensing/imu/data`
  - Output (desired): `/localization/vio/odometry`

## Decision Notes
- OpenVINS: stable, maintainable, ROS2-friendly.
- VINS-Fusion: strong performance but ROS2 integration effort.
- ORB-SLAM3: highest accuracy, heavier integration and compute.
