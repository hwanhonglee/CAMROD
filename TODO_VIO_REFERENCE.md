# TODO VIO References

<!-- HH_260109 Track VIO references, install notes, and topic wiring. -->

## Kimera-VIO
- Repository: https://github.com/MIT-SPARK/Kimera-VIO
- Workspace location: `camping_cart_localization/external/Kimera-VIO`
- Current integration: CSV bridge (`kimera_csv_bridge_node`) into `/localization/kimera_vio/*`
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
- Kimera-VIO: current project reference implementation.
- VINS-Fusion: strong performance but ROS2 integration effort.
- ORB-SLAM3: highest accuracy, heavier integration and compute.
