# VIO Reference Notes

_Last updated: HH_260528_

VIO integration is future work. Current localization runs on GNSS + IMU + wheel odometry via EKF/ESKF.

## Candidate implementations

| Library | Repo | Notes |
|---|---|---|
| Kimera-VIO | https://github.com/MIT-SPARK/Kimera-VIO | CSV bridge (`kimera_csv_bridge_node`) exists for offline testing |
| VINS-Fusion | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion | Needs ROS2 port |
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | Highest accuracy, heaviest compute |

## Target topic wiring (when integrated)

| Direction | Topic | Type |
|---|---|---|
| Input | `/sensing/camera/econ_front/image_rect/compressed` | `sensor_msgs/CompressedImage` |
| Input | `/sensing/imu/data` | `sensor_msgs/Imu` |
| Output | `/localization/vio/odometry` | `nav_msgs/Odometry` |

> **HH_260528 note:** Camera topic changed from the old `/sensing/camera/processed/image`.
> Front camera output is now `/sensing/camera/econ_front/image_rect/compressed` (GPU-rectified).
