# EKF Fusion Notes (HH_260109)

This document explains how the EKF (robot_localization) fuses GNSS, IMU, Wheel, and VIO for the camping cart.

## Core idea: one filter, multiple sensors
The EKF keeps a single state estimate and updates it with every sensor measurement.
Instead of switching topics (A/B), it adjusts *how much* each measurement influences the state by using covariance.

Typical state (2D mode):
- x, y, yaw
- vx, vy, yaw_rate
- IMU biases (implicit in process model)

## Measurement weighting (why "GNSS dropouts" do not require switching)
Each measurement has a covariance matrix R.
The EKF update uses the Kalman gain K:
K = P * H^T * (H * P * H^T + R)^-1

If a sensor becomes unreliable:
- its R is large (or updates stop),
- K becomes small,
- the filter trusts other sensors more.

So "weighting" is not a hard switch, but a continuous trust scaling.

## Example flow with GNSS dropout
1) GNSS + IMU + Wheel + VIO all available:
   - GNSS anchors global position.
   - IMU + Wheel + VIO provide smooth local motion.

2) GNSS drops for ~10 sec:
   - GNSS updates stop (or R increases).
   - EKF relies on IMU + Wheel + VIO to propagate pose.
   - Position may drift slowly, but still continuous.

3) GNSS returns:
   - EKF re-anchors global position by blending GNSS updates.

## Sensor roles in this project
- GNSS: absolute position (global anchor) -> `/sensing/gnss/pose_with_covariance`
- IMU: orientation + angular velocity + acceleration -> `/sensing/imu/data`
- Wheel: twist (velocity) -> `/platform/wheel/odometry`
- VIO: local pose + twist -> `/localization/vio/odometry`
- Lanelet constraint: soft pose correction -> `/localization/lanelet_pose`

## Where EKF config is defined
- `camping_cart_localization/config/ekf.yaml`
- `camping_cart_bringup/config/localization/ekf.yaml`

## Practical tuning notes
- If GNSS is noisy: increase its covariance in `/sensing/gnss/pose_with_covariance`.
- If wheel odom is unstable: reduce which twist axes are enabled or increase noise.
- If VIO drifts fast: keep it but reduce its covariance weight.

## Why this is better than topic switching
- Continuous fusion avoids jumps.
- No explicit "failover" state machine is required.
- Each sensor can gracefully degrade by covariance.
