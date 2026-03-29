# camrod_localization

Localization module for GNSS-to-pose conversion, filtering, supervision, and state reporting.

## Purpose
- Convert GNSS (`NavSatFix`) into map-frame pose inputs
- Run ESKF (default) or EKF path
- Publish canonical localization outputs
- Provide supervisor/state watchering and drop-zone alignment support

## Entry Point
```bash
ros2 launch camrod_localization localization.launch.py
```

## Runtime Structure
```text
/sensing/gnss/ublox_gps_node/fix
    -> navsat_to_pose
    -> (ESKF/EKF)
    -> /localization/pose
    -> /localization/pose_with_covariance
    -> /localization/odometry/filtered
```

Optional branches:
- Kimera CSV bridge (`kimera_bridge_enable:=true`)
- Pose selector (`pose_selector_enable:=true`)
- Wheel odometry bridge (`wheel_bridge_enable:=true`)
- Drop-zone matcher for initialization gating

## Main Launch Arguments
- Input topics:
  - `navsat_topic` (default `/sensing/gnss/ublox_gps_node/fix`)
  - `gnss_pose_topic`
  - `gnss_pose_cov_topic`
- Map alignment:
  - `origin_lat`, `origin_lon`, `origin_alt`, `yaw_offset_deg`
- Pipeline toggles:
  - `use_eskf`
  - `wheel_bridge_enable`
  - `kimera_bridge_enable`
  - `pose_selector_enable`
- Namespaces:
  - `module_namespace` (default `localization`)
  - `platform_namespace` (default `platform`)
  - `system_namespace` (default `system`)

## Key Topics
- Outputs:
  - `/localization/pose`
  - `/localization/pose_with_covariance`
  - `/localization/odometry/filtered`
  - `/localization/state`
- Internal/auxiliary:
  - `/localization/initialpose3d`
  - `/localization/drop_zone/*`

## Configuration
- `config/eskf.yaml`
- `config/supervisor.yaml`
- `config/drop_zone_matcher.yaml`
- `config/kimera_bridge.yaml`
- `config/pose_selector.yaml`

Bringup-level defaults are mirrored in:
- `camrod_bringup/config/localization/*`

## StatusStream
- Module-local topic: `/localization/status`
- Aggregated topic: `/status_stream`

