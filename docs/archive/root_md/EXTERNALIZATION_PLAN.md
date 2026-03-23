# CAMROD Externalization Plan (Binary -> Source)

This document defines how to reduce binary-only dependencies by moving practical
third-party stacks under each module's `external/` tree.

## Goal

- Make workspace bootstrap predictable after Git clone.
- Minimize "surprise" runtime dependency on random local overlays.
- Keep ROS 2 core in `/opt/ros` (base runtime), unless you intentionally fork ROS 2 itself.

## Current State

See:

- `DEPENDENCY_SOURCE_AUDIT.md` (module-wise dependency inventory)
- `build_camrod_all.sh` (build with nested external trees)
- `bootstrap_module_externals.sh` (sync vendorable external sources)

## What We Vendor in `external/`

- `camrod_sensing/external/ublox`
- `camrod_sensing/external/vanjee_lidar`
- `camrod_sensing/external/perception_pcl` (optional source mode)
- `camrod_localization/external/robot_localization`
- `camrod_map/external/lanelet2`
- `camrod_perception/external/vision_opencv` (`image_geometry`)
- `camrod_common/external/vision_msgs`
- `camrod_planning/external/laser_geometry`
- `camrod_planning/external/nav2_*` (already split in this workspace)

## What Stays in `/opt/ros` (Recommended)

These are ROS 2 core/foundation stacks and should remain under base runtime
for stability and maintenance cost:

- `rclcpp`, `rclpy`, `rcutils`, `launch`, `launch_ros`
- `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2*`
- ament/rosidl foundation packages

If you vendor all of them too, CAMROD effectively becomes a full ROS 2 distro fork.

## One-Command Flow

```bash
cd /home/camrod_ws
./src/bootstrap_module_externals.sh
./src/build_camrod_all.sh
source install/setup.bash
```

Update pinned externals later:

```bash
./src/bootstrap_module_externals.sh --update
```

## Notes

- `build_camrod_all.sh` auto-detects nested `external/*` roots and includes them in `--base-paths`.
- Full "no download ever" mode is realistically achieved by Docker image distribution,
  not by vendoring the entire ROS 2 core source tree.

