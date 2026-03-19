# Vanjee External Stack (SDK + Msg)

This directory vendors the Vanjee source stack used by `camrod_sensing`:

- `vanjee_lidar_msg`
- `vanjee_lidar_sdk`

Source origin:
- copied from `src/todo/vanjee_ws/src/*`
- scope intentionally limited to SDK + message package

## Build

```bash
cd /home/hong/camrod_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src src/camrod_sensing/external/vanjee_lidar \
  --packages-select vanjee_lidar_msg vanjee_lidar_sdk
source install/setup.bash
```

## Runtime check

```bash
ros2 pkg executables vanjee_lidar_sdk
ros2 interface list | rg vanjee_lidar_msg
```

Hardware note:
- without connected LiDAR / correct NIC binding, the driver may print
  `bind: Cannot assign requested address` while still proving executable startup.
