# camrod_common

Common/shared assets for CAMROD modules.

## Current Scope
- `avg_msgs/`: shared ROS interface package used across CAMROD modules

## Directory Layout
```text
camrod_common/
  └── avg_msgs/
      ├── msg/
      ├── include/avg_msgs/
      ├── CMakeLists.txt
      └── package.xml
```

## Build
```bash
cd ~/camrod_ws
colcon build --packages-select avg_msgs
source install/setup.bash
```

## Notes
- Keep common interfaces here to avoid duplication across module packages.
- If a message/action/service is shared by two or more modules, define it in `avg_msgs`.

