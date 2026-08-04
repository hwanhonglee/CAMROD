# camrod_common

<!-- HH_260804 - Replace repeated dependency diagrams with one generated
inventory and a compact ownership contract. -->

Shared ROS 2 interface workspace. Runtime interfaces are implemented by the
`avg_msgs` package; this directory does not own motion algorithms.

![Shared interface contract](../docs/assets/module-guides/common/interface-contract-and-dependencies.png)

## Actual Simulation Runtime

![Live generated interface contract](../docs/assets/module-guides/common/runtime-interface-terminal-20260804.png)

`SIM RUNTIME CAPTURE`: the live `/system/status` topic resolves to
`avg_msgs/msg/SystemStatus`; the same screen shows its generated fields and
publisher/subscriber graph.

## At A Glance

| Uses | Function | Output |
|---|---|---|
| `rosidl`, `.msg`, `.srv` | Generates the common typed boundary used by CAMROD packages | C, C++, Python, and typesupport artifacts |
| Stable numeric constants | Keeps planning, service, system, UI, and voice state IDs aligned | Identical enum values in nodes, tests, and UI |

## Current Inventory

| Item | Count | Evidence |
|---|---:|---|
| Messages | 86 | `avg_msgs/msg/*.msg` |
| Services | 2 | `avg_msgs/srv/*.srv` |
| Direct CAMROD package dependents | 12 | Package manifests |
| Runtime performance | Not measured | Measure each publisher/subscriber path separately |

## Build And Inspect

```bash
cd ~/camrod_ws
colcon build --packages-select avg_msgs --symlink-install
source install/setup.bash

ros2 interface list | rg '^avg_msgs/'
ros2 interface show avg_msgs/msg/AvgServiceState
```

See [`avg_msgs/README.md`](avg_msgs/README.md) for key interfaces, versioning,
and validation commands.
