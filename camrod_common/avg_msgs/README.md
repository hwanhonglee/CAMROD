# avg_msgs

## Role
`avg_msgs` is the shared ROS 2 interface package for CAMROD modules.

## Package Diagram
```mermaid
graph TD
  MSG[(msg interface files)] --> GEN[[rosidl generation]]
  SRV[(srv interface files)] --> GEN
  GEN -. used by .-> RUNTIME[[camrod runtime packages]]
  NOEXEC[no executable runtime node in avg_msgs]
  GEN --> NOEXEC
```

Diagram legend: `[node/process]`, `((topic stream))`, `[(config or interface)]`, `[[external package or launch]]`, dashed arrow = dependency only.

## Runtime Node/Data Flow
`avg_msgs` has no executable node.

| Item | Input | Output |
|---|---|---|
| Message definitions (`msg/`) | interface schema files | generated message classes |
| Service definitions (`srv/`) | interface schema files | generated service classes |

## Inter-Package Connections
```mermaid
graph LR
  AVG[(avg_msgs interface package)] -. interface types .-> MAP[[camrod_map]]
  AVG -. interface types .-> SENSING[[camrod_sensing]]
  AVG -. interface types .-> LOCALIZATION[[camrod_localization]]
  AVG -. interface types .-> PLANNING[[camrod_planning]]
  AVG -. interface types .-> PLATFORM[[camrod_platform]]
  AVG -. interface types .-> PERCEPTION[[camrod_perception]]
  AVG -. interface types .-> SYSTEM[[camrod_system]]
  AVG -. interface types .-> API[[camrod_api]]
```

## Typical Interface Groups
| Group | Examples | Typical Use |
|---|---|---|
| Module status payloads | `AvgMapMsgs`, `AvgPlanningMsgs`, `AvgLocalizationMsgs`, `AvgSystemMsgs` | module-level health/state reporting |
| Unified transport aliases | `PoseStamped`, `Odometry`, `OccupancyGrid`, `MarkerArray` | consistent message use across custom nodes |
| Planning/system helpers | `ModuleState`, `AvgLocalizationStatusStream`, `RequestGoalByKey.srv` | state machines, diagnostics, service control |

## Practical Usage
```bash
cd ~/camrod_ws
colcon build --packages-select avg_msgs
source install/setup.bash
```

C++ dependency:
```cmake
find_package(avg_msgs REQUIRED)
ament_target_dependencies(your_target avg_msgs)
```

Python dependency (`package.xml`):
```xml
<depend>avg_msgs</depend>
```
