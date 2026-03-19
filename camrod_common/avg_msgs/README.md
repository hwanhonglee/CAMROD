# avg_msgs

Shared interface package for CAMROD modules.

## Purpose
- Provide module-level message contracts (`Avg*Msgs`, diagnostics/health payloads)
- Provide compatibility wrappers for selected frequently-used message types
- Keep interface ownership centralized and versionable

## Message Families
- Module aggregates:
  - `AvgSensingMsgs`
  - `AvgLocalizationMsgs`
  - `AvgMapMsgs`
  - `AvgPlanningMsgs`
  - `AvgPerceptionMsgs`
  - `AvgPlatformMsgs`
  - `AvgSensorKitMsgs`
  - `AvgSystemMsgs`
  - `AvgBringupMsgs`
- Health/diagnostics:
  - `ModuleHealth`
  - `SystemDiagnostic`
  - `AvgLocalizationDiagnostics`

## Build
```bash
cd ~/camrod_ws
colcon build --packages-select avg_msgs
source install/setup.bash
```

## C++ Usage
```cpp
#include <avg_msgs/msg/avg_planning_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>
```

## Python Usage
```python
from avg_msgs.msg import AvgPlanningMsgs, ModuleHealth
```

## Dependency Snippet (`package.xml`)
```xml
<depend>avg_msgs</depend>
```

## Dependency Snippet (`CMakeLists.txt`)
```cmake
find_package(avg_msgs REQUIRED)
ament_target_dependencies(your_node avg_msgs)
```

