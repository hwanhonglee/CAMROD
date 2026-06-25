# CAMROD Message Migration Audit

HH_260617: This document defines the migration boundary for moving CAMROD-owned
runtime interfaces into `avg_msgs` without breaking ROS 2 tool, Nav2, RViz, and
driver interoperability.

## 1. Current State

`avg_msgs` currently contains two different interface styles:

- Real ROS interfaces generated from `msg/`, `srv/`, and `action/`.
- C++-only alias headers under `include/avg_msgs/msg/` that map names such as
  `avg_msgs::msg::PoseStamped` to standard ROS messages.

The alias headers are not generated `.msg` interfaces. They work only for C++
include compatibility and do not create Python imports such as
`from avg_msgs.msg import PoseStamped`.

## 2. Migration Rule

Keep standard ROS messages when the topic is a public ROS ecosystem contract:

- Nav2, RViz, TF, diagnostics, and lifecycle/action status contracts.
- Sensor/driver raw outputs such as `sensor_msgs/Imu`, `sensor_msgs/NavSatFix`,
  `sensor_msgs/PointCloud2`, `sensor_msgs/Image`, and `sensor_msgs/Range`.
- Geometry primitives such as `geometry_msgs/PoseStamped`, `nav_msgs/Path`,
  `nav_msgs/Odometry`, and `nav_msgs/OccupancyGrid` when external tools consume
  them directly.

Create `avg_msgs` interfaces when the topic is CAMROD-owned semantics:

- Mission, service, recall, destination, and state-machine commands.
- Module readiness, module status, and normalized system status.
- Platform-normalized CAN/BMS state and CAMROD-specific vehicle status.
- Planning-specific progress, route intent, gate state, and mission phase.
- Sensing/localization summaries that aggregate several raw ROS topics into one
  CAMROD status payload.

## 3. Naming Rule

Use one message per semantic contract, not one large catch-all message per package.

Recommended pattern:

- `PlanningMissionState.msg`
- `PlanningDestinationRequest.msg`
- `PlanningRouteProgress.msg`
- `PlanningGateStatus.msg`
- `SensingGnssStatus.msg`
- `SensingRadarStatus.msg`
- `LocalizationState.msg`
- `LocalizationHealth.msg`
- `PlatformBmsStatus.msg`
- `PlatformMotionStatus.msg`
- `SystemModuleStatus.msg`
- `UiDestinationSelection.msg`

Avoid new generic aggregate names such as `AvgPlanningMsgs` unless there is a
specific consumer that needs a full snapshot payload.

## 4. Alias Header Policy

Do not add more alias headers for standard ROS messages.

HH_260623 - Keep the nested forwarding alias-header tree under
`include/avg_msgs/avg_msgs/msg/` because ROSIDL exports
`install/avg_msgs/include/avg_msgs` to downstream targets in this workspace.
With that include path, source includes such as `avg_msgs/msg/range.hpp` resolve
through the nested forwarding tree. Removing it breaks current C++ consumers.

Existing alias headers should be treated as temporary compatibility glue. Long
term, replace C++ includes such as:

```cpp
#include <avg_msgs/msg/pose_stamped.hpp>
```

with the real standard include:

```cpp
#include <geometry_msgs/msg/pose_stamped.hpp>
```

or create a real generated `avg_msgs/msg/*.msg` only when the interface has
CAMROD-owned fields beyond the standard message.

## 5. Recommended Migration Order

1. Freeze public topic contracts in each package README and launch/config file.
2. Move CAMROD-owned status/command topics to generated `avg_msgs` interfaces.
3. Keep bridge topics that Nav2/RViz/drivers require as standard ROS messages.
4. Remove C++ alias headers after all code includes real standard headers or real
   generated `avg_msgs` headers.
5. Build `avg_msgs` first, then rebuild all packages that depend on it.

## 6. Current High-Value Candidates

These are good candidates for real `avg_msgs` interfaces:

- Planning mission state and destination selection.
- Planning progress and goal-reached result.
- Platform BMS charging/status payload from Ranger CAN.
- Sensing GNSS dual-antenna status summary.
- Radar device status summary separate from raw `sensor_msgs/Range`.
- System readiness gate summary consumed by UI/planning.

These should generally remain standard ROS messages:

- `/goal_pose`, `/planning/goal_pose_snapped`, `/planning/global_path`.
- `/localization/pose`, `/localization/odometry/filtered`.
- `/sensing/gnss/fix`, `/sensing/imu/data`, point clouds, images, cost grids.
- `/system/diagnostics_agg`.
