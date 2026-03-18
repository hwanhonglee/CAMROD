# AVG Message Migration Map

## HH_260312-00:00 Scope
This document tracks module-level message consolidation to `avg_msgs`.

## Module Payload Topics (Optional)
- `/map/diag_payload` -> `avg_msgs/AvgMapMsgs`
- `/sensing/diag_payload/*` -> `avg_msgs/AvgSensing*`
- `/localization/diag_payload` -> `avg_msgs/AvgLocalizationMsgs`
- `/planning/diag_payload` -> `avg_msgs/AvgPlanningMsgs`
- `/platform/diag_payload` -> `avg_msgs/AvgPlatformMsgs`
- `/perception/diag_payload` -> `avg_msgs/AvgPerceptionMsgs`
- `/sensor_kit/diag_payload` -> `avg_msgs/AvgSensorKitMsgs`
- `/system/diag_payload` -> `avg_msgs/AvgSystemMsgs`
- `/bringup/diag_payload` -> `avg_msgs/AvgBringupMsgs`

## Cross-Module Subscription Migration (Completed)
- `camrod_platform/platform_diagnostic_node.py`
  - before: direct `/localization/*` subscriptions
  - now: direct health/diagnostic aggregation on `/diagnostic`
- `camrod_perception/perception_diagnostic_node.py`
  - before: direct sensing topic checks only
  - now: consolidated module diagnostics via `/diagnostic`
- `camrod_localization/localization_diagnostic_node.py`
  - before: direct sensing/localization topic checks only
  - now: consolidated module diagnostics via `/diagnostic`

## Diagnostic Stream Policy
- consolidated diagnostic topic: `/diagnostic`
- checker status naming convention: `<module>/checker`
- module diagnostic status naming convention: `<module>/diagnostic`

## Remaining Full-Migration Work
- internal computational nodes (planner/controller/costmap/localization filters) still consume ROS core message types (`nav_msgs`, `sensor_msgs`, `geometry_msgs`) by design.
- full replacement of internal bus with `avg_msgs` would require adapter layers or full node API changes and is intentionally staged.
