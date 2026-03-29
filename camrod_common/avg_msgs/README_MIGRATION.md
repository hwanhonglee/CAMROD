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
- `camrod_platform/platform_status_node.py`
  - before: direct `/localization/*` subscriptions
  - now: direct state/status aggregation on `/status`
- `camrod_perception/perception_status_node.py`
  - before: direct sensing topic checks only
  - now: consolidated module status_stream via `/status`
- `camrod_localization/localization_status_node.py`
  - before: direct sensing/localization topic checks only
  - now: consolidated module status_stream via `/status`

## Status Stream Policy
- consolidated status topic: `/status`
- validator status naming convention: `<module>/validator`
- module status status naming convention: `<module>/status`

## Remaining Full-Migration Work
- internal computational nodes (planner/controller/costmap/localization filters) still consume ROS core message types (`nav_msgs`, `sensor_msgs`, `geometry_msgs`) by design.
- full replacement of internal bus with `avg_msgs` would require adapter layers or full node API changes and is intentionally staged.
