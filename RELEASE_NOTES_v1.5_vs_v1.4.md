# CAMROD v1.5 vs v1.4

This note summarizes the major workspace-level changes between the `v1.4` tag and current `v1.5` work.

## Diff summary

- Files changed: `37,833`
- Insertions: `52,717`
- Deletions: `4,084,201`

## Package-level highlights

- `camrod_map`
  - Launch hierarchy cleaned (`map.launch.py` remains top entry, logic split to lower launches).
  - RViz operator profile added (`camrod_operator.rviz`, `operator_theme.qss`).
  - Cost-grid and visualization launch wiring reorganized.
- `camrod_localization`
  - Launch split into focused components (`adapter/filter/monitor/map_helper`).
  - Legacy duplicated nodes/config paths removed or unified.
  - Shared map reference flow aligned with `camrod_map/config/map_info.yaml`.
- `camrod_sensing`
  - IMU/LiDAR/Radar launch files reorganized by responsibility.
  - Velocity converter topic wiring aligned for diagnostics compatibility.
- `camrod_platform`
  - Top launch simplified with sub-launch composition.
  - Platform cmd_vel gate and visualization launch flow reorganized.
- `camrod_perception`
  - Perception launch split into thinner top + functional sub-launches.
- `camrod_planning`
  - Planning launch split into lifecycle/local-path/replanner/gate/state-machine units.
- `camrod_system`
  - Legacy `robot_diagnostics/*` tree removed from active structure.
  - Current diagnostics launch/config pipeline consolidated under `camrod_system`.
- `camrod_bringup`
  - Top launch made thin (`bringup.launch.py` -> `_bringup_impl.py`).
  - Config wiring updated for per-package launch compatibility and map-info sharing.
- `camrod_parking`
  - Parking package integrated into workspace structure.

## External and submodule handling

- Submodules explicitly tracked in this workspace:
  - `camrod_platform/external/ugv_sdk`
  - `camrod_platform/external/ranger_ros2`
  - `camrod_parking/external/opennav_docking`
  - `camrod_parking/external/apriltag_ros`
  - `camrod_parking/external/apriltag_msgs`
- `.gitmodules` added and submodule pointers recorded in repository history.

## Notes

- Large deletion volume is mainly from removing legacy diagnostic trees (`todo/`, old `robot_diagnostics` layout) and deduplicating prior structures.
- This note is intended as a quick operational summary for release tracking and review.
