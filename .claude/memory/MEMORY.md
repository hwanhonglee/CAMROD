# Memory Index

- [Project overview](project-overview.md) — CAMROD: ROS 2 Humble autonomous camping delivery robot on Agilex Ranger
- [Workspace layout](workspace-layout.md) — 12 camrod_* packages, roles, key node/config locations
- [Safety-critical path](safety-critical-path.md) — sensing→localization→planning→cmd_vel gate chain, state machine, parking
- [Build and run](build-and-run.md) — setup_camrod.sh / colcon_build.sh, launch args, sim validation
- [Field baseline & conventions](field-baseline-and-conventions.md) — v2.0.0 tuning values, HW port map, HH_ tags, commit/doc conventions
- [Light/indicator feature](light-indicator-feature.md) — 전조등/방향지시등 확정 설계: B안 turn_direction 태그 방식, MCU 페일세이프, 배선/JSON 결정
