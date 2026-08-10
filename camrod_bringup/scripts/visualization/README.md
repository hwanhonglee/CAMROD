# CAMROD Visualization Tools

<!-- HH_260810 - Keep cross-package documentation renderers in one owned
location while leaving live ROS visualization nodes with their packages. -->

This directory contains offline PNG/GIF renderers for repository documentation
and committed evidence. `camrod_bringup` owns them because they combine several
package configurations, full-stack simulation records, maps, and release
artifacts.

| Tool group | Inputs | Output claim |
|---|---|---|
| `render_module_readme_assets.py` | Package YAML, messages, committed reports | Package architecture and explicitly labelled measured/source-derived figures |
| `render_tapered_rounded_*.py` | Current geometry, map-v17 ROS timeline | Current contour contract and measured ROS road replay |
| `render_park_operating_points.py` | Current OSM, Park origin, semantic YAML | Hash-bound B1-B13, drop-zone, and parking-lot coordinate overview |
| `render_*recovery*.py` | Map-bound recovery JSON | Contact, candidate, release, retry, and completion PNG/GIF |
| `render_camping_site_sequence_results.py` | Site geometry and simulation records | Turnaround/roadside state-sequence PNG/GIF |
| `render_v2_1_5_service_results.py`, `render_rpp_service_ab.py` | Service-run JSON/logs | Repeated-service and controller A/B reports |

Operational visualization remains package-local:

- `camrod_planning/scripts/path_visualizer_node.py` publishes live ROS markers.
- `camrod_sensing/scripts/radar_status_gui.py` displays real radar topics.
- `camrod_platform/scripts/velocity_kph_gui.py` displays live platform speed.
- RViz and UI launch files remain with their owning packages.

Files under repository `util/`, user `copy` files, external vendor tools, and
disabled experiments are outside this organization contract.

The source path is used for direct regeneration. Colcon installs each renderer
flat under `lib/camrod_bringup`, so existing commands such as
`ros2 run camrod_bringup render_automatic_recovery_results.py` remain valid.
