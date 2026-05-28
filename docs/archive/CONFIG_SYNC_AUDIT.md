# Config Sync Audit

> **STALE — Generated: 2026-03-19.**
> Several referenced files have been deleted or restructured since then.
> Known stale entries:
> - `sensing/camera/preprocessor.yaml` — deleted (HH_260528)
> - `sensing/sensing_params.yaml` — deleted (HH_260528)
> - `platform/ranger_params.yaml` — deleted (HH_260528)
> - `sensing/camera` now has `camera_params.yaml` + `camera_rear_calibration.yaml`
> - `sensing/imu` split into `microstrain_cv7.yaml` / `microstrain_gq7.yaml` (unified via `imu_model`)
>
> Re-run `util/generate_config_sync_audit.py` for a current snapshot.

---

## map

| file | scope | status (2026-03-19) |
|---|---|---|
| `drop_zones.yaml` | `shared` | `DIFF` |
| `lanelet_cost_grid.yaml` | `shared` | `match` |
| `map_info.yaml` | `shared` | `DIFF` |
| `nav2_params_costlayer_example.yaml` | `shared` | `DIFF` |
| `map_visualization.yaml` | `bringup_only` | `-` |

## localization

| file | scope | status (2026-03-19) |
|---|---|---|
| `ekf.yaml` | `shared` | `DIFF` |
| `health_monitor.yaml` | `shared` | `DIFF` |
| `pose_selector.yaml` | `shared` | `DIFF` |
| `drop_zone_matcher.yaml` | `bringup_only` | `-` |
| `drop_zones.yaml` | `bringup_only` | `-` |
| `eskf.yaml` | `bringup_only` | `-` |
| `supervisor.yaml` | `bringup_only` | `-` |

## planning

| file | scope | status (2026-03-19) |
|---|---|---|
| `bt/navigate_to_pose_w_planner_selector.xml` | `shared` | `DIFF` |
| `centerline_snapper.yaml` | `shared` | `DIFF` |
| `goal_replanner.yaml` | `shared` | `DIFF` |
| `goal_snapper.yaml` | `shared` | `DIFF` |
| `nav2_lanelet.yaml` | `shared` | `DIFF` |
| `path_cost_grids.yaml` | `shared` | `match` |
| `local_path_extractor.yaml` | `bringup_only` | `-` |
| `nav2_base.yaml` | `bringup_only` | `-` |
| `nav2_behavior.yaml` | `bringup_only` | `-` |
| `nav2_lanelet_overlay.yaml` | `bringup_only` | `-` |
| `nav2_vehicle.yaml` | `bringup_only` | `-` |

## sensing (partially stale — see note above)

| file | scope | status (2026-03-19) |
|---|---|---|
| `camera/camera_params.yaml` | `shared` | (restructured HH_260528) |
| `camera/camera_rear_calibration.yaml` | `module_only` | (new HH_260528) |
| `gnss/ntrip_client.yaml` | `module_only` | `-` |
| `gnss/zed_f9p_rover.yaml` | `module_only` | `-` |
| `imu/microstrain_cv7.yaml` | `module_only` | `-` |
| `imu/microstrain_gq7.yaml` | `module_only` | `-` |
| `lidar/cost_grid.yaml` | `module_only` | `-` |
| `lidar/preprocessor.yaml` | `module_only` | `-` |
| `lidar/vanjee/config.yaml` | `module_only` | `-` |
| `radar/cost_grid.yaml` | `module_only` | `-` |
| `radar/sen0592_radar.yaml` | `module_only` | `-` |

## perception

| file | scope | status (2026-03-19) |
|---|---|---|
| `perception_params.yaml` | `shared` | `match` |

## platform

| file | scope | status (2026-03-19) |
|---|---|---|
| `robot_visualization.yaml` | `bringup_only` | `-` |

## sensor_kit

| file | scope | status (2026-03-19) |
|---|---|---|
| `robot_params.yaml` | `shared` | `DIFF` |

## system

| file | scope | status (2026-03-19) |
|---|---|---|
| `system_checker.yaml` | `shared` | `DIFF` |
