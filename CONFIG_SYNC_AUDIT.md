# Config Sync Audit

Generated: 2026-03-19

Scope: compare `camrod_<module>/config` vs `camrod_bringup/config/<module>`.

## map

| file | scope | status |
|---|---|---|
| `drop_zones.yaml` | `shared` | `DIFF` |
| `lanelet_cost_grid.yaml` | `shared` | `match` |
| `map_info.yaml` | `shared` | `DIFF` |
| `nav2_params_costlayer_example.yaml` | `shared` | `DIFF` |
| `map_visualization.yaml` | `bringup_only` | `-` |

## localization

| file | scope | status |
|---|---|---|
| `ekf.yaml` | `shared` | `DIFF` |
| `health_monitor.yaml` | `shared` | `DIFF` |
| `kimera_bridge.yaml` | `shared` | `DIFF` |
| `localization_origin.yaml` | `shared` | `DIFF` |
| `localization_params.yaml` | `shared` | `DIFF` |
| `navsat_test_inputs.yaml` | `shared` | `DIFF` |
| `pose_selector.yaml` | `shared` | `DIFF` |
| `drop_zone_matcher.yaml` | `bringup_only` | `-` |
| `drop_zones.yaml` | `bringup_only` | `-` |
| `eskf.yaml` | `bringup_only` | `-` |
| `supervisor.yaml` | `bringup_only` | `-` |

## planning

| file | scope | status |
|---|---|---|
| `bt/navigate_to_pose_w_planner_selector.xml` | `shared` | `DIFF` |
| `centerline_snapper.yaml` | `shared` | `DIFF` |
| `goal_replanner.yaml` | `shared` | `DIFF` |
| `goal_snapper.yaml` | `shared` | `DIFF` |
| `nav2_lanelet.yaml` | `shared` | `DIFF` |
| `path_cost_grids.yaml` | `shared` | `match` |
| `planning_state_machine.yaml` | `shared` | `DIFF` |
| `planning_state_machine_keypoints.yaml` | `shared` | `DIFF` |
| `camping_sites.yaml` | `bringup_only` | `-` |
| `local_path_extractor.yaml` | `bringup_only` | `-` |
| `nav2_base.yaml` | `bringup_only` | `-` |
| `nav2_behavior.yaml` | `bringup_only` | `-` |
| `nav2_lanelet_overlay.yaml` | `bringup_only` | `-` |
| `nav2_vehicle.yaml` | `bringup_only` | `-` |
| `nav2_vehicle_dwb.yaml` | `bringup_only` | `-` |

## sensing

| file | scope | status |
|---|---|---|
| `camera/preprocessor.yaml` | `module_only` | `-` |
| `gnss/ntrip_client.yaml` | `module_only` | `-` |
| `gnss/zed_f9p_rover.yaml` | `module_only` | `-` |
| `imu/microstrain_cv7.yaml` | `module_only` | `-` |
| `imu/microstrain_gq7.yaml` | `module_only` | `-` |
| `imu/microstrain_org.yaml` | `module_only` | `-` |
| `imu/platform_velocity_converter.yaml` | `module_only` | `-` |
| `lidar/cost_grid.yaml` | `module_only` | `-` |
| `lidar/preprocessor.yaml` | `module_only` | `-` |
| `lidar/vanjee/config.yaml` | `module_only` | `-` |
| `radar/cost_grid.yaml` | `module_only` | `-` |
| `radar/sen0592_radar.yaml` | `module_only` | `-` |
| `sensing_params.yaml` | `module_only` | `-` |

## perception

| file | scope | status |
|---|---|---|
| `perception_params.yaml` | `shared` | `match` |

## platform

| file | scope | status |
|---|---|---|
| `vehicle_params.yaml` | `shared` | `DIFF` |
| `robot_visualization.yaml` | `bringup_only` | `-` |

## sensor_kit

| file | scope | status |
|---|---|---|
| `robot_params.yaml` | `shared` | `DIFF` |

## system

| file | scope | status |
|---|---|---|
| `system_checker.yaml` | `shared` | `DIFF` |

