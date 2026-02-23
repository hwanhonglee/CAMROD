# camping_cart_planning — Nav2 Wrapper

# HH_260119 Draft history: HH_251202 (Dec 2, 2025); Nav2 migration HH_260105 (Jan 5, 2026)

## Overview

This package no longer builds its own global planner/controller nodes. It ships only the launch/config/BT/YAML assets needed to run Nav2 (SmacPlanner2D + BT Navigator + Controller). Use `nav2_lanelet.launch.py` directly or via `camping_cart_bringup`.

## Package layout (HH_260105)

```
camping_cart_planning/
├── CMakeLists.txt                 # HH_260105 no targets, install launch/config only
├── package.xml                    # Nav2 runtime deps only
├── config/
│   └── nav2_lanelet.yaml          # Nav2 planner/controller/BT parameters
└── launch/
    └── nav2_lanelet.launch.py     # Nav2 노드 래퍼 (네임스페이스 /planning)
```

## Usage

- Standalone:  
  `ros2 launch camping_cart_planning nav2_lanelet.launch.py`
- Full stack:  
  `ros2 launch camping_cart_bringup bringup.launch.py` (includes Nav2)

## Change log (HH_260105)

## 2026-02-06 21:26
- Path cost grid no longer forces outside_value (set to -1) so planners can compute the first path before any path is available.

## 2026-02-05 14:37
- Keep planner_server output on `/planning/plan`; rely on compute_path_bridge for `/planning/global_path` to avoid duplicate publishers.

## 2026-02-04 19:45
- Prefer Smac2D planner by default to ensure /planning/unsmoothed_plan is published.
- Fix global costmap width/height type mismatch by keeping width/height as integers (meters).
- Align global costmap origin/size to lanelet map bounds for consistent planning coverage.

## 2026-02-03 18:05
- Compute path bridge republishes last path (republish_rate_hz) and latches output for RViz.

## 2026-02-02 17:40
- Compute path bridge now prefers `/planning/unsmoothed_plan` with `/planning/plan` fallback.

## 2026-02-02 15:50
- Add plan-to-global_path bridge (/planning/plan -> /planning/global_path) for RViz.
- Add proximity RangeSensorLayer to costmap plugin list (bringup/planning configs).
- Sync path grid resolution updates for clearer visualization.

## 2026-01-05 (HH_260105)
- Remove legacy `global_planner_node`, `path_follower_node`, `global_planner.launch.py`
- Drop Lanelet2 and unused ROS msg deps from CMake/manifest
- Install Nav2 runtime assets only
- Integrate with camping_cart_map
- Implement actual planning logic (placeholder note)
- Add RViz config
- Add timeout convenience (legacy note)

## 2026-02-02 10:32
- Default Nav2 params now load from `camping_cart_bringup/config/planning/nav2_lanelet.yaml`.
- `planning.launch.py` uses bringup copies for goal/centerline snapper configs.
- `path_cost_grids.launch.py` loads bringup `path_cost_grids.yaml`.

## 2026-02-02 11:55
- Add ComputePathThroughPoses BT plugin library to match Humble BT tree.

## Build

```bash
cd /home/hong/camping_cart_ws
colcon build --packages-select camping_cart_planning

# 전체 빌드
colcon build
```

## Usage

### Run global planning

```bash
ros2 launch camping_cart_planning global_planner.launch.py \
  map_path:=/path/to/map.osm \
  origin_lat:=36.12491471 \
  origin_lon:=126.77473954
```

### Run node

```bash
ros2 run camping_cart_planning global_planner_node --ros-args \
  -p map_path:=/path/to/map.osm \
  -p origin_lat:=36.12491471 \
  -p origin_lon:=126.77473954 \
  -p goal_lane_id:=-1
```

### Set goals in RViz

1. Launch RViz
2. Set Fixed Frame to `map`
3. Choose "2D Nav Goal" in the toolbar
4. Click on the map

## Subscribed topics

- `/goal_pose` (geometry_msgs/PoseStamped): RViz 2D Nav Goal (input to goal_snapper)
- `/planning/goal_pose` (geometry_msgs/PoseStamped): lanelet-snapped goal for Nav2

## Published topics

- `/planning/global_path` (nav_msgs/Path): planned path

## Parameters

- `map_path`: Lanelet2 OSM file path (default: "")
- `origin_lat`: WGS84 latitude (default: 37.0)
- `origin_lon`: WGS84 longitude (default: 127.0)
- `goal_lane_id`: target lane ID (default: -1)

## Change log (HH_251203)

### 2026-01-23 15:30
- HH_260123: Fix swapped lanelet path layer topics (global/local costmap now consume matching path grids).
- HH_260123: Simplify Nav2 costmap layers to lanelet base + lanelet path + LiDAR obstacles + inflation; path cost grids now published under /planning/global_costmap/* and /planning/local_costmap/*.
- HH_260123: Path cost grids subscribe to /localization/pose (PoseStamped) to avoid mixed message types on pose_with_covariance topic.
- HH_260123: Path cost grids keep robot-centered window (use_path_bbox=false) and apply strong backward_penalty for forward-only preference.

### 2026-01-25 14:40
- HH_260125: Global path cost grid now uses path bounding box window (use_path_bbox=true) to stay fixed in map frame; local grid remains robot-centered.
- HH_260125: Prevent global grid origin drift by deriving bbox solely from path points (not current pose).

### 2026-01-26 10:20
- HH_260126: Global path grid locked to map origin/size (no drift); new lock_window param in lanelet_cost_grid_node with origin/width/height override.

### 2026-01-19
- HH_260119: Refresh costmap comment timestamps to match current layer layout (lanelet + LiDAR/Fusion/Proximity + inflation).

### 2026-01-14 14:50
- HH_260114: Translate README to English and standardize maintainer email/license notes.

### 2026-01-12 15:41
- HH_260112: Add lanelet-contained goal snapping and register speed_bump rules for map loading.
- HH_260112: Scope Nav2 parameters under /planning and pin BT plugin list to available libraries.

### 2026-01-12 13:20
- HH_260112: Route Nav2 controller/smoother odom_topic to /localization/odometry/filtered.

### 2026-01-12 12:55
- HH_260112: Use internal Nav2 costmaps (planner/controller) and align lifecycle list.

### 2026-01-12 12:45
- HH_260112: Add explicit global/local costmap servers to the Nav2 launch.
- HH_260112: Clarify goal_pose topic flow in planning documentation.

### 2026-01-12 10:40
- HH_260112: Add goal snapper node to keep goals within lanelet centerlines.
- HH_260112: Remap Nav2 goal input to /planning/goal_pose.

### 2026-01-09 20:30
- HH_260109: Switch costmap observation source to /perception/obstacles.

### 2026-01-09 18:37
- HH_260109: Update cost grid and obstacle topic prefixes for planning.

### 2026-01-09 15:46
- HH_260109: Add obstacle costmap layer configuration and 5 kph controller tuning.

### Major refactor notes

1. **Package rename**
   - `camping_cart_navigation` → `camping_cart_planning`
   - Rationale: separate map subsystem vs planning

2. **Removed Lanelet2 loader**
   - Previously bundled; now fully in camping_cart_map
   - Benefit: clear separation of concerns

3. **File cleanup**
   - Removed:
     - `src/lanelet2_loader_node.cpp`
     - `src/lanelet2_loader.cpp`
     - `include/camping_cart_planning/lanelet2_loader.hpp`
     - `scripts/find_map_origin.py`

4. **global_planner_node.cpp simplification**
   - Old: complex lanelet loader
   - New: simple RViz subscription
   - `/planning/goal_pose` subscribe → `/planning/global_path` publish

5. **CMakeLists.txt updates**
   - Removed camping_cart_map dependency
   - Kept only required deps (ROS2 messages, Lanelet2 routing)

## Architecture (HH_251203)

```
┌─────────────────────────────────────┐
│   camping_cart_map (Map Subsystem)  │
├─────────────────────────────────────┤
│ • lanelet2_map_loader_node          │
│ • map_projector_loader_node         │
│ • map_tf_generator_node             │
│ • lanelet2_map_visualization_node   │
└──────────────────┬──────────────────┘
                   │ (Map topics)
┌──────────────────▼──────────────────┐
│ camping_cart_planning (Planner)     │
├─────────────────────────────────────┤
│ • global_planner_node               │
│   ├─ Sub: /planning/goal_pose       │
│   └─ Pub: /planning/global_path     │
└─────────────────────────────────────┘
```

## Next steps

1. Finalize camping_cart_map build
2. Implement actual path planning in global_planner_node
3. Parameter-based site mapping
4. Add timeout helper
5. Integrate RViz visualization

---

**Note**: See source comments for HH_251202–HH_251203 refactor details.

<!-- HH_260109 Append work log entry for recent planning updates. -->

## 2026-01-30 16:05
- 2026-01-30: Add RemovePassedGoals BT plugin library to support navigate_through_poses behavior tree.
- 2026-01-30: Global path cost grid now uses map-bounds window (use_map_bbox) to avoid drifting origin.
- 2026-01-30: Update planning map configs to new_lanelet2_maps.osm origin.

## 2026-01-30 14:32
- HH_260130: Remove costmap nodes from lifecycle_manager node list to avoid duplicate configure transitions.

## 2026-01-29 22:13
- HH_260129: Fix nav2 costmap plugin class names to use '::' (Obstacle/Inflation) for pluginlib lookup.

## 2026-01-29 19:29
- HH_260129: Add global/local costmap nodes to lifecycle manager for automatic configure/activate.
- HH_260129: Align BT plugin library names with Humble action BT libraries to avoid bt_navigator load failure.
- HH_260129: Remove missing BT plugin names (follow_waypoints) and use assisted_teleop action variant.

## 2026-01-27 17:45
- HH_260127: Remove HH tags from runtime logs and demote startup logs to DEBUG (goal/centerline snapper).

## 2026-01-27 14:05
- HH_260127: Add PlannerSelector BT and enable NavFn+Smac planner plugins.
- HH_260127: Remap planner plan -> /planning/global_path and controller local_plan -> /planning/local_path.
- HH_260127: Add lifecycle management for global/local costmaps to prevent unconfigured state.
