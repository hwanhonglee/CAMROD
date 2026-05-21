# 🗺️ camrod_map — Lanelet2 map, cost grids & semantic areas

## 1. 📋 Summary

`camrod_map` is the map layer of the CAMROD stack. It loads a Lanelet2 `.osm` file, projects all primitives into the `map` frame using a `local_cartesian` projector, and serves a traversability cost grid (`/map/cost_grid/lanelet`) that drives every planning and sensing cost layer downstream. It also publishes the authoritative TF `world→map` static transform, exports semantic areas (drop zones, camping sites) to YAML, and provides multi-source MarkerArray visualization for RViz.

> 📌 **Single source of truth:** `config/map_info.yaml` — all other packages read their `map_path`, `offset_lat/lon/alt`, and frame IDs from this file via launch argument forwarding.

---

## 2. 🚀 Quick Start

```bash
# 1. Set the map path and WGS84 origin in config/map_info.yaml (once per deployment).

# 2. Launch the full map stack:
ros2 launch camrod_map map.launch.py

# 3. Override map at runtime:
ros2 launch camrod_map map.launch.py \
  map_path:=/abs/path/to/lanelet2_maps.osm \
  origin_lat:=36.7292921 \
  origin_lon:=127.4429577 \
  origin_alt:=84.0

# 4. Export semantic areas to YAML (run once after map edit):
ros2 launch camrod_map area_export.launch.py \
  output_yaml_path:=/home/hong/camrod_ws/src/camrod_map/config/drop_zones.yaml \
  camping_sites_output_yaml_path:=/home/hong/camrod_ws/src/camrod_planning/config/camping_sites.yaml
```

> 💡 Verify in RViz: add topics `/map/markers` (MarkerArray) and `/map/cost_grid/lanelet` (OccupancyGrid). Both should appear within 3 s of launch.

---

## 3. 🗺️ System Position

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph LR
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef parking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
  classDef config       fill:#FFFBEB,stroke:#D97706,stroke-width:1.5px,color:#92400E;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  subgraph UP["⬆️ Upstream"]
    LOC[🧩 camrod_localization]:::localization
    SENS[🧩 camrod_sensing]:::sensing
    PLAN_IN[🧩 camrod_planning]:::planning
  end

  subgraph CM["🗺️ camrod_map"]
    MAP[🧩 camrod_map]:::highlight
  end

  subgraph DN["⬇️ Downstream"]
    SENS_OUT[🧩 camrod_sensing]:::sensing
    PLAN_OUT[🧩 camrod_planning]:::planning
    PLAT[🧩 camrod_platform]:::platform
    ALL[🧩 all packages]:::system
  end

  LOC  ==>|📡 /planning/lanelet_pose| MAP
  SENS ==>|📡 lidar/radar cost grids| MAP
  PLAN_IN -->|📡 /planning/global_path| MAP

  MAP ==>|📡 /map/cost_grid/lanelet| SENS_OUT
  MAP ==>|📡 /map/cost_grid/lanelet| PLAN_OUT
  MAP -->|📡 /map/markers| PLAT
  MAP -->|TF world→map| ALL
  MAP -.->|⚙️ drop_zones.yaml| PLAN_OUT
  MAP -.->|⚙️ camping_sites.yaml| PLAN_OUT

  linkStyle 0,1,4,5 stroke:#F59E0B,stroke-width:2.5px;
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · ⚙️ Config · 🛠️ Hardware · 📦 External · ==> critical path · -.-> optional

*Figure 1 — camrod_map is the only package that publishes the static `world→map` TF and the canonical lanelet cost grid; all navigation decisions are ultimately shaped by these outputs.*

---

## 4. 🏗️ Runtime Architecture

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph TD
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef parking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
  classDef config       fill:#FFFBEB,stroke:#D97706,stroke-width:1.5px,color:#92400E;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  subgraph STATIC["🗂️ Static Map"]
    OSM{{🛠️ Lanelet2 .osm}}:::hardware
    LMAP[🧩 lanelet_map_provider]:::mapping
    MVIS((📡 /map/markers)):::topic
    TF((📡 TF world→map)):::topic
    OSM ==> LMAP
    LMAP ==> MVIS
    LMAP ==> TF
  end

  subgraph GRID["🧮 Cost Grids"]
    LGRID[🧩 lanelet_boundary_cost_grid]:::mapping
    POSE((📡 /planning/lanelet_pose)):::topic
    GPATH((📡 /planning/global_path)):::topic
    LCOST((📡 /map/cost_grid/lanelet)):::topic
    LBASE((📡 /map/cost_grid/planning_base)):::topic
    POSE --> LGRID
    GPATH --> LGRID
    LGRID ==> LCOST
    LGRID -.-> LBASE
  end

  subgraph VIZ["🎨 Visualization"]
    MCFM[🧩 cost_grid_multi_marker]:::mapping
    LIDAR((📡 /sensing/cost_grid/lidar)):::topic
    RADAR((📡 /sensing/cost_grid/radar)):::topic
    LMRK((📡 lanelet_markers)):::topic
    LIMRK((📡 lidar_markers)):::topic
    RMRK((📡 radar_markers)):::topic
    AGG[🧩 cost_grid_marker_aggregator]:::mapping
    CONTRIB((📡 /map/cost_grid/inflation_markers)):::topic
    INFGRID((📡 /planning/global_costmap/costmap)):::topic
    CFMRK[🧩 nav2_costmap_debug_marker]:::mapping
    INFLMRK((📡 inflation_nav2_markers)):::topic
    LIDAR --> MCFM
    RADAR --> MCFM
    MCFM --> LMRK
    MCFM --> LIMRK
    MCFM --> RMRK
    LMRK --> AGG
    LIMRK --> AGG
    RMRK --> AGG
    AGG --> CONTRIB
    INFGRID -.-> CFMRK
    CFMRK -.-> INFLMRK
  end

  subgraph EXPORT["📤 Exports"]
    CFN[🧩 lanelet_cost_field_visualizer]:::mapping
    CFIELD((📡 lanelet_field_markers)):::topic
    AEX[🧩 area_exporter]:::mapping
    DZ[(⚙️ drop_zones.yaml)]:::config
    CS[(⚙️ camping_sites.yaml)]:::config
    CFN -.-> CFIELD
    AEX --> DZ
    AEX --> CS
  end

  OSM --> LGRID
  OSM -.-> CFN
  OSM --> AEX
  LCOST --> MCFM
  GPMRK((📡 global_path_markers)):::topic --> AGG
  LPMRK((📡 local_path_markers)):::topic --> AGG

  linkStyle 3,6 stroke:#F59E0B,stroke-width:2.5px;
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · ⚙️ Config · 🛠️ Hardware · ==> critical path · -.-> optional (debug/disabled)

*Figure 2 — Full runtime node graph. Dashed edges indicate nodes/topics disabled by default or used only for debug.*

### Node summary

| Node (executable) | Inputs | Outputs | Key params |
|---|---|---|---|
| `lanelet2_map_node` (`lanelet_map_provider`) | Lanelet2 .osm | `/map/markers`, TF `world→map` | `map_path`, `offset_lat/lon/alt`, `world_frame_id`, `map_frame_id` |
| `lanelet_cost_grid_node` (`lanelet_boundary_cost_grid`) | .osm, `/planning/lanelet_pose`, `/planning/global_path` | `/map/cost_grid/lanelet` (secondary, centerline, **active**); `/map/cost_grid/planning_base` (primary, bounds, **disabled**) | `cost_mode`, `resolution`, `width`, `height`, `centerline_half_width`, `outside_value`, `primary_enable`, `secondary.output_topic` |
| `multi_cost_field_marker_node` (`cost_grid_multi_marker`) | `/map/cost_grid/lanelet`, `/sensing/cost_grid/lidar`, `/sensing/cost_grid/radar` | `/map/cost_grid/lanelet_markers`, `lidar_markers`, `radar_markers` | `palettes`, `alphas`, `marker_scales`, `z_offsets` |
| `cost_field_marker_node` (`nav2_costmap_debug_marker`) | `/planning/global_costmap/costmap` | `/map/cost_grid/inflation_nav2_markers` | `palette`, `alpha`, `marker_scale` (off by default) |
| `marker_array_aggregator_node` (`cost_grid_marker_aggregator`) | All cost marker topics | `/map/cost_grid/inflation_markers` | `stale_timeout_s`, `republish_period_s` (off by default) |
| `cost_field_node` (`lanelet_cost_field_visualizer`) | .osm | `/map/cost_grid/lanelet_field_markers` | `weights.distance/curvature`, `percentile_clip` (off by default) |
| `area_exporter_node` (`area_exporter`) | .osm | `drop_zones.yaml`, `camping_sites.yaml` | `output_yaml_path`, `camping_sites_output_yaml_path`, `default_yaw_deg` |

---

## 5. 🧮 Cost Grid Modes

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontFamily': 'ui-sans-serif, system-ui, sans-serif', 'fontSize': '14px', 'primaryColor': '#EEF2FF', 'primaryTextColor': '#0F172A', 'primaryBorderColor': '#6366F1', 'lineColor': '#475569'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true, 'padding': 12}}}%%
graph LR
  classDef sensing      fill:#ECFEFF,stroke:#06B6D4,stroke-width:1.5px,color:#0E7490;
  classDef localization fill:#ECFDF5,stroke:#10B981,stroke-width:1.5px,color:#047857;
  classDef mapping      fill:#FEF3C7,stroke:#F59E0B,stroke-width:1.5px,color:#B45309;
  classDef perception   fill:#FCE7F3,stroke:#EC4899,stroke-width:1.5px,color:#9D174D;
  classDef planning     fill:#EEF2FF,stroke:#6366F1,stroke-width:1.5px,color:#4338CA;
  classDef platform     fill:#FEE2E2,stroke:#EF4444,stroke-width:1.5px,color:#B91C1C;
  classDef parking      fill:#F5F3FF,stroke:#8B5CF6,stroke-width:1.5px,color:#6D28D9;
  classDef system       fill:#F1F5F9,stroke:#64748B,stroke-width:1.5px,color:#334155;
  classDef ui           fill:#FFF7ED,stroke:#F97316,stroke-width:1.5px,color:#C2410C;
  classDef topic        fill:#F8FAFC,stroke:#94A3B8,stroke-width:1px,color:#475569,font-style:italic;
  classDef config       fill:#FFFBEB,stroke:#D97706,stroke-width:1.5px,color:#92400E;
  classDef hardware     fill:#FAFAFA,stroke:#6B7280,stroke-width:1.5px,color:#374151;
  classDef highlight    fill:#FEF9C3,stroke:#CA8A04,stroke-width:2.5px,color:#713F12;

  NODE[🧩 lanelet_boundary_cost_grid]:::mapping

  subgraph MODES["📐 Four Cost Modes"]
    CL["🎯 centerline\ncost: 0→35→99\nhalf-width 0.75 m\n✅ ACTIVE"]:::highlight
    BD["🚧 bounds\nlethal strips at\nlane edges\n⛔ disabled"]:::mapping
    LL["🗺️ lanelet\nuniform fill\ninside lanes"]:::mapping
    PT["🛤️ path\nroute-strip bias\nalong global path"]:::mapping
  end

  NODE ==>|secondary output| CL
  NODE -.->|primary output\nprimary_enable: false| BD
  NODE -.->|alt mode| LL
  NODE -.->|alt mode| PT

  CL ==>|📡 /map/cost_grid/lanelet| OUT1((📡 active output)):::topic
  BD -.->|📡 /map/cost_grid/planning_base| OUT2((📡 disabled)):::topic

  linkStyle 0,4 stroke:#F59E0B,stroke-width:2.5px;
```

> **Diagram legend** 🧩 ROS node · 📡 Topic · ==> active path · -.-> disabled/optional

*Figure 3 — The `lanelet_boundary_cost_grid` node supports four cost modes; only `centerline` (secondary profile) is currently active.*

---

## 6. 🔌 Interface Contract

### Inputs

| Topic | Type | Required | Producer | Rate | Meaning |
|---|---|---|---|---|---|
| `/planning/lanelet_pose` | `geometry_msgs/PoseStamped` | No | camrod_planning | ~10 Hz | Robot pose used to center the rolling cost-grid window |
| `/planning/global_path` | `nav_msgs/Path` | No | camrod_planning | On replan | Global path used for path-mode cost layer |
| `/sensing/cost_grid/lidar` | `nav_msgs/OccupancyGrid` | No | camrod_sensing | 10 Hz | LiDAR obstacle grid forwarded to multi-marker visualizer |
| `/sensing/cost_grid/radar` | `nav_msgs/OccupancyGrid` | No | camrod_sensing | 10 Hz | Radar obstacle grid forwarded to multi-marker visualizer |
| `/planning/cost_grid/global_path_markers` | `visualization_msgs/MarkerArray` | No | camrod_planning | ~5 Hz | Global path marker stream aggregated for RViz |
| `/planning/cost_grid/local_path_markers` | `visualization_msgs/MarkerArray` | No | camrod_planning | ~5 Hz | Local path marker stream aggregated for RViz |

### Outputs

| Topic | Type | Consumer | Rate | Meaning |
|---|---|---|---|---|
| `/map/markers` | `visualization_msgs/MarkerArray` | camrod_platform, RViz | Once (transient_local) | Lanelet geometry: centerlines, boundaries, lane-direction arrows |
| TF `world→map` | `tf2_msgs/TFMessage` | All packages | Static | Fixed transform anchoring the metric map frame to WGS84 world |
| `/map/cost_grid/lanelet` | `nav_msgs/OccupancyGrid` | camrod_sensing, camrod_planning | Static (transient_local) | Centerline-gradient cost grid: 0=centerline, 35=in-lane, 99=off-lane |
| `/map/cost_grid/planning_base` | `nav_msgs/OccupancyGrid` | camrod_planning | Static (transient_local) | Boundary-strips-only cost grid. **Currently disabled** (`primary_enable: false`) |
| `/map/cost_grid/lanelet_markers` | `visualization_msgs/MarkerArray` | RViz | ~5 Hz | Colored cubes for lanelet cost visualization |
| `/map/cost_grid/lidar_markers` | `visualization_msgs/MarkerArray` | RViz | ~8 Hz | Colored cubes for LiDAR cost visualization |
| `/map/cost_grid/radar_markers` | `visualization_msgs/MarkerArray` | RViz | ~8 Hz | Colored cubes for radar cost visualization |
| `/map/cost_grid/inflation_markers` | `visualization_msgs/MarkerArray` | RViz | ~20 Hz | Merged contributor markers (requires `enable_inflation_markers:=true`) |
| `/map/cost_grid/lanelet_field_markers` | `visualization_msgs/MarkerArray` | RViz | Once | Line markers showing weighted lanelet cost field (requires `enable_cost_field:=true`) |

### Generated files

| File | Consumer | Content |
|---|---|---|
| `config/drop_zones.yaml` | camrod_localization (drop-zone matcher), camrod_planning (state machine) | id, x/y/z (map frame), yaw_deg, corner polygon |
| `config/camping_sites.yaml` | camrod_planning (state machine goal targets) | id, x/y/z (map frame), yaw_deg |

---

## 7. ⚙️ Key Behaviors

### Map loading

| Field | Detail |
|---|---|
| Trigger | Node startup |
| Internal logic | `lanelet_map_provider` opens the `.osm` file and projects all Lanelet2 primitives into the `map` frame using a `local_cartesian` projector anchored at `offset_lat/lon/alt`. Lane centerlines, boundaries, and regulatory elements are converted to MarkerArray for RViz. |
| Output effect | `/map/markers` published once with `transient_local` QoS; TF `world→map` published as a static transform. |
| Operator-visible symptom | No markers in RViz within 5 s of launch usually means the `.osm` file path is wrong or the projector origin is far from the robot start position. |
| Related params | `map_path`, `offset_lat`, `offset_lon`, `offset_alt`, `world_frame_id`, `map_frame_id` |
| Related topics | `/map/markers`, TF `world→map` |

### Cost grid generation

| Field | Detail |
|---|---|
| Trigger | Node startup (static grid); the node does **not** rebuild on each pose update (`rebuild_on_pose: false`, `rebuild_on_path: false`). |
| Internal logic | `lanelet_boundary_cost_grid` runs **two output profiles** from a single node instance. The **primary profile** (`cost_mode: bounds`, `/map/cost_grid/planning_base`) is currently **disabled** (`primary_enable: false`). The **secondary profile** (`cost_mode: centerline`, `/map/cost_grid/lanelet`) is the only active output: center cells = 0, in-lane off-center cells = 35, off-lane cells = 99. The window is 600×600 cells at 0.20 m resolution (120 m square). Outside-lanelet cells receive 99; unreachable cells receive −1. |
| Output effect | Both grids are published with `transient_local` QoS so late-joining Nav2 costmap layers receive them immediately on subscribe. |
| Operator-visible symptom | If the robot pose is far from any lanelet at startup, the centerline profile will show only the off-lane fill value (99). The grid becomes meaningful once the robot is driven into a lanelet corridor or a route is planned. |
| Related params | `cost_mode`, `resolution`, `width`, `height`, `centerline_half_width` (secondary: 0.75 m), `outside_value`, `secondary.outside_value`, `secondary.centerline_lanelet_fill_value` |
| Related topics | `/map/cost_grid/lanelet` (active); `/map/cost_grid/planning_base` (disabled) |

### TF publishing

| Field | Detail |
|---|---|
| Trigger | Node startup |
| Internal logic | `lanelet_map_provider` computes the `world→map` static transform by inverting the `local_cartesian` projection of the WGS84 origin. This is a fixed transform; it does **not** update at runtime. |
| Output effect | All packages can look up `world→map` immediately after map launch. |
| Operator-visible symptom | If `world_frame_id` or `map_frame_id` in `map_info.yaml` differs from what other packages declare (e.g., in URDF or localization config), TF lookups will fail across the stack. |
| Related params | `world_frame_id` (default: `world`), `map_frame_id` (default: `map`), `offset_lat`, `offset_lon`, `offset_alt` |
| Related topics | TF `world→map` |

---

## 8. 🚀 Launch

```bash
# Full map stack (recommended)
ros2 launch camrod_map map.launch.py

# Sub-stacks
ros2 launch camrod_map lanelet2_map.launch.py        # map loader + TF only
ros2 launch camrod_map cost_grid.launch.py           # cost grid nodes only
ros2 launch camrod_map visualization.launch.py       # marker visualization only

# Export semantic areas (run after each map edit)
ros2 launch camrod_map area_export.launch.py \
  output_yaml_path:=/path/to/drop_zones.yaml \
  camping_sites_output_yaml_path:=/path/to/camping_sites.yaml
```

### Launch arguments for `map.launch.py`

| Argument | Default | Description |
|---|---|---|
| `map_path` | (from `map_info.yaml`) | Absolute path to Lanelet2 `.osm` file; empty = auto-discover |
| `origin_lat` | (from `map_info.yaml`) | WGS84 reference latitude override |
| `origin_lon` | (from `map_info.yaml`) | WGS84 reference longitude override |
| `origin_alt` | (from `map_info.yaml`) | WGS84 reference altitude override |
| `enable_cost_grids` | `true` | Enable `lanelet_boundary_cost_grid` node |
| `enable_map_cost_markers` | `true` | Enable `cost_grid_multi_marker` node |
| `enable_cost_field` | `false` | Enable `lanelet_cost_field_visualizer` (slow, debug only) |
| `enable_inflation_markers` | `false` | Enable `cost_grid_marker_aggregator` node |
| `enable_nav2_inflation_debug_marker` | `false` | Enable Nav2 global costmap debug marker |
| `module_namespace` | `map` | ROS 2 namespace for all map nodes |
| `system_namespace` | `system` | Namespace for system-level diagnostics |

---

## 9. 🔧 Config

### `config/map_info.yaml` — field reference

> 📌 This file is the **single source of truth** for map path and WGS84 origin. Edit it once per deployment; all launch files read it automatically.

<details><summary>Full field reference (click to expand)</summary>

| Field | Unit | Required | Default | Meaning | Used by |
|---|---|---|---|---|---|
| `map_path` | — | No | `""` | Absolute path to Lanelet2 `.osm` file. Empty = auto-discover via workspace search. | `lanelet_map_provider`, `lanelet_boundary_cost_grid` |
| `offset_lat` | degrees (WGS84) | Yes | `36.7292921` | Reference latitude for `local_cartesian` projector | All map nodes, camrod_localization, camrod_planning |
| `offset_lon` | degrees (WGS84) | Yes | `127.4429577` | Reference longitude for `local_cartesian` projector | All map nodes, camrod_localization, camrod_planning |
| `offset_alt` | m (WGS84 ellipsoidal) | Yes | `84.0` | Reference altitude for `local_cartesian` projector | All map nodes, camrod_localization, camrod_planning |
| `offset_utm_easting` | m | No | `360965.99` | Informational UTM easting of origin (not used by projector) | External tools |
| `offset_utm_northing` | m | No | `4065972.49` | Informational UTM northing of origin (not used by projector) | External tools |
| `offset_utm_alt` | m | No | `84.0` | Informational UTM altitude of origin | External tools |
| `yaw_offset_deg` | degrees | No | `0.0` | Optional rotation applied to the lat/lon → x/y projection | `lanelet_map_provider` |
| `rotate_latlon_xy_by_yaw_offset` | bool | No | `true` | Enable/disable the yaw rotation above | `lanelet_map_provider` |
| `world_frame_id` | — | Yes | `world` | TF frame name for the WGS84 world frame | All packages |
| `map_frame_id` | — | Yes | `map` | TF frame name for the projected metric map frame | All packages |
| `dir_body_scale` | m | No | `0.55` | Arrow body scale for lane-direction markers (RViz only) | `lanelet_map_provider` |
| `dir_head_scale` | m | No | `0.35` | Arrow head scale for lane-direction markers (RViz only) | `lanelet_map_provider` |
| `dir_width_scale` | m | No | `0.18` | Arrow width scale for lane-direction markers (RViz only) | `lanelet_map_provider` |
| `dir_stride` | — | No | `30` | Number of centerline points skipped between direction arrows | `lanelet_map_provider` |
| `visualization_republish_period_s` | s | No | `0.0` | Periodic republish interval for map markers (0 = once, transient_local only) | `lanelet_map_provider` |

</details>

### `config/lanelet_cost_grid.yaml` — key params

| Param | Value | Meaning |
|---|---|---|
| `cost_mode` | `bounds` | Primary profile: lethal strips at lane left/right boundaries |
| `resolution` | `0.20` m | Grid cell size |
| `width` / `height` | `600` cells | 120 m square window centred on pose |
| `centerline_half_width` | `2.2` m | (Primary profile) half-width of boundary strip |
| `outside_value` | `-1` | Cells outside lanelets are unknown |
| `secondary.cost_mode` | `centerline` | Secondary profile → `/map/cost_grid/lanelet` |
| `secondary.centerline_half_width` | `0.75` m | Tight centerline corridor |
| `secondary.outside_value` | `99` | Off-lane cells are near-lethal |
| `secondary.centerline_lanelet_fill_value` | `35` | In-lane but off-centerline cost |

### `config/map_visualization.yaml` — key params

| Param | Value | Meaning |
|---|---|---|
| `palettes` | `[pastel_blue_red, pastel_green_red, pastel_orange_red]` | Per-stream color palettes (lanelet, lidar, radar) |
| `alphas` | `[0.12, 0.40, 0.42]` | Per-stream marker transparency |
| `marker_scales` | `[0.08, 0.08, 0.10]` m | Per-stream cube size |
| `z_offsets` | `[0.02, 0.035, 0.04]` m | Vertical offsets to avoid z-fighting |
| `republish_periods_s` | `[0.20, 0.12, 0.12]` s | Per-stream periodic re-emit intervals |

### Other config files

| File | Purpose |
|---|---|
| `config/map_info.yaml` | Map path, WGS84 origin, frame IDs, visualization periods — single source of truth |
| `config/lanelet_cost_grid.yaml` | Grid geometry, cost mode, corridor widths, penalty weights, rebuild triggers |
| `config/map_visualization.yaml` | Multi-marker and debug visualization parameters |
| `config/drop_zones.yaml` | Generated drop-zone list (id, x/y/z, yaw_deg, corners) |
| `config/nav2_params_costlayer_example.yaml` | Reference Nav2 costmap layer config |

---

## 10. ✅ Validation

```bash
# Confirm TF world→map is published
ros2 run tf2_ros tf2_echo world map

# Confirm lanelet markers are available (transient_local, non-empty)
ros2 topic echo /map/markers --once | grep -c "markers"

# Confirm cost grid is published
ros2 topic echo /map/cost_grid/lanelet --once | head -20

# planning_base is currently disabled (primary_enable: false); no messages expected
# ros2 topic echo /map/cost_grid/planning_base --once

# After area_export launch: confirm drop_zones.yaml is non-empty
python3 -c "import yaml; d=yaml.safe_load(open('/home/hong/camrod_ws/src/camrod_map/config/drop_zones.yaml')); print(len(d['drop_zones']), 'drop zones')"
```

> 💡 **Expected healthy state:** TF echo returns a valid transform, `/map/markers` has > 0 markers, `/map/cost_grid/lanelet` shows a 600×600 OccupancyGrid.

---

## 11. 🔍 Troubleshooting

### Map rotated or offset in RViz

The robot position appears to float off the displayed lane geometry.

> ⚠️ Even a 1 arcsecond error in `offset_lat/lon` causes a ~30 m position offset.

1. Confirm `offset_lat/lon/alt` in `map_info.yaml` exactly matches the reference origin used when the `.osm` file was generated.
2. Check `yaw_offset_deg`. If non-zero, the entire map is rotated. Set to `0.0` unless the `.osm` was generated with a deliberate heading correction.
3. Verify all consumers (camrod_localization, camrod_planning) are reading the same `map_info.yaml` — mismatched origins produce consistent-looking maps that are shifted relative to sensor data.

### Cost grid is empty

`/map/cost_grid/lanelet` is received but every cell is −1 or 99.

1. Check that the `.osm` file path in `map_info.yaml` points to an existing file. Verify:
   ```bash
   ls -lh $(python3 -c "import yaml; p=yaml.safe_load(open('/home/hong/camrod_ws/src/camrod_map/config/map_info.yaml')); print(p['/**']['ros__parameters']['map_path'])")
   ```
2. If the file path is empty, the launch auto-discover search walks up from `~/camrod_ws/src`; place `lanelet2_maps.osm` there or set `map_path` explicitly.
3. If the grid is all 99 (off-lane fill), the `offset_lat/lon/alt` places the map far from the lanelet geometry. Re-check the origin against the OSM file.

### Markers don't appear in RViz

1. Confirm `lanelet_map_provider` is running: `ros2 node list | grep map`.
2. `/map/markers` uses `transient_local` QoS. Subscribe with a matching QoS in RViz (set "Durability Policy" to "Transient Local").
3. If RViz shows "transform not available for Header frame `map`", the TF `world→map` has not been published yet. Check that `lanelet_map_provider` started without error.

### `drop_zones.yaml` has no entries

The `area_exporter` node runs once at launch and exits. If the output file is empty:

1. Confirm the `.osm` file contains areas tagged as drop zones (the exporter searches for OSM area tags matching `dropzone_types`).
2. Verify `output_yaml_path` points to a writable location. The launch file defaults to `camrod_map/config/drop_zones.yaml` in the workspace source tree.
3. Re-run: `ros2 launch camrod_map area_export.launch.py` and observe stdout for "Exported N drop zones".

---

## 12. 📚 Related Docs

- [../README.md](../README.md) — monorepo overview and inter-package data flow
- [../camrod_localization/README.md](../camrod_localization/README.md) — consumes `world→map` TF, `/sensing/gnss/pose_with_covariance`, produces `/localization/pose`
- [../camrod_planning/README.md](../camrod_planning/README.md) — consumes `/map/cost_grid/lanelet`, produces `/planning/global_path`
- [../camrod_sensing/README.md](../camrod_sensing/README.md) — consumes `/map/cost_grid/lanelet` for inflation grid, produces `/sensing/cost_grid/lidar` and `/sensing/cost_grid/radar`
- [../PARAMETER_NAMING_STANDARD.md](../PARAMETER_NAMING_STANDARD.md) — canonical parameter naming conventions used across the stack
