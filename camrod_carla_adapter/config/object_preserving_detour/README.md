# Offline CARLA shoulder-detour candidate

<!-- HH_261013 - Distinguish the unchanged offline artifact from its later explicitly selected live trials. -->
**Later use:** the October 12 selected site-completion trials explicitly selected
this map through `CAMROD_LANELET_MAP`; the normal default remains unchanged.
[Current release scope and prerequisites](../../../docs/virtual_carla_release_20261013.md)
and [published actual driving evidence](../../../docs/evidence/virtual_carla/releases/20261013/README.md)
document that later use. The original October 7 generation note below describes
the artifact before activation, not the current host selection.

HH_261007 - This artifact is a separate experimental map copy. It is **not
activated** and is not evidence of a successful autonomous drive. No original
map, CARLA object, develop default, safety threshold or runtime setting was
changed by its generation.

The source remains `lanelet2_maps.osm`, SHA-256
`cf5490cbf2067b4ce7dc703f80e75095366cacd364097cbad431131e7b435c8d`.
The finite geometry/support audit is
`docs/evidence/virtual_carla/b1_b11_access_live_20261007/shoulder_candidate_audit.json`.
Its limitations and retained ray observations are documented beside that file.

Only the geometry of lanelets **2757 → 2285 → 2451** is replaced. Relation IDs,
roles, tags, routing topology, original external seam nodes and all other ways
remain unchanged. Both explicit centerlines and paired boundaries follow the
audited smoothed path; changing only the boundaries would not move this planner.
There is no map-wide road widening.

The checked planning-footprint sweep is buffered by 0.075 m. Generated sides
remain 0.002 m inside that finite envelope, with at most 0.00025 m chord
simplification. Every newly permitted road region is checked against the
audited corridor; full planning-footprint containment is checked across the
connected lanelets. The existing 0.397 mm discrepancy between external center
node 6213 and its authored boundary end cap is retained exactly, not repaired
by altering adjacent roads.

Read-only parameter inspection on 2026-10-07 confirmed the active
`/map/lanelet_safety_cost_grid` resolution is **0.05 m**, with inside-hit counts
7 / 9. The audit conservatively assumed 0.10 m. Neither was changed. Clearance
does not prove dynamic tracking, live sensor acceptance, or a complete mission.

Reproduce into **new, absent** output/report paths from the repository root:

```bash
/usr/bin/python3 camrod_carla_adapter/tools/generate_object_preserving_detour.py \
  --audit docs/evidence/virtual_carla/b1_b11_access_live_20261007/shoulder_candidate_audit.json \
  --output /absolute/new/lanelet2_maps_carla_shoulder_2285.osm \
  --report /absolute/new/generation.json --grid-resolution-m 0.05
```

The generated file in this directory can be explicitly selected for a separately
authorized CARLA launch using its existing argument:

```text
camrod_map_path:=/home/hong/camrod_ws/src/camrod_carla_adapter/config/object_preserving_detour/lanelet2_maps_carla_shoulder_2285.osm
```

Equivalently the caller can supply `CAMROD_LANELET_MAP` with that absolute path.
This note does not change an environment file or select the map automatically.
Do not run the older centerline-only `generate_woraksan_carla_lanelet_map.py`
for this candidate: its frozen path and unchanged bounds are a different scope.

Offline regression command:

```bash
/usr/bin/python3 -m pytest -q camrod_carla_adapter/test/test_object_preserving_detour.py
```

The tests preserve the source byte-for-byte, check all directed routing
successor sets, shared node IDs, unrelated map elements, centerline fidelity,
planning-footprint containment and the bounded new road permission.
