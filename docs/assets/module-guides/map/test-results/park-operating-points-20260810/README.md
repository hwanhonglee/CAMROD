# Historical Park Map-v22 Operating Points (2026-08-18)

<!-- HH_260810 - Bind semantic coordinates to the exact current user-authored map. -->
<!-- HH_260818 - Refresh map identity to v22 after confirming every exported
operating coordinate remains unchanged. -->

![Current Park semantic coordinates](park-operating-points.png)

This archived map-v22 record is source-derived configuration evidence, not a
physical-road PASS claim and not the current map-v23 geometry. The current map
has a relocated shared parking/docking zone and edited campsite boundaries; see the
[active map contract](../../../../../../camrod_map/README.md#current-park-operating-coordinates).
The renderer originally loaded `lanelet2_maps.osm` through Lanelet2's
`LocalCartesianProjector` with the shared Park origin, then combines the
official `area_exporter` output with the operational service policy.

| Input | Contract |
|---|---|
| OSM | map v22, SHA `8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59` |
| Park origin | `36.8435737`, `128.0925646`, altitude `0.0 m` |
| Service areas | B1-B13 plus one `drop_zone` |
| Other semantic geometry | Three `parking_lot` polygons (ways 1146, 1378, 1615) |
| Policy preserved outside OSM | B1-B10 `turnaround`; B11-B13 `roadside_stop` |

At capture time the runtime YAML mirrors were byte-identical across
map/localization/bringup for the drop-zone and planning/bringup for campsites.
The archived coordinates,
corners, parking-lot metrics, map identity, and validation limits are in
`park-operating-points.json`.

Do not run the historical single-zone renderer with current defaults: that
would overwrite this evidence with different inputs. The regression test
reproduces it only in a temporary directory using the hash-bound map-v22
snapshot and the archived report's semantic records:

```bash
pytest -q camrod_bringup/test/test_park_operating_points_assets.py -k historical
```

Field validation is still required after any road-width or semantic-area edit.
