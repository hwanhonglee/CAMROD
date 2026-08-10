# Current Park Operating Points (2026-08-10)

<!-- HH_260810 - Bind semantic coordinates to the exact current user-authored map. -->

![Current Park semantic coordinates](park-operating-points.png)

This is source-derived configuration evidence, not a physical-road PASS claim.
The renderer loads the current `lanelet2_maps.osm` through Lanelet2's
`LocalCartesianProjector` with the shared Park origin, then combines the
official `area_exporter` output with the operational service policy.

| Input | Contract |
|---|---|
| OSM | map v15, SHA `689c49854f3e5d93b59ccde13799f9748a669956cf9bbfa7c121f369ecdb1b39` |
| Park origin | `36.8435737`, `128.0925646`, altitude `0.0 m` |
| Service areas | B1-B13 plus one `drop_zone` |
| Other semantic geometry | Three `parking_lot` polygons (ways 1146, 1378, 1615) |
| Policy preserved outside OSM | B1-B10 `turnaround`; B11-B13 `roadside_stop` |

The runtime YAML mirrors are byte-identical across map/localization/bringup for
the drop-zone and planning/bringup for campsites. The exact coordinates,
corners, parking-lot metrics, map identity, and validation limits are in
`park-operating-points.json`.

Regenerate from the repository root:

```bash
python3 camrod_bringup/scripts/visualization/render_park_operating_points.py
```

Field validation is still required after any road-width or semantic-area edit.
