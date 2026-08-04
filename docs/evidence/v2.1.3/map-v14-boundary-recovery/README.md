# Map v14 Boundary-Recovery Rerun

<!-- HH_260804 - Keep the regenerated map-v14 evidence separate from the
earlier automatic-owner baseline and the B6 browser/runtime capture. -->

Full bringup command:

```bash
ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false enable_operator_ui_window:=false
```

All three records identify `lanelet2_maps.osm` as `map_version=14` with
SHA-256 `2f69deed24ae47e6762a7653e29e5574438a1ec4b9144b8a3b0a01165f404dbe`.

| Record | Production response | Measured result | Verdict |
|---|---|---|---|
| [`route-retry.json`](route-retry.json) | reverse, release, normal RPP retry | reverse `0.0703 m`; retry `0.0661 m`; yaw `-0.1235 deg`; recontact `0.366 s` | rapid retry latched; final Twist zero |
| [`static-reverse-retry.json`](static-reverse-retry.json) | reverse from boundary contact | reverse `0.0721 m`; recontact `0.362 s` | rapid retry latched; final Twist zero |
| [`one-sided-crab.json`](one-sided-crab.json) | `CRAB_LEFT` on lanelet `4677` | `0.3321 m`; max lateral output `0.05 m/s` | first hold released; no retry requested |

This proves production-node simulation behavior, not physical wheel clearance.
The route cases deliberately end fail-closed because the mapped corridor still
cannot contain the deployed planning footprint through the complete retry.
