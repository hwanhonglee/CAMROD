# Map v15 Boundary-Recovery Evidence

<!-- HH_260805 - Record the current staged recovery run separately from the
historical map-v14 translation-only evidence. -->

These JSON files were captured from full `sim:=true` bringup against the
user-authored `lanelet2_maps.osm` map v15, SHA-256
`e0b50f09c61fbd5429e528c2b3d8d2799a0dab9f83bb79b06dd0da0403efe36d`.

| Scenario | Selected motion | Observed result |
|---|---|---|
| `route_retry` | `REVERSE_YAW_RIGHT` | `0.0582 m` recovery, `0.05 rad/s` maximum yaw rate, recontact after `0.335 s`, retry latched |
| `static_reverse_retry` | `REVERSE_YAW_RIGHT` | `0.0405 m` recovery, `0.05 rad/s` maximum yaw rate, recontact after `0.400 s`, retry latched |
| `one_sided_crab` | `CRAB_LEFT` | `0.3378 m` recovery, `0.05 m/s` lateral maximum, hold released without recontact |

All three final commands are zero. The route scenarios demonstrate bounded
reverse-yaw and fail-closed rapid-recontact containment; they do not prove
route or mission completion. Physical wheel response remains field-pending.

Re-run one scenario while full simulation is active:

```bash
ros2 run camrod_bringup automatic_route_recovery_probe.py \
  --map lanelet2_maps.osm \
  --scenario route_retry \
  --output /tmp/map-v15-route-retry.json
```

Render the committed runs:

```bash
python3 camrod_bringup/scripts/render_automatic_recovery_results.py \
  --map lanelet2_maps.osm \
  --route docs/evidence/v2.1.4/map-v15-boundary-recovery/route-retry.json \
  --reverse docs/evidence/v2.1.4/map-v15-boundary-recovery/static-reverse-retry.json \
  --crab docs/evidence/v2.1.4/map-v15-boundary-recovery/one-sided-crab.json \
  --artifact-prefix map-v15-boundary-recovery \
  --output-dir docs/assets/module-guides/control
```
