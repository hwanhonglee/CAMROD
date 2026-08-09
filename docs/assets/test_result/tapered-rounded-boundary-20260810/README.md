# Tapered Rounded Boundary Visual Record

<!-- HH_260810 - Keep the active non-rectangular boundary visible and
regenerable without relabeling a source-derived transform as runtime evidence. -->

`SOURCE-DERIVED`: these visuals are generated from the checked-in sensor-kit
geometry and verified against the Nav2 local/global footprint strings. They are
not a ROS simulation, physical drive, collision test, or recovery-policy PASS.

## Current Shape

![Tapered rounded boundary geometry](tapered-rounded-boundary-geometry.png)

| Envelope | Bounding size | Shape | Runtime meaning |
|---|---:|---|---|
| Physical body | `1.39160 x 1.07000 m` | `0.12 m` front taper/depth, `R0.05 m`, 30 points | Ordinary cost-100 overlap stops motion |
| Planning boundary | `1.59160 x 1.27000 m` | Exact `0.10 m` offset, `R0.15 m`, 30 points | Endpoint/swept clearance limit for bounded recovery |

Both contours are local to `robot_center_link`. The front/rear and left/right
extrema remain `0.70837/0.68323 m` and `0.53505/0.53495 m`; rounding removes
only the former rectangular corner area.

## Motion

![Tapered rounded boundary motion](tapered-rounded-boundary-motion.gif)

| Phase | Center motion | Yaw motion |
|---|---|---|
| Forward | translates along heading | fixed |
| Curved drive | translates along an arc | changes continuously |
| Crab right | translates laterally | fixed |
| Zero turn | fixed | rotates about `robot_center_link` |

The physical and planning polygons are rigid transforms of the same robot pose,
so neither contour moves independently. Candidate acceptance against a lanelet
or obstacle grid remains the responsibility of `camrod_control`; the animation
only explains the geometry transform.

The separate [measured map-v17 ROS road record](../tapered-rounded-boundary-road-sim-20260810/README.md)
shows this exact contour reaching a planning-margin hold, performing bounded
`REVERSE_YAW_RIGHT`, releasing the hold, and completing the route.

## Reproduce

From the repository root:

```bash
python3 camrod_bringup/scripts/visualization/render_tapered_rounded_boundary.py
pytest -q camrod_bringup/test/test_tapered_rounded_boundary_assets.py
```

| File | Purpose |
|---|---|
| `tapered-rounded-boundary-geometry.png` | Exact current top-view shape, dimensions, and safety meaning |
| `tapered-rounded-boundary-motion.gif` | Forward, curve, crab, and zero-turn rigid transforms |
| `result.json` | Source hashes, all 30-point polygons, dimensions, and motion metadata |
| `SHA256SUMS` | Integrity hashes for the generated PNG, GIF, and JSON |
