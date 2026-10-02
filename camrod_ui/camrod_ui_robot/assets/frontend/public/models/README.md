# Ranger driving-display asset

## Realtime model

`ranger-navigation.glb` is the lightweight, animated-scene derivative of the
approved Ranger FBX, not a flat image. Source geometry is separated using the
existing hash-pinned rigid-weight repair before planar simplification and
bounded triangle decimation. The original FBX is never overwritten.

Coordinates are meters, **+X forward, +Y up, +Z right**. ROS coordinates map as
`(x, y, z) -> (x, z, -y)`. The source base origin is preserved; ground is Y=0.
Body dimensions are approximately 1.553 m long × 1.120 m wide × 1.324 m high.
These are the source bounds. The simplified exported body bounds are 1.564 m
long × 1.121 m wide × 1.326 m high; exact minima/maxima and the current GLB
SHA-256 are recorded in `ranger-navigation.provenance.json`.

Named animation hierarchy for each of `fl`, `fr`, `rl`, `rr`:

```text
ranger_navigation
  body
  wheel_fl_steer
    wheel_fl_spin
      wheel_fl
```

Steer around local Y; positive means left. Forward rolling decrements the
spin node's local Z angle by travelled distance / 0.153 m. These are display
pivots from the source wheel-bone centers, not control commands or proof of
real steering feedback. Exact pivots are recorded in the adjacent GLB
provenance JSON. Materials are `body_white`, `tire_rubber`, `chassis_metal`.

`wrap_left` / `wrap_right` / `wrap_front` / `wrap_rear` are UV-mapped runtime
artwork targets using the corresponding `sidewrap_*` materials. They can be
hidden until the requested Woraksan texture is loaded. Their subdivided
surfaces follow rays onto the existing body LOD with a nominal 2 mm outward
offset, rather than sitting on outer bounding-box planes. Seven rays per
triangle check its centroid, edge midpoints, and interior; triangles with
unsupported gaps, penetrations, or more than 8 mm clearance are omitted.
Small white interruptions around real ribs, cutouts, and sharp curvature are
intentional; this is display geometry, not a measured wrap installation.

Front is +X, rear is -X. Side artwork covers 1.18 m × 0.34 m at Y=0.64–0.98 m;
rear covers 0.96 m × 0.34 m at the same height. The curved front's central
facade uses 0.62 m × 0.27 m at Y=0.67–0.94 m. UVs are upright and
left-to-right when viewed from outside each facade. White upper trim, outside
borders, and lower chassis remain unwrapped.

The three runtime artwork files are:

| Surface | Artwork | Reference interpretation |
|---|---|---|
| Both sides | `woraksan-side-wrap.png` | Illustrated lower sheet in the user's portrait photograph |
| Front | `woraksan-front-wrap.png` | Coordinated white/forest front variation |
| Rear | `woraksan-rear-wrap.png` | User-selected sage-green upper sheet, with white delivery-robot text |

These are AI-assisted, reference-based reconstructions/variations—not original
printing masters, exact photo recovery, or certified official insignia. The
rear uses the corrected green-panel reference, not the discarded forest rear
proposal. See [side artwork provenance](woraksan-side-wrap.README.md) and
[front/rear artwork provenance](woraksan-wrap-variants.README.md) for source
artifacts and prompts. Replace these files with approved flat print artwork
when exact branding is required. Artwork does not turn the display-only cargo
into a measured load or restore any shelf.

`accessory_cargo_preview` contains 12 closely packed cardboard boxes in four
rows of three, with varied heights and kraft tones. They rest on the existing
bed floor (approximately Y=0.616 m), fill about 0.99 × 0.81 m of the bed,
and stay below Y=0.948 m and the upper rails. Its extras specify
`provisional: true` and `default_visible: true`. Label these as display-only
cargo: they are not measured real cargo and do not establish load capacity,
physical attachment, or collision behavior. No upper shelf remains in the GLB.

Reproduce the GLB from the workspace source root:

```sh
blender --background --factory-startup --python tools/export_ranger_navigation_asset.py
```

The exporter checks the approved FBX hash, validates authored rigid group
counts, and rejects output above 150,000 triangles / 6 MiB. The source-pinned
weight-mask script and JSON remain dependencies in the existing model package.

## Static studio reference

`ranger-driving-rear.png` is a transparent 1000×1000 studio render of the
approved `ranger_carla_rigged_template.fbx` checkpoint documented by the current
`ranger-carla-4ws-pipeline/docs/MODEL_BUILD_CURRENT.md`.

The original 635,576-vertex / 1,167,363-polygon geometry and neutral wheel pose
are preserved. The source has no material slots; olive body, warm alloy lower
chassis, and dark tires are display-only fallback materials, not source
textures. The camera is behind/right and approximately 32° elevated. Wheels
occluded by the real body are not invented or moved to make them visible.

This asset is not a live camera image, simulator acceptance capture, or evidence
of road testing. See the adjacent provenance JSON for source/output SHA-256.

Reproduce from the workspace source root:

```sh
blender --background --factory-startup --python tools/render_ranger_driving_asset.py
```

Requires Blender 3.x with NumPy and ImageMagick. The script verifies the input
hash, never exports/overwrites the FBX, and rejects blank/opaque renders before
replacing an existing image. Optional denoising and compositing are disabled
because this host returned invalid pixels with them enabled.
