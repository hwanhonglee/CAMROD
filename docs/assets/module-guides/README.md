# Module Visual Asset Layout

<!-- HH_260810 - Keep every visual beside the package that owns its claim. -->

Each package directory uses the same three-way provenance split:

| Directory | Meaning | Update rule |
|---|---|---|
| `guide/` | Source/config-derived architecture and geometry | Regenerate from the current source tree |
| `evidence/` | Captured RViz, terminal, UI, JSON, and concise raw logs | Preserve the capture and its historical geometry |
| `test-results/` | Reproducible experiment PNG/GIF/JSON and verdicts | Bind each result to its map/config/input identity |

Historical screenshots under `evidence/` may show the rectangular boundary
that was active when they were captured. Derived boundary visuals under
`guide/` and `test-results/` use the active tapered, rounded contour. A derived
historical replay must say that it combines recorded motion or metrics with
the current contour; it must not be presented as a new runtime measurement.

## Ownership

| Package directory | Primary visual scope |
|---|---|
| `bringup/` | Full-service lifecycle and scenario endurance |
| `common/` | Shared messages and interface contracts |
| `control/` | Command gates, boundary stops, and bounded recovery |
| `localization/` | Pose inputs, timing, and GNSS lever arms |
| `map/` | Lanelet map products and semantic operating points |
| `perception/` | Detection, clustering, and parking pipelines |
| `planning/` | Nav2 servers, route feasibility, and RPP tracking |
| `platform/` | Ranger command/status and published geometry |
| `runtime/` | Component/container topology and lifecycle |
| `sensing/` | Sensor processing and cost-grid fusion |
| `sensor-kit/` | Robot reference frame, mounts, and boundary geometry |
| `system/` | Diagnostics and operator-facing health state |
| `ui/` | Robot/guest UI mission and state surfaces |
| `voice/` | Voice events and priority behavior |

Experiment directories should include a concise `README.md`, machine-readable
JSON when measurements exist, and hashes for external or generated inputs.
Large rosbags remain external; record their path and SHA-256 in the result.

Summary animations must identify themselves when raw frame/event samples were
not recorded. They may animate measured aggregates or unit-tested decisions,
but must not be labelled as runtime or field captures.
