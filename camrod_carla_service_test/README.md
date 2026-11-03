# CARLA service-usability compatibility

HH_261103 - Keep the simulator's proven docking retry implementation outside native robot control.

Composition: `service_test.launch.py`; use the checked portable overlay entrypoint
below, not an arbitrary ambient ROS prefix graph.
This package depends on the installed `camrod_carla_adapter` for the existing
actual-sensor/plant profile and contact emulator. It does not publish synthetic
tag poses or claim physical charger validation.

The private source copy is the existing virtual/CARLA AprilTag controller and
closed helper dependency set, recorded in `PROVENANCE.json`. Existing fail-closed
retry/clearance tests are retained. Native dispatcher, reverse parking, cargo
departure and ordinary controller defaults remain in `camrod_control`.

The explicit test overlay enables the proven bounded 0.8 m forward retry/reacquisition
(maximum two attempts), retaining the 0.40 m tag stop, 0.03 m final lateral tolerance,
freshness, odometry step, signed progress, drift, path and timeout bounds.
`config/charging_station.yaml` belongs only to the contact emulator. Native park/cargo
station geometry is not changed by this wrapper.

This is an opt-in simulation fixture, not a unit1/unit2 deployment or field PASS.

## Portable two-source overlay

HH_261103 - Preserve the tested mixed package graph without merging virtual controllers into the native robot branch.

Prerequisites: an already built compatible virtual workspace descending from
`4c607cc6d1ac7eaf84e7d93f0420d0491fa572fe`, ROS/dependencies, Node/npm,
the Ranger/CARLA pipeline, UE/CARLA binaries, Ranger assets and all required map
assets. This package neither builds the engine nor generates fake sensors,
charger feedback, gate PASS reports or scene approvals.

The native source is `unit2-v1.0.1`, commit
`9c261f9df15abffb86fbff12d334efbb4c2663f3`, pinned in
`config/overlay_sources.env`. Setup refuses a placeholder, tag/SHA mismatch, an existing target, or an uncommitted
test package. It exports tracked Git objects only; no dirty working tree is copied.

1. Copy `config/root.env.example` to a private caller-owned `root.env` and set
   absolute paths. `CAMROD_SERVICE_WS_ROOT` must be a new path outside the source
   and virtual underlay trees. The native repository must already contain the
   tag; no fetch/checkout/reset is performed by the helper.
2. Run `bash tools/setup_overlay.sh /absolute/root.env` from this committed
   package. This archives the pinned native tree, copies only the committed
   test package beside native control, and dry-checks/applies
   `tools/native_parking_compat.patch` and `tools/native_underlay_compat.patch`
   **only in that isolated tree**. Both are checked before either is applied.
   The parking patch
   adds optional parking package/executable selection and an empty default YAML;
   it does not change native acceptance thresholds or the published native tag.
   The underlay patch reuses virtual's existing explicit-root allowlist in the
   native build script; an empty allowlist retains native filtering. This is
   necessary because the unpatched native wrapper drops external underlays.
   No second build wrapper replaces `colcon_build.sh`.
3. Run `bash tools/overlay.sh /absolute/root.env build`. It uses the existing
   `colcon_build.sh`, including its canonical UI frontend preparation/install,
   for exactly control, sensing, UI, nested ground segmentation and this package
   (two workers). Other dependencies come from the explicit prebuilt virtual
   underlay. Tests are separate, in the isolated workspace.
4. Produce fresh, legitimate **host/source/binary-bound** build/test/gate receipts
   with the existing official pipeline tools after the build. Do not reuse the
   historical host's gate JSON or edit hashes to pass validation.
5. Provision the tag-only map and a locally valid APPLIED receipt separately.
   Set `CAMROD_SERVICE_SCENE_RECEIPT`; `tools/verify_scene.py` rehashes protected
   assets and requires exact46 translation-only/unchanged-station invariants.
   Historical staging source is archived under `historical/` and deliberately
   disabled as an entrypoint: it records host-specific inputs, not a portable
   asset installer. Missing engine/assets/source-scene lineage is a prerequisite
   failure, not permission to fabricate a receipt or widen a map allowlist.
6. `bash tools/overlay.sh /absolute/root.env env` checks package resolution.
   `... run [launch arguments]` deeply validates supplied gates/scene, selects
   only five new prefixes and starts the explicit test composition. It does
   **not** start/stop the server, bridge, pacer or actors; arrange their single
   ownership separately with the existing virtual tools and matching map URI.

Virtual `camrod_bringup`/`camrod_carla_adapter` retain plant/sensor ownership;
native bringup is never overlaid. Runtime state/logs stay below the supplied
test workspace. Active tools contain no machine-specific home default.
`PROVENANCE.json` and historical notes retain original paths as audit history,
not runtime dependencies. This publication layout must still be built and
validated on the target host; it is not one-command deployment.
