# Virtual/CARLA release allowlist — 2026-10-02

This is a staging proposal, not a commit or a successful driving-test claim.
No index, branch, tag, runtime process or remote was changed by this audit.

## Exact source list

Use [the 110-path source manifest](release_virtual_carla_source_allowlist_v230_20261002.txt)
as the candidate path list for the **main virtual/carla worktree only**. It was
derived from the current develop promotion worktree's pending core files,
intersected with changed/new main files, then augmented with the bounded CARLA
integration changes. Review its staged diff before committing; it is not a
license to overwrite one branch with the other or skip functional validation.

| Commit group | Allowed source scope |
| --- | --- |
| Shared UI features | Listed `camrod_ui` frontend JS/CSS/tests, package/lock, public model assets/provenance, backend modules, launch/setup, build publication scripts, tests and `docs/mission_recording.md` |
| Shared recorder launch | `camrod_bringup/launch/_bringup_impl.py` |
| Shared observed LiDAR extents | `camrod_perception/CMakeLists.txt`, `src/obstacle_fusion_node.cpp`, new extent header and test |
| Shared voice direction | Four listed voice source/config/test files and `camrod_voice/README.md` |
| CARLA-only integration | The 19 listed `camrod_carla_adapter` files, four listed `scripts/virtual_carla` files, UI `manual_drive_policy.py` and its test |

The main UI backend/launch/App still contain the pre-existing CARLA manual
integration. Their **main versions belong on virtual/carla**. Use the already
reconciled develop worktree versions for develop; never copy these whole files
back there. The simulator 2 m/s opt-in, detector label/profile, controller period
and step-pacer startup readiness belong only on virtual/carla.

`camrod_bringup/test/test_mission_recorder_launch_contract.py` currently exists
only in the develop worktree. It is deliberately not listed as an existing main
file. Copy/test it explicitly if the integration branch should also include it.

## Optional reproducibility-tool commit

These source tools can be reviewed separately without their generated outputs:

- `camrod_ui/tools/capture_driving_hmi.py`
- `camrod_ui/tools/capture_driving_navigation.py`
- `camrod_ui/tools/capture_driving_preview.py`
- `camrod_ui/tools/capture_live_basemap.py`
- `camrod_ui/tools/capture_navigation_objects.py`
- `camrod_ui/tools/mission_recorder_ros_smoke.py`
- `camrod_ui/tools/show_driving_preview.py`
- `tools/live_navigation_obstacle_probe.py`
- `tools/live_navigation_ui_action.py`

Keep explicit distinctions between synthetic UI fixtures, ROS recorder smoke
input and actual CARLA captures. The live scenario-action tool sends commands
only when explicitly invoked; it must not become a startup hook.

Hold `tools/export_ranger_navigation_asset.py` and
`tools/render_ranger_driving_asset.py` out of the default source list: both
currently hardcode a workstation-local `/home/hong/Downloads/...` model path and
do not have the requested dated English source header. Their existing generated
GLB/PNG assets and provenance are independently usable and already listed. Either
document this local-only reproduction limitation or add portable explicit input
arguments and dated comments in a separate authorized change.

## Documentation and evidence commit

Review and select these release reports separately; do not stage all `docs/`:

- `docs/navigation_basemap_and_asset_integrity_20261002.md`
- `docs/navigation_object_geometry_20261002.md`
- `docs/release_audit_v230_20261002.md`
- `docs/release_ui_validation_v230_20261002.md`
- `docs/release_v230_verification_20261002.md`
- `docs/validation_20261002_mission_can_charge_voice.md`
- `docs/virtual_carla_failure_recovery_20261002.md`
- `docs/virtual_carla_navigation_obstacle_performance_20261002.md`
- `docs/virtual_carla_yolo80_sensor_validation_20261002.md`
- This allowlist report and its exact source manifest.

Add only finalized PNG/GIF files and small summary JSON/Markdown actually linked
from those reports. Keep failed attempts labelled as failures. The B9 test is
still in progress during this audit; the current `delivery_departure_partial.gif`
and stall images do not prove B9 delivery/return/recall completion. Useful bounded
existing captures include the 6.1 MB observed-box GIF and 0.38 MB idle-map GIF in
`docs/evidence/virtual_carla/navigation_objects_live_20261002/`.

## Explicit exclusions

- Every package's `external/` subtree, extracted/vendor files, and
  `camrod_sensing/file/radar_driver/`; this release did not require their changes.
- All broad historical comment-only cleanup outside the exact feature list.
  Retain it in the worktree for a separately audited comment-only commit; do not
  erase it. Never include unrelated parameter changes by assuming all 376+
  modified tracked files are comments.
- `docs/assets/module-guides/sensor-kit/test-results/tapered-rounded-boundary-20260810/`
  changed checksum/result artifacts and unrelated older workspace/history docs.
- `docs/evidence/virtual_carla/current/virtual_test_results.zip`:
  224,759,147 bytes (214.35 MiB), not normal Git source material.
- Raw `.mp4`, `.jsonl`, `.log`, repeated `frames/` and `*_gif_frames/` outputs.
- All runtime `*.sqlite3`, `*.db`, `.writer.lock`, database journal/WAL/SHM files,
  and `camrod_ui/docs/evidence/mission_recorder_ros_smoke_20261002/records/`.
- Any `node_modules`, build/install trees, caches and temporary symlinks. In
  particular, the develop worktree currently exposes an **untracked**
  `camrod_ui/camrod_ui_robot/assets/frontend/node_modules` symlink to the main
  worktree; do not stage it even if a broad directory command proposes it.

Exclusion from Git is not deletion authorization. Real robot records and current
runtime files must remain untouched. Check newly staged blob sizes and Markdown
links before the release commits, and leave tag/push operations to the release
owner after the required validation.
