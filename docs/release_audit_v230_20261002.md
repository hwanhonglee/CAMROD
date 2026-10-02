# v2.3.0 release separation audit — 2026-10-02

Read-only audit before release preparation. No fetch, checkout, staging, commit,
tag, push, source transfer, runtime command or restart was performed by this audit.
References below are the locally available references at inspection time; the
release owner must recheck remote references before publication.

## Baselines and scope

| Item | Observed state |
| --- | --- |
| Main worktree | `/home/hong/camrod_ws/src`, `virtual/carla`, `6859f6d68bce1a8a126d57eeff120ace3e2582ce` |
| Develop worktree | `/tmp/camrod-develop-comments.5YTNjJB7`, `develop`, `15277ab325cd835966fb41f96d16998770377bb2`, clean |
| Local origin/develop and v2.2.9 | `fe6f7815fc13eee1b482bee9e5d2b3badddec9a0` |
| Develop-only commit | `15277ab32`, historical inline comment annotations, one commit ahead of the inspected origin/develop |
| Common ancestor of develop and virtual/carla | `fe6f7815f` |
| Local v2.3.0 tag | Not present at inspection |
| Main working changes | 376 modified tracked files; 302 untracked files at inspection |

The virtual branch contains substantial **pre-existing** simulator integration
in otherwise shared CAMROD packages. Whole-directory copying, merging the virtual
branch into develop, or applying `git diff develop` without hunk selection would
promote more than the newly requested core improvements.

An AST/YAML/comment-stripped C++ comparison against the virtual HEAD classified
284 tracked edits as comment/whitespace-only, 38 as potential semantic edits, and
54 as unclassified formats. This is an audit heuristic, not proof of behavior
equivalence: for example Python docstring edits count as AST changes and the C++
lexer is not a compiler. Historical comments already committed on develop must
not be removed by overwriting files with virtual-branch versions.

## Feature-oriented core promotion

Use the incremental **virtual HEAD → current working files** delta, not the
entire difference between develop and virtual/carla. Port the following groups
onto the clean develop worktree while retaining its existing annotations.

| Functional group | Core files / new modules | Required integration |
| --- | --- | --- |
| Passive navigation and object visualization | `camrod_ui/runtime/python/camrod_ui/driving_snapshot.py`; frontend `DrivingDisplay*`, `DrivingIntegration*`, `RangerNavigationScene*`, `navigationMath*`, `navigationObjectVisuals*`, `illustrativeScenery*`, `rangerModelAsset.js`, `useDrivingDisplay*`, `DrivingPreview*` | Select navigation imports, subscriptions/callbacks, pose/platform/diagnostic hooks and `/api/driving` endpoint in `ui_backend_node.py`; select display-only integration in frontend `App.js`, `App.css`, `index.js`; include `three` package/lock changes and model assets |
| Measured foreground object extents | `camrod_perception/include/camrod_perception/observed_lidar_extent.hpp`; `test/test_observed_lidar_extent.cpp`; corresponding `CMakeLists.txt` test and `src/obstacle_fusion_node.cpp` hunks | Dedicated `navigation_boxes` publisher; preserve existing legacy sphere/Detection3D sizes and all planning/safety outputs |
| Stop confirmation and Guest cancel/retry | frontend `App.js`/`App.css`, Guest `assets/guest_frontend/index.html`, backend `ui_backend_node.py`, `ui_guest_node.py`, relevant stop/Guest/frontend tests | Retain mission revision/owner checks, backend-session identity and fail-closed handling; do not import simulator manual-drive transport as an incidental dependency |
| Mission statistics, paired legs, manual distance and CAN journal | `service_metrics.py`, `service_metrics_migration.py`, `mission_journal.py`, `mission_recorder_node.py`, `mission_recording_bridge.py`, `raw_can_capture.py`; frontend `ServiceEvidence*`, `MissionRecords*`; `camrod_ui/docs/mission_recording.md` | Select event/metrics/API hunks in `ui_backend_node.py`, recorder wiring in `camrod_ui_robot/launch/ui.launch.py` and `camrod_bringup/launch/_bringup_impl.py`, new `mission_recorder.launch.py`, `setup.py`, relevant tests |
| Charge-complete presentation and recall-stage presentation | `battery_policy.py`, backend/App presentation hunks, `test_battery_return_policy.py`, related UI tests | BMS full/charging signal only; existing mission-admission and urgent-return thresholds remain unchanged |
| Correct site/return voice direction | `camrod_voice/config/voice_event_adapter.yaml`, `src/voice_event_adapter_node.py`, `src/voice_event_policy.py`, `test/test_voice_event_policy.py` | Separate reactive return cue from UI-gated outbound cue; clear obsolete route identity on direction change |
| Safe frontend publication | `camrod_ui/scripts/build_frontend.sh`, `sync_frontend_build.sh`, package scripts/lock, `test/test_frontend_publication.py` | Stage build outside served tree; publish complete assets atomically; retain previous hashed bundles for open tabs |

The new runtime Python modules and extent helper have no CARLA imports or CARLA
runtime dependencies. Some **display text/comments** are simulator-specific:

- `MissionRecords.js:118` labels simulated telemetry `CARLA 주행 표본` and accepts
  `carla|simulator` environment aliases. Prefer generic simulation wording in
  pure develop if strict separation includes product labels; retain the explicit
  distinction between simulated telemetry and measured physical CAN.
- `rangerModelAsset.js` mentions CARLA packaging in its comment.
- `illustrativeScenery.js` and a debug source string in `RangerNavigationScene.js`
  mention CARLA geometry. These are labels/comments, not imports; generic wording
  avoids implying a simulator dependency on develop.

## Keep on virtual/carla only

- Entire `camrod_carla_adapter/` changes, notably
  `config/command_adapter_carla_site_manual.yaml`, `config/yolo_coco80.txt`,
  `config/perception_carla_site_geometry.yaml`, adapter/full/site launch changes,
  command mapping and simulator-specific tests.
- `scripts/virtual_carla/*`, `tools/live_navigation_obstacle_probe.py`,
  `tools/live_navigation_ui_action.py`, and CARLA-specific runtime evidence.
- Existing `config/diagnostics/carla/` and `cyclonedds_carla.xml` in shared packages.
- Existing simulator launch overrides, source-substitution paths, physical 4WS
  runtime switches and isolated simulator storage roots.
- Existing `ManualDrivePanel.js`, `manual_drive_policy.py`, backend manual WebSocket
  transport and manual launch parameters: these are already present in the virtual
  baseline but absent from develop. The new 2 m/s simulator-only manual ceiling
  must not become a physical-robot default or be promoted incidentally.
- Existing frontend `camrod-build-env.json`: it disables the operating-hours gate
  and is absent from develop. Do not copy it as part of a frontend-directory sync.

The virtual baseline also differs in `camrod_control`, planning launch/config,
perception configuration, external YOLO/TensorRT build support and diagnostics.
None of these pre-existing simulator differences is required merely to add the
new read-only UI, recorder, Guest recovery or voice fixes to develop.

## Mechanical transfer checks

Read-only `git apply --check` was run on individual current incremental tracked
file patches against the clean develop worktree. It did not change either index.

Already applies cleanly (still review feature grouping and existing annotations):

- `camrod_perception/CMakeLists.txt`
- `camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html`
- frontend `package.json`, `package-lock.json`, `src/ServiceEvidence.js`, `src/index.js`
- backend `battery_policy.py`, `service_metrics.py`, `ui_guest_node.py`
- `camrod_ui/scripts/sync_frontend_build.sh`, `camrod_ui/setup.py`
- `test_drop_zone_departure_origin.py`, `test_service_metrics.py`,
  `test_ui_guest_contract.py`, `test_ui_robot_frontend_contract.py`
- `camrod_voice/config/voice_event_adapter.yaml`, `src/voice_event_adapter_node.py`,
  `src/voice_event_policy.py`, `README.md`

Important patches needing explicit hunk reconciliation:

- `camrod_bringup/launch/_bringup_impl.py`
- `camrod_perception/src/obstacle_fusion_node.cpp`
- frontend `src/App.js`, `src/App.css`
- `camrod_ui/camrod_ui_robot/launch/ui.launch.py`
- `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`
- `test_battery_return_policy.py`, `test_ui_backend_stop.py`,
  `camrod_voice/test/test_voice_event_policy.py`

Do not resolve these by replacing the whole file with the virtual copy. Port
feature hunks onto develop with `apply_patch`; add new independent modules/assets
from an explicit allow-list; retain develop's comments and physical-platform
defaults. For the shared backend, separate commits may touch the same file:
navigation, Guest/stop recovery, mission journal/statistics, charging presentation.
Run each group's tests after its integration, then combined build/runtime checks.

Suggested commit sequence on develop:

1. Passive navigation/object visualization plus measured display extents and assets.
2. Stop confirmation and Guest cancellation/session recovery.
3. Mission/leg statistics and independent CAN/state journal.
4. Charge-complete and recall-stage presentation, voice-direction correction.
5. Atomic frontend publication and release documentation/tests, if not grouped above.

Historical comment cleanup is already a separate develop commit. Keep subsequent
virtual-specific implementation and historical comment normalization separate
from core feature commits. Do not create/move `v2.3.0` until the chosen develop
commit has passed the required validation. This request authorizes preparation,
not an actual remote push.

## Assets, evidence and unsafe blanket staging

New `frontend/public/models/` totals approximately 10.4 MiB:

| Asset | Bytes |
| --- | ---: |
| `ranger-navigation.glb` | 5,576,620 |
| `woraksan-side-wrap.png` | 1,987,476 |
| `woraksan-front-wrap.png` | 1,399,204 |
| `woraksan-rear-wrap.png` | 1,084,559 |
| `ranger-driving-rear.png` | 814,382 |

These files and provenance/readme files are currently untracked and not ignored.
They are required portable UI assets, not a CARLA Python/Unreal dependency. Include
the renderer's required assets with frontend changes: both `npm start` and the
staged build now hash the GLB and three wrap files. Missing assets break that build
step. Include generated-source provenance without requiring its workstation-local
input paths at runtime.

Do not blanket-stage these untracked files:

- `docs/evidence/virtual_carla/current/virtual_test_results.zip`: **214.35 MiB**,
  not ignored. Keep out of normal Git history; it exceeds the repository's own
  documented 100 MB per-blob publication limit.
- `camrod_ui/docs/evidence/mission_recorder_ros_smoke_20261002/records/.writer.lock`
  and `records/mission_journal.sqlite3`: runtime lock/database, not source fixtures.
- Repeated passive collector JSONL files (18.30, 8.46 and 3.38 MiB), redundant
  experimental captures, and earlier mock preview images: select only clear
  release evidence with honest source/attempt labels. Do not delete physical
  robot data as part of this source release operation.

`build/` and `node_modules/` are correctly ignored. The giant ZIP, SQLite journal,
writer lock and public model assets are not ignored. Source tests, live simulator
tests and illustrative preview captures must remain explicitly distinguished in
the release report; no simulator screenshot proves physical CAN or all-site
scenario completion.
