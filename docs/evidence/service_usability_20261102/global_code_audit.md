# CAMROD owned-code audit, 2026-11-02

<!-- HH_261102 - Record measured scope and small verified reductions without treating safety checks as bloat. -->

## Scope and baseline

Baseline: `c7d2f9c88` (`origin/unit2/main`), before the service-usability branch changes.
The complete selected-file inventory is [owned_code_inventory.tsv](owned_code_inventory.tsv).

| Category | Files | Physical lines |
| --- | ---: | ---: |
| Owned source, including interfaces, launch files and developer tools | 386 | 134,575 |
| Configuration and build files | 297 | 28,199 |
| Tests | 143 | 36,867 |
| Total | 826 | 199,641 |

Physical lines include blank lines and comments; they are not executable-SLOC counts.
The inventory includes CAMROD-managed frontend source under `assets/frontend`.
It excludes external/vendor dependencies, the bundled vendor radar kernel driver under
`camrod_sensing/file`, generated map exports (16 files), dependency lockfiles,
documentation, evidence, audio/resources, and non-source assets. New untracked test-branch
files are not silently counted as baseline code.

## Global checks and targeted review

- Enumerated all selected source/config/test paths from the baseline Git tree.
- Hashed selected files larger than 200 bytes: 104 exact-duplicate groups contain
  9,017 additional physical lines. These are overwhelmingly intentional package/deployment
  configuration mirrors, not duplicate executable implementations.
- Scanned owned Python function names and imported names, then checked repository callers
  and tests for the candidate private helpers. Checked C++ declaration-name candidates
  conservatively; exported APIs and framework overrides were not removed merely because
  an in-repository caller was absent.
- Read the existing battery conversion, admission, parking-method selection and ownership
  handoff paths before simplifying them.

This is a whole-tree inventory and static candidate scan with targeted semantic review.
It is not a claim that every line has received a complete semantic review or that every
runtime path has been exercised.

## Small changes and evidence

### Unused simulation helper

`SimValidationRunner._event_contains` had no caller in owned source, launch/config or tests.
The actual repeated-service validation uses `_route_recovery_progress`, which requires
an ordered motion event followed by an `IDLE` event.

- Removed four physical lines (decorator, unused helper, separator).
- Added one `HH_261102` reason beside the retained recovery helper.
- Net existing source reduction: three physical lines.
- Added five pure-helper cases in `test_sim_runner_recovery_progress.py` (31 test lines).
  The tests compile only the actual helper AST; they do not start a ROS node or simulator.

### Shared battery conversion

Guest previously evaluated `floor(float(value) * 100)`. A real float32 CAN value for
35% widens to `0.3499999940395355`, so Guest displayed 34 while Robot displayed 35.
The conversion now lives once in `battery_policy.normalize_platform_battery_percent`;
Guest imports/re-exports the existing name so callers retain their import contract.
The Robot backend now uses the same helper while retaining its existing availability,
finite-value and `[0, 1]` admission guard.

- `battery_policy.py`: 12 added physical lines.
- `ui_guest_node.py`: three added, eight removed physical lines; old helper doc/comment
  moved into the shared implementation rather than discarded.
- These two files alone have a net seven-line increase: this is a real consistency fix,
  not a claimed line-count saving. Robot caller integration removes its second conversion
  implementation separately.
- Added 14 cases in `test_platform_battery_normalization.py` (28 test lines), including
  real float32 25%/35% values and each immediately lower wire neighbor.
- Preserved finite clamp behavior, NaN/infinity as unknown, and floor behavior. Shared
  conversion is not charger contact, BMS health, mission ownership or motion permission.

## Candidates handed to the owning agent

Parent integration completed the first three removals below and retained the compatibility callback.
Robot/Guest shared conversion passed the subsequent UI regression run. Newly added sensor/control
features and tests increase the overall diff; there is no claimed whole-repository percentage reduction.

- `UiBackendNode._parse_destination_payload`: definition-only private helper, 25-line body
  including signature. Current typed destination parsing does not call it.
- `UiBackendNode._snapshot_disk_free_mb`: definition-only five-line private wrapper;
  current snapshot budgets use `_snapshot_disk_usage` directly.
- `AvgTwistStamped`: unused import in the Robot backend.
- `CampingSiteManeuverControllerNode::isEntryPhase`: definition-only six-line private
  predicate. Left unchanged in this audit because removal would save little and needs
  native controller compilation/regression coverage.

The explicit `_on_telemetry_lidar` compatibility callback was preserved. Public C++
accessors and the Nav2 `isClearable` override were not treated as safely removable
private dead code.

## Intentional duplication retained

`camrod_bringup/test/test_package_config_mirrors.py` documents and tests why standalone
package configurations coexist with self-contained deployment mirrors. Deleting one
side, replacing arbitrary files with links, dropping calibration, or stripping historical
`HH_...` comments is outside this safe reduction. No mirror configuration was deleted.

## Charging/parking policy confirmed without movement

`parking_selection_policy.hpp:67` selects AprilTag docking for SOC strictly below 35%,
unknown/stale SOC, or an explicit force-docking request. Exactly 35% selects normal
noncharging reverse parking. A normal Return starts with reverse parking; dispatcher
handoff to AprilTag requires an actually completed, fresh owned reverse park, healthy
fresh CAN authority, and no current charging feedback (`parking_dispatcher_node.cpp:395`).
The existing normal Return policy therefore does not require a new force-redocking
completion button.

The existing mapped noncharging station anchor is `dz_area_7144`,
`x=-8.47366`, `y=40.391`, reverse-axis yaw `-88.2127 deg`, in the active drop-zone YAML.
It is inside the authored polygon and is already the reverse-parking XY goal. This
identifies an existing anchor, not a verified physical cargo-clearance pose. A new
movement must still validate current position, direction, full footprint, bounds and
actual charger hardware. No native target/controller was changed by this audit.

## Verification and limits

118 targeted tests passed: shared SOC conversion, existing Guest contract, battery policy,
the retained simulation recovery helper, and existing configuration mirror contracts.
Python compilation and `git diff --check` passed.

The targeted command, after sourcing the existing ROS/interface environment, was:

```bash
PYTHONPATH=$PWD/camrod_ui/runtime/python:$PYTHONPATH python3 -m pytest -q \
  camrod_ui/test/test_platform_battery_normalization.py \
  camrod_ui/test/test_ui_guest_contract.py camrod_ui/test/test_battery_return_policy.py \
  camrod_bringup/test/test_sim_runner_recovery_progress.py \
  camrod_bringup/test/test_package_config_mirrors.py --disable-warnings --maxfail=1
```

No robot/simulator service was started, restarted or stopped. No real CARLA movement,
charger electrical isolation, physical connector clearance or hardware test is claimed.
No commit or push was made by this audit task.
