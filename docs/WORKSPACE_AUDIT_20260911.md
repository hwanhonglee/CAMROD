# Workspace entrypoint, documentation and configuration audit

<!-- HH_260911 - Separate actual-source checks, build results and unresolved release findings. -->

## Scope and evidence

This audit inspected the actual develop and virtual/carla working trees, not a
copied source tree. Generated build/install/log files were isolated under
`/home/hong/camrod_ws/_sync_backups/script_config_audit_20260911T014936Z/` so the
running CARLA installation was not replaced. No remote Git push was executed.

The initial tracked-file inventory contained 5,957 develop entries and 6,782
virtual/carla entries. Structural inspection covered YAML/YML, JSON, XML,
Xacro/URDF/OSM, shell files, and Markdown. Counts below are file instances per
branch; shared files are counted once in each branch, not as unique content.

| Project category | develop | virtual/carla |
|---|---:|---:|
| YAML/YML | 262 | 301 |
| JSON | 73 | 76 |
| XML-family | 51 | 58 |
| Shell | 12 | 23 |
| Markdown | 101 | 105 |

No project YAML/JSON/XML parse error, duplicate YAML key, or shell syntax error
was found in that scan. This does not establish that physical calibration,
all parameter consumers, every document statement, or every launch is correct.
Vendor/disabled data produced seven standard-parser candidates per branch:
six OpenCV-format calibration files and one devcontainer JSON candidate.
They were not changed or counted as proven CAMROD runtime defects.

## Applied corrections

- Both real entrypoints now bind `SRC_ROOT` to the checkout containing the script.
  A sibling worktree no longer silently operates on the main workspace's src.
- The regular src checkout retains its original workspace outputs. A sibling
  worktree defaults to a separate `.camrod-build/<checkout-name>` output root.
  `CAMROD_BUILD_ROOT` explicitly overrides outputs, never the source checkout.
- Both scripts offer `--print-paths` without creating output directories.
  Build `--help` exits before cleanup, dependency installation or frontend work.
- A new regression file invokes the actual entrypoints from unrelated working
  directories and checks source identity, output isolation and harmless help.
- Current README labels were corrected for configured GNSS 10 Hz, the field
  0.65 m longitudinal lever arm, auto parking, and develop's map-v24 revision.
  Historical evidence and the user's calibration settings were not rewritten.

## Actual executions

| Check | develop | virtual/carla |
|---|---|---|
| Existing selected shell/config contracts, initial | 78 passed | 82 passed, 2 failed |
| Entire bringup contract suite | 240 passed, 3 failed | 285 passed, 8 failed |
| Actual YOLO dependency build wrapper | 3 packages built | 3 packages built |

The three built packages were vision_msgs, yolov9mit and yolov9mit_ros. Both
YOLO packages were discovered without COLCON_IGNORE. TensorRT deprecation and
ament include-install warnings remain. This was a real compilation/install,
not a mocked build; no new detector inference accuracy claim is made.
The full bringup suite above predates the newly added entrypoint-only tests.

## Open findings: not a blanket release approval

1. The build wrapper can exit 0 for an unknown explicitly selected package.
   Actual negative tests recorded `Summary: 0 packages finished` and exit 0.
   A zero exit status alone must not be treated as proof that a package built.
2. Missing SDL2_mixer still causes camrod_voice to be skipped automatically.
   The CARLA wrapper provisions its local SDL environment, but a standalone
   invocation can skip voice. Package-level completion must be checked.
3. Frontend scope inference is incomplete: develop compares selector names
   rather than the recursive dependency closure; virtual/carla's closure query
   omits explicit nested external discovery roots. The latter printed two
   unknown-yolov9mit_ros warnings after an otherwise successful YOLO build.
4. Setup still logs rosdep-init failure as already initialized, and can print
   completion after apt/rosdep failure. Full privileged installation/update was
   NOT run on this active workstation. Attempts to apply broader setup/build
   error-handling fixes were blocked by the execution tool and were not applied.
5. develop's three bringup failures concern old map-version/hash assumptions
   and the permitted differences from a historical map snapshot. The current
   root map is v24 (d4a76062...), while the preserved CARLA root map is v23
   (2c96514f...). This is not proof that all map geometry is interchangeable.
6. CARLA's eight failures involve parking yaw tolerance 7.5 vs 2.5 degrees,
   package/bringup GNSS x 0.60 vs 0.65 m in the actual worktree, drop-zone YAML
   yaw -88.2127 vs OSM -82.2127 degrees, and stale source-hash evidence.
   Original uncommitted sensor calibration was preserved. Do not weaken tests
   or overwrite measurements merely to make these checks green.
7. Markdown target candidates include intentional template placeholders and
   missing historical evidence pages. No fabricated history was supplied.

## Final entrypoint checks and use

The final real-entrypoint regression group passed 18 tests on develop and
24 on virtual/carla, including eight newly added path/help checks per branch.
These results do not erase the 3/8 failures in the wider bringup suite.

```bash
# Safe inspection; neither command installs dependencies or starts a build.
./setup_camrod.sh --print-paths
./colcon_build.sh --print-paths
./setup_camrod.sh --help
./colcon_build.sh --help

# Direct build entrypoint only: actual sources, isolated output directories.
CAMROD_BUILD_ROOT=/absolute/empty/output/root ./colcon_build.sh \
  --packages-up-to yolov9mit_ros --parallel-workers 2
```

Do not call full setup/update merely to reproduce this audit on a running
workstation. System installations, external checkout updates, complete fresh
machine provisioning, ARM64 execution, all B1-B13 missions and full physical
calibration acceptance remain outside this audit's completed scope.

The local worak-test branch is aligned to the final develop audit commit.
Only explicitly listed branches/tags should be pushed; no force push is needed
for an unchanged remote. An atomic dry run is not a substitute for the remote
server's actual protection-rule checks or the unresolved validation items.
