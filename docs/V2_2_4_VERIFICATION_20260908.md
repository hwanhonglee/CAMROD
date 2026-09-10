# CAMROD v2.2.4 focused verification — 2026-09-08

Base: `f3a177038567f8f1b568e2f49d4c127bc298d9a5` (`develop`, v2.2.4).
The checks below ran from an isolated source checkout using Ubuntu 22.04,
ROS 2 Humble, Python 3.10.12 and GCC 11.4.0. Existing installed ROS dependencies
were used; the tested native controller and GoalSnapper fixtures were rebuilt
from this checkout. The running robot workspace was not reinstalled by these
checks.

## Corrections

| Files | Change and reason |
| --- | --- |
| `camrod_localization/config/source/input_adapter.yaml` | Match the package heading trim to the existing bringup value, `-92°`. Standalone localization previously used `-90°`. Antenna offsets remain x=0, y=+0.45 m. |
| `camrod_bringup/config/sim/fake_sensors.yaml` | Match the built-in simulator's raw heading bias to `+92°`, so the `-92°` input correction recovers the original heading. The focused test initially caught the old `+90°`/`-92°` mismatch. |
| Control/planning map-geometry fixtures | Replace stale map-v22 expected coordinates with the already-deployed map-v23 B1–B13 sites. Map and runtime campsite coordinates are unchanged. The native GoalSnapper computes each projection from the active map and checks its expected signed site side. |
| `test_camping_site_recall_return.cpp` | Add a regression that battery-urgent return yaw alignment reaches ERROR without publishing a return request after its existing timeout. The production timeout behavior already exists in v2.2.4. |
| `test_recall_target_policy.py` | Verify the return handoff margin still requires a fresh successful Nav2 result; stale or absent success cannot authorize arrival. |
| Control/Nav2 comments, handoff tests and parameter/release references | Correct stale 0.05/0.30 m descriptions to existing field values: Nav2 route 0.10 m, local parking approach 0.20 m, final reverse station 0.25 m. No tolerance is widened by this patch. |

The local parking tolerance is not a GNSS lever-arm fix. The controller does
not recheck XY after its 90° yaw alignment; correct antenna geometry and
timestamp-matched heading still matter.

## Results

| Verification | Result |
| --- | --- |
| Bringup config mirrors, route/parking handoff, robot-center frame, map revision and operating-point contracts; planning recall policy | 77/77 Python tests passed |
| Native control policy, local parking approach, actual campsite recall controller, SOC parking selection, reverse completion and actual reverse controller | 6/6 CTest targets passed; 154/154 GoogleTest cases |
| Native production GoalSnapper against active B1–B13 map geometry | 1/1 CTest target passed; 2/2 GoogleTest cases |
| Patch whitespace/error check | Passed |

The compiler emitted existing anonymous-namespace field linkage warnings from
test fixtures that include production `.cpp` files. Builds and all listed
checks completed successfully.

These are config, policy and deterministic controller tests. They do not
measure real-world traversal time/distance or prove successful physical
B1–B13 delivery, Return, Recall, UI rendering or docking. These tests generate
text/JUnit results, not driving PNG/GIF recordings.

## Reproduction

From this source checkout, with the matching ROS dependencies installed:

```bash
source /opt/ros/humble/setup.bash
source /home/hong/camrod_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=191

python3 -m pytest -q \
  camrod_bringup/test/test_nav2_controller_profile_sync.py \
  camrod_bringup/test/test_package_config_mirrors.py \
  camrod_bringup/test/test_robot_center_frame_contract.py \
  camrod_bringup/test/test_lanelet_map_revision_sync.py \
  camrod_bringup/test/test_park_operating_points_assets.py \
  camrod_planning/test/test_recall_target_policy.py

verification_build=$(mktemp -d /tmp/camrod-v224-verification.XXXXXXXX)
cmake -S camrod_control -B "$verification_build/control" \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$verification_build/control" --parallel 2 --target \
  test_control_policies test_camping_site_recall_return \
  test_parking_selection_policy test_reverse_parking_completion \
  test_reverse_parking_controller test_drop_zone_parking_approach
ctest --test-dir "$verification_build/control" --output-on-failure \
  -R 'test_control_policies|test_camping_site_recall_return|test_parking_selection_policy|test_reverse_parking_completion|test_reverse_parking_controller|test_drop_zone_parking_approach'

cmake -S camrod_planning -B "$verification_build/planning" \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$verification_build/planning" --parallel 2 \
  --target test_active_campsite_geometry
ROS_DOMAIN_ID=192 ctest --test-dir "$verification_build/planning" \
  --output-on-failure -R '^test_active_campsite_geometry$'
```

Controller fixtures additionally set their own isolated DDS domains 187/188.
Use the install path of the target workspace when reproducing on another host.

## Follow-up: nested detector launch replaced controller parameters

`parking.launch.py` included the detector launch with an argument also named
`parameter_file`. The unscoped include left the detector YAML in its parent's
launch context. In `auto`/`apriltag` mode with the detector enabled, the later
parking nodes consequently received perception YAML rather than `parking.yaml`.
Missing node parameters then fell back to constructor defaults, including
`complete_without_charging=false` despite the configured value being `true`.

The detector include now runs inside `GroupAction(scoped=True)`. It receives
the configured detector YAML and restores the outer controller YAML before
the dispatcher, reverse controller and AprilTag controller are evaluated.

`test_parking_launch_parameter_scope.py` executes the actual nested launch
descriptions, arguments, conditions and scope push/pop in `LaunchContext`.
Only process startup is replaced with parameter-file observations. It checks
auto/apriltag with detector on and reverse/auto with detector off, including
the controller's `complete_without_charging=true` and 5.0 m reverse bound.
The test produced 2 failures and 2 passes before the fix, then 4 passes after
the fix. The existing 10 AprilTag docking contracts also passed (14 total).

```bash
python3 -m pytest -q \
  camrod_bringup/test/test_parking_launch_parameter_scope.py \
  camrod_bringup/test/test_apriltag_docking_contract.py
```

The regression is registered with bringup CTest. Restart the launch after
installing the change; existing controller processes retain their old parameters.

## Final departure regression audit

The registered parking-scope CTest target passed after fresh bringup CMake
configuration. A bounded UI/departure run also exposed an outdated source-text
contract: it still expected service-state-only admission, while v2.2.4 already
uses `_station_departure_origin()` and fresh map-frame polygon containment.
The contract now checks that current logic, including unknown origins and
charging/outside-polygon mismatches. No UI or departure runtime code changed.
The selected station/departure/origin UI tests and departure contracts passed:
18 passed, 60 unrelated tests deselected.

## Final pure-CAMROD revision and retained evidence

The final runtime-code revision covered by this supplement is
`ddd599672dd7977295406b24e1ffa3fc2f0225e2`, including the UI regression commit
`af3d7b6e58876f57b4fb9c3ff8cd4cb7a3b8c3df`. The later documentation commit does
not change algorithms, configuration, launch behavior or the running workspace.

| Focused suite | Result | Retained original JUnit |
| --- | --- | --- |
| Robot UI frontend contracts, including completion after a battery heartbeat | 32 passed, 0 failed/skipped; rerun on the final pure revision | [robot_ui_frontend.junit.xml](evidence/v2_2_4_20260908/robot_ui_frontend.junit.xml) |
| Native production reverse-parking controller | 16 passed, 0 failed/disabled | [reverse_parking_controller.junit.xml](evidence/v2_2_4_20260908/reverse_parking_controller.junit.xml) |
| Reverse-parking Euclidean goal-completion helper | 4 passed, 0 failed/disabled | [reverse_parking_completion.junit.xml](evidence/v2_2_4_20260908/reverse_parking_completion.junit.xml) |

[verification_summary.json](evidence/v2_2_4_20260908/verification_summary.json)
records the source revision, test provenance, limits of the claims, artifact
sizes and SHA-256 values. The native XML files are byte-identical to the
successful native build's original results. Its CMake source directory was
the isolated pure checkout's `camrod_control`, not a simulator controller.
Only the small result files are retained here; temporary build trees and large
runtime logs are not required to read or reproduce them. These 52 latest
focused cases are a supplement to the earlier results, not a rerun of every
historical suite in this document.

### UI completion survives an unrelated battery heartbeat

Commit `af3d7b6e` extends the actual frontend WebSocket-handler replay in
`test_robot_reconnect_restores_completion_and_preserves_minimal_phase_frames`.
After an owned B1 mission reaches `WAITING_FOR_RETURN_REQUEST`, a later
`battery_return_pending=false` message must preserve the arrived site,
completion popup and ownership-checked completion permission. It must not
erase arrival state merely because no battery return is pending.

The pure develop frontend already had the correct incremental battery-state
handler, so this commit changes the regression test only, not `App.js`.
The complete frontend contract suite was run again when curating these
artifacts: 32 passed in 0.35 seconds. The replay runs extracted production
JavaScript with Node; it does not connect to live WebSockets, command a robot,
or prove actual browser rendering and physical travel.

### Reverse parking reaches the XY disk, not just its axial envelope

Commit `ddd59967` fixes `reverse_parking_controller_node.cpp` and documents
the existing Euclidean goal contract in `reverse_parking_completion.hpp`.
Previously the controller stopped as soon as signed reverse-axis distance
entered the tolerance, then correctly rejected `PARKED` if XY was still
outside the goal. For example, axis error 0.25 m and lateral error 0.128 m
produce approximately 0.281 m XY error, outside the unchanged 0.25 m radius.
This was an early stopping-boundary error, not proof of inadequate wheel torque.

The controller now continues its existing slow final approach within the
axial envelope only while the station is still ahead and the reverse-axis
line can intersect the same XY acceptance disk. It stops with an error if
the station plane has been passed outside the disk, or the lateral offset
cannot reach that disk. The maximum reverse distance and timeout remain hard
bounds. Fresh finite localization, invalid-goal rejection, immediate charging
contact handling, cancellation and the actual XY acceptance radius are
preserved. No torque, vehicle model, goal tolerance or sensor pose is changed.

The native regression includes both signs of a reachable lateral offset,
unreachable and tangential lateral misses, station-plane crossing and the
retained timeout. The two rebuilt CTest targets passed all 16 controller and
4 goal-helper cases. To reproduce after the CMake configuration above:

```bash
cmake --build "$verification_build/control" --parallel 2 --target \
  test_reverse_parking_controller test_reverse_parking_completion
ctest --test-dir "$verification_build/control" --output-on-failure \
  -R '^test_reverse_parking_(controller|completion)$'
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q -p no:cacheprovider \
  camrod_ui/test/test_ui_robot_frontend_contract.py \
  --junitxml="$verification_build/robot_ui_frontend.junit.xml"
```

### Hardware geometry and validation boundary

The stored GNSS antenna lever arm remains **x=0.0 m, y=+0.45 m**, with the
existing heading trim **-92 degrees**. A reported front-mounted antenna has
not been measured and confirmed by these software tests. Do not infer a
verified forward offset, silently move it to the robot center, or claim that
changing parking tolerances corrects its geometry. Deployment still requires
the actual antenna-to-body measurement, frame-axis convention and
timestamp-aligned heading to be checked on that platform.

This pure supplement introduces no external simulator, bridge, model or map
dependency. The earlier existing built-in simulator heading-bias correction
is recorded separately above; it is not an external simulator integration.
Real rendered simulator trials and their PNG/GIF evidence remain a separate
validation stream. The native/UI results here do not certify B1-B13 physical
delivery, recall, Guest-to-Robot handoff, optional docking or the unmeasured
hardware mount.
