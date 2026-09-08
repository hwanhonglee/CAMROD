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
