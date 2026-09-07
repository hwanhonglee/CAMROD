# Recall loading completion and return

The initial Recall still stops up to 0.30 m toward the site from its lanelet
snap. It waits for explicit loading completion, including in profiles that
automatically return after a delivery unload. The Robot UI can complete the
current Guest recall using its exact accepted site and mission generation;
the Guest completion button remains compatible with the same backend path.

| Sites | After loading completion |
| --- | --- |
| B1–B10 | First confirmation → announcement (8 s) → site entry → 180-degree turn → `RECALL_RETURN_WAIT` → second loading-complete confirmation → lateral exit and normal forward drop-zone route |
| B11–B13 | Keep the existing opposite-direction roadside return, without entering the site or turning inside it |

The spoken instruction is: “해당 캠핑 사이트를 잠시 비워 주세요. 로봇이 들어가
방향을 바꾼 뒤 복귀합니다.” The prerecorded Korean cue lasts 6.552 s. The
8 s hold is a timed clearance interval, not confirmation that perception has
proved the site empty or that speaker playback has finished. The existing
optional campsite occupancy guard and the live motion safety gate still apply.
When occupancy guarding is enabled, clearance holds until the site is clear.

Robot and Guest UIs display the controller's actual clearance, entry, rotation,
final loading wait, and exit phases. The new `RECALL_RETURN_WAIT` reports
`GUEST_LOADING_WAIT` and owns strict zero motion, even below 25% SOC. People may
still be loading, so low battery cannot replace this second confirmation.
Only a new `recall_final_return=true` completion for the same site/generation
authorizes exit. Replayed first-stage clicks, generic Return, and early final
clicks cannot start it. Cancellation stops the active motion through the existing
operator-stop path. Return routing starts from the current road-exit pose;
there is no reverse correction to the historical entry coordinate.

## Robot UI return-button authority fix

The 2026-09-07 B8 delivery investigation found a separate backend transport
defect: successful mission admission published the new site/owner/generation
to Guest ROS listeners, but not to already-connected Robot WebSocket clients.
The Robot screen could therefore retain its pre-dispatch identity and fail the
mission-scoped completion check. The common dispatch-status publisher now also
broadcasts the authoritative identity to every Robot client on admission,
rejection, and mission release. It does not forward another client's private
request result or error, and does not weaken site/generation validation.

An existing backend process must be restarted to load this Python fix; building
alone does not replace functions already loaded in that process. Do not restart
it during a live mission merely to refresh the UI. An authorized, exact-identity
Return uses the same normal command path; chassis CAN mode and all safety gates
still apply. A manual/RC-mode hold is not evidence of another missing Return.

## Verification

### Relocated shared parking/docking area

The subsequent map-v23 update retires former drop-zone relation `2320` (vehicle
entrance) and uses only area `7019` for both parking methods. Its underlying
old way/nodes and the user-authored map snapshots are preserved. The existing
`camrod_map/launch/area_export.launch.py` re-exported the new station and all
13 campsites with the shared LocalCartesian origin, retaining operational site
yaws and B11–B13 roadside service policy.

The shared station is `x=-11.3585 m`, `y=40.0901 m`, `yaw=-82.2127 deg`.
Bringup's authoritative `config/map/drop_zones.yaml`, map/localization mirrors,
planning/bringup campsite YAML, and both return keypoints agree on this update.
There is no second parking destination or station-transfer workflow: SOC >=35%
selects ordinary bounded reverse parking at the new station; SOC <35%, unknown
SOC, or an explicit Dock request selects AprilTag docking there. All previous
mission admission, urgent-return, and final Recall confirmation gates remain.

The physical charger and its AprilTag must be at the new station before a
supervised docking check. Moving a map area does not relocate the physical tag;
tag-relative docking still depends on its actual installation. Export and unit
tests do not establish physical clearance or successful field docking.

This single-area revision built and installed `camrod_map`, `camrod_control`,
`camrod_planning`, `camrod_localization`, and `camrod_bringup` successfully
(incremental build: 1 min 40 s). All eight updated installed YAML files match
their source files. The 316 Python regression checks passed, including the
active-map projection, retired-area removal, and synchronized keypoints.
All six rebuilt native control test targets also passed (148 cases).
The unchanged Robot UI source matches `main.daacedf0.js` source-map contents
byte for byte, and installed bundle hashes match; no UI bundle rebuild was
needed for this map-only follow-up. Existing ROS processes still need a
coordinated restart to load the new map/configuration. They were not restarted
and no live motion command was sent.

### Field-discovered battery unit regression

The subsequent 16:32 field return exposed a gap in the earlier passing unit
tests: `AvgPlatformStatus.battery_percentage` is a float32 fraction in [0, 1],
but the new parking dispatcher compared it directly with the 35-percent
threshold. The recorded `0.75` therefore selected AprilTag instead of reverse
parking. The previous tests supplied already-converted 35/80-percent values
and did not cover the actual wire representation.

The dispatcher now uses an explicitly named fraction-to-percent adapter that
validates [0, 1] before conversion. Conversion retains float32 precision before
widening, so `0.35F` becomes 35% and the adjacent lower float stays below the
threshold. Regression tests cover observed 74/75%, exact/adjacent 35% values,
full/low battery, invalid/stale data, and explicit docking at high SOC.

The corrected control package built and installed successfully. All six
native control targets passed (150 cases). A localhost domain-187 smoke check
against the installed dispatcher additionally published actual platform
messages with fractions 0.75/0.74/0.35/1.0 and observed 75/74/35/100 percent in
its status output. No parking operation was requested in that check.

The same field attempt successfully aligned and entered `WAITING_FOR_TAG`,
then failed after 60 seconds without a usable target observation. Live detector
output was `tag_detected=false`; available logs contained other IDs, not ID 3.
The operator confirmed that charger and target tag were moved to the new area.
This does not establish why ID 3 was unrecognized. Existing field processes
exited before a rear-image snapshot could be captured; no process restart,
tag-filter relaxation, or motion command was performed for this diagnosis.
Physical reverse parking after the unit fix and camera-based docking validation
remain pending.

### Field-discovered false parking completion and tag detection

At 17:03:22 the reverse controller logged `PARKED: reverse distance limit
reached`; voice immediately emitted `docking.succeeded`. Drive authorization
was disabled after this success state. There was no lanelet-stop log at that
boundary. The approach point was approximately 3.75 m from the station, so the
retained 1.5 m reverse limit did not establish station arrival.

The first correction stopped at the same existing distance/axis boundaries,
but reports ERROR unless fresh finite localization is within the existing
0.25 m station XY radius. It does not drive farther to compensate, enlarge the
1.5 m limit, or bypass any safety gate. Charging-confirmed stopping remains
authoritative. Pure policy and actual-controller tests cover distance-limit
misreporting, lateral error, overshoot, valid arrival, stale/invalid input,
charging and cancellation. Reaching the new station from its current approach
point was still unresolved with that travel bound; the first fix only prevented
false success. The subsequent bounded-approach correction below supersedes
the retained 1.5 m configuration, not the physical-validation requirement.

Voice now retains selected method and attempt from `/parking/status`.
Only actual AprilTag docking can emit the three docking cues. Ordinary reverse
parking does not reuse docking speech; no verified ordinary-parking WAV exists,
so these announcements are suppressed rather than replaced with a false or
unmapped cue.

The recorded rear frame at ROS time 1788768460.116318089 visibly contains the
station tag. With the same installed AprilTag library, decimation 1.75 yielded
no detection, whereas 1.5 decoded tag36h11 ID 3 with zero corrected bits.
The IPPE pose passed the unchanged 2-pixel reprojection limit (approximately
0.172 px using rounded logged calibration). The installed detector was then
run against this saved frame in isolated localhost domain 188 and published
ID 3 `tag_pose` successfully with the current 1.5 YAML profile.

Both source profiles were already changed to 1.5 when inspected; the live
node's read-only parameter response still reported 1.75. These user edits were
preserved. This constructor-loaded detector setting requires a controlled
restart; no live parameter mutation, process restart, robot motion, or audio
playback was performed. The offline replay proves recognition of this frame,
not reliability across distances or completion of physical docking.

Verification for this correction: 97 Python voice/bringup checks passed (plugin
autoload disabled to avoid an unrelated host pytest/anyio version conflict),
and all eight native control targets passed, including the new completion
fixture. `camrod_control`, `camrod_voice`, and `camrod_bringup` built/installed
successfully. The earlier combined build stopped when the unrelated
`obstacle_lidar_node` compilation was terminated; the full perception package
build therefore did not finish. Its AprilTag detector target did build, both
installed detector YAML files match source, and the installed detector replay
passed again after that build. No full-workspace or live-docking PASS is claimed.

### Follow-up: new station travel bound and GNSS reference clarification

The reverse controller and both parking profiles now use an explicit 5.0 m
maximum instead of 1.5 m, accommodating the measured 3.75 m road-to-station
approach. START rejects an invalid bound or a station farther than that bound;
it never expands the configured limit automatically. Existing 30 s timeout,
0.25 m final station XY tolerance, stale-pose stopping, charging-contact stop,
and limit/axis-miss ERROR behavior remain. Start distance, configured limit,
and remaining station XY error are exposed in logs/diagnostics. This is not a
claim of 5 cm station positioning or of completed physical parking.

The auto dispatcher status already reaches the gate through `/parking/status`.
`REVERSE_APPROACH` already permits crossing the semantic road-lanelet boundary
in the bounded parking phase. The gate still evaluates live LiDAR/radar stops;
no lanelet exception, radar range, obstacle threshold, or emergency-stop policy
was changed for this correction. The recorded early stop is explained by the
old distance limit, not a proven lanelet stop. No active field controller was
available for a fresh physical reproduction during this inspection.

The reverse executable and its completion/controller test targets compiled
successfully. Four control CTest targets passed 122 cases: 102 gate policies,
5 reverse-axis cases, 4 completion checks, and 11 actual-controller fixture
cases. These include continuing past 1.5 m toward a 3.75 m station, completion
only within goal tolerance, rejection beyond 5.0 m, and live-obstacle stopping
despite the existing parking lanelet exception. Installed reverse executable
and both installed parking profiles resolve to the updated build/source files;
the user-tuned AprilTag gains remain 1.2 and 2.0. Tests use an isolated DDS domain,
not physical robot commands. No full-workspace rebuild is claimed.

The operator confirmed that NavSatFix references the LEFT antenna. The current
sensor-kit TF and localization lever-arm configuration both specify x=0.0,
y=+0.45 m relative to robot_center_link. A left-antenna fix does not by itself
imply a configuration error. These values are correct if the antenna is at the
same fore/aft coordinate as the actual rotation center; if it is also forward,
the measured forward distance must be added as x while retaining the measured
left offset. No mounting distance or tuned heading offset was guessed/changed.

The custom adapter does not read the GNSS TF for this correction; it explicitly
subtracts R(yaw) * [offset_x, offset_y] from the projected antenna fix before
publishing the center pose. Thus changing only the TF cannot change that numeric
correction. Its fresh-heading selection accepts the latest valid receiver yaw
with up to 1.0 s absolute timestamp difference from the fix, without matching
the direct receiver yaw to the measurement epoch. This is a potential rotating
pose error source, not an established cause without synchronized field data.
The existing time-indexed EKF fallback is a separate path and does not make the
normal direct-heading path time-aligned.

Five lever-arm unit cases passed, including a fixed-center left-antenna orbit
at 0/90/180/270 degrees and a synthetic 10-degree yaw lag (about 7.8 cm residual
error at 45 cm offset). These tests verify geometry, not real GNSS accuracy or
the physical rotation center. No robot motion, process restart, TF calibration,
or live localization parameter change was performed.

### Final source reconciliation

Release-preparation reconciliation: the final working tree contained the
operator's original 1.0.13 active map (two drop-zone relations, no explicit
service metadata) while exported runtime YAML already used only area 7019.
Reapplied only the approved relation retirement and explicit site/parking
metadata; original nodes, ways, coordinates and named snapshots are unchanged.
Added a source XML comment explaining the retired vehicle-entrance role.
The resulting map SHA-256 is
`2c96514fa788e46ab5061a0ebc130a732557045d0baa3b67bb9f9dbcb132fef7`.

The final detector profiles both contain decimation 2.0 (also the HEAD default),
not the 1.5 profile used in the earlier diagnostic comparison. Preserve 2.0:
an additional installed-detector replay in isolated localhost domain 188
confirmed the startup value 2.0 and target ID 3 with finite pose, approximately
[0.0260, -0.1286, 1.8914] m in the camera frame. No detector source/profile was
retuned for this check. The regression now binds the actual final profile,
while retaining the earlier 1.75/1.5 evidence as history.

### Drop-zone departure after backend restart

In the 18:12 field launch, startup recovery set OPERATOR_STOPPED. At ROS time
1788772414.58 the backend accepted Robot B1, published MOVING_TO_SITE and the
site goal immediately, without a drop-zone EXIT operation. The gate then held
lanelet_physical_body_cost while the robot was still inside area 7019. The
drop-zone controller log contains startup/operator CANCELs, but no EXIT.

UI mission admission now verifies fresh map-frame localization against authored
drop-zone polygons from the same bringup YAML as planning/control. Both Robot
and Guest dispatch inside the area must perform the existing stopped parking
CANCEL and bounded EXIT before releasing a site goal, regardless of missing
service history or OPERATOR_STOPPED. Outside-area requests do not blindly EXIT;
after successful admission they cancel any retained station owners before
normal routing. Invalid/stale/future/missing pose data, malformed/missing area
geometry and boundary ambiguity reject before ownership or motion changes.
An initially missing service state now serializes as UNKNOWN instead of raising
int(None) while publishing a rejected request.

The final UI suite passed all 284 cases, including new Robot/Guest restart,
inside/outside, source/receipt age, invalid geometry, no premature goal, stopped
handoff, duplicate/cancel and battery refusal coverage. Separate bringup/map
routing checks passed 47 cases; voice/system/battery checks passed 98 cases;
all eight native control CTest targets passed. These are overlapping suites,
not a single aggregate count. Tests did not command the physical robot.

The canonical wrapper rebuilt main.daacedf0.js and built/installed camrod_ui,
camrod_voice and camrod_bringup after this correction. The installed backend
matches source. No operator process was restarted by the assistant; successful
physical departure and full perception/workspace build remain separate pending
acceptance. An already-running backend must be restarted while safely stopped
to load the new Python logic and station-map launch parameter.

### Earlier build evidence

The final two-confirmation and battery-policy revision built and installed
`camrod_control`, `camrod_system`, `camrod_planning`, `camrod_voice`,
`camrod_bringup`, and `camrod_ui`. Installed Robot bundle: `main.daacedf0.js`,
verified against the source build manifest. The optional Ranger diagnostic
target was explicitly rebuilt using the already installed `ranger_msgs` CMake
directory; it was not silently skipped for this validation.

Final results: 307 Python UI/Guest/bringup/planning/voice checks; 148 native
control cases across six CTest targets; all six native system CTest targets.
All passed. Native checks used localhost ROS domain 187 (agent fixture checks
also used 188), without live robot commands. Existing Eigen/CMake developer
warnings were non-fatal. No bringup restart or physical field test was run.

Automated checks cover the controller transitions for B1–B13, immutable entry
geometry, explicit completion, duplicate requests, cancellation, occupancy and
pose freshness, backend ownership and generation, UI phase presentation, and
one-shot voice events. These are software checks, not a physical field trial.

After rebuilding and starting bringup, a supervised field check can verify:

1. Recall a cleared test site B1–B10. Confirm roadside stop and continued wait
   before pressing the robot's **적재·정리 완료 · 복귀** button.
2. Finish loading and clear the site. Press completion once. Confirm the
   site-clear voice plays, the robot stays stationary during the clearance
   interval, and both UIs show the same progress.
3. Confirm site entry and one 180-degree turn, then continued zero-motion
   `RECALL_RETURN_WAIT` even if the original completion packet is repeated.
   Finish loading, clear people from the robot, and press **짐 싣기 완료 · 복귀**.
   Only now confirm lateral exit and forward return. SOC >=35% selects bounded
   reverse parking, lower/unknown SOC or explicit Dock selects charging docking.
4. Cancel during clearance or an active return maneuver in a controlled test;
   confirm stop and no delayed continuation from the cancelled request.
5. Repeat B11–B13 with their existing roadside route; verify no site-entry or
   on-site turn occurs. A stale button from another site/visit must be rejected.
