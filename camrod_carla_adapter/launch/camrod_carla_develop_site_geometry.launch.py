"""Develop-parity CAMROD with proven CARLA campsite/runtime adaptations.

This wrapper preserves the ordinary control, planning, safety, localization,
parking, and UI defaults from ``camrod_carla_full.launch.py`` except for the
explicit CARLA route-recovery adaptations listed below.  Its perception graph,
tag identity, physical tag size, AprilTag validation threshold, ROI policy,
thread count, and outputs remain production-parity; only AprilTag detector
decimation is disabled so CARLA's 960x720 rear image retains enough tag pixels
at the Drop Zone acquisition distance.  Separately, this wrapper explicitly
selects the measured Woraksan YOLO `min_confidence=0.95` overlay; ordinary
`camrod` retains develop's `0.50` value and only applies the CARLA camera-to-
LiDAR extrinsic.  It also selects the campsite-geometry
parameters exercised by the 7a095ee B1 physical round-trip and one
authenticated CARLA-plant recovery lease.  The site profile also opts into
full-route-relative recovery, zero-hold timing, corrective yaw, and bounded
goal reissue.  These switches already exist as false-by-default full-launch
arguments, so ordinary CAMROD and develop-parity CARLA remain unchanged.  The
lease does not change ordinary or manual drive authority: it is accepted only
while the bounded route-recovery controller publishes a fresh healthy state
for a 0.04--0.06 m/s recovery command.  Live B2 return evidence on 2026-09-02
showed that the unassisted 0.05 m/s crab moved only 0.263 m before the 90 s
bounded limit, leaving the robot safely stopped at the lanelet edge.

The current develop return contract is also preserved explicitly for ordinary
and turnaround sites: after ``CRAB_OUT`` the controller accepts a fresh live
lanelet projection, holds zero for 1.20 s, and asks LaneletRoute to plan from
the robot's current XY.  B11--B13 are the scoped CARLA exception: their
roadside exit must first reach the 0.03 m lanelet band, retain outbound yaw,
and select the separately tagged RPPReverse route.  The former CARLA entry-
anchor centering lease remains pinned off; enabling it before the maneuver
would defeat develop's current-pose arrival behavior.

CARLA Drop-Zone arrival poses also showed run-to-run lateral variation larger
than the immutable AprilTag completion tolerance. This wrapper alone enables
two fail-closed retries: stop at the unchanged 0.40 m range, align yaw, drive a
measured 0.8 m forward within signed-progress, drift, odometry-step, path and
timeout bounds, discard the old tag-axis estimate, and reacquire. Fresh B2 v14
evidence measured lateral errors of 0.054 m and 0.042 m around a bounded
0.804 m first exit, so the v15 profile changes only the retry-count budget from
one to two. Ordinary CAMROD and the full/develop-parity CARLA launch keep that
retry disabled.

Fresh B2 v15 evidence also reached the post-crab steering-settle latch and
then reported 0.12 m lateral error at the develop 0.12 m fail threshold
(0.02 m transition plus 0.10 m hysteresis).  The v16 wrapper keeps the
0.02 m transition exact and changes only its CARLA return hysteresis to
0.13 m, yielding a still fail-closed 0.15 m total band for the measured
physical-simulation settling.  The production/develop parameter file remains
0.10 m and the full/develop-parity CARLA launch supplies no override.

Fresh B2 v16 evidence then reached the same B2 rotation center used by the
successful v14/v15 runs, but the 0.35 rad/s request produced only about
1.68 N*m/wheel at the observed 0.78 deg/s residual rate and stalled after
about 120 degrees.  The v17 wrapper alone requests 0.45 rad/s so the unchanged
physical yaw-rate controller can reach its already accepted 2.0 N*m/wheel
rotation cap.  It does not change the 20 N*m bridge cap, the backend torque
configuration, the 60 s timeout, or any completion/safety tolerance.

Fresh B2 v17/v18 evidence completed that rotation in about 20 s, entered
``CRAB_OUT``, and then stopped after the physical body contacted the narrow
edge of the simulator-only access cut.  Increasing the post-latch hysteresis
from the develop 0.10 m value merely delayed the same yaw runaway, so the
v19 profile removes that override.  The controller therefore uses the exact
develop 0.02 m transition plus 0.10 m hysteresis again; the isolated Woraksan
CARLA map supplies the wider B2 collision-free access surface instead.

Fresh B2 v19 wheel telemetry then proved that the Ranger remained inside that
cut with all four wheels grounded and symmetric 88-degree/8 N*m commands, but
the steep right contact normals repeatedly rotated the body away from its
retrace heading.  The v20 wrapper alone enables a bounded CRAB_OUT hook: at
more than 8 degrees of drift it publishes a full zero tick, reuses the existing
bounded rotate/settle controller, and resumes the still-latched lateral stage.
The measured 1.224-to-0.798 m trace crossed that threshold about six times, so
the CARLA lease allows eight attempts inside one non-resetting 90 s steady
episode.  Exhaustion, timeout, or invalid evidence stops in ERROR.  Production
and develop-parity launches keep this hook disabled.

The optional fixed-entry-line cross-track
guard remains at the develop default (disabled): live CARLA runs measured
0.101 m and 0.121 m of normal lateral drift while the physical crab steering
was clipped to 88 degrees.  That guard therefore rejected valid motion and is
not a campsite boundary check.  In particular, this is not an alias for the
historical Woraksan-tuned profile.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Keep this mapping explicit and host-independent: the live runtime auditor
# checks the corresponding ROS parameters before a campsite matrix may move.
DEVELOP_SITE_GEOMETRY_ARGUMENTS = {
    # CARLA publishes a real rear RGB stream, but the AprilTag debug JPEG is
    # intentionally event-driven and can be stale before acquisition.  Only
    # this simulator evidence profile may show the live rear stream in that
    # empty docking slot; full/develop/production launches remain fail-closed.
    "operator_telemetry_docking_rear_camera_fallback_enabled": "true",
    # Do not halve campsite CRAB/ROTATE commands in the final gate. Ordinary
    # Nav2 is independently pre-limited by the CARLA runtime overlay to the
    # same 0.555556 m/s physical cruise produced by develop's 1.111111 * 0.5.
    # This keeps B11 on its tight route while preserving the B10-v22-proven
    # maneuver dynamics and accepted extreme-crab Kp=8.
    "carla_cmd_vel_gate_speed_scale": "1.0",
    # Arrival intentionally closes both production command gates.  In CARLA,
    # explicitly reopen mission authorization before publishing the campsite
    # RETURN operation so a short B11-B13 roadside CRAB_OUT cannot spend its
    # entire bounded timeout issuing commands into STANDBY.  The shared full
    # launch and ordinary develop profile keep the fail-closed default false.
    "return_site_exit_rearm_enabled": "true",
    # The campsite return ends in the production AprilTag controller rather
    # than reverse_parking_controller.  CARLA has no physical charger/BMS, so
    # this site-only node converts a verified stopped contact pose into the
    # same heartbeat input the real platform supplies.  The full/develop-
    # parity launch remains disabled by default.
    "launch_charging_contact_emulator": "true",
    "carla_charging_contact_parking_status_topic": (
        "/parking/apriltag_parking_controller/status"
    ),
    # CARLA plant-only low-speed breakaway.  The command adapter authenticates
    # the bounded recovery node before setting the backend authorization bit.
    "recovery_breakaway_enable": "true",
    # v20 held the correct zero-turn cut for 75.8 s but the ordinary 2 N*m
    # yaw-rate controller settled at only 0.873 N*m median and made no useful
    # progress against the B2 support slope. B10 v11 likewise completed
    # 155.5 degrees before settling at a 25.5-degree residual with all four
    # wheels grounded. This site-only lease lets the backend stage 3/4 N*m
    # only while the exact ROTATE_180 or CRAB_OUT recovery state is fresh and
    # measured yaw progress is stalled; moving rotation remains on 2 N*m.
    "rotation_recovery_breakaway_enable": "true",
    # The develop controller's status heartbeat is 1 Hz.  1.25 s covers one
    # period plus scheduling jitter while the 0.35 s command watchdog and the
    # exact phase/token predicate still fail closed on stopped control.
    "rotation_recovery_breakaway_status_timeout_sec": "1.25",
    # The generic develop recovery chooses a clearance side.  CARLA's B2
    # return can contact a cost-100 body cell after that choice and exhaust the
    # bounded controller in place.  The site-only profile instead follows the
    # current full route inward, pauses the bounded clock only for its exact
    # fail-closed zero sentinel, permits only yaw that corrects an exceeded
    # limit, and reissues the same goal after the gate proves continuous clear.
    # Judge a real pose discontinuity from unsnapped localization. The
    # lanelet-snapped pose can switch nearby branches by metres while the
    # physical Ranger remains continuous; treating that switch as a teleport
    # caused three false B2 replans and left the replacement route 2.184 m
    # from the actor on the 2026-09-02 v6 run.
    "carla_goal_snapper_pose_jump_check_topic": "/localization/pose",
    "carla_route_safety_path_relative_recovery_enable": "true",
    # The shared 0.08 m re-entry value permits only 0.03 m of CTE regression.
    # The official B2 return improved from 0.220 m to 0.162 m, then the CARLA
    # plant settled at 0.246 m after its zero-turn-to-parallel transition.  A
    # 0.15 m site-only re-entry band admits that still fully projected inward
    # correction while retaining a hard stop at 0.10 m regression from best.
    "carla_route_safety_path_center_reentry_m": "0.15",
    # B11 v13 reached the ordinary 0.15 m live-lanelet handoff while still
    # 0.41 m from its inbound anchor. The resulting forward loop immediately
    # put the physical body on a cost-100 edge, then Nav2 recovery rotated the
    # front radar to 0.16 m. Keep the develop current-pose policy everywhere
    # else, but make the existing roadside-only CARLA exception atomic: crab
    # to the 0.03 m band, retain outbound yaw, tag the goal as reverse, select
    # RPPReverse, and enforce the reverse lanelet corridor.
    "carla_roadside_reverse_return_enable": "true",
    "carla_roadside_reverse_handoff_distance_m": "0.03",
    "carla_nav2_reverse_controller": "RPPReverse",
    "carla_reverse_goal_topic": "/planning/auto_reverse_goal_raw",
    "carla_lanelet_safety_check_reverse": "true",
    # ALIGN_OUTBOUND_LANE_YAW is a CARLA roadside-return geometry lease, not a
    # develop safety default. Command ownership remains production-equivalent;
    # only this explicit wrapper lets the phase cross the measured static and
    # lanelet boundary while live classified/radar stops remain authoritative.
    "carla_camping_site_maneuver_controller_static_bypass_phases": (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,"
        "ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
    ),
    "carla_camping_site_maneuver_controller_lanelet_bypass_phases": (
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,"
        "ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT"
    ),
    # Nav2 and the planning state machine measure the same snapped return goal
    # from different live pose references. B2 operator recall recorded Nav2
    # SUCCEEDED at 0.338 m in the planning reference while the production
    # 0.300 m handoff remained closed. Match the already-proven generic 0.35 m
    # arrival bound only in this site wrapper; Nav2 success, its 15 s latch,
    # the 0.75 m local correction cap and 0.05 m parking tolerance stay exact.
    "carla_return_goal_reached_distance_m": "0.35",
    # B11 v13 completed outbound and CRAB_OUT with collision=0 and every
    # physical wheel grounded, then the road-return command repeatedly hit
    # lanelet_footprint_cost at body=(-0.63,-0.63).  That cell is outside the
    # measured body (right=0.53495 m) and touches only the extra 0.10 m planning
    # envelope (right=0.63495 m).  Disable exactly that simulator/map-mismatched
    # envelope check for this site-validation wrapper.  The fabrication-sized
    # physical-body lanelet hard stop, directional lanelet corridor, fresh
    # radar/classified-fusion obstacle checks, stale-grid stops and platform
    # interlocks remain enabled.  The shared full launch defaults this to true.
    "carla_lanelet_safety_footprint_enable": "false",
    # A semantic fusion hit is attributed on the merged raster, which also
    # contains dense static lanelet/path costs.  Release must be proven by the
    # same fresh fusion source so unrelated static cells cannot hold it forever.
    "carla_cost_stop_latch_use_trigger_source_for_merged_clear": "true",
    # Fusion owns a classified, route-only 2 m direct horizon. Only radar may
    # attribute the wider merged corridor; this prevents a fusion pixel beyond
    # 2 m from authorizing an overlapping static lanelet cell as an obstacle.
    "carla_cost_stop_merged_dynamic_source_labels": "radar",
    "carla_route_safety_zero_hold_pauses_limits": "true",
    "carla_route_safety_allow_corrective_yaw_beyond_limit": "true",
    "carla_goal_reissue_while_nav_active": "true",
    "carla_crab_approach_slowdown_distance_m": "1.0",
    "carla_crab_approach_min_speed_mps": "0.12",
    # Keep a bounded 90 s final guard for slow terrain transitions. The
    # stall-qualified CARLA lease above supplies torque only if yaw progress
    # falls below its existing backend threshold; this is not a blind wait.
    "carla_rotate_180_timeout_s": "90.0",
    # B2 v16 stalled with 61.24 deg remaining while the 0.35 rad/s request
    # yielded about 1.68 N*m/wheel. Request enough rate to saturate the
    # unchanged, independently verified 2.0 N*m/wheel rotation envelope.
    "carla_camping_site_max_angular_speed_radps": "0.45",
    "carla_entry_position_tolerance_m": "0.05",
    "carla_rotate_entry_max_position_error_m": "0.05",
    # v11 B10 reached the rotation entry 0.542 m from its authored center
    # after the physical wheels crossed the locally smoothed CARLA support.
    # Permit that measured simulator-only correction with 0.108 m margin,
    # while retaining the exact 0.05 m rotation target and bounded centering.
    "carla_rotate_entry_centering_max_initial_error_m": "0.65",
    # Develop 5ab77f246 plans the return from the fresh CRAB_OUT pose.  Keep
    # the obsolete exact inbound-anchor centering path disabled in CARLA too.
    "carla_entry_anchor_centering_max_initial_error_m": "0.0",
    "carla_entry_anchor_centering_max_speed_mps": "0.12",
    "carla_entry_anchor_centering_timeout_s": "15",
    "carla_entry_anchor_centering_tolerance_m": "0.05",
    # HH_260903 - Preserve the develop CRAB_IN continuation contract.  B1
    # remained physically mobile and inside the authored access corridor, but
    # run-to-run contact variation crossed the former CARLA-only 5 deg guard
    # by 0.10 deg before the existing bounded centering stage could act.
    # Zero disables only this simulator-added guard; production/develop already
    # uses the same identity and all target, timeout and centering bounds remain.
    "carla_crab_entry_max_heading_drift_deg": "0.0",
    "carla_crab_entry_body_yaw_compensation_deg": "2.0",
    # B1 reached -62.52 deg for a -61.25 deg prealignment target, but the
    # physical CARLA plant did not settle inside the 0.5 deg develop band
    # before the 15 s fail-closed timeout.  Admit that measured 1.27 deg
    # residual only in this site-validation wrapper.  Develop parity and the
    # historical tuned profile deliberately retain their 0.5 deg contract.
    "carla_crab_entry_body_yaw_alignment_tolerance_deg": "1.5",
    "carla_crab_entry_body_yaw_alignment_timeout_s": "15",
    "carla_crab_out_yaw_recovery_enable": "true",
    "carla_crab_out_yaw_recovery_trigger_deg": "8.0",
    "carla_crab_out_yaw_recovery_max_attempts": "8",
    "carla_crab_out_yaw_recovery_global_timeout_s": "90.0",
}


def develop_site_geometry_arguments(adapter_share):
    """Bind the proven site profile to its CARLA-only sensor/plant configs."""
    arguments = dict(DEVELOP_SITE_GEOMETRY_ARGUMENTS)
    # Unlike full/develop-parity, this evidence profile explicitly opts into
    # the measured Woraksan YOLO false-positive filter as well as the shared
    # CARLA camera-to-LiDAR extrinsic.
    arguments["carla_perception_runtime_override_param_file"] = os.path.join(
        adapter_share,
        "config",
        "perception_carla_site_geometry.yaml",
    )
    arguments["carla_apriltag_param_file"] = os.path.join(
        adapter_share,
        "config",
        "apriltag_parking_detector_carla.yaml",
    )
    arguments["carla_parking_runtime_override_param_file"] = os.path.join(
        adapter_share,
        "config",
        "apriltag_parking_controller_carla.yaml",
    )
    arguments["carla_nav2_reverse_return_param_file"] = os.path.join(
        adapter_share,
        "config",
        "nav2_carla_reverse_return.yaml",
    )
    return arguments


def generate_launch_description():
    adapter_share = get_package_share_directory("camrod_carla_adapter")
    full_launch = os.path.join(
        adapter_share, "launch", "camrod_carla_full.launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments=develop_site_geometry_arguments(
                adapter_share
            ).items(),
        )
    ])


if __name__ == "__main__":
    generate_launch_description()
