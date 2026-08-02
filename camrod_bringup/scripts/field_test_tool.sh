#!/usr/bin/env bash
# HH_260708 - Outdoor field-test helper for repeatable CAMROD debugging.
#
# This script is intentionally conservative:
# - read-only commands are the default;
# - motion-enabling commands require --allow-motion;
# - logs are written under $HOME/camrod_field_logs for later debugging.

set -euo pipefail

DEFAULT_HZ_SECONDS=5
DEFAULT_WATCH_INTERVAL=1.0
CAMERA_YOLO_MIN_SECONDS=12
CAMERA_YOLO_ACCEPTANCE_SECONDS=300

log() { echo "[field_test] $*"; }
warn() { echo "[field_test] WARN: $*" >&2; }
die() { echo "[field_test] ERROR: $*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Usage:
  field_test_tool.sh config
      Compare bringup config files with package-level configs and installed copies.

  field_test_tool.sh snapshot [log_dir]
      Save git, process, ROS graph, diagnostics, topic info, and short Hz samples.

  field_test_tool.sh record-recovery [log_dir]
      Record the TODO 11-13 route-recovery/steering evidence until Ctrl+C.
      Also saves active parameters and a PASS/FAIL field-result template.

  field_test_tool.sh hz [seconds]
      Measure the field-critical topic rates.

  field_test_tool.sh camera-yolo [seconds]
      Check the front-camera input, YOLO detections, and on-demand debug image rates.

  field_test_tool.sh profile [seconds] [label]
      Capture CPU/GPU/RAM and critical topic rates concurrently.
      Run once each for RViz, WebKit-only, and UI-window-off comparisons.

  field_test_tool.sh pose-latency [seconds] [output_json]
      Measure the real GNSS/IMU/wheel -> EKF -> selected-pose chain concurrently.
      Missing publishers, type mismatches, and invalid headers fail visibly.

  field_test_tool.sh watch [seconds]
      Print a compact live status loop for diagnostics, localization, planning, and gates.

  field_test_tool.sh launch [extra launch args...]
      Start real bringup with tee logging. Example:
        field_test_tool.sh launch rviz:=true

  field_test_tool.sh stop-gates
      Publish false with the native AvgBool contract and verify command output is disabled.

  field_test_tool.sh enable-gates --allow-motion
      Open software engage/drive-enable gates with the native AvgBool contract.
      Requires explicit --allow-motion.

Environment:
  CAMROD_FIELD_LOG_ROOT  Override log root (default: $HOME/camrod_field_logs).

Notes:
  This script does not replace the physical e-stop or operator supervision.
EOF
}

resolve_ws_root() {
  local probe="$1"
  while [[ "${probe}" != "/" ]]; do
    if [[ -d "${probe}/src/camrod_bringup" ]]; then
      echo "${probe}"
      return 0
    fi
    probe="$(dirname "${probe}")"
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_ROOT="$(resolve_ws_root "${SCRIPT_DIR}" || resolve_ws_root "$(pwd)" || true)"
[[ -n "${WS_ROOT}" ]] || die "cannot find workspace root (expected <ws>/src/camrod_bringup)"
SRC_ROOT="${WS_ROOT}/src"

source_ros() {
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
  fi
  set -u
}

new_log_dir() {
  local root stamp
  root="${CAMROD_FIELD_LOG_ROOT:-${HOME}/camrod_field_logs}"
  stamp="$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${root}/${stamp}"
  echo "${root}/${stamp}"
}

run_shell_to_log() {
  local out_file="$1"
  shift
  {
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
  } >"${out_file}" 2>&1 || {
    local rc=$?
    echo "[exit=${rc}]" >>"${out_file}"
    return 0
  }
}

run_eval_to_log() {
  local out_file="$1"
  local cmd="$2"
  {
    echo "$ ${cmd}"
    bash -lc "${cmd}"
  } >"${out_file}" 2>&1 || {
    local rc=$?
    echo "[exit=${rc}]" >>"${out_file}"
    return 0
  }
}

safe_name() {
  echo "$1" | sed 's#^/##; s#[^A-Za-z0-9_.-]#_#g'
}

topic_once_to_log() {
  local topic="$1"
  local out_file="$2"
  run_eval_to_log "${out_file}" "source /opt/ros/humble/setup.bash; source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true; timeout 3 ros2 topic echo --once '${topic}'"
}

topic_hz_to_log() {
  local topic="$1"
  local seconds="$2"
  local out_file="$3"
  run_eval_to_log "${out_file}" "source /opt/ros/humble/setup.bash; source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true; timeout --signal=INT --kill-after=2 '${seconds}' ros2 topic hz '${topic}'"
}

compare_tree_subset() {
  local label="$1"
  local bringup_dir="$2"
  local package_dir="$3"
  local status=0
  local src rel dst

  echo "## ${label}"
  if [[ ! -d "${bringup_dir}" ]]; then
    echo "MISSING bringup dir: ${bringup_dir}"
    return 1
  fi
  if [[ ! -d "${package_dir}" ]]; then
    echo "MISSING package dir: ${package_dir}"
    return 1
  fi

  while IFS= read -r -d '' src; do
    rel="${src#${bringup_dir}/}"
    dst="${package_dir}/${rel}"
    if [[ ! -f "${dst}" ]]; then
      echo "MISSING package file: ${label}/${rel}"
      status=1
      continue
    fi
    if cmp -s "${src}" "${dst}"; then
      echo "OK ${label}/${rel}"
    else
      echo "DIFF ${label}/${rel}"
      status=1
    fi
  done < <(find "${bringup_dir}" -type f -print0 | sort -z)

  # HH_260730 - Also reject package-only files. A one-way subset check could
  # leave a package default unreviewed and absent from the deployed bringup.
  while IFS= read -r -d '' src; do
    rel="${src#${package_dir}/}"
    dst="${bringup_dir}/${rel}"
    if [[ ! -f "${dst}" ]]; then
      echo "EXTRA package file: ${label}/${rel}"
      status=1
    fi
  done < <(find "${package_dir}" -type f -print0 | sort -z)

  return "${status}"
}

compare_install_subset() {
  local label="$1"
  local source_dir="$2"
  local install_dir="$3"
  local status=0
  local src rel dst

  echo "## install:${label}"
  if [[ ! -d "${install_dir}" ]]; then
    echo "MISSING install dir: ${install_dir}"
    return 1
  fi
  while IFS= read -r -d '' src; do
    rel="${src#${source_dir}/}"
    dst="${install_dir}/${rel}"
    if [[ ! -f "${dst}" ]]; then
      echo "MISSING installed file: ${label}/${rel}"
      status=1
      continue
    fi
    if cmp -s "${src}" "${dst}"; then
      echo "OK install:${label}/${rel}"
    else
      echo "DIFF install:${label}/${rel}"
      status=1
    fi
  done < <(find "${source_dir}" -type f -print0 | sort -z)

  # HH_260730 - Stale installed YAML is just as dangerous as a missing copy:
  # launch may still discover it by path even after the source was removed.
  while IFS= read -r -d '' src; do
    rel="${src#${install_dir}/}"
    dst="${source_dir}/${rel}"
    if [[ ! -f "${dst}" ]]; then
      echo "EXTRA installed file: ${label}/${rel}"
      status=1
    fi
  done < <(find "${install_dir}" -type f -print0 | sort -z)

  return "${status}"
}

cmd_config() {
  local rc=0
  local pairs=(
    "platform:camrod_bringup/config/platform:camrod_platform/config"
    "map:camrod_bringup/config/map:camrod_map/config"
    "planning:camrod_bringup/config/planning:camrod_planning/config"
    "perception:camrod_bringup/config/perception:camrod_perception/config"
    "sensing:camrod_bringup/config/sensing:camrod_sensing/config"
    "system:camrod_bringup/config/system:camrod_system/config"
    "sensor_kit:camrod_bringup/config/sensor_kit:camrod_sensor_kit/config"
    # HH_260720 - Parking and maneuver configuration now belongs to camrod_control.
    "control:camrod_bringup/config/control:camrod_control/config"
    "localization:camrod_bringup/config/localization:camrod_localization/config"
  )
  local item label bringup_rel package_rel bringup_dir package_dir package_name

  cd "${SRC_ROOT}"
  for item in "${pairs[@]}"; do
    IFS=: read -r label bringup_rel package_rel <<<"${item}"
    bringup_dir="${SRC_ROOT}/${bringup_rel}"
    package_dir="${SRC_ROOT}/${package_rel}"
    compare_tree_subset "${label}" "${bringup_dir}" "${package_dir}" || rc=1
    echo
  done

  compare_install_subset "bringup" \
    "${SRC_ROOT}/camrod_bringup/config" \
    "${WS_ROOT}/install/camrod_bringup/share/camrod_bringup/config" || rc=1
  echo

  for item in "${pairs[@]}"; do
    IFS=: read -r label bringup_rel package_rel <<<"${item}"
    package_dir="${SRC_ROOT}/${package_rel}"
    package_name="${package_rel%%/*}"
    compare_install_subset "${label}" \
      "${package_dir}" \
      "${WS_ROOT}/install/${package_name}/share/${package_name}/config" || rc=1
    echo
  done

  if [[ "${rc}" -eq 0 ]]; then
    log "config sync OK"
  else
    warn "config sync mismatch detected"
  fi
  return "${rc}"
}

critical_topics() {
  # HH_260720 - Observe the control gate contract used by hardware bringup.
  # HH_260722 - Capture raw dual-GNSS position, baseline, and RTCM evidence.
  cat <<'EOF'
/system/status
/service/state
/system/diagnostics_agg
/localization/mode
/localization/pose
/planning/state_machine/state
/planning/engage
/planning/mission_engage
/platform/drive_enable
/control/planning_engaged
/control/command_enabled
/control/cmd_vel_safety_gate/status
/goal_pose
/ui/selected_destination
/planning/mission_key
/planning/goal_pose_snapped
/planning/goal_pose_snapped_ros
/planning/goal_source
/planning/state_machine/mission_source
/planning/navigate_to_pose/_action/status
/planning/navigate_to_pose/_action/feedback
/planning/global_path
/planning/global_path_avg
/planning/local_path
/planning/tracking_error
/planning/controller_selector_ros
/planning/speed_limit
/planning/lookahead_point
/control/cmd_vel_raw
/control/nav2_cmd_vel_ros
/control/cmd_vel
/control/cmd_vel_ros
/platform/status
/platform/status/odometry
/localization/input/wheel_odometry
/localization/input/wheel_odometry_ros
/localization/primary/odometry_ros
/localization/odometry_ros
/planning/cost_grid/inflation
/sensing/cost_grid/lidar
/sensing/cost_grid/radar
/sensing/radar/obstacle_evidence
/sensing/radar/dummy_active
/sensing/radar/front1/dummy_active
/sensing/radar/front2/dummy_active
/sensing/radar/left1/dummy_active
/sensing/radar/left2/dummy_active
/sensing/radar/right1/dummy_active
/sensing/radar/right2/dummy_active
/sensing/radar/rear/dummy_active
/perception/obstacles/fused_obstacles
/perception/obstacles
/perception/camera/detections_2d
/sensing/lidar/points_filtered
/sensing/lidar/filtered_cloud
/sensing/gnss/ntrip_client/rtcm
/sensing/gnss/ublox_gps_node/navpvt
/sensing/gnss/ublox_gps_node/fix
/sensing/gnss/ublox_gps_node/fix_velocity
/sensing/gnss/navheading
/sensing/gnss/navrelposned
/sensing/gnss/rxmrtcm
/sensing/gnss/dummy_active
/sensing/imu/data
/sensing/imu/data_ros
/sensing/imu/dummy_active
/sensing/lidar/dummy_active
/sensing/camera/econ_front/dummy_active
/sensing/camera/econ_rear/dummy_active
/platform/dummy_active
/voice/voice_announcer/say
/voice/voice_announcer/state
/sensing/radar/front1/range
/sensing/radar/front2/range
/sensing/radar/left1/range
/sensing/radar/left2/range
/sensing/radar/right1/range
/sensing/radar/right2/range
/sensing/radar/rear/range
EOF
}

rate_only_topics() {
  cat <<'EOF'
/sensing/camera/econ_front/image_rect/compressed
/sensing/camera/econ_front/camera_info
/sensing/camera/econ_rear/image_raw
/sensing/camera/econ_rear/image_raw/compressed
/sensing/camera/econ_rear/camera_info
/perception/camera/yolo_image
EOF
}

# HH_260730 / TODOLIST 8 - These rates must be sampled concurrently with
# process and Jetson telemetry; sequential `hz` samples cannot explain a CPU
# spike at the same instant.
profile_topics() {
  cat <<'EOF'
/planning/global_path
/planning/global_path_avg
/planning/local_path
/planning/tracking_error
/control/cmd_vel_raw
/control/cmd_vel
/control/cmd_vel_ros
/planning/cost_grid/inflation
/sensing/cost_grid/lidar
/sensing/cost_grid/radar
/sensing/lidar/points_filtered
/sensing/camera/econ_front/image_rect/compressed
/sensing/camera/econ_rear/image_raw
/sensing/camera/econ_rear/image_raw/compressed
/perception/camera/detections_2d
EOF
}

# HH_260729 / TODOLIST 11-13 - Keep every route-hold, retained-goal,
# full-footprint, obstacle, and steering-feedback source on one rosbag clock.
recovery_topics() {
  cat <<'EOF'
/rosout
/tf
/tf_static
/service/state
/localization/pose
/planning/state_machine/state
/planning/engage
/planning/mission_engage
/planning/mission_key
/planning/state_machine/mission_source
/planning/goal_pose_snapped
/planning/goal_pose_snapped_ros
/planning/goal_source
/planning/navigate_to_pose/_action/status
/planning/navigate_to_pose/_action/feedback
/planning/global_path
/planning/global_path_avg
/planning/local_path
/planning/tracking_error
/planning/controller_selector_ros
/planning/speed_limit
/planning/lookahead_point
/control/planning_engaged
/control/command_enabled
/control/cmd_vel_safety_gate/status
/control/cmd_vel_raw
/control/nav2_cmd_vel_ros
/control/cmd_vel
/control/cmd_vel_ros
/platform/drive_enable
/platform/status
/platform/status/odometry
/platform/status/wheel
/platform/steering_transition_state
/odom
/motion_state
/actuator_state
/sensing/gnss/ublox_gps_node/fix
/sensing/gnss/ublox_gps_node/fix_velocity
/sensing/gnss/navheading
/sensing/gnss/navrelposned
/sensing/gnss/pose_with_covariance_ros
/localization/input/wheel_odometry_ros
/localization/primary/odometry_ros
/localization/odometry_ros
/platform/robot/planning_boundary
/map/cost_grid/lanelet
/planning/cost_grid/inflation
/sensing/cost_grid/lidar
/sensing/cost_grid/radar
/sensing/radar/obstacle_evidence
/sensing/radar/front1/range
/sensing/radar/front2/range
/sensing/radar/left1/range
/sensing/radar/left2/range
/sensing/radar/right1/range
/sensing/radar/right2/range
/sensing/radar/rear/range
/system/status
/system/diagnostics_agg
/localization/mode
/ui/selected_destination
EOF
}

write_recovery_result_template() {
  local output_file="$1"
  local commit
  commit="$(git -C "${SRC_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  cat >"${output_file}" <<EOF
CAMROD TODO 11-13 REAL-ROBOT FIELD RESULT
date/time:
operator:
location:
commit: ${commit}
bag directory:
bringup log:
weather/ground:
battery start/end:
physical e-stop checked: YES / NO
second safety operator present: YES / NO

[TODO 11] ROUTE_SAFETY_HOLD AND SAME-GOAL RECOVERY
trigger reason:
trigger direction:
command_enabled became false: YES / NO
ROUTE_SAFETY_HOLD observed: YES / NO
outside/stale evidence stayed blocked: YES / NO
continuous clear observed (required >= 1.0 s):
Nav2 ABORTED during hold: YES / NO
same goal/source reissued after enabled (required >= 0.5 s): YES / NO / N/A
reissue count (required <= 2):
PASS / FAIL:
notes:

[TODO 12] CONSTRAINED OPPOSITE ESCAPE AND FIRST RE-ENGAGE
same/zero/rotation remained blocked: YES / NO
rear obstacle blocked opposite escape: YES / NO
clear opposite escape moved at supervised low speed: YES / NO
safe corridor re-entered: YES / NO
first re-engage produced path and cmd_vel: YES / NO
operator cancel caused no automatic restart: YES / NO
PASS / FAIL:
notes:

[TODO 13] STEERING-LAG VELOCITY ENVELOPE
left-offset run complete: YES / NO
right-offset run complete: YES / NO
target/limited/scale logs captured: YES / NO
scale=0 observed for steering error >= 0.35 rad: YES / NO / NOT REACHED
full scale observed only at error <= 0.05 rad: YES / NO
repeated centerline crossing: YES / NO
lanelet footprint cost 100 contact: YES / NO
measured pose->controller->gate->wheel delays:
PASS / FAIL:
notes:

OVERALL RESULT: PASS / FAIL / FIELD RETEST REQUIRED
EOF
}

cmd_record_recovery() {
  source_ros
  local log_dir="${1:-}"
  [[ -n "${log_dir}" ]] || log_dir="$(new_log_dir)"
  local bag_dir="${log_dir}/route_recovery_bag"
  [[ ! -e "${bag_dir}" ]] || die "bag directory already exists: ${bag_dir}"
  mkdir -p "${log_dir}/meta"

  # HH_260729 / TODOLIST 11-13 - Freeze the exact source/config identity before
  # motion so field evidence can be traced back to one commit and parameter set.
  run_shell_to_log "${log_dir}/meta/date.txt" date --iso-8601=seconds
  run_eval_to_log \
    "${log_dir}/meta/git.txt" \
    "cd '${SRC_ROOT}' && git status --short --branch && git log -1 --format=fuller"
  run_eval_to_log \
    "${log_dir}/meta/config_sync.txt" \
    "'${SCRIPT_DIR}/field_test_tool.sh' config"
  {
    echo '$ active recovery parameters'
    ros2 param get /control/cmd_vel_safety_gate route_safety_recovery_enable
    ros2 param get /control/cmd_vel_safety_gate route_safety_recovery_clear_required_s
    ros2 param get /control/cmd_vel_safety_gate route_safety_opposite_direction_cosine_max
    ros2 param get /control/cmd_vel_safety_gate route_safety_opposite_recovery_probe_distance_m
    ros2 param get /control/cmd_vel_safety_gate route_safety_recovery_pose_max_age_s
    ros2 param get /planning/goal_snapper reissue_active_goal_after_route_recovery
    ros2 param get /planning/goal_snapper route_recovery_reissue_clear_delay_s
    ros2 param get /planning/goal_snapper route_recovery_max_reissues_per_goal
    ros2 param get /ranger_base_node steering_transition_rate_radps
    ros2 param get /ranger_base_node steering_transition_velocity_scale_enabled
    ros2 param get /ranger_base_node steering_transition_full_speed_error_rad
    ros2 param get /ranger_base_node steering_transition_stop_error_rad
    ros2 param get /ranger_base_node steering_transition_min_velocity_scale
    # HH_260730 - Capture the direct and manual-wrapper RPP profiles so a
    # straight-line A/B can prove that both goal sources tracked identically.
    ros2 param get /planning/controller_server controller_frequency
    ros2 param get /planning/controller_server default_controller
    ros2 param get /planning/controller_server RPP.desired_linear_vel
    ros2 param get /planning/controller_server RPP.min_lookahead_dist
    ros2 param get /planning/controller_server RPP.max_lookahead_dist
    ros2 param get /planning/controller_server RPP.lookahead_time
    ros2 param get /planning/controller_server RPP.use_velocity_scaled_lookahead_dist
    ros2 param get /planning/controller_server RotationShim.desired_linear_vel
    ros2 param get /planning/controller_server RotationShim.min_lookahead_dist
    ros2 param get /planning/controller_server RotationShim.max_lookahead_dist
    ros2 param get /planning/controller_server RotationShim.lookahead_time
    ros2 param get /planning/controller_server RotationShim.use_velocity_scaled_lookahead_dist
  } >"${log_dir}/meta/recovery_parameters.txt" 2>&1 || true
  write_recovery_result_template "${log_dir}/FIELD_RESULT.txt"

  declare -A available_topics=()
  while IFS= read -r topic; do
    [[ -n "${topic}" ]] && available_topics["${topic}"]=1
  done < <(ros2 topic list --include-hidden-topics)

  # HH_260730 - A non-empty constant request list does not prove bringup is
  # alive. Refuse to start field evidence without the core safety owners.
  local required_topic
  for required_topic in \
    /control/cmd_vel_safety_gate/status \
    /control/command_enabled \
    /localization/pose \
    /map/cost_grid/lanelet \
    /platform/status
  do
    [[ -n "${available_topics[${required_topic}]+present}" ]] ||
      die "required recovery topic is not active: ${required_topic}"
  done

  local topics=()
  local missing=()
  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    # HH_260729 / TODOLIST 11-13 - Pass initially missing topics to rosbag too.
    # Its discovery loop can attach after a lifecycle node activates or respawns;
    # the missing list remains a preflight warning, not a permanent exclusion.
    topics+=("${topic}")
    if [[ -z "${available_topics[${topic}]+present}" ]]; then
      missing+=("${topic}")
    fi
  done < <(recovery_topics)
  [[ "${#topics[@]}" -gt 0 ]] || die "none of the recovery topics are available"

  printf '%s\n' "${topics[@]}" >"${log_dir}/meta/requested_topics.txt"
  printf '%s\n' "${!available_topics[@]}" |
    sort >"${log_dir}/meta/available_at_start_topics.txt"
  printf '%s\n' "${missing[@]}" >"${log_dir}/meta/missing_topics.txt"
  if [[ "${#missing[@]}" -gt 0 ]]; then
    warn "some recovery topics are missing; inspect ${log_dir}/meta/missing_topics.txt"
  fi

  log "recording TODO 11-13 evidence to ${bag_dir}"
  log "keep this terminal open through the test; press Ctrl+C only after all runs"
  log "complete the PASS/FAIL form at ${log_dir}/FIELD_RESULT.txt"
  set +e
  ros2 bag record \
    --include-hidden-topics \
    --output "${bag_dir}" \
    "${topics[@]}"
  local record_rc=$?
  set -e
  if [[ "${record_rc}" -ne 0 && "${record_rc}" -ne 130 ]]; then
    warn "rosbag exited with status ${record_rc}"
  fi
  if [[ -d "${bag_dir}" ]]; then
    run_shell_to_log "${log_dir}/meta/bag_info.txt" ros2 bag info "${bag_dir}"
    sed -n 's/.*Topic: \([^| ]*\).*/\1/p' \
      "${log_dir}/meta/bag_info.txt" |
      sort -u >"${log_dir}/meta/recorded_topics.txt"
  fi
  log "recovery recording complete: ${log_dir}"
}

cmd_snapshot() {
  source_ros
  local log_dir="${1:-}"
  [[ -n "${log_dir}" ]] || log_dir="$(new_log_dir)"
  mkdir -p "${log_dir}/topics" "${log_dir}/hz" "${log_dir}/meta"
  log "writing snapshot to ${log_dir}"

  run_shell_to_log "${log_dir}/meta/date.txt" date --iso-8601=seconds
  run_shell_to_log "${log_dir}/meta/uname.txt" uname -a
  run_eval_to_log "${log_dir}/meta/git.txt" "cd '${SRC_ROOT}' && git status --short --branch && git log --oneline --decorate -5"
  run_eval_to_log "${log_dir}/meta/config_sync.txt" "'${SCRIPT_DIR}/field_test_tool.sh' config"
  # HH_260727 - Record numeric devices and their stable identities so a reboot
  # or USB power cycle cannot silently swap GNSS, IMU, modem, and light roles.
  run_eval_to_log "${log_dir}/meta/serial_ports.txt" "ls -l /dev/serial/by-id /dev/serial/by-path /dev/ttyACM* /dev/ttyUSB* /dev/ttyCH9344USB* 2>/dev/null || true"
  run_eval_to_log "${log_dir}/meta/can0.txt" "ip -details link show can0 2>&1 || true"
  run_eval_to_log "${log_dir}/meta/top.txt" "top -b -n 1 -o %CPU | head -60"
  run_eval_to_log "${log_dir}/meta/ps_cpu.txt" "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -80"
  run_eval_to_log "${log_dir}/meta/free.txt" "free -h"
  run_eval_to_log "${log_dir}/meta/df.txt" "df -h '${WS_ROOT}' '${HOME}'"
  run_shell_to_log "${log_dir}/meta/ros_nodes.txt" ros2 node list
  run_shell_to_log "${log_dir}/meta/ros_topics.txt" ros2 topic list
  run_shell_to_log "${log_dir}/meta/ros_services.txt" ros2 service list

  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    local name
    name="$(safe_name "${topic}")"
    run_eval_to_log "${log_dir}/topics/${name}.info.txt" "source /opt/ros/humble/setup.bash; source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true; ros2 topic info -v '${topic}'"
    topic_once_to_log "${topic}" "${log_dir}/topics/${name}.once.txt"
    topic_hz_to_log "${topic}" "${DEFAULT_HZ_SECONDS}" "${log_dir}/hz/${name}.hz.txt"
  done < <(critical_topics)

  # Camera frames are intentionally sampled by rate only. Saving one full
  # 1920x1080 image in every snapshot creates large, low-value text logs.
  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    local name
    name="$(safe_name "${topic}")"
    run_eval_to_log "${log_dir}/topics/${name}.info.txt" "source /opt/ros/humble/setup.bash; source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true; ros2 topic info -v '${topic}'"
    topic_hz_to_log "${topic}" "${DEFAULT_HZ_SECONDS}" "${log_dir}/hz/${name}.hz.txt"
  done < <(rate_only_topics)

  log "snapshot complete: ${log_dir}"
}

cmd_hz() {
  source_ros
  local seconds="${1:-${DEFAULT_HZ_SECONDS}}"
  log "sampling topic rates for ${seconds}s each"
  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    echo
    echo "## ${topic}"
    timeout --signal=INT --kill-after=2 "${seconds}" ros2 topic hz "${topic}" 2>/dev/null || true
  done < <(critical_topics)

  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    echo
    echo "## ${topic}"
    timeout --signal=INT --kill-after=2 "${seconds}" ros2 topic hz "${topic}" 2>/dev/null || true
  done < <(rate_only_topics)
}

cmd_camera_yolo() {
  source_ros
  local seconds="${1:-${CAMERA_YOLO_ACCEPTANCE_SECONDS}}"
  local rate_topics=(
    "/sensing/camera/econ_front/image_rect/compressed"
    "/sensing/camera/econ_front/camera_info"
    "/perception/camera/detections_2d"
    "/perception/camera/yolo_image"
  )
  local info_topics=(
    "${rate_topics[@]}"
    "/sensing/camera/econ_front/dummy_active"
  )
  local topic pid
  local pids=()
  local payload_probe_pid

  if [[ ! "${seconds}" =~ ^[0-9]+$ ]]; then
    die "camera-yolo seconds must be an integer"
  fi
  if (( seconds < CAMERA_YOLO_MIN_SECONDS )); then
    warn "camera-yolo needs at least ${CAMERA_YOLO_MIN_SECONDS}s for DDS discovery; using ${CAMERA_YOLO_MIN_SECONDS}s"
    seconds="${CAMERA_YOLO_MIN_SECONDS}"
  fi

  log "checking front camera and YOLO for ${seconds}s per topic"
  echo "## nodes"
  ros2 node list | grep -E '/sensing/camera/econ_front/camera_front_publisher|/perception/yolov9mit' || true

  for topic in "${info_topics[@]}"; do
    echo
    echo "## ${topic}"
    ros2 topic info --verbose "${topic}" || true
  done

  echo
  echo "## component native libraries"
  while IFS= read -r pid; do
    [[ -r "/proc/${pid}/maps" ]] || continue
    echo "-- pid=${pid}"
    awk '{print $6}' "/proc/${pid}/maps" 2>/dev/null |
      grep -E '/(libopencv|libnvjpeg|libcuda)' |
      sort -u || true
  done < <(pgrep -f 'component_container|yolov9mit' || true)

  echo
  echo "## simultaneous rate samples"
  ros2 run camrod_bringup camera_payload_probe.py \
    --duration "${seconds}" \
    --min-rate-hz 5.0 &
  payload_probe_pid="$!"
  for topic in "${rate_topics[@]}"; do
    (
      timeout --signal=INT --kill-after=2 "${seconds}" \
        ros2 topic hz "${topic}" 2>/dev/null | sed "s#^#[${topic}] #"
    ) &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do
    wait "${pid}" || true
  done

  if ! wait "${payload_probe_pid}"; then
    die "front-camera payload validation failed"
  fi
  log "YOLO debug images are subscriber-gated; camera-yolo creates the subscriber that enables them"
}

cmd_profile() {
  source_ros
  local seconds="${1:-300}"
  local label="${2:-baseline}"
  [[ "${seconds}" =~ ^[0-9]+$ ]] ||
    die "profile seconds must be an integer"
  (( seconds >= 10 )) ||
    die "profile duration must be at least 10 seconds"
  label="$(safe_name "${label}")"

  local log_dir
  log_dir="$(new_log_dir)"
  mkdir -p "${log_dir}/hz" "${log_dir}/meta"
  printf '%s\n' "${label}" >"${log_dir}/meta/profile_label.txt"
  run_shell_to_log "${log_dir}/meta/date.txt" date --iso-8601=seconds
  run_eval_to_log \
    "${log_dir}/meta/git.txt" \
    "cd '${SRC_ROOT}' && git status --short --branch && git log -1 --oneline"
  run_eval_to_log \
    "${log_dir}/meta/ps_before.txt" \
    "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -100"

  local pids=()
  local topic name pid
  if command -v tegrastats >/dev/null 2>&1; then
    timeout --signal=INT --kill-after=2 "${seconds}" \
      tegrastats --interval 1000 >"${log_dir}/meta/tegrastats.txt" 2>&1 &
    pids+=("$!")
  else
    warn "tegrastats is unavailable; GPU telemetry will be missing"
  fi
  timeout --signal=INT --kill-after=2 "${seconds}" \
    top -b -d 1 -o %CPU >"${log_dir}/meta/top.txt" 2>&1 &
  pids+=("$!")

  while IFS= read -r topic; do
    [[ -n "${topic}" ]] || continue
    name="$(safe_name "${topic}")"
    (
      timeout --signal=INT --kill-after=2 "${seconds}" \
        ros2 topic hz "${topic}" >"${log_dir}/hz/${name}.hz.txt" 2>&1
    ) &
    pids+=("$!")
  done < <(profile_topics)

  log "profiling '${label}' for ${seconds}s into ${log_dir}"
  for pid in "${pids[@]}"; do
    wait "${pid}" || true
  done
  run_eval_to_log \
    "${log_dir}/meta/ps_after.txt" \
    "ps -eo pid,ppid,pcpu,pmem,comm,args --sort=-pcpu | head -100"
  run_shell_to_log "${log_dir}/meta/free.txt" free -h
  run_shell_to_log "${log_dir}/meta/df.txt" df -h "${WS_ROOT}" "${HOME}"
  log "profile complete: ${log_dir}"
}

cmd_pose_latency() {
  source_ros
  local seconds="${1:-60}"
  local output_json="${2:-}"
  local log_dir

  [[ "${seconds}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    die "pose-latency seconds must be numeric"
  awk -v value="${seconds}" 'BEGIN {exit !(value >= 2.0)}' ||
    die "pose-latency duration must be at least 2 seconds"
  [[ $# -le 2 ]] || die "pose-latency accepts only [seconds] [output_json]"

  if [[ -z "${output_json}" ]]; then
    log_dir="$(new_log_dir)"
    output_json="${log_dir}/pose_latency.json"
  fi
  mkdir -p "$(dirname "${output_json}")"

  log "measuring the complete localization chain for ${seconds}s"
  ros2 run camrod_bringup pose_latency_probe.py \
    --duration "${seconds}" \
    --output-json "${output_json}"
  log "pose latency report: ${output_json}"
}

collect_watch_sample() {
  local topic="$1"
  local output_file="$2"
  timeout 1.5 ros2 topic echo --once "${topic}" 2>/dev/null |
    sed -n '1,32p' >"${output_file}" || true
  [[ -s "${output_file}" ]] || printf '%s\n' "(no sample)" >"${output_file}"
}

cmd_watch() {
  source_ros
  local interval="${1:-${DEFAULT_WATCH_INTERVAL}}"
  local watch_dir
  watch_dir="$(mktemp -d /tmp/camrod_field_watch.XXXXXX)"
  trap 'rm -rf -- "${watch_dir}"' EXIT
  trap 'exit 130' INT TERM
  local topics=(
    "/system/status"
    "/service/state"
    "/localization/mode"
    "/planning/state_machine/state"
    "/planning/engage"
    "/planning/mission_engage"
    "/platform/drive_enable"
    "/control/planning_engaged"
    "/control/command_enabled"
    "/control/cmd_vel_safety_gate/status"
    "/platform/status"
  )
  local labels=(
    "system"
    "service_state"
    "localization_mode"
    "planning_state"
    "manual_engage"
    "mission_engage"
    "platform_drive_enable"
    "planning_engaged"
    "control_command_enabled"
    "control_gate_status"
    "platform_status"
  )
  local i pid
  local pids=()
  while true; do
    pids=()
    for i in "${!topics[@]}"; do
      collect_watch_sample \
        "${topics[${i}]}" "${watch_dir}/${i}.txt" &
      pids+=("$!")
    done
    for pid in "${pids[@]}"; do
      wait "${pid}" || true
    done

    clear || true
    date --iso-8601=seconds
    # HH_260730 - Collect all safety topics concurrently. The old sequential
    # two-second echo loop could display state that was almost 20 seconds old.
    for i in "${!topics[@]}"; do
      echo "-- ${labels[${i}]}: ${topics[${i}]}"
      cat "${watch_dir}/${i}.txt"
    done
    echo
    echo "-- top cpu"
    ps -eo pcpu,pmem,comm,args --sort=-pcpu | head -12
    echo
    echo "Press Ctrl+C to stop."
    sleep "${interval}"
  done
}

cmd_launch() {
  source_ros
  local log_dir
  log_dir="$(new_log_dir)"
  mkdir -p "${log_dir}"
  log "launch log: ${log_dir}/bringup.log"
  ros2 launch camrod_bringup bringup.launch.py sim:=false "$@" 2>&1 | tee "${log_dir}/bringup.log"
}

# HH_260730 - Field gate controls must use the same avg_msgs contract as the
# UI and cmd_vel safety gate.  The old std_msgs publisher was type-incompatible
# and could print a false success while leaving the real gate unchanged.
require_gate_subscriber() {
  local topic="$1"
  local expected_type="avg_msgs/msg/AvgBool"
  local actual_type info subscription_count

  actual_type="$(timeout 3 ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual_type}" != "${expected_type}" ]]; then
    warn "${topic} type is '${actual_type:-missing}', expected ${expected_type}"
    return 1
  fi

  info="$(timeout 3 ros2 topic info "${topic}" 2>/dev/null || true)"
  subscription_count="$(
    awk -F': *' '/^Subscription count:/ {print $2}' <<<"${info}" | tail -1
  )"
  if [[ ! "${subscription_count}" =~ ^[0-9]+$ ]]; then
    warn "cannot determine subscriber count for ${topic}"
    return 1
  fi
  if (( subscription_count == 0 )); then
    warn "${topic} has no subscriber; refusing to report a gate change"
    return 1
  fi
}

publish_gate_bool() {
  local topic="$1"
  local value="$2"

  require_gate_subscriber "${topic}" || return 1
  timeout 5 ros2 topic pub --once \
    "${topic}" avg_msgs/msg/AvgBool "{data: ${value}}" >/dev/null ||
    {
      warn "failed to publish ${value} to ${topic}"
      return 1
    }
}

verify_command_disabled() {
  local command_sample planning_sample
  command_sample="$(
    timeout 4 ros2 topic echo --once /control/command_enabled 2>/dev/null || true
  )"
  planning_sample="$(
    timeout 4 ros2 topic echo --once /control/planning_engaged 2>/dev/null || true
  )"
  if ! grep -Eq \
    '^[[:space:]]*data:[[:space:]]*false[[:space:]]*$' \
    <<<"${command_sample}"
  then
    warn "gate commands were sent but /control/command_enabled did not confirm false"
    return 1
  fi
  if ! grep -Eq \
    '^[[:space:]]*data:[[:space:]]*false[[:space:]]*$' \
    <<<"${planning_sample}"
  then
    warn "gate commands were sent but /control/planning_engaged did not confirm false"
    return 1
  fi
}

cmd_stop_gates() {
  source_ros
  log "closing software engage and drive-enable gates"
  local failures=0
  local topic
  # HH_260730 - Attempt every close command even if one owner is absent. An
  # early exit here could leave a later, still-live gate open.
  for topic in \
    /planning/engage \
    /planning/mission_engage \
    /platform/drive_enable
  do
    publish_gate_bool "${topic}" false || failures=1
  done
  verify_command_disabled || failures=1
  (( failures == 0 )) ||
    die "one or more gates could not be closed and verified; keep the physical e-stop engaged"
  log "software gates closed and /control/command_enabled=false confirmed"
}

cmd_enable_gates() {
  source_ros
  local allow=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --allow-motion) allow=1; shift ;;
      *) die "unknown enable-gates arg: $1" ;;
    esac
  done
  [[ "${allow}" -eq 1 ]] || die "enable-gates requires --allow-motion"
  warn "opening software gates; physical supervision and e-stop are still required"
  publish_gate_bool /platform/drive_enable true ||
    die "platform drive-enable gate was not opened"
  publish_gate_bool /planning/engage true ||
    die "manual planning gate was not opened"
  log "software gates requested open"
}

cmd="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${cmd}" in
  help|-h|--help) usage ;;
  config) cmd_config "$@" ;;
  snapshot) cmd_snapshot "$@" ;;
  record-recovery) cmd_record_recovery "$@" ;;
  hz) cmd_hz "$@" ;;
  camera-yolo) cmd_camera_yolo "$@" ;;
  profile) cmd_profile "$@" ;;
  pose-latency) cmd_pose_latency "$@" ;;
  watch) cmd_watch "$@" ;;
  launch) cmd_launch "$@" ;;
  stop-gates) cmd_stop_gates "$@" ;;
  enable-gates) cmd_enable_gates "$@" ;;
  *) usage; die "unknown command: ${cmd}" ;;
esac
