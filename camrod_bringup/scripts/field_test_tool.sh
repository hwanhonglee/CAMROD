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

  field_test_tool.sh hz [seconds]
      Measure the field-critical topic rates.

  field_test_tool.sh watch [seconds]
      Print a compact live status loop for diagnostics, localization, planning, and gates.

  field_test_tool.sh launch [extra launch args...]
      Start real bringup with tee logging. Example:
        field_test_tool.sh launch rviz:=true

  field_test_tool.sh stop-gates
      Publish false to software engage/drive-enable gates and call /platform/set_enabled false.

  field_test_tool.sh enable-gates --allow-motion
      Open software engage/drive-enable gates. Requires explicit --allow-motion.

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
  run_eval_to_log "${out_file}" "source /opt/ros/humble/setup.bash; source '${WS_ROOT}/install/setup.bash' 2>/dev/null || true; timeout '${seconds}' ros2 topic hz '${topic}'"
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
    "parking:camrod_bringup/config/parking:camrod_parking/config"
    "localization:camrod_bringup/config/localization:camrod_localization/config"
  )
  local item label bringup_rel package_rel bringup_dir package_dir

  cd "${SRC_ROOT}"
  for item in "${pairs[@]}"; do
    IFS=: read -r label bringup_rel package_rel <<<"${item}"
    bringup_dir="${SRC_ROOT}/${bringup_rel}"
    package_dir="${SRC_ROOT}/${package_rel}"
    compare_tree_subset "${label}" "${bringup_dir}" "${package_dir}" || rc=1
    echo
  done

  compare_install_subset "bringup/platform" \
    "${SRC_ROOT}/camrod_bringup/config/platform" \
    "${WS_ROOT}/install/camrod_bringup/share/camrod_bringup/config/platform" || rc=1
  echo
  compare_install_subset "platform" \
    "${SRC_ROOT}/camrod_platform/config" \
    "${WS_ROOT}/install/camrod_platform/share/camrod_platform/config" || rc=1

  if [[ "${rc}" -eq 0 ]]; then
    log "config sync OK"
  else
    warn "config sync mismatch detected"
  fi
  return "${rc}"
}

critical_topics() {
  cat <<'EOF'
/system/status
/system/diagnostics_agg
/localization/mode
/localization/pose
/planning/state_machine/state
/planning/engaged
/planning/global_path
/planning/local_path
/planning/cmd_vel_raw
/planning/cmd_vel
/platform/cmd_vel
/platform/status/estop
/platform/status/wheel_odometry
/planning/cost_grid/inflation
/sensing/cost_grid/lidar
/sensing/cost_grid/radar
/perception/obstacles/fused_obstacles
/perception/obstacles
/sensing/lidar/points_filtered
/sensing/lidar/filtered_cloud
/sensing/radar/front1/range
/sensing/radar/front2/range
/sensing/radar/left1/range
/sensing/radar/left2/range
/sensing/radar/right1/range
/sensing/radar/right2/range
/sensing/radar/rear/range
EOF
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
  run_eval_to_log "${log_dir}/meta/serial_ports.txt" "ls -l /dev/ttyACM* /dev/ttyCH9344USB* 2>/dev/null || true"
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
    timeout "${seconds}" ros2 topic hz "${topic}" || true
  done < <(critical_topics)
}

echo_once_short() {
  local topic="$1"
  local label="$2"
  echo "-- ${label}: ${topic}"
  timeout 2 ros2 topic echo --once "${topic}" 2>/dev/null | sed -n '1,12p' || echo "(no sample)"
}

cmd_watch() {
  source_ros
  local interval="${1:-${DEFAULT_WATCH_INTERVAL}}"
  while true; do
    clear || true
    date --iso-8601=seconds
    echo_once_short "/system/status" "system"
    echo_once_short "/localization/mode" "localization_mode"
    echo_once_short "/planning/state_machine/state" "planning_state"
    echo_once_short "/planning/engaged" "planning_engaged"
    echo_once_short "/platform/status/estop" "platform_estop"
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

cmd_stop_gates() {
  source_ros
  log "closing software engage and drive-enable gates"
  ros2 topic pub --once /planning/engage std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
  ros2 topic pub --once /planning/mission_engage std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
  ros2 topic pub --once /platform/drive_enable std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
  timeout 3 ros2 service call /platform/set_enabled std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
  log "software gates requested closed"
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
  timeout 3 ros2 service call /platform/set_enabled std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 || true
  ros2 topic pub --once /platform/drive_enable std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
  ros2 topic pub --once /planning/engage std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
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
  hz) cmd_hz "$@" ;;
  watch) cmd_watch "$@" ;;
  launch) cmd_launch "$@" ;;
  stop-gates) cmd_stop_gates "$@" ;;
  enable-gates) cmd_enable_gates "$@" ;;
  *) usage; die "unknown command: ${cmd}" ;;
esac
