#!/usr/bin/env bash
# Visible-Robot-UI manual 4WS evidence orchestrator. It never starts/stops CARLA/ROS.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PYTHON_RUNNER="${SCRIPT_DIR}/manual_4ws_evidence.py"
CAPTURE_SCRIPT="${SCRIPT_DIR}/capture_ui_evidence.sh"
WHEEL_RECORDER="${SCRIPT_DIR}/record_physical_wheel_telemetry.py"
WHEEL_SUMMARIZER="${SCRIPT_DIR}/summarize_physical_wheel_telemetry.py"
SCENARIOS=(straight turn crab zero_turn)
CAPTURE_DURATION_SECONDS=86400
MINIMUM_CAPTURE_SECONDS=12
RECORDER_SHUTDOWN_SECONDS=10
CAPTURE_SHUTDOWN_SECONDS=45
TERMINATE_SHUTDOWN_SECONDS=5

usage() {
  cat <<'EOF'
Usage:
  run_manual_4ws_evidence.sh [plan]
  run_manual_4ws_evidence.sh run --output-root /absolute/new/path [options]

Actions:
  plan  print the complete workflow without querying UI, ROS, CARLA or X11
  run   use the already-running rendered stack and visible Robot UI to collect
        straight, turn, crab and zero-turn evidence

Options:
  --output-root PATH          new or empty absolute evidence root (required)
  --ui-url URL                Robot UI URL (default: CAMROD_UI_URL)
  --operator-cdp-url URL      local visible-page CDP URL (default: 127.0.0.1:9224)
  --display DISPLAY           X11 display (default: DISPLAY)
  --xauthority PATH           readable Xauthority path (default: XAUTHORITY)
  --capture-fps N             actual X11 capture rate, 1..30 (default: 5)
  --gif-fps N                 derived GIF rate, 1..20 (default: 8)
  --derived-width PX          PNG/GIF panel width, 320..1920 (default: 1600)
  --wheel-rate-hz HZ          physical wheel readback rate, 1..100 (default: 10)
  --hold-s S                  each positive/inverse key hold, 2..15 (default: 4)
  --zero-hold-s S             zero interval between directions, .5..10 (default: 2)
  --settle-s S                final zero observation, .5..10 (default: 2)
  --retain-source-video BOOL  true or false (default: false)
  -h, --help                  show this help

The script does not launch, stop, respawn, teleport, publish ROS, or call a
CARLA control API. It requires the server, bridge, pacer, Ranger, CAMROD and a
single visible Robot UI CDP page to be running already. Its only motion source
is actual CDP pointer/text/keyboard input delivered to that page. ROS and CARLA
wheel access are passive subscribers/readback recorders. Before ARM, a unique
temporary title proves that the CDP page is the exact X11 Robot UI window pinned
for every pixel capture; the normal title is restored before motion.

Output:
  <root>/session/session_setup.json
  <root>/{straight,turn,crab,zero_turn}/
    scenario_manifest.json, ui_interactions.json, ros_trace.jsonl,
    physical_wheels.jsonl, wheel_summary/, visual/*.png, visual/*.gif
  <root>/session_teardown/session_teardown.json
  <root>/manual_4ws_summary.{json,csv}, manual_4ws_report.md, SHA256SUMS
EOF
}

die() {
  printf '[manual-4ws] ERROR: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '[manual-4ws] %s\n' "$*"
}

ACTION="plan"
OUTPUT_ROOT=""
UI_URL=""
OPERATOR_CDP_URL=""
DISPLAY_VALUE="${DISPLAY:-}"
XAUTHORITY_VALUE="${XAUTHORITY:-}"
CAPTURE_FPS="${CAMROD_MANUAL_EVIDENCE_CAPTURE_FPS:-5}"
GIF_FPS="${CAMROD_MANUAL_EVIDENCE_GIF_FPS:-8}"
DERIVED_WIDTH="${CAMROD_MANUAL_EVIDENCE_DERIVED_WIDTH:-1600}"
WHEEL_RATE_HZ="${CAMROD_MANUAL_EVIDENCE_WHEEL_RATE_HZ:-10.0}"
HOLD_S="${CAMROD_MANUAL_EVIDENCE_HOLD_S:-4.0}"
ZERO_HOLD_S="${CAMROD_MANUAL_EVIDENCE_ZERO_HOLD_S:-2.0}"
SETTLE_S="${CAMROD_MANUAL_EVIDENCE_SETTLE_S:-2.0}"
RETAIN_SOURCE_VIDEO="${CAMROD_MANUAL_EVIDENCE_RETAIN_SOURCE_VIDEO:-false}"

if [[ $# -gt 0 ]]; then
  case "$1" in
    plan|run) ACTION="$1"; shift ;;
    -h|--help|help) usage; exit 0 ;;
  esac
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-root|--ui-url|--operator-cdp-url|--display|--xauthority|\
    --capture-fps|--gif-fps|--derived-width|--wheel-rate-hz|--hold-s|\
    --zero-hold-s|--settle-s|--retain-source-video)
      [[ $# -ge 2 ]] || die "missing value for $1"
      option="$1"; value="$2"; shift 2
      case "${option}" in
        --output-root) OUTPUT_ROOT="${value}" ;;
        --ui-url) UI_URL="${value}" ;;
        --operator-cdp-url) OPERATOR_CDP_URL="${value}" ;;
        --display) DISPLAY_VALUE="${value}" ;;
        --xauthority) XAUTHORITY_VALUE="${value}" ;;
        --capture-fps) CAPTURE_FPS="${value}" ;;
        --gif-fps) GIF_FPS="${value}" ;;
        --derived-width) DERIVED_WIDTH="${value}" ;;
        --wheel-rate-hz) WHEEL_RATE_HZ="${value}" ;;
        --hold-s) HOLD_S="${value}" ;;
        --zero-hold-s) ZERO_HOLD_S="${value}" ;;
        --settle-s) SETTLE_S="${value}" ;;
        --retain-source-video) RETAIN_SOURCE_VIDEO="${value}" ;;
      esac
      ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

if [[ "${ACTION}" == "plan" ]]; then
  cat <<EOF
[manual-4ws] PLAN ONLY -- no UI, ROS, CARLA, X11, or filesystem query occurred.

Prerequisites already running:
  server -> bridge -> pacer -> Ranger spawn -> CAMROD -> visible Robot UI CDP 9224

Evidence order:
  1. Actual 1.65 s administrator-entry long press, admin form input, Camera tab.
  2. Actual ARM click; page-owned /ws/manual-drive remains the sole authority.
  3. For each scenario, start passive X11 and physical-wheel recorders, then:
       straight:  W ${HOLD_S}s -> ZERO -> S ${HOLD_S}s
       turn:      W+A ${HOLD_S}s -> ZERO -> S+D ${HOLD_S}s
       crab:      Z ${HOLD_S}s -> ZERO -> C ${HOLD_S}s
       zero_turn: A ${HOLD_S}s -> ZERO -> D ${HOLD_S}s
  4. Actual Space ZERO and Escape DISARM.
  5. Offline acceptance, manifests, PNG/GIF, wheel summaries and SHA256SUMS.

Example:
  ${0@Q} run --output-root /absolute/new/manual-4ws-evidence
EOF
  exit 0
fi

validate_settings() {
  python3 - "${CAPTURE_FPS}" "${GIF_FPS}" "${DERIVED_WIDTH}" \
      "${WHEEL_RATE_HZ}" "${HOLD_S}" "${ZERO_HOLD_S}" "${SETTLE_S}" <<'PY'
import math
import sys

capture, gif, width, wheel, hold, zero, settle = sys.argv[1:]
try:
    capture_i, gif_i, width_i = int(capture), int(gif), int(width)
    wheel_f, hold_f, zero_f, settle_f = map(float, (wheel, hold, zero, settle))
except ValueError as error:
    raise SystemExit(f"[manual-4ws] ERROR: invalid numeric option: {error}")
checks = (
    (1 <= capture_i <= 30, "capture FPS must be in [1, 30]"),
    (1 <= gif_i <= 20, "GIF FPS must be in [1, 20]"),
    (320 <= width_i <= 1920, "derived width must be in [320, 1920]"),
    (math.isfinite(wheel_f) and 1 <= wheel_f <= 100, "wheel rate must be in [1, 100]"),
    (math.isfinite(hold_f) and 2 <= hold_f <= 15, "hold must be in [2, 15]"),
    (math.isfinite(zero_f) and .5 <= zero_f <= 10, "zero hold must be in [.5, 10]"),
    (math.isfinite(settle_f) and .5 <= settle_f <= 10, "settle must be in [.5, 10]"),
)
for accepted, message in checks:
    if not accepted:
        raise SystemExit(f"[manual-4ws] ERROR: {message}")
PY
}

validate_output_root() {
  [[ -n "${OUTPUT_ROOT}" ]] || die "run requires --output-root"
  [[ "${OUTPUT_ROOT}" == /* && "${OUTPUT_ROOT}" != "/" ]] || \
    die "--output-root must be absolute and not /: ${OUTPUT_ROOT}"
  [[ ! -L "${OUTPUT_ROOT}" ]] || die "--output-root must not be a symlink"
  if [[ -e "${OUTPUT_ROOT}" && ! -d "${OUTPUT_ROOT}" ]]; then
    die "--output-root exists but is not a directory"
  fi
  if [[ -d "${OUTPUT_ROOT}" && -n "$(find "${OUTPUT_ROOT}" -mindepth 1 -print -quit)" ]]; then
    die "--output-root must be empty; evidence is never overwritten"
  fi
}

validate_settings
validate_output_root

# env.sh defines paths/helpers only and does not start or command a process.
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"
UI_URL="${UI_URL:-${CAMROD_UI_URL}}"
OPERATOR_CDP_URL="${OPERATOR_CDP_URL:-${CAMROD_OPERATOR_CDP_URL}}"

[[ -f "${PYTHON_RUNNER}" ]] || die "Python runner not found: ${PYTHON_RUNNER}"
[[ -x "${CAPTURE_SCRIPT}" ]] || die "capture helper is not executable: ${CAPTURE_SCRIPT}"
[[ -f "${WHEEL_RECORDER}" ]] || die "wheel recorder not found: ${WHEEL_RECORDER}"
[[ -f "${WHEEL_SUMMARIZER}" ]] || die "wheel summarizer not found: ${WHEEL_SUMMARIZER}"
[[ -n "${DISPLAY_VALUE}" ]] || die "DISPLAY is empty; pass --display from the graphical session"
if [[ -n "${XAUTHORITY_VALUE}" && ! -r "${XAUTHORITY_VALUE}" ]]; then
  die "XAUTHORITY is not readable: ${XAUTHORITY_VALUE}"
fi
case "${RETAIN_SOURCE_VIDEO}" in true|false) ;; *) die "retain-source-video must be true or false" ;; esac
[[ "${CARLA_RENDER_MODE}" != "nullrhi" ]] || die "actual X11 evidence requires rendered CARLA"

virtual_carla_require_dds_transport
virtual_carla_verify_external_prefixes
virtual_carla_source_ros true
virtual_carla_use_python_egg
virtual_carla_verify_package_prefix camrod_carla_adapter "${CAMROD_WS_ROOT}/install"
python3 -c 'import rclpy, websocket' || die "rclpy/websocket-client is unavailable"

capture_validate=(
  "${CAPTURE_SCRIPT}" validate
  --display "${DISPLAY_VALUE}"
  --ui-window-title "Robot UI"
  --ui-kind operator
)
if [[ -n "${XAUTHORITY_VALUE}" ]]; then
  capture_validate+=(--xauthority "${XAUTHORITY_VALUE}")
fi
log "preflight: validating already-visible CarlaUE4 + Robot UI windows"
capture_validation_output="$("${capture_validate[@]}")"
printf '%s\n' "${capture_validation_output}"
CARLA_WINDOW_ID="$(sed -n 's/.*geometry VERIFIED: carla=\(0x[0-9a-fA-F]*\) .*/\1/p' <<<"${capture_validation_output}")"
UI_WINDOW_ID="$(sed -n 's/.*geometry VERIFIED: ui=\(0x[0-9a-fA-F]*\) .*/\1/p' <<<"${capture_validation_output}")"
[[ "${CARLA_WINDOW_ID}" =~ ^0x[0-9a-fA-F]+$ ]] || \
  die "could not bind the validated CarlaUE4 X11 window id"
[[ "${UI_WINDOW_ID}" =~ ^0x[0-9a-fA-F]+$ ]] || \
  die "could not bind the validated Robot UI X11 window id"
[[ "${CARLA_WINDOW_ID,,}" != "${UI_WINDOW_ID,,}" ]] || \
  die "CarlaUE4 and Robot UI resolved to the same X11 window"
log "CDP commands and every capture are pinned to Robot UI X11 id ${UI_WINDOW_ID}"

mkdir -p -- "${OUTPUT_ROOT}"
OUTPUT_ROOT="$(readlink -m "${OUTPUT_ROOT}")"
TEMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/camrod-manual-4ws.XXXXXX")"
CURRENT_CAPTURE_PID=""
CURRENT_RECORDER_PID=""
CURRENT_CAPTURE_FD=""
CURRENT_CAPTURE_FIFO=""
CURRENT_CAPTURE_READY_MONOTONIC=""
CURRENT_SCENARIO=""
MANUAL_ARMED=false
RUN_COMPLETE=false
FAILURE_REASON="unexpected runner exit"

wait_for_file_from_process() {
  local pid="$1" path="$2" label="$3" attempts="$4" index
  for ((index=0; index<attempts; index+=1)); do
    [[ -s "${path}" ]] && return 0
    kill -0 "${pid}" 2>/dev/null || {
      wait "${pid}" 2>/dev/null || true
      return 1
    }
    sleep 0.1
  done
  log "${label} did not create ${path} in time"
  return 1
}

process_is_running() {
  local pid="$1" state
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null | awk 'NR == 1 {print $1}')"
  [[ -n "${state}" && "${state}" != Z* ]]
}

bounded_reap() {
  local pid="$1" label="$2" graceful_seconds="$3"
  local deadline status forced=false
  deadline=$((SECONDS + graceful_seconds))
  while process_is_running "${pid}"; do
    if (( SECONDS >= deadline )); then
      forced=true
      log "${label} exceeded ${graceful_seconds}s graceful shutdown; sending TERM"
      kill -TERM "${pid}" 2>/dev/null || true
      break
    fi
    sleep 0.1
  done
  if [[ "${forced}" == "true" ]]; then
    deadline=$((SECONDS + TERMINATE_SHUTDOWN_SECONDS))
    while process_is_running "${pid}"; do
      if (( SECONDS >= deadline )); then
        log "${label} ignored TERM for ${TERMINATE_SHUTDOWN_SECONDS}s; sending KILL"
        kill -KILL "${pid}" 2>/dev/null || true
        break
      fi
      sleep 0.1
    done
  fi
  if wait "${pid}"; then
    status=0
  else
    status=$?
  fi
  [[ "${forced}" == "false" && ${status} -eq 0 ]]
}

wait_for_minimum_capture() {
  local started="$1"
  [[ -n "${started}" ]] || return 0
  python3 - "${started}" "${MINIMUM_CAPTURE_SECONDS}" "${CAPTURE_FPS}" <<'PY'
import math
import sys
import time

started, minimum, fps = map(float, sys.argv[1:])
if not all(math.isfinite(value) for value in (started, minimum, fps)):
    raise SystemExit("invalid monotonic capture timing")
# The extra frame/encoder margin makes the ffprobe duration safely >= 12 s,
# including a 1 fps run and fractional process/file-start timing.
target = minimum + max(1.0, 2.0 / fps)
remaining = target - (time.monotonic() - started)
if remaining > 0.0:
    time.sleep(remaining)
PY
}

finish_observers() {
  local capture_status=0 recorder_status=0
  if [[ -n "${CURRENT_CAPTURE_PID}" ]] && kill -0 "${CURRENT_CAPTURE_PID}" 2>/dev/null; then
    wait_for_minimum_capture "${CURRENT_CAPTURE_READY_MONOTONIC}"
    [[ -z "${CURRENT_CAPTURE_FD}" ]] || printf 'q' >&"${CURRENT_CAPTURE_FD}" || true
  fi
  if [[ -n "${CURRENT_CAPTURE_FD}" ]]; then
    exec {CURRENT_CAPTURE_FD}>&-
    CURRENT_CAPTURE_FD=""
  fi
  if [[ -n "${CURRENT_RECORDER_PID}" ]] && kill -0 "${CURRENT_RECORDER_PID}" 2>/dev/null; then
    kill -INT "${CURRENT_RECORDER_PID}" 2>/dev/null
  fi
  if [[ -n "${CURRENT_RECORDER_PID}" ]]; then
    bounded_reap "${CURRENT_RECORDER_PID}" "physical wheel recorder" \
      "${RECORDER_SHUTDOWN_SECONDS}" || recorder_status=$?
  fi
  if [[ -n "${CURRENT_CAPTURE_PID}" ]]; then
    bounded_reap "${CURRENT_CAPTURE_PID}" "X11 capture/derivative process" \
      "${CAPTURE_SHUTDOWN_SECONDS}" || capture_status=$?
  fi
  CURRENT_RECORDER_PID=""
  CURRENT_CAPTURE_PID=""
  [[ -z "${CURRENT_CAPTURE_FIFO}" ]] || rm -f -- "${CURRENT_CAPTURE_FIFO}"
  CURRENT_CAPTURE_FIFO=""
  CURRENT_CAPTURE_READY_MONOTONIC=""
  (( recorder_status == 0 && capture_status == 0 ))
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  # A recorder/ffmpeg finalizer can block or hang.  ZERO/DISARM therefore
  # always precedes child shutdown in the asynchronous cleanup path.
  if [[ "${MANUAL_ARMED}" == "true" ]]; then
    failsafe_dir="${OUTPUT_ROOT}/failsafe_teardown"
    if [[ ! -e "${failsafe_dir}" ]]; then
      mkdir -- "${failsafe_dir}"
      python3 "${PYTHON_RUNNER}" teardown \
        --output-dir "${failsafe_dir}" \
        --ui-url "${UI_URL}" \
        --operator-cdp-url "${OPERATOR_CDP_URL}" \
        --x11-window-id "${UI_WINDOW_ID}" \
        --display "${DISPLAY_VALUE}" \
        --xauthority "${XAUTHORITY_VALUE}" \
        --role-name "${CARLA_ROLE_NAME}" || true
    fi
  fi
  finish_observers || true
  if [[ "${RUN_COMPLETE}" != "true" && ! -e "${OUTPUT_ROOT}/manual_4ws_summary.json" ]]; then
    python3 "${PYTHON_RUNNER}" summarize \
      --output-root "${OUTPUT_ROOT}" --status FAIL \
      --failure-scenario "${CURRENT_SCENARIO}" \
      --failure-reason "${FAILURE_REASON}" || true
  fi
  if [[ -n "${TEMP_ROOT:-}" && "${TEMP_ROOT}" != "/" && -d "${TEMP_ROOT}" ]]; then
    rm -rf -- "${TEMP_ROOT}"
  fi
  exit "${status}"
}

on_signal() {
  FAILURE_REASON="manual evidence runner interrupted"
  trap - INT TERM
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

mkdir -- "${OUTPUT_ROOT}/session"
FAILURE_REASON="visible administrator login or manual ARM failed"
# Treat the whole setup attempt as requiring a fail-safe teardown.  The Python
# helper already handles a partially successful ARM click, while this outer
# guard remains a second safety layer if that helper itself is interrupted.
MANUAL_ARMED=true
python3 "${PYTHON_RUNNER}" setup \
  --output-dir "${OUTPUT_ROOT}/session" \
  --ui-url "${UI_URL}" \
  --operator-cdp-url "${OPERATOR_CDP_URL}" \
  --x11-window-id "${UI_WINDOW_ID}" \
  --display "${DISPLAY_VALUE}" \
  --xauthority "${XAUTHORITY_VALUE}" \
  --role-name "${CARLA_ROLE_NAME}"

for scenario in "${SCENARIOS[@]}"; do
  CURRENT_SCENARIO="${scenario}"
  scenario_dir="${OUTPUT_ROOT}/${scenario}"
  mkdir -- "${scenario_dir}"
  wheel_output="${scenario_dir}/physical_wheels.jsonl"
  wheel_manifest="${scenario_dir}/physical_wheels.manifest.json"
  wheel_summary_dir="${scenario_dir}/wheel_summary"
  visual_dir="${scenario_dir}/visual"
  egg_cache="${TEMP_ROOT}/${scenario}-egg-cache"
  capture_fifo="${TEMP_ROOT}/${scenario}-capture.fifo"
  mkdir -- "${egg_cache}"
  mkfifo -- "${capture_fifo}"

  log "${scenario}: starting passive physical wheel recorder"
  (
    export PYTHON_EGG_CACHE="${egg_cache}"
    exec python3 "${WHEEL_RECORDER}" \
      --output "${wheel_output}" \
      --manifest "${wheel_manifest}" \
      --host "${CARLA_HOST}" \
      --port "${CARLA_PORT}" \
      --rate-hz "${WHEEL_RATE_HZ}"
  ) >"${scenario_dir}/physical_wheels.log" 2>&1 &
  CURRENT_RECORDER_PID=$!
  FAILURE_REASON="${scenario} physical wheel recorder failed before motion"
  wait_for_file_from_process \
    "${CURRENT_RECORDER_PID}" "${wheel_output}" "${scenario} wheel recorder" 150

  CURRENT_CAPTURE_FIFO="${capture_fifo}"
  exec {CURRENT_CAPTURE_FD}<>"${capture_fifo}"
  capture_command=(
    "${CAPTURE_SCRIPT}" capture
    --output-dir "${visual_dir}"
    --duration-seconds "${CAPTURE_DURATION_SECONDS}"
    --capture-fps "${CAPTURE_FPS}"
    --gif-fps "${GIF_FPS}"
    --derived-width "${DERIVED_WIDTH}"
    --retain-source-video "${RETAIN_SOURCE_VIDEO}"
    --allow-short-capture true
    --display "${DISPLAY_VALUE}"
    --ui-window-title "Robot UI"
    --ui-kind operator
    --carla-window-id "${CARLA_WINDOW_ID}"
    --ui-window-id "${UI_WINDOW_ID}"
  )
  if [[ -n "${XAUTHORITY_VALUE}" ]]; then
    capture_command+=(--xauthority "${XAUTHORITY_VALUE}")
  fi
  log "${scenario}: starting actual side-by-side X11 capture"
  "${capture_command[@]}" <"${capture_fifo}" >"${scenario_dir}/capture.log" 2>&1 &
  CURRENT_CAPTURE_PID=$!
  FAILURE_REASON="${scenario} X11 capture failed before motion"
  wait_for_file_from_process \
    "${CURRENT_CAPTURE_PID}" "${visual_dir}/carla_camrod_desktop.mp4" \
    "${scenario} X11 capture" 300
  CURRENT_CAPTURE_READY_MONOTONIC="$(python3 -c 'import time; print(time.monotonic())')"

  process_is_running "${CURRENT_RECORDER_PID}" || \
    die "${scenario} physical wheel recorder exited immediately before motion"
  process_is_running "${CURRENT_CAPTURE_PID}" || \
    die "${scenario} X11 capture exited immediately before motion"

  FAILURE_REASON="${scenario} visible Robot UI or ROS observation failed"
  log "${scenario}: issuing only real Robot UI keyboard events"
  python3 "${PYTHON_RUNNER}" scenario \
    --scenario "${scenario}" \
    --output-dir "${scenario_dir}" \
    --ui-url "${UI_URL}" \
    --operator-cdp-url "${OPERATOR_CDP_URL}" \
    --x11-window-id "${UI_WINDOW_ID}" \
    --display "${DISPLAY_VALUE}" \
    --xauthority "${XAUTHORITY_VALUE}" \
    --role-name "${CARLA_ROLE_NAME}" \
    --hold-s "${HOLD_S}" \
    --zero-hold-s "${ZERO_HOLD_S}" \
    --settle-s "${SETTLE_S}"

  FAILURE_REASON="${scenario} observer finalization failed"
  finish_observers
  log "${scenario}: validating finalized physical wheel stream"
  python3 "${WHEEL_SUMMARIZER}" \
    --input "${wheel_output}" \
    --output-dir "${wheel_summary_dir}" \
    >"${scenario_dir}/wheel_summary.log" 2>&1

  FAILURE_REASON="${scenario} offline acceptance failed"
  python3 "${PYTHON_RUNNER}" evaluate \
    --scenario "${scenario}" --scenario-dir "${scenario_dir}"
  log "${scenario}: PASS"
done

CURRENT_SCENARIO="teardown"
mkdir -- "${OUTPUT_ROOT}/session_teardown"
FAILURE_REASON="visible Robot UI ZERO/DISARM teardown failed"
python3 "${PYTHON_RUNNER}" teardown \
  --output-dir "${OUTPUT_ROOT}/session_teardown" \
  --ui-url "${UI_URL}" \
  --operator-cdp-url "${OPERATOR_CDP_URL}" \
  --x11-window-id "${UI_WINDOW_ID}" \
  --display "${DISPLAY_VALUE}" \
  --xauthority "${XAUTHORITY_VALUE}" \
  --role-name "${CARLA_ROLE_NAME}"
MANUAL_ARMED=false

CURRENT_SCENARIO=""
FAILURE_REASON="collection summary or hash generation failed"
python3 "${PYTHON_RUNNER}" summarize --output-root "${OUTPUT_ROOT}"
RUN_COMPLETE=true
log "PASS: ${OUTPUT_ROOT}/manual_4ws_summary.json"
