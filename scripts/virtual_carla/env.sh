#!/usr/bin/env bash
# Source this file to define the portable CAMROD <-> Ranger/CARLA environment.
# It declares paths and helper functions only; no package is installed, built,
# launched, or commanded when the file is sourced.

_virtual_carla_env_dir="$(
  cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd
)"
# shellcheck disable=SC1091
source "${_virtual_carla_env_dir}/map_profiles.sh"

export CAMROD_SRC_ROOT="${CAMROD_SRC_ROOT:-$(
  cd "${_virtual_carla_env_dir}/../.." >/dev/null 2>&1 && pwd
)}"
export CAMROD_WS_ROOT="${CAMROD_WS_ROOT:-$(
  cd "${CAMROD_SRC_ROOT}/.." >/dev/null 2>&1 && pwd
)}"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-188}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# RANGER_CARLA_ROOT is the only required cross-repository anchor. The Ranger
# repository remains independent and is never added as a Git submodule.
export RANGER_CARLA_ROOT="${RANGER_CARLA_ROOT:-}"
if [[ -n "${RANGER_CARLA_ROOT}" ]]; then
  RANGER_CARLA_ROOT="$(readlink -m "${RANGER_CARLA_ROOT}")"
  export RANGER_CARLA_ROOT
fi

# A Ranger bootstrap may write machine-specific, untracked selections here.
# Source those portable selections when present, while keeping values explicitly
# exported by the caller authoritative. This is configuration only; the file is
# not allowed to launch, build, or send commands by the Ranger repository's own
# environment contract.
export RANGER_ENV_FILE="${RANGER_ENV_FILE:-${RANGER_CARLA_ROOT:+${RANGER_CARLA_ROOT}/config/environment.env}}"
_virtual_carla_config_names=(
  CAMROD_CYCLONEDDS_CONFIG
  CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES
  CAMROD_MANUAL_LINEAR_LIMIT_MPS
  CAMROD_MANUAL_LATERAL_LIMIT_MPS
  CAMROD_MANUAL_ANGULAR_LIMIT_RADPS
  CAMROD_MANUAL_DEADMAN_TIMEOUT_S
  CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE
  CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M
  CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ
  CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ
  CAMROD_CARLA_SENSOR_MIN_RATE_HZ
  CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS
  CAMROD_GUEST_UI_URL
  CAMROD_GUEST_CDP_URL
  CAMROD_CARLA_YOLO_MODEL_PATH
  CAMROD_CARLA_YOLO_DEVICE
  CAMROD_CARLA_YOLO_WORKSPACE_MIB
  CAMROD_DEVELOP_LANELET_MAP
  CAMROD_WORAKSAN_TUNED_LANELET_MAP
  CAMROD_LANELET_MAP
  RANGER_WORK_ROOT
  RANGER_ROS_WS
  CARLA_ROOT
  RANGER_UE_ROOT
  UE_ROOT
  RANGER_ROS_BRIDGE_WS
  CARLA_ROS_BRIDGE_WS
  RANGER_ROS_BRIDGE_SOURCE
  RANGER_EVIDENCE_ROOT
  RANGER_CARLA_PYTHON_EGG
  RANGER_PYTHON_EGG_CACHE
  CARLA_PYTHON_EGG
  CARLA_PYTHON_EGG_CACHE
  CAMROD_CARLA_STEP_PACING
  CAMROD_CARLA_STEP_PERIOD_SECONDS
  RANGER_BASELINE_MANIFEST
  RANGER_PHYSICAL_MANIFEST
  RANGER_SPAWN_FILE
  CAMROD_LAUNCH_SENSOR_RELAY
  CAMROD_CARLA_MAP_PROFILE
  CAMROD_VIRTUAL_CARLA_ENTRYPOINT
  CARLA_UE_MAP
  CARLA_TOWN
)
declare -A _virtual_carla_explicit_values=()
for _virtual_carla_name in "${_virtual_carla_config_names[@]}"; do
  if [[ -v "${_virtual_carla_name}" ]]; then
    _virtual_carla_explicit_values["${_virtual_carla_name}"]="${!_virtual_carla_name}"
  fi
done
if [[ -n "${RANGER_ENV_FILE}" && -f "${RANGER_ENV_FILE}" ]]; then
  _virtual_carla_restore_nounset=0
  _virtual_carla_restore_allexport=0
  [[ $- == *u* ]] && _virtual_carla_restore_nounset=1
  [[ $- == *a* ]] && _virtual_carla_restore_allexport=1
  set +u
  set -a
  # shellcheck disable=SC1090
  source "${RANGER_ENV_FILE}"
  [[ "${_virtual_carla_restore_allexport}" -eq 0 ]] && set +a
  [[ "${_virtual_carla_restore_nounset}" -eq 1 ]] && set -u
fi
for _virtual_carla_name in "${!_virtual_carla_explicit_values[@]}"; do
  printf -v "${_virtual_carla_name}" '%s' \
    "${_virtual_carla_explicit_values[${_virtual_carla_name}]}"
  export "${_virtual_carla_name}"
done

export RANGER_WORK_ROOT="${RANGER_WORK_ROOT:-${RANGER_CARLA_ROOT:+${RANGER_CARLA_ROOT}/.work}}"
export CAMROD_CARLA_YOLO_MODEL_PATH="${CAMROD_CARLA_YOLO_MODEL_PATH:-${RANGER_WORK_ROOT:+${RANGER_WORK_ROOT}/camrod/model_cache/yolov9mit/v1.0.0/v9-s.vec2box.sim.fp16.engine}}"
export CAMROD_CARLA_YOLO_DEVICE="${CAMROD_CARLA_YOLO_DEVICE:-0}"
export CAMROD_CARLA_YOLO_WORKSPACE_MIB="${CAMROD_CARLA_YOLO_WORKSPACE_MIB:-2048}"

_virtual_carla_default_ranger_ros_ws() {
  if [[ -z "${RANGER_CARLA_ROOT}" ]]; then
    return 0
  fi
  if [[ -d "${RANGER_CARLA_ROOT}/ros_ws" ]]; then
    printf '%s\n' "${RANGER_CARLA_ROOT}/ros_ws"
  elif [[ -d "${RANGER_CARLA_ROOT}/carla_ws" ]]; then
    # Compatibility with the accepted pre-split workspace layout.
    printf '%s\n' "${RANGER_CARLA_ROOT}/carla_ws"
  else
    printf '%s\n' "${RANGER_CARLA_ROOT}/ros_ws"
  fi
}

export RANGER_ROS_WS="${RANGER_ROS_WS:-$(_virtual_carla_default_ranger_ros_ws)}"
export CARLA_ROOT="${CARLA_ROOT:-${RANGER_WORK_ROOT:+${RANGER_WORK_ROOT}/src/carla}}"
export UE_ROOT="${UE_ROOT:-${RANGER_UE_ROOT:-${RANGER_WORK_ROOT:+${RANGER_WORK_ROOT}/src/UnrealEngine_4.26}}}"
export RANGER_UE_ROOT="${RANGER_UE_ROOT:-${UE_ROOT}}"
export CARLA_ROS_BRIDGE_WS="${CARLA_ROS_BRIDGE_WS:-${RANGER_ROS_BRIDGE_WS:-${RANGER_WORK_ROOT:+${RANGER_WORK_ROOT}/ros-bridge-ws}}}"
export RANGER_ROS_BRIDGE_WS="${RANGER_ROS_BRIDGE_WS:-${CARLA_ROS_BRIDGE_WS}}"
export RANGER_ROS_BRIDGE_SOURCE="${RANGER_ROS_BRIDGE_SOURCE:-${RANGER_ROS_BRIDGE_WS:+${RANGER_ROS_BRIDGE_WS}/src/ros-bridge}}"

export CARLA_VERSION="${CARLA_VERSION:-0.9.15}"
export CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
export CARLA_PORT="${CARLA_PORT:-2000}"
export CARLA_ROLE_NAME="${CARLA_ROLE_NAME:-ego_vehicle}"
export CARLA_RENDER_MODE="${CARLA_RENDER_MODE:-offscreen}"
export CARLA_RENDER_MAX_FPS="${CARLA_RENDER_MAX_FPS:-30}"
export CARLA_FIXED_DELTA_SECONDS="${CARLA_FIXED_DELTA_SECONDS:-0.05}"
export CARLA_SYNCHRONOUS_MODE="${CARLA_SYNCHRONOUS_MODE:-True}"
export CARLA_WAIT_FOR_CONTROL_COMMAND="${CARLA_WAIT_FOR_CONTROL_COMMAND:-False}"
if [[ -z "${CAMROD_CARLA_STEP_PACING:-}" ]]; then
  if [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]]; then
    CAMROD_CARLA_STEP_PACING=False
  else
    # HH_260830 - fixed_delta_seconds fixes simulated time, not wall time.
    # Keep the gate-bound upstream bridge unchanged and pace its public
    # /carla/control PAUSE/STEP_ONCE contract from CAMROD instead.
    CAMROD_CARLA_STEP_PACING=True
  fi
fi
export CAMROD_CARLA_STEP_PACING
export CAMROD_CARLA_STEP_PERIOD_SECONDS="${CAMROD_CARLA_STEP_PERIOD_SECONDS:-${CARLA_FIXED_DELTA_SECONDS}}"

export CARLA_UE_MAP="${CARLA_UE_MAP:-/Game/map_package/Maps/Woraksan_v1_0_3_parking_lot_hegiht_fit/Woraksan_v1_0_3_parking_lot_hegiht_fit}"
export CARLA_TOWN="${CARLA_TOWN:-map_package/Maps/Woraksan_v1_0_3_parking_lot_hegiht_fit/Woraksan_v1_0_3_parking_lot_hegiht_fit}"
export CAMROD_CARLA_MAP_PROFILE="${CAMROD_CARLA_MAP_PROFILE:-}"

export UE_EDITOR="${UE_EDITOR:-${UE_ROOT:+${UE_ROOT}/Engine/Binaries/Linux/UE4Editor}}"
export CARLA_UPROJECT="${CARLA_UPROJECT:-${CARLA_ROOT:+${CARLA_ROOT}/Unreal/CarlaUE4/CarlaUE4.uproject}}"

_virtual_carla_python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || true)"
_virtual_carla_default_egg="${CARLA_ROOT:+${CARLA_ROOT}/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py${_virtual_carla_python_version}-linux-x86_64.egg}"
export RANGER_CARLA_PYTHON_EGG="${RANGER_CARLA_PYTHON_EGG:-${CARLA_PYTHON_EGG:-${_virtual_carla_default_egg}}}"
export CARLA_PYTHON_EGG="${CARLA_PYTHON_EGG:-${RANGER_CARLA_PYTHON_EGG}}"
export RANGER_PYTHON_EGG_CACHE="${RANGER_PYTHON_EGG_CACHE:-${CARLA_PYTHON_EGG_CACHE:-}}"
export CARLA_PYTHON_EGG_CACHE="${CARLA_PYTHON_EGG_CACHE:-${RANGER_PYTHON_EGG_CACHE}}"

# Runtime authorization is intentionally bound to freshly generated portable
# gates under the untracked Ranger work root. Timestamped historical reports
# are documentation only and are never selected as runnable defaults.
export RANGER_EVIDENCE_ROOT="${RANGER_EVIDENCE_ROOT:-${RANGER_WORK_ROOT:+${RANGER_WORK_ROOT}/evidence}}"
export RANGER_BASELINE_MANIFEST="${RANGER_BASELINE_MANIFEST:-${RANGER_EVIDENCE_ROOT:+${RANGER_EVIDENCE_ROOT}/ranger_ros_backend_gate.json}}"
export RANGER_PHYSICAL_MANIFEST="${RANGER_PHYSICAL_MANIFEST:-${RANGER_EVIDENCE_ROOT:+${RANGER_EVIDENCE_ROOT}/ranger_physical_4ws_acceptance_gate.json}}"
if [[ -z "${RANGER_SPAWN_FILE:-}" ]]; then
  if [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]]; then
    RANGER_SPAWN_FILE="${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/ranger_spawn_camrod_control_only.json"
  else
    # Rendered CARLA must own deterministic camera/LiDAR actors.  Topic relays
    # cannot populate the CAMROD UI when the upstream sensors were never
    # spawned, even if the topic names themselves are configured correctly.
    RANGER_SPAWN_FILE="${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/ranger_spawn_camrod_full_sensors.json"
  fi
fi
export RANGER_SPAWN_FILE
if [[ -z "${CAMROD_LAUNCH_SENSOR_RELAY:-}" ]]; then
  if [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]]; then
    CAMROD_LAUNCH_SENSOR_RELAY=false
  else
    CAMROD_LAUNCH_SENSOR_RELAY=true
  fi
fi
export CAMROD_LAUNCH_SENSOR_RELAY
export CAMROD_MAP_ALIGNMENT_FILE="${CAMROD_MAP_ALIGNMENT_FILE:-${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/woraksan_lane_anchor_alignment.yaml}"
# Keep the develop map and the historical terrain-clearance map as two named
# cohorts.  `camrod` uses the former; only `camrod-tuned` may select the latter.
# CAMROD_LANELET_MAP remains the caller-overridable parity map for backwards
# compatibility with existing launch scripts.
export CAMROD_DEVELOP_LANELET_MAP="${CAMROD_DEVELOP_LANELET_MAP:-${CAMROD_SRC_ROOT}/lanelet2_maps.osm}"
export CAMROD_WORAKSAN_TUNED_LANELET_MAP="${CAMROD_WORAKSAN_TUNED_LANELET_MAP:-${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/woraksan_carla_lanelet2.osm}"
export CAMROD_LANELET_MAP="${CAMROD_LANELET_MAP:-${CAMROD_DEVELOP_LANELET_MAP}}"
export CAMROD_LAUNCH_DEFAULTS_FILE="${CAMROD_LAUNCH_DEFAULTS_FILE:-${CAMROD_SRC_ROOT}/camrod_bringup/config/bringup/launch_defaults.yaml}"

export CAMROD_UI_PORT="${CAMROD_UI_PORT:-8010}"
export CAMROD_UI_URL="${CAMROD_UI_URL:-http://127.0.0.1:${CAMROD_UI_PORT}}"
export CAMROD_GUEST_UI_URL="${CAMROD_GUEST_UI_URL:-http://127.0.0.1:8012}"
export CAMROD_GUEST_CDP_URL="${CAMROD_GUEST_CDP_URL:-http://127.0.0.1:9223}"
export CAMROD_ENABLE_OPERATOR_WINDOW="${CAMROD_ENABLE_OPERATOR_WINDOW:-true}"
export CAMROD_ENABLE_VOICE="${CAMROD_ENABLE_VOICE:-false}"
export CAMROD_ENABLE_RVIZ="${CAMROD_ENABLE_RVIZ:-false}"
export CAMROD_MANUAL_LINEAR_LIMIT_MPS="${CAMROD_MANUAL_LINEAR_LIMIT_MPS:-0.20}"
export CAMROD_MANUAL_LATERAL_LIMIT_MPS="${CAMROD_MANUAL_LATERAL_LIMIT_MPS:-0.20}"
export CAMROD_MANUAL_ANGULAR_LIMIT_RADPS="${CAMROD_MANUAL_ANGULAR_LIMIT_RADPS:-0.20}"
export CAMROD_MANUAL_DEADMAN_TIMEOUT_S="${CAMROD_MANUAL_DEADMAN_TIMEOUT_S:-0.25}"
# The default CARLA composition keeps the develop final-gate scale exactly.
# The historical Woraksan profile selects 1.0 inside its explicit tuned launch
# wrapper, so a map-specific uphill workaround cannot leak into parity runs.
export CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE="${CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE:-0.5}"
# Develop leaves the final Nav2 Ackermann-radius clamp disabled.  The accepted
# Ranger boundary and its 0.82 m anti-chatter margin belong only to the explicit
# Woraksan tuned wrapper.
export CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M="${CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M:-0.0}"
# Five wall-clock JPEG frames per camera are sufficient for the operator view
# and leave CPU headroom for the independent 10 Hz manual command heartbeat.
# Raw frames remain available to algorithms, but the CARLA relay caps their
# publication when a consumer is actually present.
export CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ="${CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ:-5.0}"
export CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ="${CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ:-10.0}"
export CAMROD_CARLA_SENSOR_MIN_RATE_HZ="${CAMROD_CARLA_SENSOR_MIN_RATE_HZ:-2.0}"
export CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS="${CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS:-3.0}"

# A CARLA-only CycloneDDS transport contract is shared by every virtual-CARLA
# ROS process. The repository-root file keeps the develop defaults; raw 800x600
# simulation frames use this explicit larger profile so CameraInfo and images
# cannot silently diverge between terminals.
export CAMROD_CYCLONEDDS_CONFIG="${CAMROD_CYCLONEDDS_CONFIG:-${CAMROD_SRC_ROOT}/camrod_system/config/cyclonedds_carla.xml}"
CAMROD_CYCLONEDDS_CONFIG="$(readlink -m "${CAMROD_CYCLONEDDS_CONFIG}")"
export CAMROD_CYCLONEDDS_CONFIG
export CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES="${CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES:-20971520}"
export CYCLONEDDS_URI="file://${CAMROD_CYCLONEDDS_CONFIG}"

_virtual_carla_prefix_roots=()
if [[ -n "${CARLA_ROS_BRIDGE_WS}" ]]; then
  _virtual_carla_prefix_roots+=("${CARLA_ROS_BRIDGE_WS}/install")
fi
if [[ -n "${RANGER_ROS_WS}" ]]; then
  _virtual_carla_prefix_roots+=("${RANGER_ROS_WS}/install")
fi
if [[ ${#_virtual_carla_prefix_roots[@]} -gt 0 ]]; then
  _virtual_carla_joined_prefix_roots="$(
    IFS=:
    printf '%s' "${_virtual_carla_prefix_roots[*]}"
  )"
else
  _virtual_carla_joined_prefix_roots=""
fi
export CAMROD_EXTRA_PREFIX_ROOTS="${CAMROD_EXTRA_PREFIX_ROOTS:-${_virtual_carla_joined_prefix_roots}}"

virtual_carla_log() {
  printf '[virtual_carla] %s\n' "$*"
}

virtual_carla_die() {
  printf '[virtual_carla] ERROR: %s\n' "$*" >&2
  return 1
}

virtual_carla_require_var() {
  local name="$1" value="${!1:-}"
  [[ -n "${value}" ]] || virtual_carla_die "${name} is not set"
}

virtual_carla_require_dir() {
  local path="$1" label="${2:-directory}"
  [[ -d "${path}" ]] || virtual_carla_die "missing ${label}: ${path}"
}

virtual_carla_require_file() {
  local path="$1" label="${2:-file}"
  [[ -f "${path}" ]] || virtual_carla_die "missing ${label}: ${path}"
}

virtual_carla_require_executable() {
  local path="$1" label="${2:-executable}"
  [[ -x "${path}" ]] || virtual_carla_die "missing ${label}: ${path}"
}

virtual_carla_require_dds_transport() {
  local required_floor=20971520
  local required="${CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES}"
  local key current failures=0

  virtual_carla_require_file \
    "${CAMROD_CYCLONEDDS_CONFIG}" "CAMROD CycloneDDS config" || return 1
  if [[ "${RMW_IMPLEMENTATION}" != "rmw_cyclonedds_cpp" ]]; then
    virtual_carla_die \
      "virtual CARLA camera transport requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (got ${RMW_IMPLEMENTATION})"
    return 1
  fi
  if [[ ! "${required}" =~ ^[0-9]+$ || "${required}" -lt "${required_floor}" ]]; then
    virtual_carla_die \
      "CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES must be an integer >= ${required_floor} (got ${required})"
    return 1
  fi
  if [[ "${CYCLONEDDS_URI}" != "file://${CAMROD_CYCLONEDDS_CONFIG}" ]]; then
    virtual_carla_die \
      "CYCLONEDDS_URI must select ${CAMROD_CYCLONEDDS_CONFIG} (got ${CYCLONEDDS_URI})"
    return 1
  fi
  command -v sysctl >/dev/null 2>&1 || {
    virtual_carla_die "sysctl is required to validate DDS camera socket buffers"
    return 1
  }

  for key in net.core.rmem_max net.core.wmem_max; do
    current="$(sysctl -n "${key}" 2>/dev/null)" || {
      virtual_carla_die "could not read kernel setting ${key}"
      failures=$((failures + 1))
      continue
    }
    if [[ ! "${current}" =~ ^[0-9]+$ ]]; then
      virtual_carla_die \
        "kernel setting ${key} is not an integer (got ${current})"
      failures=$((failures + 1))
      continue
    fi
    if (( current < required )); then
      virtual_carla_die \
        "kernel setting ${key}=${current} is below the DDS camera minimum ${required}"
      failures=$((failures + 1))
    fi
  done

  if (( failures > 0 )); then
    printf '%s\n' \
      "[virtual_carla] ACTION: sudo sysctl -w net.core.rmem_max=${required} net.core.wmem_max=${required}" \
      "[virtual_carla] Persist both values in /etc/sysctl.d/99-camrod-carla-dds.conf before opening the ROS terminals again." >&2
    return 1
  fi
  virtual_carla_log \
    "DDS camera transport ready: ${CYCLONEDDS_URI}, socket buffers >= ${required} bytes"
}

virtual_carla_source_ros() {
  local include_camrod="${1:-false}"
  local include_ranger="${2:-true}"
  local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  local bridge_setup="${CARLA_ROS_BRIDGE_WS:+${CARLA_ROS_BRIDGE_WS}/install/local_setup.bash}"
  local ranger_setup="${RANGER_ROS_WS:+${RANGER_ROS_WS}/install/local_setup.bash}"
  local camrod_setup="${CAMROD_WS_ROOT}/install/local_setup.bash"
  local restore_nounset=0

  virtual_carla_require_file "${ros_setup}" "ROS ${ROS_DISTRO} setup" || return 1

  # Execution scripts run in a child shell. Start their ROS prefix graph from a
  # known base so an ambient Autoware or unrelated CARLA overlay cannot win.
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
  unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

  [[ $- == *u* ]] && restore_nounset=1
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  if [[ -n "${bridge_setup}" && -f "${bridge_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${bridge_setup}"
  fi
  if [[ "${include_ranger}" == "true" && \
        -n "${ranger_setup}" && -f "${ranger_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${ranger_setup}"
  fi
  if [[ "${include_camrod}" == "true" && -f "${camrod_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${camrod_setup}"
  fi
  [[ "${restore_nounset}" -eq 1 ]] && set -u

  export ROS_DOMAIN_ID RMW_IMPLEMENTATION CAMROD_EXTRA_PREFIX_ROOTS
  export CAMROD_CYCLONEDDS_CONFIG CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES
  export CYCLONEDDS_URI
  export CAMROD_MANUAL_LINEAR_LIMIT_MPS CAMROD_MANUAL_LATERAL_LIMIT_MPS
  export CAMROD_MANUAL_ANGULAR_LIMIT_RADPS
  export CAMROD_MANUAL_DEADMAN_TIMEOUT_S
  export CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE
  export CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M
  export CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ
  export CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ
  export CAMROD_CARLA_SENSOR_MIN_RATE_HZ
  export CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS
  export CAMROD_CARLA_YOLO_MODEL_PATH CAMROD_CARLA_YOLO_DEVICE
  export CAMROD_CARLA_YOLO_WORKSPACE_MIB
  export CAMROD_CARLA_STEP_PACING CAMROD_CARLA_STEP_PERIOD_SECONDS
}

virtual_carla_use_python_egg() {
  virtual_carla_verify_gate_aliases || return 1
  virtual_carla_require_file "${CARLA_PYTHON_EGG}" "CARLA Python egg" || return 1
  case ":${PYTHONPATH:-}:" in
    *":${CARLA_PYTHON_EGG}:"*) ;;
    *) export PYTHONPATH="${CARLA_PYTHON_EGG}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
}

virtual_carla_verify_gate_aliases() {
  if [[ "$(readlink -m "${CARLA_PYTHON_EGG}")" != \
        "$(readlink -m "${RANGER_CARLA_PYTHON_EGG}")" ]]; then
    virtual_carla_die \
      "CARLA_PYTHON_EGG and RANGER_CARLA_PYTHON_EGG must identify the same gate-bound artifact"
    return 1
  fi
  if [[ -n "${CARLA_PYTHON_EGG_CACHE}" && \
        -n "${RANGER_PYTHON_EGG_CACHE}" && \
        "$(readlink -m "${CARLA_PYTHON_EGG_CACHE}")" != \
        "$(readlink -m "${RANGER_PYTHON_EGG_CACHE}")" ]]; then
    virtual_carla_die \
      "CARLA_PYTHON_EGG_CACHE and RANGER_PYTHON_EGG_CACHE must match"
    return 1
  fi
}

virtual_carla_verify_package_prefix() {
  local package="$1" expected_root="$2" prefix prefix_real root_real
  command -v ros2 >/dev/null 2>&1 || virtual_carla_die "ros2 is not available" || return 1
  prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)" || {
    virtual_carla_die "ROS package is not installed: ${package}"
    return 1
  }
  prefix_real="$(readlink -m "${prefix}")"
  root_real="$(readlink -m "${expected_root}")"
  case "${prefix_real}" in
    "${root_real}"|"${root_real}"/*)
      virtual_carla_log "package ${package}: ${prefix_real}"
      ;;
    *)
      virtual_carla_die \
        "package ${package} resolved outside ${root_real}: ${prefix_real}"
      return 1
      ;;
  esac
}

virtual_carla_verify_external_prefixes() {
  virtual_carla_require_var RANGER_CARLA_ROOT || return 1
  virtual_carla_require_file \
    "${CARLA_ROS_BRIDGE_WS}/install/local_setup.bash" \
    "CARLA ROS bridge install" || return 1
  virtual_carla_require_file \
    "${RANGER_ROS_WS}/install/local_setup.bash" \
    "Ranger ROS install" || return 1
  virtual_carla_source_ros false || return 1
  virtual_carla_verify_package_prefix \
    carla_ros_bridge "${CARLA_ROS_BRIDGE_WS}/install" || return 1
  virtual_carla_verify_package_prefix \
    carla_extended_ackermann_control "${RANGER_ROS_WS}/install" || return 1
}

virtual_carla_map_asset_file() {
  local relative_map="${CARLA_UE_MAP#/Game/}"
  printf '%s\n' \
    "${CARLA_ROOT}/Unreal/CarlaUE4/Content/${relative_map}.umap"
}

virtual_carla_normalize_map_name() {
  local value="$1"
  value="${value#/Game/}"
  value="${value#/}"
  value="${value%/}"
  printf '%s\n' "${value}"
}

virtual_carla_validate_map_selection() {
  local normalized_ue_map normalized_town
  normalized_ue_map="$(virtual_carla_normalize_map_name "${CARLA_UE_MAP}")"
  normalized_town="$(virtual_carla_normalize_map_name "${CARLA_TOWN}")"

  if [[ -z "${normalized_ue_map}" || \
        "${normalized_ue_map}" != "${normalized_town}" ]]; then
    virtual_carla_die \
      "CARLA_UE_MAP and CARLA_TOWN must identify the same map: ${CARLA_UE_MAP} != ${CARLA_TOWN}"
    return 1
  fi

  case "${CAMROD_CARLA_MAP_PROFILE}" in
    "") ;;
    "${CAMROD_CARLA_SITE_ACCESS_PROFILE_ID}")
      if [[ "${CARLA_UE_MAP}" != "${CAMROD_CARLA_SITE_ACCESS_UE_MAP}" || \
            "${CARLA_TOWN}" != "${CAMROD_CARLA_SITE_ACCESS_TOWN}" ]]; then
        virtual_carla_die \
          "map profile ${CAMROD_CARLA_MAP_PROFILE} is bound to ${CAMROD_CARLA_SITE_ACCESS_UE_MAP}"
        return 1
      fi
      ;;
    "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_PROFILE_ID}")
      if [[ "${CARLA_UE_MAP}" != "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_UE_MAP}" || \
            "${CARLA_TOWN}" != "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_TOWN}" ]]; then
        virtual_carla_die \
          "map profile ${CAMROD_CARLA_MAP_PROFILE} is bound to ${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_UE_MAP}"
        return 1
      fi
      ;;
    "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_PROFILE_ID}")
      if [[ "${CARLA_UE_MAP}" != "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_UE_MAP}" || \
            "${CARLA_TOWN}" != "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_TOWN}" ]]; then
        virtual_carla_die \
          "map profile ${CAMROD_CARLA_MAP_PROFILE} is bound to ${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_UE_MAP}"
        return 1
      fi
      ;;
    *)
      virtual_carla_die \
        "unsupported CAMROD_CARLA_MAP_PROFILE: ${CAMROD_CARLA_MAP_PROFILE}"
      return 1
      ;;
  esac
}

virtual_carla_print_environment() {
  cat <<EOF
CONFIG_PRECEDENCE=caller environment > RANGER_ENV_FILE > derived defaults
CAMROD_SRC_ROOT=${CAMROD_SRC_ROOT}
CAMROD_WS_ROOT=${CAMROD_WS_ROOT}
RANGER_CARLA_ROOT=${RANGER_CARLA_ROOT}
RANGER_WORK_ROOT=${RANGER_WORK_ROOT}
RANGER_ENV_FILE=${RANGER_ENV_FILE}
RANGER_ROS_WS=${RANGER_ROS_WS}
CARLA_ROOT=${CARLA_ROOT}
UE_ROOT=${UE_ROOT}
RANGER_UE_ROOT=${RANGER_UE_ROOT}
CARLA_ROS_BRIDGE_WS=${CARLA_ROS_BRIDGE_WS}
RANGER_ROS_BRIDGE_WS=${RANGER_ROS_BRIDGE_WS}
RANGER_ROS_BRIDGE_SOURCE=${RANGER_ROS_BRIDGE_SOURCE}
RANGER_EVIDENCE_ROOT=${RANGER_EVIDENCE_ROOT}
RANGER_BASELINE_MANIFEST=${RANGER_BASELINE_MANIFEST}
RANGER_PHYSICAL_MANIFEST=${RANGER_PHYSICAL_MANIFEST}
RANGER_CARLA_PYTHON_EGG=${RANGER_CARLA_PYTHON_EGG}
CARLA_PYTHON_EGG=${CARLA_PYTHON_EGG}
CARLA endpoint=${CARLA_HOST}:${CARLA_PORT}
CARLA map profile=${CAMROD_CARLA_MAP_PROFILE:-direct-runner-default-or-caller}
CARLA map=${CARLA_UE_MAP}
CARLA town=${CARLA_TOWN}
CARLA render mode=${CARLA_RENDER_MODE}
CARLA render maximum fps=${CARLA_RENDER_MAX_FPS}
CARLA synchronous=${CARLA_SYNCHRONOUS_MODE}
CARLA wait for control=${CARLA_WAIT_FOR_CONTROL_COMMAND}
CARLA fixed delta seconds=${CARLA_FIXED_DELTA_SECONDS}
CAMROD CARLA step pacing=${CAMROD_CARLA_STEP_PACING}
CAMROD CARLA step period seconds=${CAMROD_CARLA_STEP_PERIOD_SECONDS}
CAMROD sensor relay=${CAMROD_LAUNCH_SENSOR_RELAY}
CAMROD develop/parity lanelet map=${CAMROD_LANELET_MAP}
CAMROD Woraksan-tuned lanelet map=${CAMROD_WORAKSAN_TUNED_LANELET_MAP}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
CAMROD CycloneDDS config=${CAMROD_CYCLONEDDS_CONFIG}
CYCLONEDDS_URI=${CYCLONEDDS_URI}
DDS socket buffer minimum bytes=${CAMROD_DDS_SOCKET_BUFFER_MIN_BYTES}
CAMROD manual linear limit mps=${CAMROD_MANUAL_LINEAR_LIMIT_MPS}
CAMROD manual lateral limit mps=${CAMROD_MANUAL_LATERAL_LIMIT_MPS}
CAMROD manual angular limit radps=${CAMROD_MANUAL_ANGULAR_LIMIT_RADPS}
CAMROD manual UI deadman timeout seconds=${CAMROD_MANUAL_DEADMAN_TIMEOUT_S}
CARLA final cmd_vel speed scale=${CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE}
CARLA Nav2 minimum Ackermann turn radius m=${CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M}
CARLA compressed image maximum wall rate hz=${CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ}
CARLA raw image maximum wall rate hz=${CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ}
CARLA visual sensor minimum wall rate hz=${CAMROD_CARLA_SENSOR_MIN_RATE_HZ}
CARLA visual sensor maximum sample age seconds=${CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS}
CARLA YOLO model path=${CAMROD_CARLA_YOLO_MODEL_PATH}
CARLA YOLO CUDA device=${CAMROD_CARLA_YOLO_DEVICE}
CARLA YOLO TensorRT workspace MiB=${CAMROD_CARLA_YOLO_WORKSPACE_MIB}
EOF
}

unset _virtual_carla_prefix_roots
unset _virtual_carla_joined_prefix_roots
unset _virtual_carla_python_version
unset _virtual_carla_env_dir
unset _virtual_carla_config_names
unset _virtual_carla_explicit_values
unset _virtual_carla_name
unset _virtual_carla_restore_nounset
unset _virtual_carla_restore_allexport
unset _virtual_carla_default_egg
