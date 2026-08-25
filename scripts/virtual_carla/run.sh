#!/usr/bin/env bash
# Explicit lifecycle runner for the CAMROD <-> Ranger/CARLA integration.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run.sh <commands|server|bridge|spawn|camrod|doctor>

  commands  print copyable terminal commands; start nothing
  server    run the UE 4.26 CARLA server in CARLA_RENDER_MODE
  bridge    run the standard carla_ros_bridge
  spawn     spawn the configured Ranger actor/sensors
  camrod    run the Ranger 4WS controller, adapter, full CAMROD and UI
  doctor    validate paths, overlays, gates, Python API and renderer readiness

Start in separate terminals: server -> bridge -> spawn -> camrod.
Stop in reverse order. No subcommand publishes motion or sends a Nav2 goal.

CARLA_RENDER_MODE is one of offscreen (default), onscreen, or nullrhi.
Set RANGER_SPAWN_FILE to an aligned full-sensor JSON for rendered perception;
the default checked-in JSON is the control-only smoke profile.
EOF
}

if [[ $# -eq 0 ]]; then
  usage
  exit 2
fi

subcommand="$1"
shift
if [[ $# -ne 0 ]]; then
  printf '[virtual_carla] ERROR: unexpected argument: %s\n' "$1" >&2
  usage >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/env.sh"

print_command() {
  local argument
  for argument in "$@"; do
    printf '%q ' "${argument}"
  done
  printf '\n'
}

server_command() {
  SERVER_COMMAND=(
    "${UE_EDITOR}"
    "${CARLA_UPROJECT}"
    "${CARLA_UE_MAP}"
    -game
    -quality-level=Low
    "-world-port=${CARLA_PORT}"
    -nosound
    -unattended
    -nop4
    -NoSplash
    -stdout
    -FullStdOutLogOutput
  )
  case "${CARLA_RENDER_MODE}" in
    offscreen) SERVER_COMMAND+=( -RenderOffScreen ) ;;
    onscreen) ;;
    nullrhi) SERVER_COMMAND+=( -nullrhi ) ;;
    *)
      virtual_carla_die \
        "CARLA_RENDER_MODE must be offscreen, onscreen, or nullrhi (got ${CARLA_RENDER_MODE})"
      return 1
      ;;
  esac
}

bridge_command() {
  BRIDGE_COMMAND=(
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
    "host:=${CARLA_HOST}"
    "port:=${CARLA_PORT}"
    "ego_vehicle_role_name:=${CARLA_ROLE_NAME}"
    "town:=${CARLA_TOWN}"
    "synchronous_mode:=${CARLA_SYNCHRONOUS_MODE}"
    "synchronous_mode_wait_for_vehicle_control_command:=${CARLA_WAIT_FOR_CONTROL_COMMAND}"
    "fixed_delta_seconds:=${CARLA_FIXED_DELTA_SECONDS}"
  )
}

spawn_command() {
  SPAWN_COMMAND=(
    ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
    "objects_definition_file:=${RANGER_SPAWN_FILE}"
  )
}

camrod_command() {
  CAMROD_COMMAND=(
    ros2 launch camrod_carla_adapter camrod_carla_full.launch.py
    "role_name:=${CARLA_ROLE_NAME}"
    "host:=${CARLA_HOST}"
    "port:=${CARLA_PORT}"
    "accepted_carla_python_egg:=${CARLA_PYTHON_EGG}"
    "python_egg_cache:=${CARLA_PYTHON_EGG_CACHE}"
    "verified_baseline_manifest:=${RANGER_BASELINE_MANIFEST}"
    "verified_physical_four_wheel_manifest:=${RANGER_PHYSICAL_MANIFEST}"
    extended_mode_backend:=PHYSX_FOUR_WHEEL_STEERING
    "map_alignment_file:=${CAMROD_MAP_ALIGNMENT_FILE}"
    "camrod_launch_defaults_file:=${CAMROD_LAUNCH_DEFAULTS_FILE}"
    "camrod_map_path:=${CAMROD_LANELET_MAP}"
    enable_plugin_api:=true
    enable_api_ui:=true
    "enable_operator_ui_window:=${CAMROD_ENABLE_OPERATOR_WINDOW}"
    "api_ui_port:=${CAMROD_UI_PORT}"
    "operator_ui_window_url:=${CAMROD_UI_URL}"
    "rviz:=${CAMROD_ENABLE_RVIZ}"
    "enable_voice:=${CAMROD_ENABLE_VOICE}"
  )
}

port_is_listening() {
  command -v ss >/dev/null 2>&1 || return 1
  ss -H -ltn 2>/dev/null | awk -v suffix=":${CARLA_PORT}" '
    $4 == suffix || substr($4, length($4) - length(suffix) + 1) == suffix {
      found = 1
    }
    END { exit(found ? 0 : 1) }
  '
}

require_renderer() {
  [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]] && return 0
  command -v nvidia-smi >/dev/null 2>&1 || {
    virtual_carla_die "nvidia-smi is required for rendered CARLA"
    return 1
  }
  nvidia-smi -L >/dev/null 2>&1 || {
    virtual_carla_die "NVIDIA driver/GPU is not ready for rendered CARLA"
    return 1
  }
  command -v vulkaninfo >/dev/null 2>&1 || {
    virtual_carla_die "vulkaninfo is required for rendered CARLA"
    return 1
  }
  vulkaninfo --summary >/dev/null 2>&1 || {
    virtual_carla_die "Vulkan is not ready for rendered CARLA"
    return 1
  }
}

require_common_runtime_files() {
  virtual_carla_require_var RANGER_CARLA_ROOT
  virtual_carla_verify_gate_aliases
  virtual_carla_require_file \
    "${RANGER_BASELINE_MANIFEST}" "Ranger baseline gate"
  virtual_carla_require_file \
    "${RANGER_PHYSICAL_MANIFEST}" "Ranger physical 4WS gate"
  virtual_carla_require_file "${CARLA_PYTHON_EGG}" "CARLA Python egg"
  virtual_carla_require_file \
    "${CAMROD_MAP_ALIGNMENT_FILE}" "CARLA-to-CAMROD map alignment"
  virtual_carla_require_file "${CAMROD_LANELET_MAP}" "CAMROD lanelet map"
  virtual_carla_require_file \
    "${CAMROD_LAUNCH_DEFAULTS_FILE}" "CAMROD launch defaults"
}

require_bridge_authorization_files() {
  virtual_carla_require_var RANGER_CARLA_ROOT
  virtual_carla_require_dir \
    "${RANGER_ROS_BRIDGE_SOURCE}" "CARLA ROS bridge source"
  virtual_carla_require_dir \
    "${RANGER_EVIDENCE_ROOT}" "Ranger evidence root"
  virtual_carla_require_file \
    "${RANGER_BASELINE_MANIFEST}" "Ranger baseline gate"
  virtual_carla_require_file \
    "${RANGER_PHYSICAL_MANIFEST}" "Ranger physical 4WS gate"
  validate_gate_status "${RANGER_BASELINE_MANIFEST}" "baseline gate"
  validate_gate_status \
    "${RANGER_PHYSICAL_MANIFEST}" "physical 4WS gate"
}

validate_spawn_file() {
  virtual_carla_require_file "${RANGER_SPAWN_FILE}" "Ranger spawn JSON"
  python3 - "${RANGER_SPAWN_FILE}" "${CARLA_ROLE_NAME}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
role = sys.argv[2]
document = json.loads(path.read_text(encoding="utf-8"))
objects = document.get("objects", [])
if not isinstance(objects, list) or not objects:
    raise SystemExit(f"spawn JSON has no objects: {path}")
if not any(item.get("id") == role for item in objects if isinstance(item, dict)):
    raise SystemExit(f"spawn JSON has no actor id {role!r}: {path}")
PY
}

validate_gate_status() {
  local manifest="$1" label="$2"
  python3 - "${manifest}" "${label}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
label = sys.argv[2]
document = json.loads(path.read_text(encoding="utf-8"))
if document.get("status") != "VERIFIED":
    raise SystemExit(f"{label} status is not VERIFIED: {path}")
PY
}

validate_carla_python_api() {
  local cache_dir
  cache_dir="$(mktemp -d "${TMPDIR:-/tmp}/camrod-carla-doctor.XXXXXX")"
  (
    trap 'rm -rf -- "${cache_dir}"' EXIT
    virtual_carla_use_python_egg
    PYTHON_EGG_CACHE="${cache_dir}" python3 - <<'PY'
import carla

expected = (
    "set_wheel_physics_steer_angles_and_drive_torques",
    "get_wheel_physics_telemetry",
)
missing = [name for name in expected if not callable(getattr(carla.Vehicle, name, None))]
if missing:
    raise SystemExit("CARLA Python API lacks physical 4WD v2: " + ", ".join(missing))
print("[virtual_carla] CARLA Python API physical 4WD v2: ready")
PY
  )
}

prepare_ros_carla_python() {
  local owner="$1" cache_dir
  cache_dir="$(mktemp -d \
    "${TMPDIR:-/tmp}/camrod-carla-${owner}-egg.XXXXXX")"
  export PYTHON_EGG_CACHE="${cache_dir}"
  virtual_carla_use_python_egg
  virtual_carla_log \
    "${owner} CARLA Python egg cache: ${PYTHON_EGG_CACHE}"
}

run_doctor() {
  local failures=0 map_asset

  virtual_carla_print_environment
  virtual_carla_require_var RANGER_CARLA_ROOT || failures=$((failures + 1))
  virtual_carla_require_dir \
    "${RANGER_CARLA_ROOT}" "Ranger/CARLA repository" || failures=$((failures + 1))
  virtual_carla_require_executable \
    "${UE_EDITOR}" "UE4Editor" || failures=$((failures + 1))
  virtual_carla_require_file \
    "${CARLA_UPROJECT}" "CarlaUE4 project" || failures=$((failures + 1))
  map_asset="$(virtual_carla_map_asset_file)"
  virtual_carla_require_file \
    "${map_asset}" "CARLA custom map asset" || failures=$((failures + 1))
  validate_spawn_file || failures=$((failures + 1))
  require_common_runtime_files || failures=$((failures + 1))
  require_bridge_authorization_files || failures=$((failures + 1))

  if virtual_carla_source_ros true true; then
    virtual_carla_verify_package_prefix \
      carla_ros_bridge "${CARLA_ROS_BRIDGE_WS}/install" || failures=$((failures + 1))
    virtual_carla_verify_package_prefix \
      carla_spawn_objects "${CARLA_ROS_BRIDGE_WS}/install" || failures=$((failures + 1))
    virtual_carla_verify_package_prefix \
      carla_extended_ackermann_control \
      "${RANGER_ROS_WS}/install" || failures=$((failures + 1))
    virtual_carla_verify_package_prefix \
      rqt_extended_ackermann \
      "${RANGER_ROS_WS}/install" || failures=$((failures + 1))
    virtual_carla_verify_package_prefix \
      camrod_carla_adapter \
      "${CAMROD_WS_ROOT}/install" || failures=$((failures + 1))
    virtual_carla_verify_package_prefix \
      camrod_ui "${CAMROD_WS_ROOT}/install" || failures=$((failures + 1))
    validate_carla_python_api || failures=$((failures + 1))
  else
    failures=$((failures + 1))
  fi

  require_renderer || failures=$((failures + 1))
  if port_is_listening; then
    virtual_carla_log \
      "TCP port ${CARLA_PORT} is listening (do not start a second server)"
  else
    virtual_carla_log \
      "TCP port ${CARLA_PORT} is free; start the server first"
  fi

  if [[ "${failures}" -ne 0 ]]; then
    virtual_carla_die "doctor found ${failures} failed check(s)"
    return 1
  fi
  virtual_carla_log "doctor passed"
}

case "${subcommand}" in
  -h|--help|help)
    usage
    ;;
  commands)
    virtual_carla_require_var RANGER_CARLA_ROOT
    server_command
    bridge_command
    spawn_command
    if [[ -z "${CARLA_PYTHON_EGG_CACHE}" ]]; then
      CARLA_PYTHON_EGG_CACHE='<fresh-empty-absolute-directory>'
    fi
    camrod_command

    printf '# Export once in every terminal (adjust overrides as needed)\n'
    printf 'export RANGER_CARLA_ROOT=%q\n' "${RANGER_CARLA_ROOT}"
    printf 'export CARLA_ROOT=%q\n' "${CARLA_ROOT}"
    printf 'export UE_ROOT=%q\n' "${UE_ROOT}"
    printf 'export CARLA_ROS_BRIDGE_WS=%q\n' "${CARLA_ROS_BRIDGE_WS}"
    printf 'export RANGER_ROS_WS=%q\n\n' "${RANGER_ROS_WS}"
    printf '# Recommended copyable lifecycle commands (four terminals)\n'
    printf '%q server\n' "${script_dir}/run.sh"
    printf '%q bridge\n' "${script_dir}/run.sh"
    printf '%q spawn\n' "${script_dir}/run.sh"
    printf '%q camrod\n\n' "${script_dir}/run.sh"

    printf '# Expanded server command\n'
    print_command "${SERVER_COMMAND[@]}"
    printf '\n# Expanded ROS bridge command (after sourcing validated overlays)\n'
    print_command "${BRIDGE_COMMAND[@]}"
    printf '\n# Expanded actor spawn command\n'
    print_command "${SPAWN_COMMAND[@]}"
    printf '\n# Expanded CAMROD command; run.sh creates the fresh egg cache\n'
    print_command "${CAMROD_COMMAND[@]}"
    printf '\n# No motion command or navigation goal is emitted here.\n'
    ;;
  server)
    virtual_carla_require_executable "${UE_EDITOR}" "UE4Editor"
    virtual_carla_require_file "${CARLA_UPROJECT}" "CarlaUE4 project"
    virtual_carla_require_file \
      "$(virtual_carla_map_asset_file)" "CARLA custom map asset"
    require_renderer
    if port_is_listening; then
      virtual_carla_die \
        "TCP port ${CARLA_PORT} is already listening; refusing a second server"
      exit 1
    fi
    server_command
    virtual_carla_log "starting CARLA server (${CARLA_RENDER_MODE})"
    exec "${SERVER_COMMAND[@]}"
    ;;
  bridge)
    require_bridge_authorization_files
    virtual_carla_verify_external_prefixes
    prepare_ros_carla_python bridge
    bridge_command
    virtual_carla_log "starting standard CARLA ROS bridge"
    exec "${BRIDGE_COMMAND[@]}"
    ;;
  spawn)
    virtual_carla_verify_external_prefixes
    prepare_ros_carla_python spawn
    virtual_carla_verify_package_prefix \
      carla_spawn_objects "${CARLA_ROS_BRIDGE_WS}/install"
    validate_spawn_file
    spawn_command
    virtual_carla_log "spawning configured Ranger actor (no motion command)"
    exec "${SPAWN_COMMAND[@]}"
    ;;
  camrod)
    require_common_runtime_files
    virtual_carla_source_ros true true
    virtual_carla_verify_package_prefix \
      carla_extended_ackermann_control "${RANGER_ROS_WS}/install"
    virtual_carla_verify_package_prefix \
      camrod_carla_adapter "${CAMROD_WS_ROOT}/install"
    virtual_carla_verify_package_prefix \
      camrod_ui "${CAMROD_WS_ROOT}/install"
    validate_gate_status "${RANGER_BASELINE_MANIFEST}" "baseline gate"
    validate_gate_status \
      "${RANGER_PHYSICAL_MANIFEST}" "physical 4WS gate"

    if [[ -z "${CARLA_PYTHON_EGG_CACHE}" ]]; then
      CARLA_PYTHON_EGG_CACHE="$(
        mktemp -d "${TMPDIR:-/tmp}/camrod-carla-egg.XXXXXX"
      )"
      RANGER_PYTHON_EGG_CACHE="${CARLA_PYTHON_EGG_CACHE}"
      export CARLA_PYTHON_EGG_CACHE
      export RANGER_PYTHON_EGG_CACHE
      virtual_carla_log \
        "created fresh egg cache: ${CARLA_PYTHON_EGG_CACHE}"
    fi
    [[ "${CARLA_PYTHON_EGG_CACHE}" == /* ]] || \
      virtual_carla_die "CARLA_PYTHON_EGG_CACHE must be an absolute path"
    virtual_carla_require_dir \
      "${CARLA_PYTHON_EGG_CACHE}" "CARLA Python egg cache"
    if find "${CARLA_PYTHON_EGG_CACHE}" -mindepth 1 -print -quit | grep -q .; then
      virtual_carla_die \
        "CARLA_PYTHON_EGG_CACHE must be empty: ${CARLA_PYTHON_EGG_CACHE}"
      exit 1
    fi
    virtual_carla_use_python_egg
    camrod_command
    virtual_carla_log \
      "starting Ranger 4WS controller, CAMROD algorithms and production UI"
    virtual_carla_log \
      "no motion is sent; arm and command only after runtime gates are healthy"
    exec "${CAMROD_COMMAND[@]}"
    ;;
  doctor)
    run_doctor
    ;;
  *)
    printf '[virtual_carla] ERROR: unknown subcommand: %s\n' \
      "${subcommand}" >&2
    usage >&2
    exit 2
    ;;
esac
