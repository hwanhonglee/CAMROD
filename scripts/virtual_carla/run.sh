#!/usr/bin/env bash
# Explicit lifecycle runner for the CAMROD <-> Ranger/CARLA integration.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run.sh <commands|server|bridge|spawn|camrod|manual|doctor>

  commands  print copyable terminal commands; start nothing
  server    run the UE 4.26 CARLA server in CARLA_RENDER_MODE
  bridge    run the standard carla_ros_bridge
  spawn     spawn the configured Ranger actor/sensors
  camrod    run the Ranger 4WS controller, adapter, full CAMROD and UI
  manual    terminal-keyboard fallback through CAMROD safety/physical 4WS
  doctor    validate paths, overlays, gates, Python API and renderer readiness

Required order in separate terminals: server -> bridge -> spawn -> camrod.
Wait for each preceding stage to report success. The camrod stage refuses to
start unless exactly one vehicle.ranger.default with CARLA_ROLE_NAME exists.
After camrod is healthy, the preferred manual control is in the UI Admin >
Camera tab. Run manual only as a terminal fallback; it sends nothing until the
operator presses a motion key and never engages the robot. Stop in reverse
order. The lifecycle subcommands do not publish motion or send a Nav2 goal.

CARLA_RENDER_MODE is one of offscreen (default), onscreen, or nullrhi.
Rendered modes default to the aligned checked-in full-sensor profile. NullRHI
defaults to the control-only smoke profile and cannot validate camera output.
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
    "launch_sensor_relay:=${CAMROD_LAUNCH_SENSOR_RELAY}"
    enable_plugin_api:=true
    enable_api_ui:=true
    "enable_operator_ui_window:=${CAMROD_ENABLE_OPERATOR_WINDOW}"
    "api_ui_port:=${CAMROD_UI_PORT}"
    "operator_ui_window_url:=${CAMROD_UI_URL}"
    "rviz:=${CAMROD_ENABLE_RVIZ}"
    "enable_voice:=${CAMROD_ENABLE_VOICE}"
  )
}

manual_command() {
  # Keep diagonal teleop commands in Dual-Ackermann mode:
  # radius = speed / turn = 1.0 m > Ranger minimum 0.810330349 m.
  MANUAL_COMMAND=(
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    --ros-args
    -r cmd_vel:=/control/manual_cmd_vel_ros
    -p speed:=0.20
    -p turn:=0.20
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
  if ! command -v vulkaninfo >/dev/null 2>&1; then
    printf '%s\n' \
      '[virtual_carla] WARNING: vulkaninfo is not installed; skipping the optional Vulkan summary preflight' >&2
    return 0
  fi
  vulkaninfo --summary >/dev/null 2>&1 || {
    virtual_carla_die "Vulkan is not ready for rendered CARLA"
    return 1
  }
}

validate_render_timing_contract() {
  local synchronous="${CARLA_SYNCHRONOUS_MODE,,}"
  local wait_for_control="${CARLA_WAIT_FOR_CONTROL_COMMAND,,}"
  if [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]]; then
    return 0
  fi
  case "${synchronous}" in
    true|1|yes|on) ;;
    *)
      virtual_carla_die \
        "rendered CARLA requires synchronous_mode=True for deterministic 10 Hz CAMROD sensors"
      return 1
      ;;
  esac
  case "${wait_for_control}" in
    false|0|no|off) ;;
    *)
      virtual_carla_die \
        "rendered CARLA requires synchronous_mode_wait_for_vehicle_control_command=False so camera/LiDAR can stream before CAMROD starts"
      return 1
      ;;
  esac
  python3 - "${CARLA_FIXED_DELTA_SECONDS}" <<'PY'
import math
import sys

try:
    delta = float(sys.argv[1])
except ValueError:
    delta = math.nan
if not math.isfinite(delta) or delta <= 0.0 or delta > 0.1:
    raise SystemExit(
        "CARLA_FIXED_DELTA_SECONDS must be finite and in (0, 0.1] "
        "for the rendered 10 Hz sensor contract"
    )
PY
}

require_common_runtime_files() {
  virtual_carla_require_var RANGER_CARLA_ROOT || return 1
  virtual_carla_verify_gate_aliases || return 1
  virtual_carla_require_file \
    "${RANGER_BASELINE_MANIFEST}" "Ranger baseline gate" || return 1
  virtual_carla_require_file \
    "${RANGER_PHYSICAL_MANIFEST}" "Ranger physical 4WS gate" || return 1
  virtual_carla_require_file \
    "${CARLA_PYTHON_EGG}" "CARLA Python egg" || return 1
  virtual_carla_require_file \
    "${CAMROD_MAP_ALIGNMENT_FILE}" "CARLA-to-CAMROD map alignment" || return 1
  virtual_carla_require_file \
    "${CAMROD_LANELET_MAP}" "CAMROD lanelet map" || return 1
  virtual_carla_require_file \
    "${CAMROD_LAUNCH_DEFAULTS_FILE}" "CAMROD launch defaults" || return 1
}

require_bridge_authorization_files() {
  virtual_carla_require_var RANGER_CARLA_ROOT || return 1
  virtual_carla_require_dir \
    "${RANGER_ROS_BRIDGE_SOURCE}" "CARLA ROS bridge source" || return 1
  virtual_carla_require_dir \
    "${RANGER_EVIDENCE_ROOT}" "Ranger evidence root" || return 1
  virtual_carla_require_file \
    "${RANGER_BASELINE_MANIFEST}" "Ranger baseline gate" || return 1
  virtual_carla_require_file \
    "${RANGER_PHYSICAL_MANIFEST}" "Ranger physical 4WS gate" || return 1
  validate_runtime_gates || return 1
}

validate_spawn_file() {
  local accepted_control accepted_visual
  accepted_control="${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/ranger_spawn_camrod_control_only.json"
  accepted_visual="${CAMROD_SRC_ROOT}/camrod_carla_adapter/config/ranger_spawn_camrod_full_sensors.json"
  virtual_carla_require_file \
    "${RANGER_SPAWN_FILE}" "Ranger spawn JSON" || return 1
  virtual_carla_require_file \
    "${accepted_control}" "accepted Ranger control-only spawn contract" || return 1
  virtual_carla_require_file \
    "${accepted_visual}" "accepted Ranger visual spawn contract" || return 1
  python3 - "${RANGER_SPAWN_FILE}" "${CARLA_ROLE_NAME}" \
    vehicle.ranger.default "${CARLA_RENDER_MODE}" \
    "${accepted_control}" "${accepted_visual}" <<'PY'
import json
import math
from pathlib import Path
import sys

path = Path(sys.argv[1])
role = sys.argv[2]
expected_type = sys.argv[3]
render_mode = sys.argv[4]
control_path = Path(sys.argv[5])
visual_path = Path(sys.argv[6])


def load_document(document_path):
    try:
        return json.loads(document_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SystemExit(
            f"spawn JSON is unreadable or invalid: {document_path}: {error}"
        ) from None


def select_vehicle(document, document_path):
    objects = document.get("objects", [])
    if not isinstance(objects, list) or not objects:
        raise SystemExit(f"spawn JSON has no objects: {document_path}")
    matches = [
        item for item in objects
        if isinstance(item, dict) and item.get("id") == role
    ]
    if len(matches) != 1:
        raise SystemExit(
            f"spawn JSON must have exactly one actor id {role!r}, "
            f"found {len(matches)}: {document_path}")
    return matches[0]


def require_contract(actual, expected, label):
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            raise SystemExit(f"{label} must be an object: {path}")
        for key, expected_value in expected.items():
            if key not in actual:
                raise SystemExit(f"{label}.{key} is missing: {path}")
            require_contract(actual[key], expected_value, f"{label}.{key}")
        return
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        try:
            numeric = float(actual)
        except (TypeError, ValueError):
            numeric = math.nan
        if not math.isfinite(numeric) or abs(numeric - float(expected)) > 1.0e-6:
            raise SystemExit(
                f"{label} breaks the accepted spawn/alignment cohort: "
                f"expected {expected!r}, got {actual!r}: {path}")
        return
    if actual != expected:
        raise SystemExit(
            f"{label} breaks the accepted spawn/alignment cohort: "
            f"expected {expected!r}, got {actual!r}: {path}")


document = load_document(path)
vehicle = select_vehicle(document, path)
if vehicle.get("type") != expected_type:
    raise SystemExit(
        f"spawn JSON actor {role!r} type must be exactly "
        f"{expected_type!r}, got {vehicle.get('type')!r}: {path}")

reference_path = control_path if render_mode == "nullrhi" else visual_path
reference_vehicle = select_vehicle(load_document(reference_path), reference_path)
require_contract(
    vehicle.get("spawn_point"),
    reference_vehicle.get("spawn_point"),
    f"actor {role!r}.spawn_point",
)
target_sensors = vehicle.get("sensors", [])
if not isinstance(target_sensors, list):
    raise SystemExit(f"spawn JSON actor {role!r} has no sensor list: {path}")
for expected_sensor in reference_vehicle.get("sensors", []):
    sensor_id = expected_sensor.get("id")
    sensor_matches = [
        sensor for sensor in target_sensors
        if isinstance(sensor, dict) and sensor.get("id") == sensor_id
    ]
    if len(sensor_matches) != 1:
        profile = "control" if render_mode == "nullrhi" else "rendered"
        raise SystemExit(
            f"{profile} spawn JSON must contain exactly one {sensor_id!r}, "
            f"found {len(sensor_matches)}: {path}")
    require_contract(
        sensor_matches[0], expected_sensor, f"sensor {sensor_id!r}"
    )
PY
}

validate_carla_sensor_streams() {
  local status_code
  if [[ "${CARLA_RENDER_MODE}" == "nullrhi" ]]; then
    virtual_carla_log \
      "NullRHI selected: skipping camera/LiDAR payload preflight"
    return 0
  fi
  if timeout --signal=INT --kill-after=2s 12s \
      python3 "${script_dir}/check_carla_sensor_streams.py" \
        --role-name "${CARLA_ROLE_NAME}" \
        --timeout-seconds 8; then
    virtual_carla_log "CARLA camera/LiDAR payload preflight passed"
    return 0
  else
    status_code=$?
  fi
  case "${status_code}" in
    124|137)
      virtual_carla_die \
        "CARLA visual sensor preflight exceeded its hard 12s bound; keep bridge and spawn running, then retry camrod"
      ;;
    130)
      virtual_carla_die "CARLA visual sensor preflight was interrupted"
      ;;
    *)
      return "${status_code}"
      ;;
  esac
  return 1
}

refuse_hot_respawn() {
  local active_nodes status_code
  if active_nodes="$(timeout --signal=INT --kill-after=1s 5s ros2 node list 2>&1)"; then
    :
  else
    status_code=$?
    virtual_carla_die \
      "cannot verify whether CAMROD is active before spawn (ros2 node list exit=${status_code}): ${active_nodes}"
    return 1
  fi
  if printf '%s\n' "${active_nodes}" | grep -Eq \
      '/(camrod_twist_to_4ws|physical_four_wheel_bridge|ui_backend|carla_spawn_objects)$'; then
    virtual_carla_die \
      "CAMROD, physical 4WS, UI, or an existing spawn stage is active; duplicate/hot respawn would change the actor ID and break the controller binding. Stop manual -> camrod -> spawn before spawning again"
    return 1
  fi
}

validate_no_ranger_actor_present() {
  local status_code
  if timeout --signal=INT --kill-after=2s 8s \
      python3 "${script_dir}/check_ranger_actor.py" \
        --host "${CARLA_HOST}" \
        --port "${CARLA_PORT}" \
        --role-name "${CARLA_ROLE_NAME}" \
        --accepted-python-egg "${CARLA_PYTHON_EGG}" \
        --private-egg-cache "${PYTHON_EGG_CACHE}" \
        --timeout-seconds 4 \
        --expected-count 0; then
    return 0
  else
    status_code=$?
  fi
  case "${status_code}" in
    124|137)
      virtual_carla_die \
        "pre-spawn Ranger inventory check exceeded its hard 8s bound; verify the CARLA endpoint before retrying spawn"
      ;;
    130)
      virtual_carla_die "pre-spawn Ranger inventory check was interrupted"
      ;;
    *)
      return "${status_code}"
      ;;
  esac
  return 1
}

validate_runtime_gates() {
  local validator_source
  validator_source="${RANGER_ROS_WS}/src/carla_extended_ackermann_control/src"
  python3 "${script_dir}/validate_runtime_gates.py" \
    --validator-source "${validator_source}" \
    --baseline-manifest "${RANGER_BASELINE_MANIFEST}" \
    --physical-manifest "${RANGER_PHYSICAL_MANIFEST}"
}

validate_ranger_actor_ready() {
  local actor_id cache_dir status_code
  cache_dir="$(mktemp -d \
    "${TMPDIR:-/tmp}/camrod-carla-actor-preflight.XXXXXX")"
  if actor_id="$(
    (
      trap 'rm -rf -- "${cache_dir}"' EXIT
      export PYTHON_EGG_CACHE="${cache_dir}"
      export CARLA_PYTHON_EGG_CACHE="${cache_dir}"
      export RANGER_PYTHON_EGG_CACHE="${cache_dir}"
      virtual_carla_use_python_egg || exit 1
      PYTHONDONTWRITEBYTECODE=1 python3 \
        "${script_dir}/check_ranger_actor.py" \
        --host "${CARLA_HOST}" \
        --port "${CARLA_PORT}" \
        --role-name "${CARLA_ROLE_NAME}" \
        --accepted-python-egg "${CARLA_PYTHON_EGG}" \
        --private-egg-cache "${cache_dir}" \
        --actor-id-only
    )
  )"; then
    :
  else
    status_code=$?
    return "${status_code}"
  fi
  if [[ ! "${actor_id}" =~ ^[1-9][0-9]*$ ]]; then
    virtual_carla_die \
      "Ranger actor preflight returned a malformed actor id: ${actor_id@Q}"
    return 1
  fi
  RANGER_LIVE_ACTOR_ID="${actor_id}"
  virtual_carla_log \
    "Ranger actor preflight ready: actor_id=${RANGER_LIVE_ACTOR_ID} type=vehicle.ranger.default role_name=${CARLA_ROLE_NAME} endpoint=${CARLA_HOST}:${CARLA_PORT}"
}

validate_physical_bridge_ready() {
  local actor_id status_code
  if actor_id="$(timeout --signal=INT --kill-after=2s 9s \
      python3 "${script_dir}/check_physical_bridge_status.py" \
        --role-name "${CARLA_ROLE_NAME}" \
        --timeout-seconds 5 \
        --actor-id-only)"; then
    if [[ ! "${actor_id}" =~ ^[1-9][0-9]*$ ]]; then
      virtual_carla_die \
        "physical 4WS preflight returned a malformed actor id: ${actor_id@Q}"
      return 1
    fi
    PHYSICAL_BRIDGE_ACTOR_ID="${actor_id}"
    virtual_carla_log \
      "physical 4WS bridge ready: role=${CARLA_ROLE_NAME} actor_id=${PHYSICAL_BRIDGE_ACTOR_ID} backend=PHYSX_FOUR_WHEEL_STEERING"
    return 0
  else
    status_code=$?
  fi
  case "${status_code}" in
    124|137)
      virtual_carla_die \
        "physical 4WS status preflight exceeded its hard 9s bound and was killed; start server -> bridge -> spawn -> camrod, then retry manual"
      ;;
    130)
      virtual_carla_die "physical 4WS status preflight was interrupted"
      ;;
    *)
      return "${status_code}"
      ;;
  esac
  return 1
}

validate_manual_actor_binding() {
  local live_actor_id="${RANGER_LIVE_ACTOR_ID:-}"
  local bridge_actor_id="${PHYSICAL_BRIDGE_ACTOR_ID:-}"
  if [[ ! "${live_actor_id}" =~ ^[1-9][0-9]*$ || \
        ! "${bridge_actor_id}" =~ ^[1-9][0-9]*$ ]]; then
    virtual_carla_die \
      "manual actor identity check is missing a valid live or bridge actor id; manual remains disabled"
    return 1
  fi
  if [[ "${live_actor_id}" != "${bridge_actor_id}" ]]; then
    virtual_carla_die \
      "manual actor identity mismatch: live CARLA actor_id=${live_actor_id}, physical 4WS bridge actor_id=${bridge_actor_id}; manual remains disabled. Stop CAMROD completely, rerun ./scripts/virtual_carla/run.sh camrod, wait until ready, then retry manual"
    return 1
  fi
  virtual_carla_log \
    "manual actor identity matched: actor_id=${live_actor_id}"
}

validate_camrod_ui_manual_ready() {
  local status_code
  if timeout --signal=INT --kill-after=2s 7s \
      python3 "${script_dir}/check_camrod_ui_manual_ready.py" \
        --base-url "${CAMROD_UI_URL}" \
        --timeout-seconds 2; then
    return 0
  else
    status_code=$?
  fi
  case "${status_code}" in
    124|137)
      virtual_carla_die \
        "CAMROD UI manual preflight exceeded its hard 7s bound and was killed; verify ${CAMROD_UI_URL}, press STOP, then retry manual"
      ;;
    130)
      virtual_carla_die "CAMROD UI manual preflight was interrupted"
      ;;
    *)
      return "${status_code}"
      ;;
  esac
  return 1
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
  local failures=0 configuration_failures=0 static_failures=0 map_asset

  virtual_carla_print_environment >&2
  virtual_carla_require_var RANGER_CARLA_ROOT || failures=$((failures + 1))
  virtual_carla_require_dir \
    "${RANGER_CARLA_ROOT}" "Ranger/CARLA repository" || \
    configuration_failures=$((configuration_failures + 1))
  case "${RANGER_UE_ROOT}" in
    /absolute/path/to/*|/path/to/*)
      virtual_carla_die \
        "RANGER_UE_ROOT is still a template placeholder; edit ${RANGER_ENV_FILE} or export the intended path" || true
      configuration_failures=$((configuration_failures + 1))
      ;;
  esac
  if [[ -x "${CARLA_ROOT}/CarlaUE4.sh" && ! -f "${CARLA_UPROJECT}" ]]; then
    virtual_carla_die \
      "CARLA_ROOT points to a packaged runtime, not the pipeline source checkout: ${CARLA_ROOT}; unset the caller CARLA_ROOT or point it to RANGER_WORK_ROOT/src/carla" || true
    configuration_failures=$((configuration_failures + 1))
  fi
  failures=$((failures + configuration_failures))
  if [[ "${configuration_failures}" -ne 0 ]]; then
    virtual_carla_die \
      "doctor configuration phase failed; dependent file, gate, ROS and renderer checks were skipped" || true
    return 1
  fi

  virtual_carla_require_executable \
    "${UE_EDITOR}" "UE4Editor" || static_failures=$((static_failures + 1))
  virtual_carla_require_file \
    "${CARLA_UPROJECT}" "CarlaUE4 project" || static_failures=$((static_failures + 1))
  map_asset="$(virtual_carla_map_asset_file)"
  virtual_carla_require_file \
    "${map_asset}" "CARLA custom map asset" || static_failures=$((static_failures + 1))
  virtual_carla_require_file \
    "${RANGER_SPAWN_FILE}" "Ranger spawn JSON" || static_failures=$((static_failures + 1))
  require_common_runtime_files || static_failures=$((static_failures + 1))
  virtual_carla_require_dir \
    "${RANGER_ROS_BRIDGE_SOURCE}" "CARLA ROS bridge source" || \
    static_failures=$((static_failures + 1))
  virtual_carla_require_dir \
    "${RANGER_EVIDENCE_ROOT}" "Ranger evidence root" || \
    static_failures=$((static_failures + 1))
  virtual_carla_require_file \
    "${CARLA_ROS_BRIDGE_WS}/install/local_setup.bash" \
    "CARLA ROS bridge install" || static_failures=$((static_failures + 1))
  virtual_carla_require_file \
    "${RANGER_ROS_WS}/install/local_setup.bash" \
    "Ranger ROS install" || static_failures=$((static_failures + 1))
  failures=$((failures + static_failures))
  if [[ "${static_failures}" -ne 0 ]]; then
    virtual_carla_die \
      "doctor static prerequisite phase failed; dependent JSON, ROS, Python API and renderer checks were skipped" || true
    virtual_carla_die "doctor found ${failures} failed check(s)" || true
    return 1
  fi

  validate_spawn_file || failures=$((failures + 1))
  validate_render_timing_contract || failures=$((failures + 1))
  validate_runtime_gates || failures=$((failures + 1))

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
    manual_command

    printf '# Export once in every terminal (adjust overrides as needed)\n'
    printf 'export RANGER_CARLA_ROOT=%q\n' "${RANGER_CARLA_ROOT}"
    printf 'export CARLA_ROOT=%q\n' "${CARLA_ROOT}"
    printf 'export UE_ROOT=%q\n' "${UE_ROOT}"
    printf 'export RANGER_UE_ROOT=%q\n' "${RANGER_UE_ROOT}"
    printf 'export CARLA_ROS_BRIDGE_WS=%q\n' "${CARLA_ROS_BRIDGE_WS}"
    printf 'export RANGER_ROS_WS=%q\n' "${RANGER_ROS_WS}"
    printf 'export ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID}"
    printf 'export RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION}"
    printf 'export CARLA_RENDER_MODE=%q\n\n' "${CARLA_RENDER_MODE}"
    printf '# REQUIRED lifecycle order (four terminals)\n'
    printf '# Wait for each preceding stage to report success before continuing.\n'
    printf '%q server\n' "${script_dir}/run.sh"
    printf '%q bridge\n' "${script_dir}/run.sh"
    printf '%q spawn\n' "${script_dir}/run.sh"
    printf '%q camrod\n\n' "${script_dir}/run.sh"
    printf '# Optional after CAMROD and physical 4WS status are healthy\n'
    printf '%q manual\n\n' "${script_dir}/run.sh"

    printf '# Expanded server command\n'
    print_command "${SERVER_COMMAND[@]}"
    printf '\n# Expanded ROS bridge command (after sourcing validated overlays)\n'
    print_command "${BRIDGE_COMMAND[@]}"
    printf '\n# Expanded actor spawn command\n'
    print_command "${SPAWN_COMMAND[@]}"
    printf '\n# Expanded CAMROD command; run.sh creates the fresh egg cache\n'
    print_command "${CAMROD_COMMAND[@]}"
    printf '\n# Expanded guarded keyboard command (manual preflight runs first)\n'
    print_command "${MANUAL_COMMAND[@]}"
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
    validate_render_timing_contract
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
    refuse_hot_respawn
    validate_spawn_file
    validate_no_ranger_actor_present
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
    validate_runtime_gates
    validate_spawn_file
    validate_render_timing_contract
    validate_ranger_actor_ready
    validate_carla_sensor_streams

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
  manual)
    [[ -t 0 ]] || virtual_carla_die \
      "manual requires an interactive terminal (TTY)"
    require_common_runtime_files
    virtual_carla_source_ros true true
    virtual_carla_verify_package_prefix \
      teleop_twist_keyboard "/opt/ros/${ROS_DISTRO}"
    virtual_carla_verify_package_prefix \
      carla_extended_ackermann_control "${RANGER_ROS_WS}/install"
    virtual_carla_verify_package_prefix \
      camrod_carla_adapter "${CAMROD_WS_ROOT}/install"
    validate_runtime_gates
    validate_spawn_file
    validate_ranger_actor_ready
    validate_camrod_ui_manual_ready
    validate_physical_bridge_ready
    # Close the actor hot-respawn race between the initial actor preflight and
    # the physical bridge status sample before allowing any keyboard command.
    validate_ranger_actor_ready
    validate_manual_actor_binding
    manual_command
    virtual_carla_log \
      "manual input is safety-gated at /control/manual_cmd_vel_ros"
    virtual_carla_log \
      "UI: cancel any active goal with STOP, then press the standalone ENGAGE on the last page (not a B1-B13 destination button); this command never engages automatically"
    virtual_carla_log \
      "keys: i/, forward/reverse; u/o and m/. steer; j/l zero-turn; Shift+J/L crab; k or any unbound key stop; Ctrl-C exits with zero"
    virtual_carla_log \
      "speed=0.20 m/s, turn=0.20 rad/s; diagonal keys keep a 1.0 m Ackermann radius; keep the CARLA and wheel telemetry views visible"
    exec "${MANUAL_COMMAND[@]}"
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
