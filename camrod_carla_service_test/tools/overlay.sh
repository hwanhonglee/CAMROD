#!/usr/bin/env bash
# HH_261103 - Reuse virtual plant ownership and selected tagged-native packages, without replacing virtual bringup.
set -eo pipefail
[[ $# -ge 2 ]] || { printf 'Usage: bash overlay.sh root.env build|env|run [launch args]\n' >&2; exit 2; }
config="$1"; action="$2"; shift 2
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set -a
source "${config}"
set +a
: "${CAMROD_VIRTUAL_WS_ROOT:?}" "${CAMROD_SERVICE_WS_ROOT:?}" "${RANGER_CARLA_ROOT:?}"
# HH_261103 - Never inherit a physical-robot DDS domain or native manual envelope into this simulation fixture.
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 CAMROD_RECORDING_ENVIRONMENT=simulation
export CAMROD_MANUAL_LINEAR_LIMIT_MPS=2.0 CAMROD_MANUAL_LATERAL_LIMIT_MPS=2.0
export CAMROD_WS_ROOT="${CAMROD_VIRTUAL_WS_ROOT}" CAMROD_SRC_ROOT="${CAMROD_VIRTUAL_WS_ROOT}/src"
source "${CAMROD_SRC_ROOT}/scripts/virtual_carla/env.sh"
virtual_carla_source_ros true true
test_ws="${CAMROD_SERVICE_WS_ROOT}"
[[ -f "${test_ws}/overlay_sources.json" && -f "${test_ws}/src/colcon_build.sh" ]] || exit 2
export CAMROD_EXTRA_PREFIX_ROOTS="${CAMROD_VIRTUAL_WS_ROOT}/install:${RANGER_ROS_WS}/install:${CARLA_ROS_BRIDGE_WS}/install"
if [[ "${action}" == build ]]; then
  # HH_261103 - Canonical wrapper rebuilds the selected UI frontend before installation; nested ground segmentation is explicit.
  exec bash "${test_ws}/src/colcon_build.sh" --base-paths "${test_ws}/src" \
    "${test_ws}/src/camrod_sensing/external/ground_segmentation_ros2" \
    --packages-select camrod_control camrod_sensing camrod_ui ground_segmentation_ros2 camrod_carla_service_test \
    --parallel-workers 2 "$@"
fi
for package in camrod_control camrod_sensing camrod_ui ground_segmentation_ros2 camrod_carla_service_test; do
  COLCON_CURRENT_PREFIX="${test_ws}/install/${package}" \
    source "${test_ws}/install/${package}/share/${package}/package.bash"
  [[ "$(ros2 pkg prefix "${package}")" == "${test_ws}/install/${package}" ]] || exit 2
done
for package in camrod_bringup camrod_carla_adapter; do
  [[ "$(ros2 pkg prefix "${package}")" == "${CAMROD_VIRTUAL_WS_ROOT}/install/${package}" ]] || exit 2
done
case "${action}" in
  env) printf 'Tagged native overlay: %s\nVirtual bringup/adapter: %s\n' "${test_ws}" "${CAMROD_VIRTUAL_WS_ROOT}"; exit 0;;
  run) ;;
  *) printf 'Unknown action\n' >&2; exit 2;;
esac
: "${RANGER_BASELINE_MANIFEST:?}" "${RANGER_PHYSICAL_MANIFEST:?}" "${CAMROD_SERVICE_SCENE_RECEIPT:?}"
virtual_carla_require_dds_transport
python3 "${CAMROD_SRC_ROOT}/scripts/virtual_carla/validate_runtime_gates.py" \
  --validator-source "${RANGER_ROS_WS}/src/carla_extended_ackermann_control/src" \
  --baseline-manifest "${RANGER_BASELINE_MANIFEST}" --physical-manifest "${RANGER_PHYSICAL_MANIFEST}"
scene_exports="$(python3 "${package_dir}/tools/verify_scene.py" "${CAMROD_SERVICE_SCENE_RECEIPT}")"
eval "${scene_exports}"
export ROS_LOCALHOST_ONLY=1 CAMROD_RECORDING_ENVIRONMENT=simulation
export XDG_STATE_HOME="${test_ws}/state/service_usability" ROS_LOG_DIR="${test_ws}/log/runtime"
export CAMROD_CARLA_MISSION_RECORDS_ROOT="${XDG_STATE_HOME}/mission_records"
cache="$(mktemp -d "${test_ws}/.carla-egg.XXXXXXXX")"
export CARLA_PYTHON_EGG_CACHE="${cache}" RANGER_PYTHON_EGG_CACHE="${cache}" PYTHON_EGG_CACHE="${cache}"
virtual_carla_use_python_egg
if [[ -n "${CAMROD_CARLA_YOLO_MODEL_PATH:-}" ]]; then
  export YOLOV9_MODEL_PATH="${CAMROD_CARLA_YOLO_MODEL_PATH}"
fi
exec ros2 launch camrod_carla_service_test service_test.launch.py \
  "role_name:=${CARLA_ROLE_NAME}" "host:=${CARLA_HOST}" "port:=${CARLA_PORT}" \
  "accepted_carla_python_egg:=${CARLA_PYTHON_EGG}" "python_egg_cache:=${cache}" \
  "verified_baseline_manifest:=${RANGER_BASELINE_MANIFEST}" \
  "verified_physical_four_wheel_manifest:=${RANGER_PHYSICAL_MANIFEST}" \
  extended_mode_backend:=PHYSX_FOUR_WHEEL_STEERING \
  launch_sensor_relay:=true compressed_image_max_rate_hz:=5.0 raw_image_max_rate_hz:=10.0 \
  "map_alignment_file:=${CAMROD_MAP_ALIGNMENT_FILE}" "camrod_launch_defaults_file:=${CAMROD_LAUNCH_DEFAULTS_FILE}" \
  "camrod_map_path:=${CAMROD_LANELET_MAP}" "planning_state_machine_camping_sites_yaml:=${CAMROD_CARLA_CAMPING_SITES_YAML}" \
  "mission_records_root:=${CAMROD_CARLA_MISSION_RECORDS_ROOT}" "$@"
