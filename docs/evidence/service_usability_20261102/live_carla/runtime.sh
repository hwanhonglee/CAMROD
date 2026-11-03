#!/usr/bin/env bash
# HH_261102 - Keep the CARLA plant/bringup underlay while testing only the changed pure CAMROD packages.
set -eo pipefail
trial_root=/home/hong/camrod_service_test_ws
legacy_src=/home/hong/camrod_ws/src
source /home/hong/camrod_ws/config/virtual_carla.env
export XDG_STATE_HOME="${trial_root}/state/service_usability_live_20261102"
export CAMROD_CARLA_MISSION_RECORDS_ROOT="${XDG_STATE_HOME}/mission_records"
export ROS_LOG_DIR="${trial_root}/log/service_usability_live_20261102"
export RANGER_ENV_FILE=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/runtime/service_usability_live_20261102/candidate.env
export RANGER_PHYSICAL_MANIFEST=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/service_usability_live_20261102/ranger_physical_4ws_acceptance_gate.json
export CAMROD_RECORDING_ENVIRONMENT=simulation
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
export CAMROD_MANUAL_LINEAR_LIMIT_MPS=2.0 CAMROD_MANUAL_LATERAL_LIMIT_MPS=2.0
export CAMROD_ENABLE_OPERATOR_WINDOW=false CAMROD_ENABLE_RVIZ=false
export CAMROD_CARLA_DISABLE_GPU_PROFILER=true
# HH_261102 - A tag-only cloned scene is an explicit test selection, never the historical V224 identity.
if [[ "${CAMROD_SERVICE_TAG_SCENE:-0}" == 1 ]]; then
  python3 - <<'PY'
import hashlib, json
from pathlib import Path
receipt = Path('/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/service_usability_tag_alignment_20261102/apply.json')
d = json.loads(receipt.read_text())
assert d['status'] == 'APPLIED' and not d['errors']
assert d['source_disk_reload_contract_unchanged'] and d['non_tag_actor_contracts_unchanged'] and d['target_disk_reload_verified']
assert d['target_map_package'] == '/Game/map_package/Maps/Woraksan_v27_service_tag_20261102/Woraksan_v27_service_tag_20261102'
for group in ('source_files', 'target_files_after'):
    for item in d[group].values():
        assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['sha256'], item['path']
print('Tag-only test scene and unchanged source files verified from APPLIED receipt')
PY
  export CARLA_UE_MAP=/Game/map_package/Maps/Woraksan_v27_service_tag_20261102/Woraksan_v27_service_tag_20261102
  export CARLA_TOWN=map_package/Maps/Woraksan_v27_service_tag_20261102/Woraksan_v27_service_tag_20261102
  export CAMROD_CARLA_MAP_PROFILE=""
fi
source "${legacy_src}/scripts/virtual_carla/env.sh"

mixed_ros() {
  virtual_carla_source_ros true true
  # HH_261102 - The unit2 bringup is not an external-CARLA launcher; do not overlay that package.
  for trial_package in camrod_control camrod_sensing camrod_ui ground_segmentation_ros2; do
    COLCON_CURRENT_PREFIX="${trial_root}/install/${trial_package}" \
      source "${trial_root}/install/${trial_package}/share/${trial_package}/package.bash"
  done
  for trial_package in camrod_control camrod_sensing camrod_ui; do
    actual_prefix="$(ros2 pkg prefix "${trial_package}")"
    [[ "${actual_prefix}" == "${trial_root}/install/${trial_package}" ]] || return 1
  done
  [[ "$(ros2 pkg prefix camrod_bringup)" == /home/hong/camrod_ws/install/camrod_bringup ]] || return 1
  [[ "$(ros2 pkg prefix camrod_carla_adapter)" == /home/hong/camrod_ws/install/camrod_carla_adapter ]] || return 1
}

action="${1:-env}"
if [[ $# -gt 0 ]]; then shift; fi
case "${action}" in
  server|bridge|pacer|spawn|doctor|spectator|operator-ui|guest-ui)
    exec "${legacy_src}/scripts/virtual_carla/run.sh" "${action}" "$@"
    ;;
  ros)
    mixed_ros
    exec "$@"
    ;;
  env)
    mixed_ros
    printf 'ROS_DOMAIN_ID=%s\nCARLA_HOST=%s\nCARLA_PORT=%s\nXDG_STATE_HOME=%s\n' \
      "${ROS_DOMAIN_ID}" "${CARLA_HOST}" "${CARLA_PORT}" "${XDG_STATE_HOME}"
    for trial_package in camrod_control camrod_sensing camrod_ui camrod_bringup camrod_carla_adapter; do
      ros2 pkg prefix "${trial_package}"
    done
    ;;
  camrod)
    mixed_ros
    virtual_carla_require_dds_transport
    # HH_261102 - Never suppress gate verification to launch a modified control binary.
    python3 "${legacy_src}/scripts/virtual_carla/validate_runtime_gates.py" \
      --validator-source "${RANGER_ROS_WS}/src/carla_extended_ackermann_control/src" \
      --baseline-manifest "${RANGER_BASELINE_MANIFEST}" --physical-manifest "${RANGER_PHYSICAL_MANIFEST}"
    egg_cache="$(mktemp -d /tmp/camrod-service-usability-egg.XXXXXX)"
    export CARLA_PYTHON_EGG_CACHE="${egg_cache}" RANGER_PYTHON_EGG_CACHE="${egg_cache}"
    virtual_carla_use_python_egg
    export YOLOV9_MODEL_PATH="${CAMROD_CARLA_YOLO_MODEL_PATH}"
    exec ros2 launch camrod_carla_adapter camrod_carla_develop_site_geometry.launch.py \
      "role_name:=${CARLA_ROLE_NAME}" "host:=${CARLA_HOST}" "port:=${CARLA_PORT}" \
      "accepted_carla_python_egg:=${CARLA_PYTHON_EGG}" "python_egg_cache:=${egg_cache}" \
      "verified_baseline_manifest:=${RANGER_BASELINE_MANIFEST}" \
      "verified_physical_four_wheel_manifest:=${RANGER_PHYSICAL_MANIFEST}" \
      extended_mode_backend:=PHYSX_FOUR_WHEEL_STEERING \
      "map_alignment_file:=${CAMROD_MAP_ALIGNMENT_FILE}" \
      "camrod_launch_defaults_file:=${CAMROD_LAUNCH_DEFAULTS_FILE}" \
      "camrod_map_path:=${CAMROD_LANELET_MAP}" \
      "planning_state_machine_camping_sites_yaml:=${CAMROD_CARLA_CAMPING_SITES_YAML}" \
      "mission_records_root:=${CAMROD_CARLA_MISSION_RECORDS_ROOT}" \
      launch_sensor_relay:=true compressed_image_max_rate_hz:=5.0 raw_image_max_rate_hz:=10.0 \
      enable_plugin_api:=true enable_api_ui:=true enable_operator_ui_window:=false \
      api_ui_port:=8010 operator_ui_window_url:=http://127.0.0.1:8010 rviz:=false \
      "enable_voice:=${CAMROD_ENABLE_VOICE}" "$@"
    ;;
  *) printf 'Unknown test action: %s\n' "${action}" >&2; exit 2 ;;
esac
