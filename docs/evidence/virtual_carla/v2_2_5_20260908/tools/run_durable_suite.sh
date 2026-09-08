#!/usr/bin/env bash
# Evidence orchestration only. Default is an offline plan, never motion.
# --start-at STAGE / --continuation-of PRIOR_ROOT pass through unchanged.
set -euo pipefail
suite_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CAMROD_SRC_ROOT=/home/hong/camrod_ws/src
export CAMROD_WS_ROOT=/home/hong/camrod_ws
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
export RANGER_EVIDENCE_ROOT="${RANGER_CARLA_ROOT}/.work/evidence"
export ROS_DOMAIN_ID=5 ROS_LOCALHOST_ONLY=0 DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000
export CARLA_RENDER_MODE=onscreen CAMROD_CARLA_PARKING_COMPLETION=reverse
export CAMROD_GUEST_FINAL_RETURN_AUTHORITY=robot
export CAMROD_OPERATOR_CDP_URL=http://127.0.0.1:9224
export CAMROD_OPERATOR_UI_URL=http://127.0.0.1:8010
export CAMROD_GUEST_CDP_URL=http://127.0.0.1:9223
export CAMROD_GUEST_UI_URL=http://127.0.0.1:8012
unset CARLA_ROOT CARLA_PYTHON_EGG CARLA_UE_MAP CARLA_TOWN CAMROD_CARLA_MAP_PROFILE
source "${CAMROD_SRC_ROOT}/scripts/virtual_carla/env.sh"
# env.sh supplies a direct-run default. Pin the canonical current site map
# AFTER sourcing it, before any nested matrix/site_access invocation.
source "${CAMROD_SRC_ROOT}/scripts/virtual_carla/map_profiles.sh"
export CAMROD_CARLA_MAP_PROFILE="${CAMROD_CARLA_SITE_ACCESS_PROFILE_ID}"
export CARLA_UE_MAP="${CAMROD_CARLA_SITE_ACCESS_UE_MAP}"
export CARLA_TOWN="${CAMROD_CARLA_SITE_ACCESS_TOWN}"
export CAMROD_VIRTUAL_CARLA_ENTRYPOINT="${CAMROD_SRC_ROOT}/scripts/virtual_carla/site_access.sh"
case "${1:-plan}" in
  guest)
    exec "${CAMROD_VIRTUAL_CARLA_ENTRYPOINT}" guest-ui
    ;;
  run)
    virtual_carla_source_ros true true
    source "${RANGER_CARLA_ROOT}/.work/deps/desktop-tools/env.sh"
    ;;
esac
exec /usr/bin/python3 "${suite_dir}/run_durable_suite.py" "$@"
