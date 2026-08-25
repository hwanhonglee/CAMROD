#!/usr/bin/env bash
# Provision CAMROD dependencies while preserving the approved CARLA underlays.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: setup.sh [--no-rosdep] [--update]

Runs CAMROD's canonical setup_camrod.sh after validating the independent
Ranger/CARLA repository and the standard CARLA ROS bridge underlay.

  --no-rosdep  skip rosdep in setup_camrod.sh
  --update     forward the interactive external-repository update request
  -h, --help   show this help without changing the host

Required before running:
  export RANGER_CARLA_ROOT=/absolute/path/to/ranger-carla-4ws-pipeline

CARLA_ROOT, UE_ROOT, CARLA_ROS_BRIDGE_WS and RANGER_ROS_WS may be overridden.
This script installs dependencies only. It never launches CARLA or sends motion.
EOF
}

args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rosdep|--update)
      args+=("$1")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[virtual_carla] ERROR: unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/env.sh"

virtual_carla_require_var RANGER_CARLA_ROOT
virtual_carla_require_dir "${RANGER_CARLA_ROOT}" "Ranger/CARLA repository"
virtual_carla_require_dir "${RANGER_ROS_WS}/src" "Ranger ROS source workspace"
virtual_carla_require_file \
  "${CARLA_ROS_BRIDGE_WS}/install/local_setup.bash" \
  "CARLA ROS bridge install"
virtual_carla_require_executable \
  "${CAMROD_SRC_ROOT}/setup_camrod.sh" "CAMROD setup script"

# Start from ROS plus the explicit bridge/Ranger underlays. A Ranger install is
# optional on the first setup pass; build.sh creates and then verifies it.
virtual_carla_source_ros false true
virtual_carla_verify_package_prefix \
  carla_ros_bridge "${CARLA_ROS_BRIDGE_WS}/install"
if [[ -f "${RANGER_ROS_WS}/install/local_setup.bash" ]]; then
  virtual_carla_verify_package_prefix \
    carla_extended_ackermann_control "${RANGER_ROS_WS}/install"
else
  virtual_carla_log \
    "Ranger install is not present yet; build.sh will create it"
fi

virtual_carla_log "running CAMROD canonical dependency setup"
CAMROD_EXTRA_PREFIX_ROOTS="${CAMROD_EXTRA_PREFIX_ROOTS}" \
  "${CAMROD_SRC_ROOT}/setup_camrod.sh" "${args[@]}"

# Verify that setup did not replace the intentionally selected underlays.
virtual_carla_source_ros false true
virtual_carla_verify_package_prefix \
  carla_ros_bridge "${CARLA_ROS_BRIDGE_WS}/install"
if [[ -f "${RANGER_ROS_WS}/install/local_setup.bash" ]]; then
  virtual_carla_verify_package_prefix \
    carla_extended_ackermann_control "${RANGER_ROS_WS}/install"
fi
virtual_carla_log "setup complete; next: ${script_dir}/build.sh"
