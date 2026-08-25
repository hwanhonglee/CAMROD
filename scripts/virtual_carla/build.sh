#!/usr/bin/env bash
# Build the independent Ranger ROS overlay before the CAMROD workspace.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: build.sh [--ranger-only | --camrod-only] [colcon-build-args...]

Default order:
  1. build Ranger 4WS messages/controller/rqt in RANGER_ROS_WS/install
  2. build CAMROD through its canonical colcon_build.sh (including its UI)

--ranger-only and --camrod-only select one stage. Remaining arguments are
forwarded only to CAMROD's colcon_build.sh. The Ranger repository is an external
path dependency identified by RANGER_CARLA_ROOT; it is never a Git submodule.

This builds ROS workspaces only. Build UE 4.26/CARLA and import the Ranger asset
with the scripts in the Ranger repository before this step.
EOF
}

build_ranger=true
build_camrod=true
camrod_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ranger-only)
      build_ranger=true
      build_camrod=false
      shift
      ;;
    --camrod-only)
      build_ranger=false
      build_camrod=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      camrod_args+=("$1")
      shift
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/env.sh"

virtual_carla_require_var RANGER_CARLA_ROOT
virtual_carla_require_dir "${RANGER_CARLA_ROOT}" "Ranger/CARLA repository"
virtual_carla_require_file \
  "${CARLA_ROS_BRIDGE_WS}/install/local_setup.bash" \
  "CARLA ROS bridge install"

ranger_packages=(
  carla_extended_ackermann_msgs
  carla_extended_ackermann_control
  rqt_extended_ackermann
)

if [[ "${build_ranger}" == "true" ]]; then
  virtual_carla_require_dir "${RANGER_ROS_WS}/src" "Ranger ROS source workspace"
  virtual_carla_source_ros false false
  virtual_carla_verify_package_prefix \
    carla_ros_bridge "${CARLA_ROS_BRIDGE_WS}/install"

  mapfile -t discovered_packages < <(
    colcon list --base-paths "${RANGER_ROS_WS}/src" --names-only
  )
  for package in "${ranger_packages[@]}"; do
    if ! printf '%s\n' "${discovered_packages[@]}" | grep -Fxq "${package}"; then
      virtual_carla_die \
        "Ranger ROS source package is missing: ${package} (${RANGER_ROS_WS}/src)"
      exit 1
    fi
  done

  mkdir -p \
    "${RANGER_ROS_WS}/build" \
    "${RANGER_ROS_WS}/install" \
    "${RANGER_ROS_WS}/log"
  virtual_carla_log "building Ranger ROS overlay: ${RANGER_ROS_WS}"
  (
    cd "${RANGER_ROS_WS}"
    colcon --log-base "${RANGER_ROS_WS}/log" build \
      --symlink-install \
      --base-paths "${RANGER_ROS_WS}/src" \
      --build-base "${RANGER_ROS_WS}/build" \
      --install-base "${RANGER_ROS_WS}/install" \
      --packages-select "${ranger_packages[@]}"
  )
fi

virtual_carla_require_file \
  "${RANGER_ROS_WS}/install/local_setup.bash" "Ranger ROS install"
virtual_carla_source_ros false true
virtual_carla_verify_package_prefix \
  carla_extended_ackermann_control "${RANGER_ROS_WS}/install"
virtual_carla_verify_package_prefix \
  rqt_extended_ackermann "${RANGER_ROS_WS}/install"

if [[ "${build_camrod}" == "true" ]]; then
  virtual_carla_require_executable \
    "${CAMROD_SRC_ROOT}/colcon_build.sh" "CAMROD build wrapper"
  virtual_carla_log \
    "building CAMROD with approved external prefixes and canonical UI build"
  CAMROD_EXTRA_PREFIX_ROOTS="${CAMROD_EXTRA_PREFIX_ROOTS}" \
    "${CAMROD_SRC_ROOT}/colcon_build.sh" "${camrod_args[@]}"

  virtual_carla_source_ros true true
  virtual_carla_verify_package_prefix \
    camrod_carla_adapter "${CAMROD_WS_ROOT}/install"
fi

virtual_carla_log "build complete"
virtual_carla_log \
  "source ${CAMROD_WS_ROOT}/install/local_setup.bash before manual ROS use"
