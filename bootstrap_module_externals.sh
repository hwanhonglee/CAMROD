#!/usr/bin/env bash
set -euo pipefail

# Bootstrap module-level external sources that are practical to vendor.
# This does NOT attempt to vendor the full ROS 2 core stack (rclcpp/rclpy/...).
#
# Usage:
#   ./src/bootstrap_module_externals.sh
#   ./src/bootstrap_module_externals.sh --update

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ "$(basename "${SCRIPT_DIR}")" == "src" && -d "${SCRIPT_DIR}/camrod_bringup" && -d "${PARENT_DIR}/.colcon" ]]; then
  WS_ROOT="${PARENT_DIR}"
  SRC_PREFIX="src/"
elif [[ -d "${SCRIPT_DIR}/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  SRC_PREFIX=""
elif [[ -d "${SCRIPT_DIR}/src/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  SRC_PREFIX="src/"
else
  echo "[bootstrap] unsupported tree layout at: ${SCRIPT_DIR}" >&2
  exit 1
fi

cd "${WS_ROOT}"

UPDATE_MODE=0
if [[ "${1:-}" == "--update" ]]; then
  UPDATE_MODE=1
fi

clone_or_update() {
  local url="$1"
  local ref="$2"
  local target="$3"

  mkdir -p "$(dirname "${target}")"
  if [[ -d "${target}" && ! -d "${target}/.git" ]]; then
    echo "[bootstrap] keep existing non-git tree ${target}"
    return
  fi
  if [[ ! -d "${target}/.git" ]]; then
    echo "[bootstrap] clone ${url} -> ${target} (${ref})"
    git clone --branch "${ref}" --depth 1 "${url}" "${target}"
    return
  fi

  if [[ "${UPDATE_MODE}" -eq 1 ]]; then
    echo "[bootstrap] update ${target} (${ref})"
    git -C "${target}" fetch --depth 1 origin "${ref}"
    git -C "${target}" checkout -f FETCH_HEAD
  else
    echo "[bootstrap] keep existing ${target}"
  fi
}

# Sensing/localization externals already used by this workspace.
clone_or_update "https://github.com/KumarRobotics/ublox.git" "master" \
  "${SRC_PREFIX}camrod_sensing/external/ublox"
clone_or_update "https://github.com/cra-ros-pkg/robot_localization.git" "humble-devel" \
  "${SRC_PREFIX}camrod_localization/external/robot_localization"

# NOTE: camrod_planning/external already contains split Nav2 packages.
# Do not clone a full navigation2 tree here, otherwise duplicate package names
# can break colcon package discovery.

# Additional commonly used dependency stacks to reduce /opt/ros binary-only scope.
clone_or_update "https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git" "master" \
  "${SRC_PREFIX}camrod_map/external/lanelet2"
clone_or_update "https://github.com/ros-perception/vision_opencv.git" "humble" \
  "${SRC_PREFIX}camrod_perception/external/vision_opencv"
clone_or_update "https://github.com/ros-perception/vision_msgs.git" "ros2" \
  "${SRC_PREFIX}camrod_common/external/vision_msgs"
clone_or_update "https://github.com/ros-perception/perception_pcl.git" "humble" \
  "${SRC_PREFIX}camrod_sensing/external/perception_pcl"
clone_or_update "https://github.com/ros-perception/laser_geometry.git" "ros2" \
  "${SRC_PREFIX}camrod_planning/external/laser_geometry"

cat <<'EOF'
[bootstrap] done.
Next steps:
  1) ./build_camrod_all.sh
  2) source install/setup.bash
EOF
