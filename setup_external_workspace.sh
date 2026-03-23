#!/usr/bin/env bash
set -euo pipefail

# One-shot setup for CAMROD externalized workspace.
# - bootstrap external sources
# - install system dependencies via rosdep
# - build with external trees
#
# Usage:
#   ./setup_external_workspace.sh
#   ./setup_external_workspace.sh --packages-up-to camrod_bringup

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ "$(basename "${SCRIPT_DIR}")" == "src" && -d "${SCRIPT_DIR}/camrod_bringup" && -d "${PARENT_DIR}/.colcon" ]]; then
  WS_ROOT="${PARENT_DIR}"
  BASE_ROOT="src"
elif [[ -d "${SCRIPT_DIR}/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  BASE_ROOT="."
elif [[ -d "${SCRIPT_DIR}/src/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  BASE_ROOT="src"
else
  echo "[setup_external_workspace] unsupported tree layout: ${SCRIPT_DIR}" >&2
  exit 1
fi

cd "${WS_ROOT}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[setup] bootstrap externals"
"${SCRIPT_DIR}/bootstrap_module_externals.sh"

echo "[setup] rosdep install"
# Build a canonical package path list to avoid duplicate package discovery
# from symlinked aliases (e.g. avg_msgs -> camrod_common/avg_msgs).
mapfile -t ROSDEP_PATHS < <(
  find "${BASE_ROOT}" -name package.xml -type f -print \
    | xargs -r -n1 dirname \
    | while read -r p; do readlink -f "${p}"; done \
    | sort -u
)
rosdep install --from-paths "${ROSDEP_PATHS[@]}" --ignore-src -r -y

echo "[setup] build"
"${SCRIPT_DIR}/build_camrod_all.sh" "$@"

echo "[setup] done. source install/setup.bash"
