#!/usr/bin/env bash
set -euo pipefail

# Sets runtime/build environment for standalone Kimera-VIO binaries/libraries
# built under camrod_localization/external/vio_bridge.
#
# This does NOT replace sourcing ROS workspace install/setup.bash.
# Use both when needed:
#   source /home/hong/camrod_ws/install/setup.bash
#   source /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge/setup_env.sh

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
VIO_BRIDGE_ROOT="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"

DBOW2_LIB="${VIO_BRIDGE_ROOT}/install/dbow2/lib"
DBOW2_CMAKE="${VIO_BRIDGE_ROOT}/install/dbow2"
KIMERA_BUILD_BIN="${VIO_BRIDGE_ROOT}/build/Kimera-VIO"

if [[ -d "${DBOW2_LIB}" ]]; then
  export LD_LIBRARY_PATH="${DBOW2_LIB}:${LD_LIBRARY_PATH:-}"
fi

if [[ -d "${DBOW2_CMAKE}" ]]; then
  export CMAKE_PREFIX_PATH="${DBOW2_CMAKE}:${CMAKE_PREFIX_PATH:-}"
fi

if [[ -d "${KIMERA_BUILD_BIN}" ]]; then
  export PATH="${KIMERA_BUILD_BIN}:${PATH}"
fi

echo "[vio_bridge/setup_env] configured"
echo "  VIO_BRIDGE_ROOT=${VIO_BRIDGE_ROOT}"
echo "  DBOW2_LIB=${DBOW2_LIB}"
echo "  KIMERA_BUILD_BIN=${KIMERA_BUILD_BIN}"
