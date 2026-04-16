#!/usr/bin/env bash
# HH_260408: Source this before colcon build/launch to resolve local vendor dependencies.
# HH_260408: Do not change caller shell options (avoid set -u breakage on setup.bash).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# HH_260408: Keep vendor dependencies inside camrod_map/external/tier4_adapi/vendor.
VENDOR_PREFIX="${SCRIPT_DIR}/vendor/autoware_apt/extract/opt/ros/humble"

if [ ! -d "${VENDOR_PREFIX}" ]; then
  echo "[tier4-adapi] vendor prefix not found: ${VENDOR_PREFIX}" >&2
  echo "[tier4-adapi] run: ${SCRIPT_DIR}/scripts/install_tier4_adapi_vendor_deps.sh" >&2
  return 1 2>/dev/null || exit 1
fi

export AMENT_PREFIX_PATH="${VENDOR_PREFIX}:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${VENDOR_PREFIX}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${VENDOR_PREFIX}/lib:${VENDOR_PREFIX}/local/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${VENDOR_PREFIX}/local/lib/python3.10/dist-packages:${VENDOR_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH:-}"
export PATH="${VENDOR_PREFIX}/bin:${PATH}"
