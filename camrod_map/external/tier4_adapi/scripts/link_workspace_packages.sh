#!/usr/bin/env bash
# HH_260409: Symlink workflow deprecated.
# Use explicit base-paths instead:
#   colcon build --base-paths src src/camrod_map/external/tier4_adapi ...

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

for name in tier4_system_msgs autoware_component_interface_utils tier4_adapi_rviz_plugin; do
  target="${SRC_ROOT}/${name}"
  if [[ -L "${target}" ]]; then
    rm -f "${target}"
    echo "[tier4-adapi] removed legacy symlink: ${target}"
  fi
done

echo "[tier4-adapi] use:"
echo "  colcon build --base-paths src src/camrod_map/external/tier4_adapi \\"
echo "    --packages-select tier4_system_msgs autoware_component_interface_utils tier4_adapi_rviz_plugin --symlink-install"
