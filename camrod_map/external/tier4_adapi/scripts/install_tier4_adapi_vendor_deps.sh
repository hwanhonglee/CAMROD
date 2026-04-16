#!/usr/bin/env bash
# HH_260408: Install Tier4 ADAPI RViz plugin binary dependencies without sudo.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# HH_260408: Install vendor deps under camrod_map/external/tier4_adapi/vendor.
VENDOR_DIR="${BASE_DIR}/vendor/autoware_apt"
EXTRACT_DIR="${VENDOR_DIR}/extract"
PREFIX="${EXTRACT_DIR}/opt/ros/humble"

mkdir -p "${VENDOR_DIR}" "${EXTRACT_DIR}"

cd /tmp
apt download \
  ros-humble-autoware-adapi-specs \
  ros-humble-autoware-adapi-v1-msgs \
  ros-humble-autoware-cmake \
  ros-humble-autoware-vehicle-msgs \
  ros-humble-autoware-common-msgs \
  ros-humble-autoware-planning-msgs

for deb in \
  /tmp/ros-humble-autoware-adapi-specs_*.deb \
  /tmp/ros-humble-autoware-adapi-v1-msgs_*.deb \
  /tmp/ros-humble-autoware-cmake_*.deb \
  /tmp/ros-humble-autoware-vehicle-msgs_*.deb \
  /tmp/ros-humble-autoware-common-msgs_*.deb \
  /tmp/ros-humble-autoware-planning-msgs_*.deb
do
  dpkg-deb -x "${deb}" "${EXTRACT_DIR}"
done

# HH_260408: colcon expects local_setup.* on AMENT_PREFIX_PATH entries.
cat > "${PREFIX}/local_setup.bash" <<'EOS'
#!/usr/bin/env bash
return 0 2>/dev/null || exit 0
EOS
chmod +x "${PREFIX}/local_setup.bash"
cp "${PREFIX}/local_setup.bash" "${PREFIX}/local_setup.sh"

cat > "${VENDOR_DIR}/setup.bash" <<'EOS'
#!/usr/bin/env bash
_VEND_PREFIX="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/extract/opt/ros/humble"
if [ -d "${_VEND_PREFIX}" ]; then
  export AMENT_PREFIX_PATH="${_VEND_PREFIX}:${AMENT_PREFIX_PATH}"
  export CMAKE_PREFIX_PATH="${_VEND_PREFIX}:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${_VEND_PREFIX}/lib:${_VEND_PREFIX}/local/lib:${LD_LIBRARY_PATH}"
  export PYTHONPATH="${_VEND_PREFIX}/local/lib/python3.10/dist-packages:${_VEND_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH}"
  export PATH="${_VEND_PREFIX}/bin:${PATH}"
fi
unset _VEND_PREFIX
EOS
chmod +x "${VENDOR_DIR}/setup.bash"

echo "[tier4-adapi] vendor dependencies installed: ${PREFIX}"
echo "[tier4-adapi] next: colcon build --base-paths src src/camrod_map/external/tier4_adapi --packages-select tier4_system_msgs autoware_component_interface_utils tier4_adapi_rviz_plugin --symlink-install"
