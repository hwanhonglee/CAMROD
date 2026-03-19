#!/usr/bin/env bash
# 2026-03-19: Avoid `set -u` because ROS setup scripts reference optional
# env vars (e.g., AMENT_TRACE_SETUP_FILES, AMENT_PYTHON_EXECUTABLE) that may
# be unset in clean containers.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-/home/hong/camrod_ws}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "[entrypoint.camrod] WARN: ${WS_DIR}/install/setup.bash not found"
fi

exec "$@"
