#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-/home/hong/camrod_ws}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "[entrypoint.camrod] WARN: ${WS_DIR}/install/setup.bash not found"
fi

exec "$@"
