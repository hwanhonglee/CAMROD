#!/usr/bin/env bash
# Run bringup.launch.py and exit the terminal when it terminates.
# Usage: ./run_bringup.sh [extra launch args...]
#   e.g. ./run_bringup.sh sim:=true
#        ./run_bringup.sh rviz:=false

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
WS="${SCRIPT_DIR}/../../.."
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

echo "[run_bringup] Starting bringup.launch.py $*"
ros2 launch camrod_bringup bringup.launch.py "$@"

# Terminal closes automatically when ros2 launch exits (Ctrl+C or crash).
# HH_260805 - ROS launch performs graceful child shutdown; clean_before_launch
# clears any stale process left by an abnormal termination before the next run.
