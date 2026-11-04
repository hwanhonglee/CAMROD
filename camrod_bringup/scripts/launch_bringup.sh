#!/usr/bin/env bash
# 261104 - CAMROD bringup launcher for systemd.
# Reproduces the environment an interactive login shell provides (~/.bashrc),
# because systemd services do not read .bashrc and cannot "source" by themselves.
set -euo pipefail

export HOME=/home/nvidia

# --- ~/.bashrc 와 동일한 ROS 런타임 환경 ---
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTRTPS_DEFAULT_PROFILES_FILE="${HOME}/.ros/fastdds.xml"
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
export PATH="/usr/local/cuda/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

# setup.bash 들은 미정의 변수를 참조하므로 nounset 을 잠시 해제한다.
set +u
source /opt/ros/humble/setup.bash
source "${HOME}/camrod_ws/install/setup.bash"
set -u

cd "${HOME}/camrod_ws"
exec ros2 launch camrod_bringup bringup.launch.py "$@"
