#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <image_tag> <ros2 launch ...>"
  echo "Example: $0 camrod_bringup:humble ros2 launch camrod_bringup bringup.launch.py"
  exit 1
fi

IMAGE="$1"
shift
CMD="$*"

docker run --rm -it \
  --gpus all \
  --network host \
  --ipc host \
  --pid host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "$(pwd)/..:/workspaces/camrod_ws/src:rw" \
  -w /workspaces/camrod_ws \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && ${CMD}"
