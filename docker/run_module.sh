#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <image_tag> <ros2 launch ...>"
  echo "Example: $0 camping_cart_bringup:humble ros2 launch camping_cart_bringup bringup.launch.py"
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
  -v "$(pwd)/..:/workspaces/cart_test_ws/src:rw" \
  -w /workspaces/cart_test_ws \
  "${IMAGE}" \
  bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && ${CMD}"
