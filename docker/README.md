# Module Docker Workflow (GPU-ready)

## 1) Build base image
```bash
cd /home/hong/cart_test_ws/src/docker
docker build -f Dockerfile.base -t camping_cart/base:humble ..
```

## 2) Build one module image
```bash
./build_module.sh camping_cart_planning camping_cart_planning:humble
```

## 3) Run one module
```bash
./run_module.sh camping_cart_planning:humble \
  ros2 launch camping_cart_planning planning.launch.py
```

## 4) Compose multi-module
```bash
docker compose -f compose.modules.yaml up planning map sensing
```

## Notes
- GPU exposure uses `--gpus all` / `gpus: all`.
- Runtime host networking is used to keep ROS2 DDS discovery simple.
- For production, pin package versions and split build/runtime stages.
