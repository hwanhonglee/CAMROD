# Tier4 ADAPI RViz Plugin (Humble)

`HH_260408` 기준으로, root 권한 없이 워크스페이스 로컬 경로에 의존성을 설치하고
`tier4_adapi_rviz_plugin`을 빌드/사용할 수 있도록 구성했습니다.

## 1) 의존성 설치 (1회)

```bash
cd ~/camrod_ws/src
./camrod_map/external/tier4_adapi/scripts/install_tier4_adapi_vendor_deps.sh
```

## 2) 빌드

```bash
cd ~/camrod_ws
source /opt/ros/humble/setup.bash
source ~/camrod_ws/src/camrod_map/external/tier4_adapi/setup_vendor_env.bash
colcon build --base-paths src src/camrod_map/external/tier4_adapi \
  --packages-select tier4_system_msgs autoware_component_interface_utils tier4_adapi_rviz_plugin \
  --symlink-install
```

## 3) 실행 전 환경

```bash
source /opt/ros/humble/setup.bash
source ~/camrod_ws/src/camrod_map/external/tier4_adapi/setup_vendor_env.bash
source ~/camrod_ws/install/setup.bash
```

## 4) RViz 패널

- `camrod_map/rviz/camrod_dark.rviz`
- `camrod_map/rviz/camrod_operator.rviz`

두 파일 모두 `tier4_adapi_rviz_plugins::RoutePanel`이 추가되어 있습니다.
