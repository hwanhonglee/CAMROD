# vio_bridge Run Guide

이 문서는 `vio_bridge`를 Docker 기준으로 실행하는 실전 가이드입니다.

> CAMROD workspace canonical path:  
> `/home/hong/camrod_ws/src/camrod_localization/external/vio_bridge`

## 1) 경로 기준 (중요)

- Host(PC): `/home/hong/camrod_ws/src/camrod_localization/external/vio_bridge`
- Container(Docker 내부): `/kimera_ws`

기본값 기준:
- `./docker/build_live.sh`는 Host 소스를 읽어 이미지에 복사합니다.
- `./docker/run_live.sh`는 Host 작업폴더를 마운트하지 않습니다.
- 따라서 Host와 Container는 같은 이름의 폴더를 가지더라도 서로 다른 파일시스템입니다.

예:
- Host local build: `/home/hong/camrod_ws/src/camrod_localization/external/vio_bridge/Kimera-VIO/build`
- Container image build: `/kimera_ws/Kimera-VIO/build`

둘 다 폴더 이름은 `build`지만 내용은 분리되어 있습니다.

## 2) 폴더 역할

- `Kimera-VIO`: 실제 VIO 실행 바이너리(`zedLiveVIO`, `orbbecLiveVIO`, `zedDumpCalibration`, `orbbecDumpCalibration`)
- `sensor_inspection_tools`: 센서 상태 확인 + Kimera 센서 YAML 생성 + 실행용 params 폴더 조합
- `third_party`: 의존성 소스(직접 실행 대상 아님)
- `docker`: Docker 빌드/실행 스크립트

## 3) Docker 이미지 빌드 (Host에서 실행)

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
./docker/build_live.sh
```

성공하면 이미지 `kimera-vio:live`가 생성됩니다.

## 4) Docker 컨테이너 실행 (Host에서 실행)

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge
./docker/run_live.sh
```

이 스크립트는 기본으로 아래를 설정합니다.
- `--gpus all`
- `--network host`
- `NVIDIA_VISIBLE_DEVICES=all`
- `NVIDIA_DRIVER_CAPABILITIES=all`
- Host workspace bind mount 없음
- `Kimera-VIO/output`만 Docker volume으로 보존

`libnvcuvid.so.1`, `libnvidia-encode.so.1` 문제를 피하려면 수동 `docker run`보다 이 스크립트 사용을 권장합니다.

필요 시 선택 옵션:

- `MOUNT_WORKSPACE=1 ./docker/run_live.sh`
  - 개발용. Host 작업폴더를 다시 `/kimera_ws`에 마운트합니다.
  - 이 경우 Host/Container build가 다시 섞일 수 있으므로 권장하지 않습니다.
- `OUTPUT_BIND_DIR=/some/host/path ./docker/run_live.sh`
  - CSV를 Docker volume 대신 Host 폴더에 직접 저장합니다.

리포지토리 없이 이미지 만으로 다른 PC에서 바로 띄우고 싶다면:

```bash
docker run -it --rm \
  --name kimera_live \
  --network host \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --privileged \
  -v /run/udev:/run/udev:ro \
  -v kimera_live_output:/kimera_ws/Kimera-VIO/output \
  -w /kimera_ws \
  kimera-vio:live \
  /bin/bash
```

이 경우에도 컨테이너 안에는 이미 아래가 들어 있습니다.
- `Kimera-VIO`
- `sensor_inspection_tools`
- 각 폴더의 `build/`

## 5) ZED 파라미터 생성 + 실행 (Container 내부에서 실행)

### 5-1) 센서 읽어서 실행용 params 폴더 생성

```bash
cd /kimera_ws

./sensor_inspection_tools/save_sensor_yaml.sh \
  --sensor zed \
  --compose_params_dir ./Kimera-VIO/params/ZED_live \
  -- --invert_imu_to_cam --resolution HD720 --fps 30
```

설명:
- `--compose_params_dir`를 쓰면 센서 YAML 3종 + `Pipeline/Frontend/Backend/Lcd/Display` YAML까지 한 번에 구성됩니다.
- `--invert_imu_to_cam`은 기존 `parseImuData: expected identity body_Pose_cam_` 문제가 있을 때 필요할 수 있습니다.

### 5-2) Kimera ZED live 실행 (권장: pose 안정/저부하)

권장 기본 실행(메시 비활성 + 저부하 + CSV 없음):

> HH_260317: 이 섹션과 아래 5-3/5-4/5-5는 실제 운영 커맨드/로깅 조건/ROS2 publish 옵션 정리로 추가됨.

```bash
cd /kimera_ws/Kimera-VIO/build

./zedLiveVIO \
  --params_folder_path ../params/ZED_inverted \
  --zed_resolution HD720 \
  --zed_fps 15 \
  --visualize=false \
  --viz_type=2 \
  --use_lcd=false \
  --zed_use_rectified=false \
  --logtostderr=1 \
  --stderrthreshold=0 \
  --zed_imu_time_reference=current
```

주의:
- `2D Mesh is empty` 경고를 줄이려면 `--viz_type=2`(none) 또는 `--viz_type=1`(pointcloud)을 사용하세요.
- `--visualize=true` + `viz_type=0`(mesh)은 디버깅용이고, 런타임 부하가 커서 추적 불안정 체감이 커질 수 있습니다.
- 이 명령은 `--log_output`을 켜지 않았기 때문에 Kimera CSV(`output_frontend_stats.csv`, `traj_vio.csv` 등)가 생성되지 않습니다.
- Local/Global pose CSV(`--local_pose_output_csv`, `--abs_pose_output_csv`)도 지정하지 않으면 생성되지 않습니다.

### 5-3) ZED CSV 로깅 포함 실행

```bash
cd /kimera_ws/Kimera-VIO/build

./zedLiveVIO \
  --params_folder_path /kimera_ws/Kimera-VIO/params/ZED_live \
  --zed_resolution HD720 \
  --zed_fps 30 \
  --visualize=false \
  --viz_type=2 \
  --use_lcd=false \
  --log_output=true \
  --output_path /kimera_ws/Kimera-VIO/output/zed_live
```

CSV 결과:
- Container: `/kimera_ws/Kimera-VIO/output/zed_live`
- 기본: Docker volume `${CONTAINER_NAME}_output`
- 선택: `OUTPUT_BIND_DIR`를 지정했다면 그 Host 폴더

로깅 관련 핵심:
- Kimera 내부 CSV 로깅은 `--log_output=true`를 명시해야 켜집니다.
- 로깅 경로는 `--output_path`를 명시해서 고정하는 것을 권장합니다.
- `--output_path`를 생략하면 기본값(`./`) 기준으로 저장되어 찾기 어려워질 수 있습니다.
- `--output_path`가 쓰기 불가 경로면 로깅 시작 시 프로세스가 중단될 수 있습니다.

### 5-4) ZED local/global pose CSV 추가 출력

```bash
cd /kimera_ws/Kimera-VIO/build

./zedLiveVIO \
  --params_folder_path ../params/ZED_inverted \
  --zed_resolution HD720 \
  --zed_fps 30 \
  --visualize=false \
  --viz_type=2 \
  --use_lcd=false \
  --log_output=true \
  --output_path /kimera_ws/Kimera-VIO/output/zed_live \
  --abs_pose_enable=true \
  --local_pose_output_csv /kimera_ws/Kimera-VIO/output/zed_live/traj_local.csv \
  --abs_pose_output_csv /kimera_ws/Kimera-VIO/output/zed_live/traj_abs.csv
```

설명:
- `traj_local.csv`: VIO local frame (`W_vio`) 기준 pose
- `traj_abs.csv`: map/global 기준 pose (`map <- W_vio` 정렬 적용 후)

### 5-5) ZED local/global pose를 ROS2 NavSatFix 토픽으로 publish

이 기능은 빌드 시 ROS2 옵션이 켜져 있어야 합니다.
또한 실행 셸에서 ROS2 환경을 먼저 로드해야 합니다.

```bash
source /opt/ros/humble/setup.bash
```

```bash
cd /kimera_ws/Kimera-VIO/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DKIMERA_VIO_USE_ZED=ON \
  -DKIMERA_VIO_USE_ROS2=ON
cmake --build . -j"$(nproc)"
```

실행:

```bash
cd /kimera_ws/Kimera-VIO/build

./zedLiveVIO \
  --params_folder_path ../params/ZED_inverted \
  --zed_resolution HD720 \
  --zed_fps 30 \
  --visualize=false \
  --viz_type=2 \
  --use_lcd=false \
  --abs_pose_enable=true \
  --ros2_publish_local_fix=true \
  --ros2_publish_global_fix=true \
  --ros2_local_fix_topic /kimera_vio/local_fix \
  --ros2_global_fix_topic /kimera_vio/global_fix \
  --ros2_global_fix_use_utm=true \
  --abs_pose_utm_zone=52 \
  --abs_pose_utm_north=true
```

토픽 확인:

```bash
ros2 topic echo /kimera_vio/local_fix
ros2 topic echo /kimera_vio/global_fix
```

메시지 의미:
- 타입: `sensor_msgs/msg/NavSatFix`
- `local_fix`: 호환성 목적의 pseudo-fix (`x,y,z -> latitude,longitude,altitude` 매핑)
- `global_fix`: 기본적으로 map x/y(UTM)를 lat/lon으로 변환해 publish

주의:
- `--params_folder_path`는 `/kimera_ws/Kimera-VIO/params/...`를 사용하세요.
- `/kimera_ws/sensor_inspection_tools/Kimera-VIO/params/...` 경로는 잘못된 경로입니다.

## 6) Orbbec 파라미터 생성 + 실행 (Container 내부에서 실행)

`Gemini 335Le`는 Ethernet 모델입니다.
즉, Orbbec 쪽 실행 기준은 아래처럼 잡으면 됩니다.

- 센서 확인/캘리브레이션 덤프: `sensor_inspection_tools/build/orbbec_sensor_inspector`
- Kimera live 실행: `Kimera-VIO/build/orbbecLiveVIO`
- Docker 네트워크: 반드시 `host` 권장
- 장치 선택 방식: `auto discovery` 또는 `--address <IPv4> --port 8090`
- 스테레오 입력: 코드에서 가능한 pair를 검사해 `IR_LEFT + IR_RIGHT`를 우선 자동 선택
- 프로파일 기본값:
  `width/height/fps`를 모두 명시하지 않으면 `640x400 @ 15` IR stereo로 자동 선택
  이유: Gemini 335Le Ethernet에서 고해상도 stereo는 RTP timeout이 나기 쉬움

Host NIC를 카메라 대역으로 올릴 때는 별도 helper 스크립트 대신 직접 설정합니다.

```bash
sudo ip addr add 192.168.1.20/24 dev enp4s0
ip route get 192.168.1.10
```

인터넷 쪽으로 되돌릴 때:

```bash
sudo ip addr del 192.168.1.20/24 dev enp4s0
```

실행은 `Kimera-VIO/build` 안의 바이너리 두 개만 사용합니다.

```bash
cd /kimera_ws/Kimera-VIO/build

./orbbecDumpCalibration \
  --output_folder_path ../params/Orbbec_live \
  --template_folder_path ../params/Orbbec \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090
```

Ethernet 장치를 IP로 직접 지정:

```bash
cd /kimera_ws/Kimera-VIO/build

./orbbecLiveVIO \
  --params_folder_path /kimera_ws/Kimera-VIO/params/Orbbec_live \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090 \
  --orbbec_stereo_stream auto \
  --orbbec_enable_visualization=false \
  --orbbec_enable_csv_log=false \
  --output_path /kimera_ws/Kimera-VIO/output/orbbec_live
```

기본 동작:
- `Gemini 335Le`는 width/height/fps를 모두 지정하지 않으면 `640x400 @ 15` 안전 프로파일 사용
- visualization 비활성화
- Kimera CSV logging 비활성화

### 6-1) 실행용 params 폴더 생성

Ethernet Orbbec의 경우:
- 자동 검색이 되면 그냥 실행하면 됩니다.
- 자동 검색이 안 되면 `--address <IPv4> --port 8090`로 직접 지정할 수 있습니다.

```bash
cd /kimera_ws/Kimera-VIO/build

./orbbecDumpCalibration \
  --output_folder_path ../params/Orbbec_live \
  --template_folder_path ../params/Orbbec
```

특정 장치만 고르고 싶으면:

```bash
./orbbecDumpCalibration \
  --output_folder_path ../params/Orbbec_live \
  --template_folder_path ../params/Orbbec \
  --orbbec_serial_number ACTUAL_SERIAL_NUMBER
```

Ethernet 장치를 IP로 직접 지정:

```bash
./orbbecDumpCalibration \
  --output_folder_path ../params/Orbbec_live \
  --template_folder_path ../params/Orbbec \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090
```

주의:
- `<SN>`은 예시 표기일 뿐입니다.
- 실제 명령에는 꺾쇠괄호를 넣지 말고 진짜 시리얼 문자열을 넣어야 합니다.
- `Gemini 335Le`는 width/height/fps를 모두 안 주면 `640x400 @ 15` 안전 프로파일을 사용합니다.
- 잘못된 예: `--serial <SN>`
- 올바른 예: `--orbbec_serial_number CL9A1310001`

### 6-2) Kimera Orbbec live 실행

```bash
cd /kimera_ws/Kimera-VIO/build

./orbbecLiveVIO \
  --params_folder_path /kimera_ws/Kimera-VIO/params/Orbbec_live \
  --orbbec_enable_visualization=false \
  --orbbec_enable_csv_log=false \
  --orbbec_stereo_stream auto \
  --output_path /kimera_ws/Kimera-VIO/output/orbbec_live
```

Ethernet 장치를 IP로 직접 지정:

```bash
./orbbecLiveVIO \
  --params_folder_path /kimera_ws/Kimera-VIO/params/Orbbec_live \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090 \
  --orbbec_enable_visualization=false \
  --orbbec_enable_csv_log=false \
  --orbbec_stereo_stream auto \
  --output_path /kimera_ws/Kimera-VIO/output/orbbec_live
```

CSV 결과:
- Container: `/kimera_ws/Kimera-VIO/output/orbbec_live`
- 기본: Docker volume `${CONTAINER_NAME}_output`
- 선택: `OUTPUT_BIND_DIR`를 지정했다면 그 Host 폴더

## 7) 자주 발생하는 오류

- `error while loading shared libraries: libnvcuvid.so.1`
  - 원인: 컨테이너 실행 시 NVIDIA video capability 미주입
  - 해결: `./docker/run_live.sh`로 컨테이너 실행

- `bash: SN: No such file or directory`
  - 원인: `--serial <SN>`처럼 placeholder를 그대로 입력해서 shell redirection으로 해석됨
  - 해결: `<SN>` 대신 실제 시리얼 번호를 그대로 입력

- `[ERR] No Orbbec device found.`
  - 원인: Ethernet 장치 자동 검색이 안 되는 상태
  - 해결 순서:
    - Host와 카메라가 같은 서브넷인지 확인
    - `./docker/run_live.sh`로 `host network` 컨테이너 사용
    - 자동 검색이 안 되면 `--address <IPv4> --port 8090`로 직접 지정

- `Send control transfer failed!` 또는 `ob_create_net_device_ex`
  - 원인 후보 1: Host NIC가 카메라 IP 대역이 아니라 다른 게이트웨이로 라우팅 중
  - 확인:
    - `ip route get 192.168.1.10`
    - 결과에 `via ...`가 보이면 카메라 직결이 아님
  - 예시 복구:
    - `sudo ip addr add 192.168.1.20/24 dev enp4s0`
    - 다시 `ip route get 192.168.1.10`
  - 원인 후보 2: 장치 검색은 되지만 `default access` open이 실패하는 SDK access mode 문제
  - 확인:
    - `./sensor_inspection_tools/build/orbbec_force_ip_tool --list`
    - `./sensor_inspection_tools/build/orbbec_force_ip_tool --open-test --ip 192.168.1.10`
  - 현재 코드 동작:
    - `orbbec_sensor_inspector`, `orbbecDumpCalibration`, `orbbecLiveVIO`는 `control -> default -> exclusive -> monitor` 순서로 자동 fallback 시도

- `parseImuData: we expected identity body_Pose_cam_`
  - 원인: IMU-Camera extrinsic 방향 불일치
  - 해결: ZED YAML 생성 시 `-- --invert_imu_to_cam` 사용

- `StereoCamera.cpp:72] Check failed: stereo_baseline_ > 0.0`
  - 원인: Orbbec camera extrinsic이 `cam<-imu` 방향으로 저장된 예전 YAML 사용
  - 해결:
    - `./Kimera-VIO/build/orbbecDumpCalibration --output_folder_path ./Kimera-VIO/params/Orbbec_live --template_folder_path ./Kimera-VIO/params/Orbbec --orbbec_address <camera_ip> --orbbec_port 8090`
    - 이미 만들어진 `Kimera-VIO/params/Orbbec_live`를 계속 재사용 중이면 먼저 갱신

- `[ERROR] Image not found locally: kimera-vio:live`
  - 해결: Host에서 `./docker/build_live.sh` 먼저 실행

- `failed to initialize rcl init options ... AMENT_PREFIX_PATH is not set or empty`
  - 원인: ROS2 NavSatFix publish를 켰지만 ROS2 setup이 로드되지 않음
  - 해결:
    - 실행 전 `source /opt/ros/humble/setup.bash`
    - 또는 컨테이너의 셸 init(`~/.bashrc`)에 source를 추가

## 8) 빠른 체크 명령

Container 내부:

```bash
# ZED inspector 도움말
/kimera_ws/sensor_inspection_tools/build/zed_sensor_inspector --help

# Orbbec inspector 도움말
/kimera_ws/sensor_inspection_tools/build/orbbec_sensor_inspector --help

# Orbbec access mode/open 진단
/kimera_ws/sensor_inspection_tools/build/orbbec_force_ip_tool --list
/kimera_ws/sensor_inspection_tools/build/orbbec_force_ip_tool --open-test --ip 192.168.1.10
```
