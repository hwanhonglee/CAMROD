# Sensor Inspection Tools

이 폴더는 `ZED/Orbbec` 센서 상태 확인과 Kimera용 센서 YAML 덤프를 위한 유틸 모음입니다.

`Orbbec Gemini 335Le`는 Ethernet 모델입니다.
따라서 Orbbec 쪽은 기본적으로 아래 흐름으로 보면 됩니다.

- 장치 확인: `build/orbbec_sensor_inspector`
- YAML 생성: `save_sensor_yaml.sh --sensor orbbec ...`
- Kimera 실행: `../Kimera-VIO/build/orbbecDumpCalibration` 후 `../Kimera-VIO/build/orbbecLiveVIO`
- 장치 선택: 자동 검색 또는 `--address <IPv4> --port 8090`
- 기본 프로파일: `width/height/fps`를 모두 주지 않으면 `640x400 @ 15` IR stereo

## Build

```bash
cd sensor_inspection_tools
cmake -S . -B build
cmake --build build -j
```

옵션:

- `-DSENSOR_TOOLS_USE_ZED=ON|OFF`
- `-DSENSOR_TOOLS_USE_ORBBEC=ON|OFF`
- `-DZED_SDK_ROOT=/usr/local/zed`
- `-DORBBEC_SDK_ROOT=/usr/local/OrbbecSDK`

## Executables

- `build/zed_sensor_inspector`
- `build/orbbec_sensor_inspector` (Orbbec SDK가 발견될 때)

## One-command YAML save (권장)

ZED/Orbbec 모두 아래 스크립트로 바로 Kimera 센서 YAML을 저장할 수 있습니다.

Kimera를 바로 실행하려면 `--compose_params_dir`를 사용하는 것이 가장 빠릅니다
(템플릿의 `Pipeline/Frontend/Backend/Lcd/Display` + 센서 3종 YAML 자동 반영).
`Kimera-VIO/...` 형태 경로는 현재 작업 디렉토리와 무관하게 워크스페이스 루트
(`/home/hong/camrod_ws/src/camrod_localization/external/vio_bridge`) 기준으로 해석됩니다.

```bash
# 권장: Kimera 실행 가능한 params 폴더까지 자동 생성
./save_sensor_yaml.sh --sensor zed --compose_params_dir ./Kimera-VIO/params/ZED_live
./save_sensor_yaml.sh --sensor orbbec --compose_params_dir ./Kimera-VIO/params/Orbbec_live

# 센서 YAML 3종만 저장
./save_sensor_yaml.sh --sensor zed --output_dir ./Kimera-VIO/params/ZED_live_sensor
./save_sensor_yaml.sh --sensor orbbec --output_dir ./Kimera-VIO/params/Orbbec_live_sensor
```

추가 옵션 전달:

```bash
./save_sensor_yaml.sh --sensor zed --compose_params_dir ./Kimera-VIO/params/ZED_live -- --rectified --resolution HD720 --fps 30
./save_sensor_yaml.sh --sensor orbbec --compose_params_dir ./Kimera-VIO/params/Orbbec_live -- --serial <SN>

# Ethernet Orbbec를 IP로 직접 지정
./save_sensor_yaml.sh --sensor orbbec --compose_params_dir ./Kimera-VIO/params/Orbbec_live -- --address 192.168.1.10 --port 8090

# 템플릿을 직접 지정하고 싶을 때
./save_sensor_yaml.sh --sensor zed \
  --compose_params_dir ./Kimera-VIO/params/ZED_custom \
  --template_params_dir ./Kimera-VIO/params/Euroc

# zedDumpCalibration의 ZED_inverted와 유사한 extrinsic이 필요할 때
./save_sensor_yaml.sh --sensor zed \
  --compose_params_dir ./Kimera-VIO/params/ZED_live_inverted \
  -- --invert_imu_to_cam --resolution HD720 --fps 30
```

## YAML -> Kimera 실행 연결

`--compose_params_dir`를 사용하면 전체 params 폴더가 자동으로 완성됩니다.

ZED 예시(권장):

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge

# 1) 템플릿 + 센서 YAML을 합친 완성 params 생성
./sensor_inspection_tools/save_sensor_yaml.sh \
  --sensor zed \
  --compose_params_dir ./Kimera-VIO/params/ZED_live \
  -- --rectified --resolution HD720 --fps 30

# 2) 실행
cd ./Kimera-VIO
./build/zedLiveVIO \
  --params_folder_path ./params/ZED_live \
  --zed_resolution HD720 --zed_fps 30 \
  --zed_use_rectified=true \
  --log_output=true --output_path ../output/zed_live
```

Orbbec 예시(권장):

```bash
cd /home/hong/camrod_ws/src/camrod_localization/external/vio_bridge

# 1) build 바이너리에서 바로 완성 params 생성
cd ./Kimera-VIO/build
./orbbecDumpCalibration \
  --output_folder_path ../params/Orbbec_live \
  --template_folder_path ../params/Orbbec \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090

# 2) 실행
./orbbecLiveVIO \
  --params_folder_path ../params/Orbbec_live \
  --orbbec_address 192.168.1.10 \
  --orbbec_port 8090 \
  --orbbec_stereo_stream auto \
  --orbbec_enable_visualization=false \
  --orbbec_enable_csv_log=false \
  --output_path ../output/orbbec_live
```

참고: `save_sensor_yaml.sh`를 `--compose_params_dir` 없이 사용하면 센서 3종 YAML만 생성됩니다.

- `LeftCameraParams.yaml`
- `RightCameraParams.yaml`
- `ImuParams.yaml`

수동 방식이 필요하면 템플릿 폴더에 위 3개를 덮어써서 직접 params를 구성해도 됩니다.

## ZED quick usage

```bash
./build/zed_sensor_inspector --print_intrinsics
./build/zed_sensor_inspector --run --frames 300
./build/zed_sensor_inspector --dump_kimera_yaml --output_dir ./tmp/zed_sensor
```

## Orbbec quick usage

```bash
./build/orbbec_sensor_inspector --print_intrinsics
./build/orbbec_sensor_inspector --run --frames 300
./build/orbbec_sensor_inspector --dump_kimera_yaml --output_dir ./tmp/orbbec_sensor
./build/orbbec_sensor_inspector --describe

# Ethernet Orbbec를 IP로 직접 지정
./build/orbbec_sensor_inspector --print_intrinsics --address 192.168.1.10 --port 8090

# 장치 검색은 되는데 open이 실패할 때 access mode 진단
./build/orbbec_force_ip_tool --list
./build/orbbec_force_ip_tool --open-test --ip 192.168.1.10
```

`StereoCamera.cpp:72] Check failed: stereo_baseline_ > 0.0`가 나오면
기존에 생성된 `Orbbec_live` YAML이 예전 extrinsic 방향(`cam<-imu`)을
담고 있는 경우가 많습니다. `orbbecDumpCalibration`을 다시 실행해 params를 갱신하세요.

## Dump output meaning

`--dump_kimera_yaml`는 아래 3개만 생성합니다.

- `LeftCameraParams.yaml`
- `RightCameraParams.yaml`
- `ImuParams.yaml`

즉, Kimera 실행용 전체 파라미터 폴더를 완성하는 도구가 아니라
센서 고유 파라미터(내/외부 파라미터 + IMU 노이즈)만 빠르게 생성하는 도구입니다.

`save_sensor_yaml.sh --compose_params_dir ...`를 사용하면
`Pipeline/Frontend/Backend/Lcd/Display` YAML까지 포함된 실행용 폴더를 함께 만들 수 있습니다.
