# 후방 AprilTag 컨테이너 CPU 실측 — 2026-10-06

## 핵심 결과

**이 PC의 실제 `scoped_component_container_mt`에 AprilTag 컴포넌트를 로드하고 CARLA 후방 영상을 넣어 측정했습니다.** 물리 카메라를 포함한 Jetson 전체 컨테이너가 아니라 **검출기만 포함한 x86 컨테이너**입니다. 로봇 이동·도킹 명령을 보내지 않았습니다.

CPU 100%는 논리 CPU 한 개입니다. 다음은 20초 구간 평균입니다.

| 시험 프로파일 | 디버그 영상 구독 없음 | 디버그 JPEG 구독 중 | 구독 해제 후 |
|---|---:|---:|---:|
| 기존 CARLA 설정 `quad_decimate=1` | 55.01% | 69.55% | 55.77% |
| **기존 실물 로봇 기본값 `quad_decimate=2`** | **30.25%** | **45.28%** | **30.33%** |

실물 기본 검출 설정에서는 디버그 구독을 닫으면 **45.28% → 30.33%**, **14.95%p / 상대 약 33.0% 감소**했습니다. 구독을 열면 입력 프레임마다 디버그 JPEG를 생성하고, 닫으면 기존 subscriber gate가 이를 중단한 결과입니다. 구독을 끄더라도 태그 검출은 계속 처리됩니다.

**이것은 v2.3.1 UI 수정 전후의 CPU 개선률이 아닙니다.** 이번 UI 배포 수정은 카메라·AprilTag 처리 코드나 기본값을 변경하지 않았습니다. 위 프로파일 비교도 기존 CARLA/실물 설정 차이를 관측한 것이며, 실물 기본값을 새로 2로 바꾸었다는 의미가 아닙니다. 월악산의 200% 문제가 해결되었다거나 Jetson에서 100% 미만이라고 결론 낼 수 없습니다.

## 동일 입력 조건 및 검증

- 호스트 `htop`, x86_64, 논리 CPU 12개. Jetson ARM64가 아닙니다.
- 실제 CARLA 후방 센서 → ROS bridge → 두 개의 동일한 멀티스레드 컴포넌트 컨테이너.
- 입력 `/carla/ego_vehicle/rgb_rear/image`, CameraInfo도 같은 센서에서 수신.
- 960×720 BGRA8, 실제 처리 약 9.45~9.49 Hz. 구간마다 각 검출기가 189~190장을 처리했습니다. 디버그 구독 중에는 각각 JPEG 190개를 실제 수신했습니다.
- 두 컨테이너에 같은 컴포넌트 바이너리를 동시에 로드했습니다. 차이는 `quad_decimate` 1/2뿐이며, `n_threads=2`, ROI 배율 3, full-search 주기 30, JPEG 품질 80 등은 기존 production YAML을 읽어 사용했습니다.
- 시험용 입력 경로와 출력 namespace만 분리하고 TF 출력을 끄는 독립 launch입니다. production YAML·물리 로봇 파라미터는 변경하지 않았습니다.
- target tag ID 3 검출은 두 프로파일 모두 0회였습니다. 따라서 이 결과는 해당 장면의 전체 프레임 검색 부하이며, 태그 검출 정확도·도킹 성공·ROI 추적 성능 검증이 아닙니다.
- 구독 없는 검출기 CPU 시간은 decimate1 약 58.00 ms/장, decimate2 약 31.89 ms/장. 디버그 구독 중은 각각 73.32 / 47.74 ms/장입니다.
- 평균 RSS: decimate1 약 244.7~245.4 MiB, decimate2 약 142.5~150.3 MiB. 단기 측정이므로 메모리 누수 여부를 판정하지 않습니다.
- 측정 중 소스 빌드는 끝난 상태였습니다. 전체 호스트 CPU는 42.4~45.7%였으며 CARLA 렌더링 등도 포함합니다. 표는 호스트 전체가 아닌 각 컨테이너 프로세스의 user+system CPU 시간으로 계산했습니다.
- 20초씩 세 구간을 한 번 수행한 단기 A/B/A입니다. 여러 차례 반복한 통계·장시간 열 포화·각기 다른 장면 검증은 아니며, 두 검출기가 같은 호스트에서 동시에 실행되었다는 한계도 있습니다.

## ARM / Jetson에서 남은 확인

`camrod_sensing/CMakeLists.txt`는 물리 카메라 컴포넌트를 ARM64 + OpenCV/CUDA/VPI 조건으로 빌드합니다. 이 x86 설치에는 `libcamera_rear_publisher_component.so`가 없어 해당 컴포넌트를 포함하지 않았습니다. 따라서 **카메라 캡처, VIC 변환, CPU 보정(remap), 물리 카메라 JPEG 비용은 이 표에서 빠져 있습니다.** 실제 물리 구성의 카메라-검출기 intra-process 전달과도 다릅니다.

기존 bringup 후방 설정은 1920×1080이고 `fps` 기본값은 30입니다. 이 시험의 약 960×720@9.48Hz와 입력 부하부터 다릅니다. 해상도·주파수·Jetson 전력 모드·온도·OpenCV 스레드·카메라 가속 경로에 따라 CPU가 달라져, x86 측정값을 ARM으로 비례 환산하면 안 됩니다. ARM 에뮬레이션도 물리 카메라/VIC의 성능 검증을 대신하지 못합니다.

실물에서는 기존 로봇 ROS 환경을 유지한 채 다음을 **읽기 전용으로** 확인하면 됩니다.

```bash
# 실제 컨테이너 PID를 먼저 확인합니다. 종료/재시작 명령이 아닙니다.
pgrep -af scoped_component_container_mt

# 위에서 확인한 rear_camera_apriltag_container의 PID를 넣습니다.
# 도킹 진단 화면 닫힘 / 열림 / 다시 닫힘을 각각 30초 기록합니다.
pidstat -u -r -p <PID> 1 30

ros2 param get /sensing/camera/econ_rear/camera_rear_publisher compressed_publish_rate_hz
ros2 param get /perception/apriltag_parking_detector quad_decimate
ros2 param get /perception/apriltag_parking_detector n_threads
ros2 param get /perception/apriltag_parking_detector debug_jpeg_quality
```

bringup의 rear compressed monitor 설정은 0.5 Hz이고 sensing 패키지 단독 YAML은 2 Hz입니다. 둘은 기존에 구분되어 있던 값입니다. **AprilTag의 `debug_image/compressed`는 이 monitor 주기 설정과 별개로, 구독 시 프레임마다 생성됩니다.** 실물 실행에서 실제로 어떤 YAML을 사용했는지 먼저 확인해야 합니다. 영상 Hz를 별도 측정할 경우 그 구독 자체의 부하를 CPU 구간에 섞지 않도록 종료한 뒤 CPU를 측정합니다.

## 자료 및 재현

- [원본 수치·실제 파라미터·바이너리 SHA-256](results.json)
- [컴포넌트 로드 로그](containers.log)
- [독립 비교 launch](container_probe.launch.py)
- [구독 전환·CPU 측정 코드](measure.py)

로컬 simulator server → bridge → pacer → spawn 준비 후, 동일 simulator ROS 환경에서 다음을 별도 터미널로 실행합니다. 물리 로봇 환경에서는 실행하지 않습니다.

```bash
ros2 launch /absolute/path/to/container_probe.launch.py
python3 /absolute/path/to/measure.py
```

측정기는 ROS parameter를 읽고 debug JPEG를 구독/해제할 뿐, 차량 제어나 production parameter 변경을 하지 않습니다. 두 출력 namespace는 `/rear_perf_decimate_1`, `/rear_perf_decimate_2`입니다. 측정은 약 15:23:22에 끝났으며, 컨테이너는 이후 15:23:48에 미리 설정한 5분 시간 제한으로 SIGINT를 받아 정상 종료했습니다. 그 뒤 이번 시험의 spawn·bridge·pacer·server도 종료했습니다. 서비스의 timeout 표시는 시간 제한 종료 원인이며 측정 중 검출기 crash가 아닙니다.
