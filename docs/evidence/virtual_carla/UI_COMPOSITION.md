# Virtual CARLA UI 구성과 재현 방법

## 결론: 별도 실행 파일은 있지만, UI 전체가 단일 바이너리는 아니다

설치 후 `camrod_ui` 패키지가 제공하는 console executable은 세 개다.

| 실행 이름 | 역할 | 실제 구현 |
|---|---|---|
| `ui_backend_node` | ROS 2 ↔ HTTP/WebSocket 변환, React 정적 파일 제공 | `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py` |
| `ui_guest_node` | 게스트용 경량 UI backend | `camrod_ui/runtime/python/camrod_ui/ui_guest_node.py` |
| `camrod_ui_window` | backend URL을 여는 GTK/WebKit 또는 Chromium kiosk 창 | `camrod_ui/runtime/python/camrod_ui/operator_ui_window.py` |

두 entrypoint는 `camrod_ui/setup.py`에 선언된다. 빌드 후 실제 설치 경로는 다음 명령으로
확인한다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/camrod_ws/install/setup.bash
ros2 pkg prefix camrod_ui
ls "$(ros2 pkg prefix camrod_ui)/lib/camrod_ui/" \
  | grep -E '^(ui_backend_node|ui_guest_node|camrod_ui_window)$'
```

`camrod_ui_window`는 UI 업무 로직을 별도로 복제한 프로그램이 아니다. backend가 제공하는
동일한 React build를 WebKit/Chromium으로 여는 창 wrapper다. 따라서 실제 UI 구성은
운영자 화면 기준
`ui_backend_node + camrod_ui_robot/assets/frontend/build + camrod_ui_window(또는 브라우저)`다.
`ui_guest_node`는 선택적 게스트 화면을 위한 별도 backend이며 07의 오른쪽 운영자 화면을
구성한 프로세스는 아니다.

## `07_b12_round_trip_e2e` 화면이 만들어진 방식

`07_b12_round_trip_e2e.png`와 `.gif`는 한 애플리케이션 내부의 분할 화면이 아니다.

```text
X11 desktop의 한 연속 영역
├─ 왼쪽: onscreen CarlaUE4 spectator window
└─ 오른쪽: CAMROD Operator UI window
```

역사 원본 MP4는 `1920×540`, 약 857초, 5 fps였다. PNG는 원본의 대표 여섯 시각을
`2×3`으로 배치한 `1920×810` contact sheet이고 GIF는 `960×270` 축소
파생물이다. committed PNG SHA-256은 `790ebb71…be42b`, GIF는
`ecb4f9f8…be899`이며 전체 값은 `camrod_carla_b12_round_trip_e2e_20260831.json`에
있다. 당시 정확한 X11/ffmpeg
명령은 저장되지 않았으므로 07은 2026-08-31 당시의 역사 증거이며, 현재 controller 수정
후 재시험으로 사용할 수 없다. 화면 자체는 이미 보이는 두 창을 기록했을 뿐이다.
actor identity, 실제 휠 각도·토크, sensor owner, B12 mission 성공 주장은 반드시
`camrod_carla_b12_round_trip_e2e_20260831.json` 같은 동반 보고서와 함께 해석한다.

## backend, frontend, ROS/CARLA의 데이터 흐름

```text
CARLA actors/sensors
  → carla_ros_bridge topics
  → camrod_carla_adapter relay/remap/TF
  → CAMROD sensing·planning·control ROS graph
  → ui_backend_node (FastAPI + uvicorn)
  → React frontend
  → camrod_ui_window 또는 일반 browser
```

production frontend가 실제로 쓰는 대표 경로는 다음과 같다.

| 기능 | frontend ↔ backend 경로 |
|---|---|
| campsite 선택, ENGAGE ON | `/ws` WebSocket command |
| STOP | `POST /ui/stop` |
| 수동 주행 deadman/속도/모드 | `/ws/manual-drive` |
| operator telemetry | `/ws/telemetry?view=...`, 실패 시 HTTP fallback |
| 전방/후방/도킹 카메라 JPEG | `GET /api/camera/front`, `/rear`, `/docking` |

`/ui/destination`, `/ui/auto` REST API도 backend에 존재하지만 현재 production 화면의
campsite/engage 대표 경로는 `/ws`다. CARLA full launch는 별도 가짜 UI를 만들지 않고
동일 production UI에 CARLA sensor relay, TF tolerance, compressed-camera 정책, manual
limit 파라미터를 주입한다.

## 포트가 8000과 8010으로 다르게 보이는 이유

| 실행 방식 | 기본 포트 |
|---|---:|
| `ui_backend_node`를 Python entrypoint로 직접 실행 | 8000 |
| `ros2 launch camrod_ui ui.launch.py` | 8010 |
| `camrod_carla_full.launch.py` | 8010 |
| guest UI가 활성화된 full launch | 8012 |

backend bind address는 `0.0.0.0`이고 로컬 window URL은 보통
`http://127.0.0.1:8010`이다. state-changing API를 공용 인터넷에 노출하는 배치가
아니므로 trusted workstation/LAN에서만 사용하고 방화벽 밖으로 직접 공개하지 않는다.

## 빌드

```bash
cd /path/to/camrod_ws/src
export RANGER_CARLA_ROOT=/path/to/ranger-carla-4ws-pipeline
./scripts/virtual_carla/build.sh
source /opt/ros/humble/setup.bash
source ../install/setup.bash
```

WebKit window와 화면 증빙에는 배포판 패키지 `python3-gi`,
`gir1.2-webkit2-4.0`, `x11-utils`(`xwininfo`), `ffmpeg`가 필요하다. frontend build는
`camrod_ui_robot/assets/frontend/build`에서 ROS package data로 설치된다.

## onscreen CARLA와 창 형태 UI를 정확히 띄우는 순서

각 단계는 별도 terminal에서 실행한다. 모든 terminal에 같은
`RANGER_CARLA_ROOT`, `CARLA_RENDER_MODE`, ROS/DDS 환경을 적용한다.

```bash
cd /path/to/camrod_ws/src
export RANGER_CARLA_ROOT=/path/to/ranger-carla-4ws-pipeline
export CARLA_RENDER_MODE=onscreen
./scripts/virtual_carla/run.sh doctor
./scripts/virtual_carla/run.sh commands
```

그 다음 별도 terminal 다섯 개에서 순서대로 실행한다.

```bash
./scripts/virtual_carla/run.sh server
./scripts/virtual_carla/run.sh bridge
./scripts/virtual_carla/run.sh pacer
./scripts/virtual_carla/run.sh spawn
CAMROD_ENABLE_OPERATOR_WINDOW=false ./scripts/virtual_carla/run.sh camrod
```

full launch의 managed operator window는 기본 fullscreen이므로 좌우 증빙 화면에는 적합하지
않다. 위처럼 managed window만 끄고, 여섯 번째 terminal에서 windowed wrapper를 연다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/camrod_ws/install/setup.bash
ros2 run camrod_ui camrod_ui_window \
  --url http://127.0.0.1:8010 \
  --engine webkit --width 960 --height 540 --no-fullscreen
```

일반 브라우저로 `http://127.0.0.1:8010`을 열 수도 있다. 이 경우 X11 title은
`CAMROD Operator UI`가 아니라 `Robot UI`일 수 있으므로 capture 시
`--ui-window-title "Robot UI"`를 명시한다.

UI만 단독 실행하려면 CARLA full launch와 중복 실행하지 않은 상태에서 다음을 사용한다.

```bash
ros2 launch camrod_ui ui.launch.py \
  enable_ui_guest:=false \
  enable_operator_ui_window:=true \
  operator_ui_window_fullscreen:=false \
  operator_ui_window_width:=960 \
  operator_ui_window_height:=540
```

종료는 `camrod → spawn → bridge → pacer → server` 역순이다. bridge를 내리는 동안에는
pacer를 먼저 죽이지 않는다.

## 새 화면 증빙 캡처

`scripts/virtual_carla/capture_ui_evidence.sh`는 CARLA/ROS를 시작하거나 UI를 클릭하지
않는다. 이미 배치된 왼쪽 CARLA 창과 오른쪽 UI 창의 XID, title, geometry를 녹화 전후로
검사하고 하나의 X11 desktop region을 기록한다.

repository의 07은 위의 역사 캡처다. 이후 2026-08-31 B1 최종 run에서
`camrod.virtual_carla.desktop_ui_capture.v2` manifest를 실제 생성해 다음 파일로
보존했다.

- `08_b1_round_trip_live_20260831.png`: 1920px panel 여섯 시각의 2×3 contact sheet
- `08_b1_round_trip_live_20260831.gif`: 같은 여섯 시각 주변 실제 frame 구간
- `camrod_carla_b1_capture_manifest_20260831.json`: XID/title/geometry, exact command,
  ffprobe와 artifact SHA-256
- `camrod_carla_b1_round_trip_final_20260831.json`: 차량 motion/mission/parking/charging
  판정
- `camrod_carla_b1_sensor_source_audit_20260831.json`: UI stream과 CARLA actor 소유권

원본은 X11의 단일 `3700×2000+140+128` 영역을 4 fps로 정확히 900초 녹화한 H.264
MP4다. 왼쪽 CARLA window는 `1900×2000`, 오른쪽 UI window는 `1800×2000`이고 창 사이
gap은 0 px였다. 원본은 3,600 frame, 2,654,872,278 byte이며 SHA-256은
`56347502f6e18c829868c89b517fb4991136265f49901c8246afa42a956f2ce4`다. Git에 2.5 GB
MP4를 넣지 않고, host-local 원본 경로와 전체 명령은 capture manifest에 남겼다.

08 화면과 동반 live report를 함께 해석하면 B1 dispatch부터 크랩, 180° 회전, 복귀,
후진 주차, 충전까지 실제 actor 130이 이동했음을 검증할 수 있다. visual capture 하나만으로
actor나 sensor provenance를 주장하지 않는 원칙은 그대로다.

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

./scripts/virtual_carla/capture_ui_evidence.sh validate

evidence_dir=/absolute/path/to/new/evidence/$(date -u +%Y%m%dT%H%M%SZ)
./scripts/virtual_carla/capture_ui_evidence.sh capture \
  --output-dir "$evidence_dir" \
  --duration-seconds 900 --capture-fps 4 --gif-fps 8 \
  --derived-width 1920

(cd "$evidence_dir" && sha256sum -c sha256sums.txt)
```

title이 중복되면 `--carla-window-id`, `--ui-window-id`를 쓸 수 있지만, 명시한 XID도
기대 title을 포함하지 않으면 거부된다. X11의 `IsViewable`은 mapped 상태만 뜻하므로 다른
창이 한동안 위를 가리지 않았다는 것까지 증명하지는 못한다. 이 제한은 v2 capture
manifest에 명시된다. 또한 title은 확인하지만 `_NET_WM_PID`, `WM_CLASS`,
backend `/ui/health`, 실행 바이너리 hash까지 창 신원에 묶지는 않으므로 UI/CARLA
프로세스 신원과 기능 통과 판정은 동반 live report로 별도 묶어야 한다.
