# CAMROD × CARLA 4WS 시나리오 시각화

이 폴더에는 직진, 좌회전, 우회전, 크랩 횡이동, 제자리 회전, Nav2 단거리 목표와
2026-08-31 Woraksan-tuned B12/B1 왕복의 PNG/GIF와 원본 JSON이 들어 있습니다.

01~06은 당시 CARLA 서버가 `NullRHI`였기 때문에 실제 카메라 프레임이 없는
제어·자세 데이터 기술 시각화다. 07은 별도의 2026-08-31 onscreen 실행에서 실제
CARLA window와 production CAMROD UI를 동시에 녹화한 렌더 증거다. 08은 2026-08-31
당시 `virtual/carla` 작업 트리에서 같은 두 창을 900초 동안 다시 녹화하고,
B1 왕복 matrix와 실제 sensor-source audit를 함께 보존한 역사 증거다. 현재
분리된 기준으로는 07·08 둘 다 `camrod-tuned`에 해당하며, 당시 실행 명령이
현재의 `camrod-tuned` subcommand였다는 뜻은 아니다. 각 범위를 섞지 않으며
역사 `manifest.json`은 01~06만 설명한다.

07은 **역사 증거**다. 원본은 보고서에 기록된 host-local
`/tmp/camrod-b12-retune-clean-e2e-20260831.mp4`였고 PNG/GIF의 hash와 원본 사양은
`camrod_carla_b12_round_trip_e2e_20260831.json`에 남아 있지만, 당시 사용한 정확한
X11/ffmpeg 명령은 commit되지 않았다. 따라서 07을 현재 wheel/torque/controller 수정의
재시험으로 간주하거나 새 checkout의 증거로 재사용하지 않는다. 아래 capture runner는
새 실행부터 exact command·창 geometry·hash를 남기며, 기존 07 파일을 자동으로
덮어쓰지 않는다. visual capture만으로 actor·sensor·mission provenance 전체가 닫히지는
않으므로 동반 live report는 여전히 필요하다.

| 번호 | 시나리오 | 증거 범위 |
|---|---|---|
| 01 | 직진 Dual Ackermann | CARLA 실측 시작/종료 자세 |
| 02 | 좌회전 Dual Ackermann | CAMROD 제어식 기반 개략 예시, 미실측 |
| 03 | 우회전 Dual Ackermann | CAMROD 제어식 기반 개략 예시, 미실측 |
| 04 | 크랩 횡이동 | CARLA 실측 물리 반응, 화면 변위 정규화 |
| 05 | 제자리 회전 | CARLA 실측 yaw/평행이동 |
| 06 | Nav2 단거리 목표 | 실측 시작·최종·목표점 선형 보간 |
| 07 | Drop Zone → B12 → 복귀 → 충전 | 과거 Woraksan-tuned onscreen CARLA/UI 녹화에서 추출; 수동주행·teleport 없음 |
| 08 | Drop Zone → B1 → 복귀 → 주차·충전 | 과거 Woraksan-tuned onscreen CARLA/UI 900초 녹화 + 32-stream/13-actor audit + live matrix PASS |

빠른 확인 파일:

- `current/README.md`: 2026-09-01 현재 develop-parity rendered smoke의 actor/gate,
  36-stream/13-actor 감사, 실제 rate, physical wheel command와 판정 경계
- `current/ranger_actor_actual_carla_20260901.png`: 실제 onscreen CARLA Ranger actor
- `current/ui_camera_actual_carla_20260901.png`: production UI에 들어온 실제 CARLA
  front/rear camera
- `current/ui_semantic_perception_actual_carla_20260901.png`: raw cost가 아니라
  semantic fusion만 보이는 현재 지도·인지 화면
- `current/manual_straight_4ws_live_20260901.gif`: bounded 직진 smoke
- `current/manual_zero_turn_crab_4ws_live_20260901.gif`: mode-cut을 포함한
  zero-turn/crab smoke
- `00_scenario_overview.png`: 전체 정적 모음
- `00_all_scenarios.gif`: 전체 연속 애니메이션
- `01_...` ~ `06_...`: 시나리오별 PNG/GIF
- `camrod_carla_ui_latest_develop_20260825.png`: 2026-08-25 당시 CAMROD UI 실행
  화면. 파일명의 `latest_develop`은 현재 parity PASS를 의미하지 않음
- `camrod_carla_live_smoke_20260823T152312Z.json`: CARLA 실측 원본
- `camrod_carla_latest_develop_full_test_20260825.json`: 2026-08-25 당시 develop 전체 시험 결과
- `07_b12_round_trip_e2e.{png,gif}`: 실제 렌더 B12 왕복 요약
- `camrod_carla_b12_round_trip_e2e_20260831.json`: 07의 gate, 센서, 좌표 정합,
  타임라인, 테스트와 제한
- `08_b1_round_trip_live_20260831.{png,gif}`: 과거 tuned B1 왕복의 실제
  CARLA/UI 대화면 요약. 파일명의 `live`는 develop-parity 재검증을 의미하지 않음
- `camrod_carla_b1_round_trip_final_20260831.json`: 08의 전체 milestone, pose, 물리 4WS,
  주행거리, 주차·충전 결과
- `camrod_carla_b1_sensor_source_audit_20260831.json`: 32개 UI stream과 13개 CARLA
  sensor actor의 실제 publisher/actor 소유권 감사
- `camrod_carla_b1_capture_manifest_20260831.json`: 900초 원본 MP4 사양, X11 geometry,
  exact ffmpeg command와 PNG/GIF/MP4 SHA-256
- `CAMPING_SITE_MATRIX.md`: B1~B13 기존 simulation과 실제 CARLA 왕복 상태를
  혼동 없이 비교하는 단일 표
- `UI_COMPOSITION.md`: 07 화면이 어떤 런타임 조합으로 만들어졌는지 설명
- `manifest.json`: 역사 01~06 원본 보고서 SHA-256, 파일 SHA-256, 프레임 수, 범위 선언

`current/`의 2026-09-01 자료는 현재 develop-parity live smoke다. actual sensor와
manual physical 4WS까지는 새로 검증했지만 B1~B13 dispatch/Return/parking은 실행하지
않았으므로 mission PASS로 확대 해석하지 않는다.

## 07·08 화면 구성

`07_b12_round_trip_e2e`와 `08_b1_round_trip_live_20260831`은 한 프로그램의 내부
화면이 아니다.

- 왼쪽은 onscreen CARLA 렌더 창이다.
- 오른쪽은 production `CAMROD` 운영 UI다.
- 사용한 UI는 별도 가짜 테스트 UI가 아니라 `camrod_ui` 패키지의 정식 backend와 정적 frontend다.

08의 녹화 영역은 `3700×2000`이고 왼쪽 CARLA `1900×2000`, 오른쪽 production UI
`1800×2000`을 공백 없이 한 번에 x11grab했다. 실행 구조와 파일 위치는
[`UI_COMPOSITION.md`](UI_COMPOSITION.md)에 정리했다.

## B1~B13 왕복 데이터

[`CAMPING_SITE_MATRIX.md`](CAMPING_SITE_MATRIX.md)에 기존 13/13 ROS 2 simulation,
과거 tuned B12/B1 CARLA PASS, 아직 실행되지 않은 tuned 사이트와 현재
develop-parity B1~B13 PENDING을 서로 다른 열로 분리했다. 최신 gate와
다섯 runtime process가 모두 준비된 뒤 다음 한 명령이 실제 production UI 경로로 각
사이트 도착·Return·Drop Zone 주차/충전 결과를 machine-local JSON에 누적한다.

```bash
./scripts/virtual_carla/run.sh camping-sites
```

계획만 볼 때는 `camping-sites-plan`을 사용한다. 이 명령은 파일과 명령을 생성하지 않는다.

## 01~06 기술 시각화 재생성

```bash
source ./scripts/virtual_carla/env.sh
python3 "$RANGER_CARLA_ROOT/tools/render_camrod_carla_scenarios.py" \
  --report "$CAMROD_SRC_ROOT/docs/evidence/virtual_carla/camrod_carla_live_smoke_20260823T152312Z.json" \
  --output-dir "$CAMROD_SRC_ROOT/docs/evidence/virtual_carla"
```

원본 계측 보고서는 이 폴더의
`camrod_carla_live_smoke_20260823T152312Z.json`입니다.

## 새 onscreen CARLA + production UI 증거 캡처

[`capture_ui_evidence.sh`](../../../scripts/virtual_carla/capture_ui_evidence.sh)는
CARLA나 ROS를 시작·종료하지 않고, UI 버튼·ROS topic·차량 명령도 전혀 보내지 않는다.
이미 실행 중인 `CarlaUE4` 창과 `CAMROD Operator UI` 창의 XID/title/위치/크기를
X11에서 읽기 전용으로 확인한 뒤 두 창의 합집합인 **하나의 desktop region**을
`ffmpeg x11grab`으로 녹화한다. 두 창을 별도 녹화해서 사후 합성하지 않으며 AI 생성,
보간, sensor 대체 영상도 사용하지 않는다.

스크립트의 기본 action은 `plan`이다. 인자 없이 실행하면 파일이나 디렉터리를 만들지
않고 실행 계획만 출력한다.

```bash
cd /path/to/camrod_ws/src
./scripts/virtual_carla/capture_ui_evidence.sh
```

먼저 `CARLA_RENDER_MODE=onscreen`으로
`server → bridge → pacer → spawn → camrod`가 기동된 상태를 만든다. 여기서
`camrod`는 현재 develop-parity 프로필이며, 과거 조정값 재현은 명시적으로
`camrod-tuned`를 선택한다. full launch의
operator window는 기본 fullscreen이므로 증빙 시에는
`CAMROD_ENABLE_OPERATOR_WINDOW=false ./scripts/virtual_carla/run.sh camrod`로 backend를
띄우고, 별도 terminal에서 다음처럼 windowed wrapper를 연다. 위는 parity
캡처 예시다. tuned 재현을 녹화할 때는 동일 위치의 마지막 인자만
`camrod-tuned`로 바꾸고 manifest에 profile을 명시한다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/camrod_ws/install/setup.bash
ros2 run camrod_ui camrod_ui_window \
  --url http://127.0.0.1:8010 \
  --engine webkit --width 960 --height 540 --no-fullscreen
```

GNOME의 좌/우 창 맞춤 기능 등으로 `CarlaUE4`를 왼쪽, UI를 오른쪽에 놓는다. capture
runner는 창을 이동하거나 크기를 바꾸지 않는다. 현재 graphical session의 X11 권한으로
다음 읽기 전용 검사를 실행한다.

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
./scripts/virtual_carla/capture_ui_evidence.sh validate
```

title이 중복이면 `xwininfo -root -tree`에서 두 XID를 확인한 뒤 명시한다.

```bash
xwininfo -root -tree | rg 'CarlaUE4|CAMROD Operator UI|Robot UI'
./scripts/virtual_carla/capture_ui_evidence.sh validate \
  --carla-window-id 0x01234567 \
  --ui-window-id 0x07654321
```

일반 브라우저를 쓴 경우 title이 `Robot UI`일 수 있으므로
`--ui-window-title "Robot UI"`도 함께 사용한다. 명시한 XID도 기대 title과 다르면
거부된다.

실제 캡처에는 `capture` action과 **절대 경로인 새 디렉터리 또는 빈 디렉터리**가 둘 다
필수다. 다음 예시는 15분을 5 fps H.264로 기록한다. 이미 파일이 하나라도 있는 출력
디렉터리는 fail closed하며 `ffmpeg -n`도 overwrite를 거부한다.

```bash
evidence_dir=/absolute/path/to/ranger_visual_evidence/$(date -u +%Y%m%dT%H%M%SZ)
./scripts/virtual_carla/capture_ui_evidence.sh capture \
  --output-dir "$evidence_dir" \
  --duration-seconds 900 \
  --capture-fps 5 \
  --gif-fps 8 \
  --derived-width 960
```

B1~B13처럼 장시간 사이트별 PNG/GIF만 보존할 때는 입력을 명시적으로
`--retain-source-video false`로 둔다. 기본값은 기존 동작과 같은 `true`다. `false`도
녹화 중에는 MP4가 필요하므로 해당 한 번의 캡처를 담을 여유 공간은 있어야 한다. 녹화와
ffprobe, contact sheet, GIF 생성·검증이 모두 끝난 뒤 **그 출력 디렉터리의 원본 MP4
하나만** 제거한다.

```bash
./scripts/virtual_carla/capture_ui_evidence.sh capture \
  --output-dir "$evidence_dir" \
  --duration-seconds 900 --capture-fps 1 \
  --gif-fps 8 --derived-width 960 \
  --retain-source-video false \
  --allow-short-capture true
```

`--allow-short-capture true`는 사이트 matrix가 900초 전에 끝났을 때 같은 캡처 terminal의
foreground `ffmpeg`에 `q`를 한 번 입력해 정상 종료할 수 있게 한다. 실제 녹화가 12초
이상이고 `요청 시간 + fps 기반 허용 오차` 이하일 때만 PNG/GIF를 실제 길이에서 다시
샘플링한다. manifest와 `exact_commands.txt`에는 요청/실제 시간,
`allow_short_capture=true`, 실제 조기 종료일 때 `early_finalized=true`가 함께 남는다.
기본값 `false`는 요청 시간과 일치해야 하는 기존 strict 검증을 그대로 유지한다. 12초 전
종료, 신호에 의한 ffmpeg 실패, 상한 초과는 opt-in 상태에서도 REJECTED다.

필요하면 위 capture 명령에도 두 `--*-window-id`를 그대로 붙인다. 생성물은 다음과 같다.

| 파일 | 내용 |
|---|---|
| `carla_camrod_desktop.mp4` | geometry 검증 뒤 한 번에 녹화한 원본 desktop region; `--retain-source-video false`이면 파생물 검증 뒤 남기지 않음 |
| `representative_contact_sheet.png` | 원본 시간의 5%, 22%, 39%, 56%, 73%, 90% 여섯 frame을 2×3으로 배치 |
| `representative_motion.gif` | 같은 여섯 시각 주변의 짧은 실제 frame 구간 |
| `ffprobe.json` | 원본 codec, frame rate, resolution, duration 원문 |
| `exact_commands.txt` | 실제 실행한 x11grab/PNG/GIF ffmpeg 명령, 요청/실제 시간과 조기 종료 판정, hash 명령 |
| `capture_manifest.json` | PASS 상태, 녹화 전후 XID/title/geometry, capture region, 요청/실제 시간과 조기 종료 판정, sample 시각, scope와 artifact SHA-256; 제거 모드에서는 원본의 제거 전 byte/hash와 `retained=false`를 기록 |
| `sha256sums.txt` | 실제로 남아 있는 원본(유지 모드만)·파생물·명령·manifest 무결성 목록 |

```bash
(cd "$evidence_dir" && sha256sum -c sha256sums.txt)
```

PNG/GIF는 이 visual runner가 차량을 움직였다는 증거가 아니다. 실행 중 control/actor/gate,
actual wheel angle·torque, sensor source와 mission 결과는 별도 live report와
`./scripts/virtual_carla/run.sh audit-sensors` 결과로 묶어야 한다. UI 내부 camera 화면도
CARLA sensor topic의 증거이고, 이 runner가 녹화하는 왼쪽 `CarlaUE4` spectator window와는
증거 범위가 다르다. 또한 X11 `IsViewable`은 mapped 상태만 확인하므로 제3의 창이 잠시
가린 적이 전혀 없다는 사실은 증명하지 못하며, 이 한계는 v4 manifest에 기록된다.

## 역사 Woraksan-tuned B1 live 결과

08과 함께 저장한 2026-08-31 실행은 현재 `camrod-tuned`로 분류한
구성에서 B1 한 곳만 시험했다. B2~B13으로 확대 해석하지 않고, 현재
`camrod` develop-parity 실행의 PASS로도 재사용하지 않는다.

- 결과: `PASS`, 710.787초 / 사이트 제한 900초
- 실제 이동: 170.222831 m, `/cmd_vel` 13,570 sample, CARLA odometry 13,462 sample
- B1 도착 오차: 0.094504 m
- Drop Zone 복귀 오차: 0.306233 m / 허용 3.0 m
- 최종 상태: `PARKED`, `CHARGING`, 정지 속도 약 `7.45e-8 m/s`
- 물리 구동: 동일 actor 130, `PHYSX_FOUR_WHEEL_STEERING`, 독립 wheel drive 사용 가능,
  torque safety cap 20 Nm
- 금지 동작: pose teleport `false`, fake sensor data `false`
- 센서 감사: 32/32 stream, 13/13 CARLA sensor actor, failure/warning 0
- 순서:
  `ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → WAIT_RETURN → ALIGN_RETRACE_YAW →`
  `CRAB_OUT → Drop Zone ALIGN_PARKING_YAW → REVERSE_APPROACH → PARKED → CHARGING`

원본 900초 MP4는 Git 크기 제한 때문에 host-local evidence 디렉터리에 두고, 재생 가능한
PNG/GIF와 원본 matrix/audit/capture manifest는 이 폴더에 함께 보존했다. MP4 경로와
SHA-256은 `camrod_carla_b1_capture_manifest_20260831.json`에서 확인한다.

07 실행의 라이다는 기계식 360°가 아니다. actor 53의 전방 고정 solid-state 근사이며
수평 `-60..+60°`, 수직 `-25..+10°`, 16 channel, 60,000 points/s,
`sensor_tick=0.1 s`다. `rotation_frequency=20`은 이 부채꼴 ray-cast sampling
parameter이지 360° 회전을 뜻하지 않는다.

08도 같은 전방 고정 solid-state 계약을 사용했으며 해당 과거 tuned run의 LiDAR actor는
133이다. 08의 센서 actor ID 전체는 `camrod_carla_b1_sensor_source_audit_20260831.json`에
있다.
