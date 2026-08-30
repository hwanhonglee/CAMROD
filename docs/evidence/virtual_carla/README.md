# CAMROD × CARLA 4WS 시나리오 시각화

이 폴더에는 직진, 좌회전, 우회전, 크랩 횡이동, 제자리 회전, Nav2 단거리 목표와
rendered B12 왕복의 PNG/GIF가 들어 있습니다.

01~06은 당시 CARLA 서버가 `NullRHI`였기 때문에 실제 카메라 프레임이 없는
제어·자세 데이터 기술 시각화다. 07은 별도의 2026-08-31 onscreen 실행에서 실제
CARLA window와 production CAMROD UI를 동시에 녹화한 렌더 증거다. 두 범위를 섞지
않으며 역사 `manifest.json`은 01~06만 설명한다.

| 번호 | 시나리오 | 증거 범위 |
|---|---|---|
| 01 | 직진 Dual Ackermann | CARLA 실측 시작/종료 자세 |
| 02 | 좌회전 Dual Ackermann | CAMROD 제어식 기반 개략 예시, 미실측 |
| 03 | 우회전 Dual Ackermann | CAMROD 제어식 기반 개략 예시, 미실측 |
| 04 | 크랩 횡이동 | CARLA 실측 물리 반응, 화면 변위 정규화 |
| 05 | 제자리 회전 | CARLA 실측 yaw/평행이동 |
| 06 | Nav2 단거리 목표 | 실측 시작·최종·목표점 선형 보간 |
| 07 | Drop Zone → B12 → 복귀 → 충전 | 실제 onscreen CARLA/UI 녹화에서 추출; 수동주행·teleport 없음 |

빠른 확인 파일:

- `00_scenario_overview.png`: 전체 정적 모음
- `00_all_scenarios.gif`: 전체 연속 애니메이션
- `01_...` ~ `06_...`: 시나리오별 PNG/GIF
- `camrod_carla_ui_latest_develop_20260825.png`: 실제 CAMROD UI 실행 화면
- `camrod_carla_live_smoke_20260823T152312Z.json`: CARLA 실측 원본
- `camrod_carla_latest_develop_full_test_20260825.json`: 최신 develop 전체 시험 결과
- `07_b12_round_trip_e2e.{png,gif}`: 실제 렌더 B12 왕복 요약
- `camrod_carla_b12_round_trip_e2e_20260831.json`: 07의 gate, 센서, 좌표 정합,
  타임라인, 테스트와 제한
- `manifest.json`: 역사 01~06 원본 보고서 SHA-256, 파일 SHA-256, 프레임 수, 범위 선언

재생성:

```bash
source ./scripts/virtual_carla/env.sh
python3 "$RANGER_CARLA_ROOT/tools/render_camrod_carla_scenarios.py" \
  --report "$CAMROD_SRC_ROOT/docs/evidence/virtual_carla/camrod_carla_live_smoke_20260823T152312Z.json" \
  --output-dir "$CAMROD_SRC_ROOT/docs/evidence/virtual_carla"
```

원본 계측 보고서는 이 폴더의
`camrod_carla_live_smoke_20260823T152312Z.json`입니다.

07 실행의 라이다는 기계식 360°가 아니다. actor 53의 전방 고정 solid-state 근사이며
수평 `-60..+60°`, 수직 `-25..+10°`, 16 channel, 60,000 points/s,
`sensor_tick=0.1 s`다. `rotation_frequency=20`은 이 부채꼴 ray-cast sampling
parameter이지 360° 회전을 뜻하지 않는다.
