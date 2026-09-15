# 왕복 미션·자율/수동·CAN 기록기 (HH_260915)

저장되는 CAN/state 필드의 실제 예와 JSON/JSONL 차이는
[데이터 형식 설명](mission_recording_data_format.md)을 참고한다.
최신 전체 화면 PNG/GIF 위치, 빌드 결과와 알려진 기존 검사 실패는
[v2.2.8 검증 기록](release_v2_2_8_validation.md)을 참고한다.

## 요청과 이번 변경의 범위

B1–B13 각각의 **일반 이동→복귀**, **recall→복귀**를 한 미션으로 묶고,
날짜별 순번·사이트·요청 종류·거리·수동 개입·정지 사유와 CAN 주행 정보를
UI가 닫혀 있어도 기록한다. 실제 조향·가감속·engage·STOP·장애물 판단은 변경하지 않는다.
구현은 `camrod_ui` 패키지 안에 있으며 CARLA 의존성을 추가하지 않았다.

## 연결 구조와 핵심 코드

```text
기존 UI 백엔드의 승인된 미션/복귀/STOP 및 확인된 서비스 상태
  → MissionRecordingEmitter → /ui/mission_recording/events
                                      ↓
/platform/status ──────────────→ mission_recorder_node
/control/cmd_vel_safety_gate/status ──→ │  (독립 ROS 노드)
선택적 SocketCAN 수신 ───────────────→ │
                                      ↓ bounded queue / 단일 저장 worker
                                MissionJournal
                           SQLite + 날짜/미션별 JSONL
                                      ↓ 원자적 snapshot.json (1 Hz)
                  GET /api/mission-records → 실증 현황의 왕복 미션 기록
```

| 파일 | 담당 변경 |
|---|---|
| `runtime/python/camrod_ui/mission_recorder_node.py` | 상태 구독, 원본 시각/decoded 필드 정규화, bounded queue, worker, stale/overflow 표시, 1 Hz export |
| `runtime/python/camrod_ui/mission_journal.py` | 일별 순번/미션/재시도, 속도 적분, 모드·정지 이벤트, 재시작·중복 방지, DB/JSONL/용량 제한 |
| `runtime/python/camrod_ui/mission_recording_bridge.py` | 명령 승인 뒤 관측 이벤트 발행, STOP 이후 기록 ID 유지, 고정 경로 snapshot 읽기 |
| `runtime/python/camrod_ui/raw_can_capture.py` | 선택적 수신 전용 SocketCAN, CAN ID/바이트/시각/플래그 보존, **송신 없음** |
| `runtime/python/camrod_ui/ui_backend_node.py` | 기존 승인 콜백에 관측 훅 추가, 읽기 전용 API 추가; 기존 실증 집계 로직 유지 |
| `camrod_ui_robot/assets/frontend/src/MissionRecords.js/.css` | 미션별 자율/수동/미확인/합계, 필터, 정지 타임라인, 저장 위치 표시 |
| `camrod_ui_robot/assets/frontend/src/ServiceEvidence.js` | 기존 화면에 접을 수 있는 새 패널만 삽입; 기존 누적 KPI 계산 불변 |
| `camrod_ui_robot/launch/{ui,mission_recorder}.launch.py`, `setup.py` | 기본 UI 실행 시 기록 노드 시작, 기록 노드 단독 실행/설치 지원 |

## 기록 기준

- 플랫폼 정의의 `control_mode=1(CAN)` → 자율, `0(RC)` → 수동, 그 외 → 미확인.
  UI의 준비 완료/engage 표시만으로 수동이라고 추정하지 않는다.
  **CAN 모드는 명령 소유자 인증은 아니다.** CAN 모드 안에서 별도 CAN 수동 제어기를
  사용하거나 시뮬레이터가 항상 1을 보내면 그것도 현재 기준에서는 자율로 분류된다.
  실제 플랫폼과 다른 모드를 보내는 어댑터는 별도 확인해야 한다.
- 거리 입력은 명령 속도나 경로 길이가 아닌 `/platform/status.velocity`의
  `hypot(vx,vy)`. 전·후진/크랩에 모두 양의 거리로 적분한다.
  바퀴 기반 속도에 슬립 오차가 있으면 기록 거리에도 반영되므로 지상 실측 거리 인증은 아니다.
- 인접한 유효 샘플 사이 사다리꼴 적분. 모드가 바뀐 한 구간은 어느 모드가 몇 초였는지
  확정할 수 없으므로 미확인 거리로 **한 번만** 더한다.
- 기본 0.03 m/s 미만은 정지로 취급, 최대 3 m/s 초과 입력은 이상으로 표시하고 제외.
  중복 시각, 역행, 2초 초과 간격, stale/잘못된 입력을 외삽하지 않는다.
- 0속도가 0.5초 이상 유지되면 정지 이벤트를 만들고, 정지 시간/횟수와 게이트의 실제
  메시지·estop·error_code·vehicle_state를 보존한다. **멈춰 있는 동안 거리 증가 0**.
  감속하며 정지하는 직전 구간의 이동 거리는 정상적으로 포함한다.
  사유가 보고되지 않으면 모른다고 남긴다. 단순히 모든 정지를 장애물 때문이라고 만들지 않는다.
- HH_260915 릴리스 점검: 동일한 정지 사유에서 인증 경과 시간·경로 진행 거리·배터리
  백분율 같은 숫자만 달라지는 메시지는 새 상태 전이로 반복 적재하지 않는다.
  실제 사유·권한·복구 선택·진단 수준/상태가 바뀌면 기록하며, 다음 실제 정지에는
  최신 원문을 보존한다. 이는 기록의 중복 억제이지 안전 게이트 판단 변경이 아니다.
- STOP은 제어상 기존대로 취소/안전 정지하되, 기록 미션만 `paused`로 남겨 RC 복구 및
  승인된 복귀를 같은 왕복에 연결한다. 새로운 별개 미션 승인 시 이전 미완료 기록은
  `interrupted`로 남기고 새로운 일별 순번을 만든다. 같은 미션 재시도는 attempt로 남긴다.
- recall의 첫 적재/회전 확인과 실제 최종 복귀를 구분한다. 복귀가 확인된 뒤 주차 완료
  상태 0/12/13에서 왕복을 닫으며 `ROAD_HANDOFF_READY`를 완료로 착각하지 않는다.
  **단순 주차가 끝난 뒤 선택적으로 수행하는 도킹/충전은 이미 닫힌 왕복 밖 기록이다.**
- 미션 밖 RC 이동도 `outside_missions`에 별도 기록한다. 새 기록기 lifetime 합계에는
  미션 안/밖의 자율+수동+미확인 거리가 포함된다. 사이트 집계에는 해당 미션만 포함한다.
- UI의 일반 STOP을 영구 취소/완료로 바꾸는 새 버튼은 만들지 않았다.

## CAN 정보와 데이터 보존

기본으로 저장하는 것은 플랫폼 드라이버가 이미 해독한 속도·모드·estop·에러·배터리·
모터 rpm/속도/각도·원본 메시지 시각이다. 드라이버가 캐시된 CAN 값을 새 ROS 시각으로
재발행하면 개별 CAN 프레임의 신선도까지 이 토픽만으로 증명할 수 없다. 출처 메타데이터에
이 제한을 명시한다. 빈 배터리 정보를 측정값처럼 만들지 않는다.

`raw_can_interface:=can0`처럼 **실제 인터페이스를 명시한 경우에만** 원시 CAN 프레임을
추가 수신한다. 버스 인터페이스 설정/bitrate 변경/프레임 송신은 하지 않는다. 관측 방향은
`bus_observed`이며 CAMROD 송신/상대 수신을 단정하지 않는다. 미설정·연결 실패를 성공으로
표시하지 않는다. 이번 검증에는 실제 CAN 버스를 사용하지 않았다.

기본 저장 위치는 `$XDG_STATE_HOME/camrod/mission_records` 또는
`~/.local/state/camrod/mission_records`이다.

```text
mission_records/
  mission_journal.sqlite3       # 새 전용 DB, 운영 중 WAL/SHM 동반 가능
  snapshot.json                 # UI 읽기 전용 export
  .writer.lock                  # 같은 저장소 중복 노드 방지
  YYYY-MM-DD/
    001_B7_delivery_<고유값>/
      events_000001.jsonl
      telemetry_000001.jsonl
      raw_can_000001.jsonl      # 설정/실제 수신 시에만 존재
    002_B7_recall_<고유값>/...
    outside/...                # 미션 밖의 상태/주행/원시 자료
```

기존 `service_metrics.sqlite3`는 열거나 변경하지 않는다. 따라서 기존 실증 기록은 남지만
그 과거 기록에 없던 CAN 원문·수동 구간·정지 사유를 소급해서 만들어 넣지는 않는다.
**기존 누적과 새 기록기의 누적은 서로 겹칠 수 있으므로 더하지 않는다.** 기존 누적 화면은
그대로 두고, 이번 기능을 켠 이후의 상세 기록임을 새 패널에서 구분한다.

기존 사이트별 평균 거리·시간 그래프, 사이트 정량 표, 일별/최근 이력과 PNG/GIF도 유지한다.
새 패널을 펼치면 그 아래 기존 화면이 내려갈 뿐이며, 패널을 접으면 기존 항목을 바로 볼 수 있다.
새 기록 패널만 촬영한 이미지는 전체 실증 현황 화면의 대체본이 아니다.

JSONL은 파일당 기본 4 MiB 회전, 이 저장소의 JSONL 총합 기본 **256 MiB 한도**다.
실운행 장기 보관 전 기록률/보관 기간/남은 용량에 맞게 `quota_bytes`를 지정한다.
한도 도달 시 기존 자료를 자동 삭제하지 않고 추가 원본 기록 불완전/DEGRADED를 표시한다.
DB/합계는 계속 남을 수 있으므로 DEGRADED 기록을 완전한 CAN 증빙이라고 사용하면 안 된다.
DB와 snapshot 크기는 JSONL 한도에 포함되지 않는다. 완전한 기록을 영구 보장하는
자동 백업/아카이빙 서비스는 이번에 추가하지 않았다.

노드 재시작 시 열린 미션과 순번/producer 중복 방지 정보를 복원하며, 꺼져 있던 시간의
거리는 추정하지 않는다. 백엔드 재시작의 명시 STOP은 기존 미션을 pause하고, 승인된
복귀만 저장된 한 미션에 다시 연결한다. 브라우저를 닫아도 노드는 계속 돌지만 ROS launch나
PC 자체를 종료하면 노드도 종료된다. systemd 서비스 설치는 별도다.

## 빌드와 실행

새 코드를 배포한 워크스페이스에서 `camrod_ui`를 다시 빌드해야 신규 실행 파일이 등록된다.
기존 전체 CAMROD 빌드 방법을 사용할 수 있으며, UI 패키지만 빌드하는 예시는 다음과 같다.
아래 `npm run build`는 생성된 frontend bundle을 갱신하는 실제 배포 작업이다.

```bash
cd /home/hong/camrod_ws/src/camrod_ui/camrod_ui_robot/assets/frontend
npm run build

cd /home/hong/camrod_ws
colcon build --base-paths /home/hong/camrod_ws/src/camrod_ui \
  --packages-select camrod_ui --symlink-install
source /home/hong/camrod_ws/install/setup.bash
```

그 다음 기존 CAMROD bringup을 실행하면 포함된 `ui.launch.py`에서 기록기도 기본 시작된다.
UI만 따로 실행하는 경우(이미 켜진 UI/기록기에 중복 실행하지 말 것):

```bash
ros2 launch camrod_ui ui.launch.py \
  mission_recorder_environment:=real \
  mission_records_root:=/home/hong/.local/state/camrod/mission_records
```

UI 화면: **실증 현황 → 왕복 미션 기록 보기 → 사이트/유형 선택 → 미션 선택**.
미션 이름, 자율/수동/미확인/합계 m·km, 정지 시간/횟수, 수동 개입, 이벤트와 파일 위치를 본다.
상세 목록은 최근 100미션, 각 타임라인은 최근 최대 50이벤트이며 축약 여부를 표시한다.
전체 기록은 DB와 원본 JSONL에 남는다. 웹 파일 다운로드/날짜 범위 전체 내보내기는 추가하지 않았다.

기록 노드 단독 실행(다른 `mission_recorder`가 없는 경우):

```bash
ros2 launch camrod_ui mission_recorder.launch.py \
  storage_root:=/home/hong/.local/state/camrod/mission_records \
  environment:=real raw_can_interface:=can0 quota_bytes:=1073741824
```

이는 1 GiB의 **예시** 한도이며 실제 CAN 인터페이스/디스크 보관 정책에 맞춰 지정한다.
단독 노드도 미션 구분에는 새 백엔드의 승인 이벤트가 필요하다. 별도 노드를 실행하면서
UI도 켤 때는 UI 측에 `enable_mission_recorder:=false`와 동일 `mission_records_root`를 지정한다.
백엔드가 이벤트를 발행하지 않는 이전 버전이면 자동으로 사이트 미션을 복구/추측하지 않는다.

```bash
ros2 topic echo /ui/mission_recording/status --once
curl -f http://127.0.0.1:8010/api/mission-records
```

입력 단절·overflow·저장 오류·발행 오류는 명시하며, snapshot이 없거나 5초보다 오래됐으면
API는 503을 반환한다. 정상인 것처럼 0으로 보이지 않는다. 실환경/시뮬레이션/시험은 반드시
서로 다른 저장소를 사용한다. `use_sim_time=true`인데 environment가 기본 real이면
simulation으로 보정하고 기존 real 프로필 저장소와 섞는 것은 거부한다.

## 이번 검증과 재현

검증 입력은 **실제 ROS 노드에 보낸 합성 상태/이벤트**이며 실차/실제 CAN/CARLA 주행 성공이 아니다.
원래 운행 DB의 SHA256도 변경 전후 동일함을 확인했다.

- 코어 테스트: B1–B13 × 일반/recall = 26개 왕복 조합과 모드/정지/재시작/중복/오류/용량 제한.
- ROS 검증: B7·B8·B9 각 일반/recall, 총 6미션 완료; 정지 중 거리 증가 0,
  RC 개입 각 1회, 같은 ID 복귀/주차 완료, 미션 밖 수동 이동 별도 누적.
- 기록 노드의 publisher는 ROS 기본 진단 이외 `/ui/mission_recording/status`뿐임을 확인.
- PNG/GIF는 해당 ROS 결과를 실제 React 컴포넌트에 표시해 캡처한 UI 증빙.
  주행 장면이나 모델 애니메이션이 아니다.

```bash
cd /home/hong/camrod_ws/src
python3 -m pytest -q camrod_ui/test
# --output은 기존 데이터와 분리된 새 디렉터리여야 함.
python3 camrod_ui/tools/mission_recorder_ros_smoke.py \
  --output /tmp/camrod-mission-recording-unique-run
```

시험 도구는 ROS domain 188 / localhost로 격리하고 주행 명령을 발행하지 않는다.
이번 작업의 원본 결과·화면·촬영 도구는 로컬
`/home/hong/camrod_ws/_maintenance/20260915_mission_recorder/`에 보존했다.
이는 Git 태그/커밋/Push 완료를 뜻하지 않는다.
