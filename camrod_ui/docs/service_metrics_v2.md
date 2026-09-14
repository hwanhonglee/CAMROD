# 서비스 이동거리 집계 v2 — 일반 이동·호출·복귀

2026-09-14. 변경 범위는 `camrod_ui`의 관측·저장·화면입니다.
차량 제어기, 경로 계획, 안전 정지, GNSS/TF, 음성, 배터리 기준값은 변경하지 않습니다.

## 해결한 문제

1. 출차 제어기의 `ROAD_HANDOFF_READY`는 수치상 상태 0을 사용합니다.
   종전 집계기는 이를 실제 `DROP_ZONE_WAIT` 완료로 처리하여
   일반 이동·호출 모두 출차 직후 기록이 종료될 수 있었습니다.
   별도 완료 Bool보다 상태 알림이 먼저 도착할 때 재현되었습니다.
2. 총거리는 있었지만 일반 이동·호출·복귀의 세부 구간은 저장되지 않았습니다.
3. 상세 UI 목록은 처음 받아온 이력이 남아 현재 요약과 다르게 보일 수 있었습니다.
4. 출차 실패 후 같은 사이트를 다시 선택하면 제어 미션 번호는 같을 수 있습니다.
   실패한 기록의 ID를 그대로 중복 방지하면 재시도 거리도 누락됩니다.

## 코드와 역할

| 파일 | 변경 |
| --- | --- |
| `runtime/python/camrod_ui/service_metrics.py` | schema v2, 구간 저장·분리 집계·기존 DB 호환, handoff 종료 방지, 실패 중단 API |
| `runtime/python/camrod_ui/ui_backend_node.py` | 승인된 요청의 유형/식별자 전달, 실제 상태/단계 연계, 실패 및 재시도 구분, 별도 복귀 기록 |
| `camrod_ui_robot/assets/frontend/src/ServiceEvidence.js` | 현재·오늘·전체·사이트·일별·최근 기록의 구간별 거리와 합계, 자동 갱신 |
| `camrod_ui_robot/assets/frontend/src/ServiceEvidence.css` | 해당 실적 화면에만 적용하는 별도 스타일 |
| `test/test_service_metrics.py`, `test/test_service_metrics_backend.py` | 집계기·백엔드 실제 콜백의 오프라인 회귀 |
| `camrod_ui_robot/assets/frontend/src/ServiceEvidence.test.js` | 실제 React 컴포넌트의 표시·자동 갱신·오류 회귀 |

### 백엔드

- `_start_service_metrics()`: 승인된 목적지에 한해 `delivery` 또는 `recall` intent를 전달합니다.
  프로세스 UUID + 기록용 attempt 번호 + 기존 motion generation을 request_id로 사용합니다.
  같은 승인 요청의 반복은 중복 기록하지 않습니다.
- `_observe_service_metrics()`: 숫자 상태뿐 아니라 이름과 제어기 단계를 기록합니다.
  `ROAD_HANDOFF_READY`는 집계 종료에서 제외하며 속도 적분 기준점을 보존합니다.
  집계기 자체도 같은 방어를 하므로 직접 호출되어도 조기 종료하지 않습니다.
- `_mark_drop_zone_exit_failed()`: 안전한 대기/충전 상태 발행 전에 해당 기록을
  `interrupted`로 종료합니다. 완료 서비스 수를 늘리지 않습니다.
  실제 중단 성공 때만 기록용 attempt 번호를 증가시켜 같은 사이트 재시도를 새로 기록합니다.
- `_ensure_return_service_metrics()`: 활성 서비스가 없는 상태에서 승인된 복귀 명령을
  실제 발행할 때 별도 `return` 기록을 만듭니다. 거절된 버튼 입력이나 상태 heartbeat만으로 시작하지 않습니다.
- recall의 상태 9는 최종 복귀 전 현지 정렬/회전에도 쓰입니다.
  최종 복귀 승인 generation, 실제 복귀 상태 3, 긴급 배터리 복귀 출처를 함께 확인하여
  호출 현지 동작과 최종 복귀를 분리합니다.
- 동일 숫자 상태에서 제어기 phase가 바뀌는 경우도 관측합니다.
  phase 알림만으로 상태 0/12/13/16의 종료를 새로 발생시키지 않습니다.
- 실제 주차 완료 상태 0/12/13은 기존처럼 서비스 완료입니다. 이후 선택적 도킹만 수행하는
  독립 동작은 이 변경으로 별도 자율운행 실적으로 새로 추정하지 않습니다.

## 거리 계산과 합계

`AvgPlatformStatus.velocity`의 평면 속도로 계산합니다.

`speed = hypot(vx, vy)`

`distance += (previous_speed + speed) / 2 × dt`

따라서 후진·크랩의 음수 성분도 실제 이동거리로 양수 누적합니다.
제자리 회전의 각속도를 임의의 전진거리로 환산하지 않습니다.

`distance_breakdown_m`:

- `delivery`: 일반 목적지 이동/진입 및 출차 구간.
- `recall`: 호출 목적지 이동 및 최종 복귀 승인 전 호출 동작.
- `return`: 복귀와 서비스 종료 전 후진 주차.
- `unknown`: 과거 구간 정보가 없거나 서로 다른 구간 사이의 정확한 경계를 알 수 없는 거리.

**전체 거리 = delivery + recall + return + unknown.**

현재 진행·중단·요청 대체된 기록의 실제 측정 거리도 전체에 한 번만 포함합니다.
완료 건수는 완료된 서비스만 셉니다. 호출과 복귀를 각각 완료 서비스 한 건씩 더하지 않습니다.
사이트 평균은 기존처럼 완료 기록만 대상으로 하며, 사이트 누적 합계는 모든 시도입니다.

서비스 상태의 벽시계와 속도 샘플의 ROS/시뮬레이션 시간이 다를 수 있으므로,
구간 변경을 가로지르는 샘플 간격을 임의 비율로 나누지 않습니다.
그 간격은 `LEG_TRANSITION_UNRESOLVED`/unknown으로 한 번 보존합니다.
같은 유형의 출차→도로 이동 전환에는 이 미확인 분리가 생기지 않습니다.

기존 수치 기준은 유지합니다.

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `service_metrics_minimum_speed_mps` | 0.03 m/s | 더 작은 속도는 0으로 취급 |
| `service_metrics_maximum_speed_mps` | 3.0 m/s | 이상 속도 제외 |
| `service_metrics_maximum_sample_gap_s` | 2.0 s | 긴 샘플 공백을 임의로 적분하지 않음 |
| `service_metrics_checkpoint_interval_s` | 5.0 s | 정기 저장, 상태 변화에는 즉시 저장 |

NaN/무효·이상 속도 이후에는 다음 정상 샘플부터 다시 기준점을 설정합니다.
샘플이 없는 시간의 거리를 추정해 채우지 않습니다.

## 시간과 실적의 의미

- `duration_s`: 요청 시작부터 현재/완료까지 **대기 포함 전체 경과시간**.
- `moving_s`, `waiting_s`: **유효한 속도 샘플 간격**에서 측정한 이동·정지 시간.
- `timing_complete`: 알려진 시간 누락 여부를 나타내는 보조 정보.
  통신 공백·과거 기록에 대해 전체 경과시간이 모두 측정되었다고 해석하면 안 됩니다.
- 일별 집계는 기존과 같은 **서비스 시작일**, Asia/Seoul 기준입니다.
  자정을 넘은 서비스 전체는 시작일에 속하며 달력 날짜별 실제 이동시간 분할은 아닙니다.
- 이것은 서비스 활성 기간의 플랫폼 속도 기반 실측입니다.
  운용 모드를 별도 분리한 “순수 자율주행만의 인증 마일리지”와는 다릅니다.

## DB와 API 호환

기존 `service_runs` 테이블에 다음 컬럼만 추가합니다.

`intent, request_id, phase, segments_json, interruption_reason`

기존 ID·시각·출처·결과·거리 컬럼은 유지합니다.
schema는 `PRAGMA user_version=2`이며 더 높은 미지원 버전을 덮어쓰지 않습니다.
기존 집계 API와 `distance_m`, `distance_km`, 완료 건수 필드는 유지합니다.
새 필드는 현재/최근 기록, today/lifetime, 일별, 사이트 집계에 추가됩니다.

- `GET /api/service-metrics/summary`
- `GET /api/service-metrics?days=30`

기존 데이터는 source 문자열이나 최종 상태만으로 유형을 역산하지 않습니다.
모든 과거 미분류 거리는 unknown으로 표시합니다.
과거에 이미 누락된 거리는 원시 로그 없이 이번 변경만으로 복원되지 않습니다.

### 기존 실증 기록 보존과 이전 점검

기존 DB를 지우거나 과거 기록을 새 행으로 다시 넣지 않습니다.
`historical_unclassified`는 구간 자료가 없는 기존 기록의 건수와 거리이며,
**이미 lifetime 합계에 포함된 부분**입니다. 합계에 다시 더하면 중복입니다.
새 운행의 구간 경계 미확인(`LEG_TRANSITION_UNRESOLVED`)과는 구분합니다.
실증 현황 화면은 과거 미분류 거리가 전체 누적에 포함되어 있음을 따로 안내합니다.

최종 상태가 복귀/충전이라고 왕복 합계를 전부 복귀로 분류할 수는 없습니다.
요청 source가 recall이라고 그 합계를 전부 호출로 분류할 수도 없습니다.
구간별 원시 거리·시각에 맞는 상태 로그가 없으면 일반/호출/복귀로 추정 배분하지 않습니다.
일반/복귀/호출과 구분하지 못한 과거 거리도 전체 km에는 모두 유지됩니다.

다른 로봇 PC로 적용할 때는 **그 장비의 DB**를 보존하십시오.
이 개발 PC의 DB나 CARLA 테스트 DB로 덮어쓰거나 합치면 안 됩니다.
DB에는 구형 기록의 실차/시뮬레이션 출처를 입증할 별도 필드가 없습니다.

로봇과 UI 백엔드를 안전하게 정지한 뒤, 적용 전 다음 복사 검증을 실행합니다.
`--source`는 해당 장비의 실제 DB 경로, `--output-dir`은 아직 없는 디렉터리입니다.
부모 디렉터리는 있어야 하며 심볼릭 링크 경로는 거부합니다.

```bash
cd /home/hong/camrod_ws/src
PYTHONPATH="$PWD/camrod_ui/runtime/python" python3 -m camrod_ui.service_metrics_migration \
  --source /home/hong/.local/state/camrod/service_metrics.sqlite3 \
  --output-dir /home/hong/camrod_metrics_backup_20260914
```

- `backup.sqlite3`: SQLite의 일관된 원본 스냅샷(WAL 반영).
- `migrated.sqlite3`: 새 형식으로 변환한 별도 사본. 자동 활성화하지 않습니다.
- `report.json`: 기존 모든 컬럼/행의 값·건수·원시 거리 보존, 무결성, 해시 검증.
- `snapshot.json`: 검증 성공 시 새 API 집계 표시용 스냅샷.

이 명령은 원본을 읽기 전용으로 열며 원본 교체·초기화·거리 재분류를 하지 않습니다.
검증이 실패하면 사본을 승인하지 않고 오류를 기록합니다.
운영 중 다른 프로세스가 원본에 기록할 수 있으므로 원본 파일의 바이트가 항상
불변이라고 주장하지 않고 **같은 SQLite 스냅샷**의 행·값끼리 대조합니다.

검증 PASS 이후 최신 UI 백엔드를 기존 DB 경로로 실행하면 원래 파일에 필요한
컬럼만 추가하여 이전 기록과 함께 계속 누적합니다. 별도 사본으로 바꿔 실행할
필요는 없습니다. `service_metrics_database_path` 파라미터 또는
`XDG_STATE_HOME`을 임의로 바꾸면 다른 DB가 선택되므로 경로를 확인하십시오.
이 도구는 활성 기록의 복구 때문에 기존 값이 달라지는 경우도 실패로 알려줍니다.
그때는 실패한 사본을 운영 DB로 복사하지 말고 진행 중인 기록부터 확인해야 합니다.

## UI

- 배송(가는 길) / 호출(가는 길) / 복귀 / 구간 미확인 + 합산 거리.
- 짧은 거리는 m로 보이고 합산 km도 함께 제공합니다.
- 현재 서비스의 요청 유형과 진행 단계, 최근 완료, 날짜별·사이트별·최근 이력.
- 상세 화면 4초 자동 갱신, 항상 가능한 새로고침.
- 요청 8초 제한, 중첩 요청 제한, 닫힌 화면의 요청 취소, 늦은 응답이 최신 값을 덮는 문제 방지.
- API 실패는 오류로 표시하며 정상적인 0 실적으로 위장하지 않습니다.

## 빌드 및 확인

이 작업의 검증은 별도 DB·가짜 publisher를 사용하는 자동 검사입니다.
아래 빌드와 서비스 재시작 이후 새 요청부터 실제 장비에 반영됩니다.
운영 중인 로봇은 안전하게 정지한 뒤 UI 백엔드를 재시작하십시오.

저장소가 `/home/hong/camrod_ws/src`이고 의존성이 설치된 환경의 예:

```bash
cd /home/hong/camrod_ws/src/camrod_ui/camrod_ui_robot/assets/frontend
npm run build

cd /home/hong/camrod_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths /home/hong/camrod_ws/src/camrod_ui --packages-select camrod_ui --symlink-install
source install/setup.bash
```

기존 플랫폼의 launch 방식으로 UI 백엔드를 다시 시작합니다.
운영 DB를 지우거나 초기화할 필요가 없습니다. 최초 실행 전 DB 백업을 권장합니다.

소스 회귀:

```bash
cd /home/hong/camrod_ws/src
python3 -m pytest -q camrod_ui/test
```

물리적 확인은 “호출 출차→사이트→적재/회전→최종 복귀→후진 주차” 전후 API를 저장하여
current_service가 출차 완료에서 사라지지 않는지, 호출·복귀 소계가 증가하는지,
전체와 네 소계 합이 같은지 확인합니다. 실차 결과는 이 오프라인 검증과 별도로 기록합니다.
