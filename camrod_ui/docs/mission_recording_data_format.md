# CAN·상태 정보는 어디에, 어떻게 저장되는가?

<!-- HH_260915 - Explain actual record fields and provenance, not invented CAN samples. -->

**기본 저장 대상은 CAN 드라이버가 해독해서 ROS로 보낸 플랫폼 정보다.**
CAN ID와 바이트 원문까지 저장하는 기능은 별도 옵션이며, 이번 시험에서는 사용하지 않았다.
화면의 `snapshot.json`에 모든 원본이 들어가는 것은 아니다.

## 1. 수집 → 저장 → 표시

```text
플랫폼 CAN 드라이버 → /platform/status
                       └→ 수신 시각 + 속도/모드/모터/진단 → telemetry_000001.jsonl

승인된 미션·복귀·STOP 이벤트 ─┐
안전 게이트 상태/속도·모드 변화 ├→ 상태 전이·정지·재개 → events_000001.jsonl
                            ┘

can0 (명시적으로 옵션을 켠 경우만) → 수신 프레임 → raw_can_000001.jsonl

미션 집계 → mission_journal.sqlite3 → snapshot.json → 읽기 전용 API → UI
```

`*.jsonl`은 **한 줄마다 독립된 JSON 객체 하나**인 append 로그다. 파일 전체를 감싼
`[ ... ]` 배열이 아니므로 JSONL 파일 전체를 한 번에 `JSON.parse()` 하면 안 된다.
읽을 때는 각 줄을 JSON으로 해석한다. 같은 미션 폴더 안에서 시간으로 이벤트와 주행 상태를 비교한다.

| 파일 | 기록 내용 | 기록 시점 / 용도 |
|---|---|---|
| `telemetry_*.jsonl` | 실제 수신한 `/platform/status`의 선택된 해독 필드와 시각·출처 | ROS 상태 메시지를 받을 때마다, worker에서 저장 |
| `events_*.jsonl` | 미션 단계, 정지·재개, 모드 전환, 보고된 정지 사유 | 의미 있는 이벤트/상태 변화가 생길 때. 같은 상태의 반복은 줄임 |
| `raw_can_*.jsonl` | CAN ID, payload hex, 프레임 플래그, 인터페이스와 수신 시각 | 옵션을 켜고 실제 프레임을 수신한 경우만 |
| `mission_journal.sqlite3` | 미션 ID·날짜 순번·거리·시간·결과·파일 목록·재시작 정보 | 새 기록기 전용 영구 집계 저장소 |
| `snapshot.json` | 현재/최근 미션, 자율·수동·미확인 거리, 횟수, 최근 이벤트, 파일 위치 | 약 1초마다 원자 교체. 원본 전체가 아닌 UI 요약 |

UI는 패널이 열려 있을 때 API를 3초 간격으로 읽는다. UI가 닫혀도 기록 노드가 실행 중이면
원본은 계속 저장된다. 실제 CAN 버스의 모든 프레임마다 기본 telemetry가 생기는 것은 아니다.
원시 CAN 프레임률과 `/platform/status` 재발행률은 별개다.

## 2. 주행 상태 한 줄의 실제 예

아래는 이번 **검증용 ROS 입력**으로 생성된 B7 일반 미션의
`telemetry_000001.jsonl` **2번째 줄에서 일부 필드만 발췌**한 것이다.
실차/CAN 측정 예시로 해석하지 않는다. 실제 행에는 추가 시각·품질·배터리·진단 필드가 있다.

```json
{
  "received_unix": 1789438886.9211838,
  "sample": {
    "sample_time_s": 1789438886.920439,
    "vx": 1.0,
    "vy": 0.0,
    "yaw_rate_radps": 0.0,
    "control_mode": 1,
    "estop": false,
    "error_code": 0,
    "vehicle_state": 0,
    "motion_mode": 0,
    "battery_percentage": 80.0,
    "decoded": {
      "motor_rpm": [100.0, 100.0, 100.0, 100.0],
      "motor_speed": [],
      "motor_angle": []
    },
    "source_topic": "/platform/status"
  }
}
```

이 행은 “해당 메시지 시각에 x방향 속도 1 m/s, CAN 모드1을 받았고,
모터 RPM 배열을 받았다”는 기록이다. `[]`는 값이 비어 있었다는 뜻이지 네 바퀴가 0이라는 뜻이 아니다.
이번 시험 도구에서 넣지 않은 ROS 필드에는 기본값 0/빈 문자열이 있을 수 있으므로,
그 값만 보고 실제 전압 0V 또는 진단 정상이라고 판정하면 안 된다.

| 저장 키 | 입력/의미 |
|---|---|
| `sample.vx`, `sample.vy` | `velocity.twist.linear.x/y`, m/s. 거리 적분의 입력 |
| `sample.yaw_rate_radps` | `velocity.twist.angular.z`, rad/s |
| `sample.control_mode` | 플랫폼 CAN/RC 제어 모드. 현재 분류 기준 1→자율, 0→수동 |
| `sample.motion_mode` | 플랫폼 운동 모드 코드. CAN/RC 모드와 다른 필드 |
| `sample.vehicle_state` | 플랫폼 차량 상태 코드. 미션 상태 번호와 다름 |
| `sample.estop`, `sample.error_code` | 수신한 비상정지·플랫폼 오류 코드 |
| `sample.battery_*`, `sample.is_charging` | SOC/사용 가능 여부/충전 여부. 전압·전류·온도는 `decoded` 아래 |
| `sample.decoded.motor_rpm/speed/angle` | 드라이버가 전달한 모터 배열 그대로. 이번 시험에는 speed/angle이 비어 있음 |
| `sample.decoded.state` | 플랫폼 `ModuleState` 진단: level, operating_state, message. 미션 FSM이 아님 |
| `sample.source_quality` | 유효성·ROS 시각 지연·시각 선택 기준·캐시 재발행 출처 제한 |

`sample_time_s`는 우선 `velocity.header.stamp`, 없으면 상위 header, 그마저 없으면
수신 ROS 시각을 쓴다. 선택 이유는 `source_quality.timestamp_basis`로 남는다.
`received_unix*`는 기록 노드 수신 시각, `received_monotonic_s`는 수신 순서/지연 점검용이다.
이것들이 실제 CAN 하드웨어 측정 시각을 대신 인증하는 것은 아니다.

## 3. 정지와 미션 상태는 이벤트로 저장

같은 미션의 `events_000001.jsonl` **6번째 줄 발췌**:

```json
{
  "at": "2026-09-15T11:21:28.520+09:00",
  "event": "stopped",
  "mode": "auto",
  "phase": "outbound",
  "reason": "reasons=obstacle_stop; TEST_INPUT_ONLY",
  "source": "/control/cmd_vel_safety_gate/status",
  "evidence": {
    "estop": false,
    "error_code": 0,
    "vehicle_state": 0
  }
}
```

뜻은 “일반 이동 구간에서 자율 모드로 정지했고, 안전 게이트가 obstacle_stop이라는
사유를 보고했다”이다. 기록기가 영상으로 장애물을 직접 검출한 것이 아니다.
테스트에서는 이 게이트 사유를 검증 입력으로 보냈으므로 `TEST_INPUT_ONLY`가 포함되어 있다.
실제 행의 `evidence.gate`에는 게이트 level/operating_state/message도 함께 있다.

같은 파일 **14번째 줄**의 미션 상태 변화는 다음 필드로 표현된다(일부 발췌).

```json
{
  "event": "phase",
  "phase": "return",
  "state": 3,
  "state_name": "RETURNING_TO_DROP_ZONE"
}
```

따라서 “state를 저장한다”는 말에는 서로 다른 정보가 들어 있다.

- `control_mode=1/0`: CAN/RC **제어 모드**.
- `vehicle_state`: **차량 플랫폼 상태**.
- `decoded.state`: 플랫폼 모듈의 **진단 상태**.
- 이벤트의 `state/state_name`: 이동·복귀·주차 등의 **미션 서비스 상태**.

하나의 `state=0`을 모든 의미에 공통으로 쓰는 구조가 아니다. 정지 중에도 CAN 모드1일 수 있고,
이때 `mode=auto` 안에 `stopped` 이벤트가 생기며 정지 유지 구간의 거리는 늘지 않는다.
수동으로 바뀌면 별도의 `mode_changed`에 previous_mode/current_mode를 남긴다.

주의: 게이트 ROS 메시지 전체를 무손실 저장하는 rosbag은 아니다. 현재 이벤트에는 선택된
level/operating_state/message/source 등이 남고, 게이트의 모든 원본 stamp/누락 토픽 배열 등이
그대로 보존되지는 않는다. “모든 ROS 필드와 CAN 프레임을 저장했다”고 표현하면 안 된다.

## 4. 원시 CAN을 켜면 무엇이 다른가?

`raw_can_interface:=can0`처럼 실제 사용 중인 인터페이스를 지정했을 때,
수신된 프레임을 `raw_can_*.jsonl`의 `frame` 아래에 기록한다.

| 원시 키 | 의미 |
|---|---|
| `frame.can_id`, `raw_can_id` | CAN ID 및 플래그를 포함한 원래 ID |
| `frame.data_hex`, `length` | 실제 수신 payload의 16진수 문자열과 길이 |
| `frame.extended/remote/error`, `format`, `fd_flags` | 프레임 종류/플래그 |
| `frame.channel` | 수신한 인터페이스 |
| `frame.received_unix_ns`, `received_monotonic_ns` | 소켓에서 프레임을 받은 시각 |
| `frame.kernel_timestamp_ns` | 커널 시각 옵션을 지원하고 받은 경우만 값 존재 |
| `frame.direction` | `bus_observed`. CAMROD의 송신인지 상대 노드 송신인지 단정하지 않음 |

이번 자료의 `raw_can_status`는 `disabled`이고 **원시 CAN 파일은 없다**.
가짜 CAN ID/바이트 예시를 실제 캡처처럼 추가하지 않았다. 수신기는 CAN을 송신하거나
인터페이스 bitrate를 변경하지 않는다. 수집 권한·링크·실제 프레임은 실차에서 따로 확인해야 한다.

## 5. 화면에서 볼 수 있는 범위

거리 막대는 저장된 자율/수동/미확인 거리의 구성이다. 실제 주행 경로 그림이 아니다.
단계 흐름은 최근 이벤트에서 관측된 단계를 표시하며, 중간 단계가 없다고 임의로 채우지 않는다.
정지·모드 전환을 먼저 보여주고 반복 진단/기술 원문은 펼쳐 보는 구조다.

현재 API는 snapshot만 읽으므로 **속도/RPM 원본 시계열 전체를 화면으로 보내지 않는다.**
CAN 파형·RPM 시간 그래프를 그리려면 선택한 미션의 telemetry를 제한적으로 읽는 조회 기능이
추가로 필요하다. 거리 집계나 몇 개 이벤트를 실제 속도 그래프처럼 만들어 보여주지 않는다.
snapshot의 최근 이벤트는 작은 요약 필드만 가지므로, 원본의 numeric state/state_name과
evidence 상세는 JSONL에서 확인한다. 현재 코드에서는 이전/현재 모드와 정지 시간도
이벤트에 값이 있을 때 요약에 포함하지만, 과거에 저장한 snapshot에는 없을 수 있다.

기존 실증 DB와 새 상세 누적은 서로 겹칠 수 있으므로 더하지 않는다.
원본 파일 회전/한도/실행 방법은 [기록기 운영 설명](mission_recording.md)을 따른다.

## 확인한 원본 경로

로컬 증빙(실차가 아닌 ROS 검증 입력):

`/home/hong/camrod_ws/_maintenance/20260915_mission_recorder/ros_smoke_01/records/2026-09-15/001_B7_delivery_227fa1ce/`

```bash
# 실제 파일의 특정 행을 읽기만 하는 예시
sed -n '2p' telemetry_000001.jsonl
sed -n '6p' events_000001.jsonl
sed -n '14p' events_000001.jsonl
```
