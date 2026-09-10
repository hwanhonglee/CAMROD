# CAMROD 목적지 임무 권한 및 UI 연동

## 적용 기준

- 기준 브랜치: `origin/develop`
- 기준 커밋: `b194614b45d24f9a1a42b5c393417a97a28d9d7e`
- 검증일: 2026-09-07
- 적용 패키지: `camrod_planning`, `camrod_ui`

이 변경은 일반 배송, Recall, Guest UI, Robot UI가 동시에 같은 목적지
명령을 다룰 때 임무를 덮어쓰거나 오래된 화면이 Return/OFF를 실행하지
못하도록 공통 권한 계약을 추가한다.

## 임무 권한 모델

백엔드는 활성 임무마다 다음 네 필드를 하나의 원자적인 상태로 관리한다.

| 필드 | 의미 |
| --- | --- |
| `site` | `B1`부터 `B13`까지의 목적지 식별자 |
| `intent` | `delivery` 또는 `recall` |
| `owner` | 임무 후속 조작을 소유한 `operator`, `guest`, `robot` |
| `generation` | 백엔드 실행 세션과 임무 발행 순서를 함께 식별하는 양의 정수 |

같은 임무의 재전송은 같은 generation으로 멱등 처리한다. 다른 site, intent,
owner가 활성 임무를 덮어쓰려 하면 거부한다. Return, OFF, 취소 명령도 활성
site, owner, generation이 모두 일치할 때만 실행한다.

권한 상태는 transient-local QoS를 사용하는
`/ui/destination_dispatch_status`에 JSON으로 발행한다. 늦게 접속한 UI는
마지막 권한 상태를 즉시 받고, WebSocket 연결 시에도 같은 snapshot을 가장
먼저 받는다.

## 시나리오 흐름

### 일반 배송과 Return

1. Operator 또는 Robot UI가 `intent=delivery` 목적지 명령을 보낸다.
2. 백엔드가 새 generation을 발급하고 목적지 및 주행 명령을 발행한다.
3. 목적지 도착 후 `SITE_ARRIVED`, `UNLOAD_WAIT`,
   `WAITING_FOR_RETURN_REQUEST` 상태에서만 소유 UI의 Return을 허용한다.
4. Return은 원래 임무의 site와 generation을 유지한 채 한 번만 승인된다.
5. 복귀 완료 terminal 상태에서 백엔드가 활성 임무 권한을 해제한다.

### Recall과 Return

1. Guest UI 또는 Robot UI가 `intent=recall`과 목적지를 보낸다.
2. Planning 요청 header stamp를 auto-goal과 GoalSnapper 출력까지 보존한다.
   따라서 해당 Recall 요청과 UI의 경로 anchor가 정확히 연결된다.
3. 목적지 도착 후 `GUEST_LOADING_WAIT` 상태에서만 해당 임무 소유자의
   Return을 허용한다.
4. Return 요청에는 활성 site와 generation을 반드시 포함한다.
5. 오래된 응답, 이전 generation의 버튼 동작, 다른 UI 소유자의 동작은
   거부한다.

## Guest UI 동작

- 목적지 요청마다 nonce를 만들고 5초 안에 도착한 정확한 ACK만 수락한다.
- ACK의 site, source, owner, intent, generation이 현재 요청과 일치해야 한다.
- 사용 완료와 취소 요청에 site와 generation을 포함한다.
- Return 버튼은 Guest가 소유한 Recall 임무에서만 한 번 활성화된다.
- 재연결 시 초기 snapshot을 적용한 뒤 연결 중 쌓인 갱신을 순서대로 처리해
  오래된 상태로 되돌아가는 것을 막는다.

## Robot UI 동작

- 백엔드의 mission snapshot을 임무 권한의 단일 기준으로 사용한다.
- 권한 변경은 React state에도 반영하여 owner 또는 generation만 바뀌어도
  화면을 다시 그린다.
- Guest 소유 임무에서는 Robot Return 버튼을 숨기고 실행 함수에서도
  이중으로 차단한다.
- WebSocket 세대와 socket identity를 검사해 끊긴 연결의 callback을 버린다.
- 늦게 도착한 Recall HTTP 응답은 요청 epoch와 최신 권한 revision이 모두
  맞을 때만 적용한다.
- Return과 OFF 요청에 현재 mission generation을 포함한다.

## 백엔드 재시작 안전 정책

백엔드 시작 시 활성 임무 identity를 비운 뒤 기존 Nav2 goal 취소와 disengage를
요청한다. 대상 cancel future가 모두 완료되기 전에는 새 목적지와 Return을
받지 않는 fail-closed 상태를 유지한다. 서비스가 준비되지 않았거나 취소가
실패하면 다음 주기에 다시 시도한다.

전역 운영자 정지는 generation 없이도 항상 허용한다. 반면 개별 UI의 OFF,
취소, Return은 소유권 검사를 통과해야 한다.

## 변경 파일과 책임

| 파일 | 책임 |
| --- | --- |
| `camrod_planning/scripts/planning_state_machine_node.py` | Recall 요청 stamp를 경로 목표까지 보존 |
| `camrod_planning/test/test_recall_target_policy.py` | B1~B13 Recall 대상 및 stamp 상관관계 검증 |
| `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py` | 임무 권한, 경쟁 요청 차단, Return/OFF 검증, 재시작 복구 |
| `camrod_ui/runtime/python/camrod_ui/ui_guest_node.py` | Guest 요청/ACK/재연결/Return 권한 처리 |
| `camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html` | Guest 화면의 권한별 상태와 문구 |
| `camrod_ui/camrod_ui_robot/assets/frontend/src/App.js` | Robot 화면의 권한 snapshot과 stale callback 차단 |
| `camrod_ui/test/test_ui_backend_stop.py` | 백엔드 권한·정지·재시작 회귀 테스트 |
| `camrod_ui/test/test_ui_guest_contract.py` | Guest 상태/전송 계약 테스트 |
| `camrod_ui/test/test_ui_robot_frontend_contract.py` | Robot 권한 및 화면 계약 테스트 |

## 검증 명령과 결과

집중 기능 테스트:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q \
  camrod_planning/test/test_recall_target_policy.py \
  camrod_ui/test/test_ui_backend_stop.py \
  camrod_ui/test/test_ui_guest_contract.py \
  camrod_ui/test/test_ui_guest_transport.py \
  camrod_ui/test/test_ui_robot_frontend_contract.py
```

결과: `131 passed`

패키지 Python 회귀 테스트:

```bash
python3 -m pytest -q camrod_planning/test camrod_ui/test
```

결과: `194 passed`

독립 build/install 디렉터리의 패키지 빌드와 colcon 테스트도 통과했다.
최종 test-result는 `67 tests, 0 errors, 0 failures, 8 skipped`이다. skip은
선택적 실행 조건에 따른 항목이며 실패 항목은 없다.

Robot UI production build도 성공했으며 생성된 압축 JavaScript bundle은
약 80.3 kB였다.

## 검증 범위와 남은 런타임 확인

자동 검증은 목적지 경쟁, B1~B13 Recall 대상, 정상 배송/Recall의 Return
권한, stale ACK·OFF·취소·Return 차단, 재연결 순서, 재시작 차단, UI build를
포함한다.

실제 전체 시스템을 올린 뒤에는 다음 두 항목을 마지막으로 확인한다.

1. 각 UI에서 목적지를 요청했을 때 `/ui/destination_dispatch_status`의
   site, intent, owner, generation이 화면과 일치하는지 확인한다.
2. 목적지 도착 전 Return, 다른 UI의 Return, 이전 generation의 OFF가
   거부되고, 도착 후 소유 UI의 Return만 한 번 승인되는지 확인한다.
