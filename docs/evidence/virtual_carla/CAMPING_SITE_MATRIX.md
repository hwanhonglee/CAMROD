# B1~B13 실제 CARLA 왕복 검증 매트릭스

이 문서는 Woraksan v15의 `Drop Zone → B1~B13 → Drop Zone 주차·충전` 최종
exact-v27 결과를 한곳에서 보여준다. 판정 근거는
[`current/`](current/)의 native matrix, strict validation, actual wheel summary,
실제 X11 PNG/GIF다.

## 최종 판정

| 사이트 | Operator delivery + Return | Operator recall + Return | visible Guest recall + 이용완료 |
|---|---:|---:|---:|
| B1 | PASS | PASS | PASS |
| B2 | PASS | PASS | PASS |
| B3 | PASS | PASS | PASS |
| B4 | PASS | PASS | PASS |
| B5 | PASS | PASS | PASS |
| B6 | PASS | PASS | PASS |
| B7 | PASS | PASS | PASS |
| B8 | PASS | PASS | PASS |
| B9 | PASS | PASS | PASS |
| B10 | PASS | PASS | PASS |
| B11 | PASS | PASS | PASS |
| B12 | PASS | PASS | PASS |
| B13 | PASS | PASS | PASS |
| **합계** | **13/13 PASS** | **13/13 PASS** | **13/13 PASS** |

세 경로의 공통 결과는 collision 0, 모든 wheel 전 구간 접지, 최종
`PARKED + CHARGING`이다. 각 경로는 독립적인 실제 왕복 실행이며 수동 pose teleport나
fake sensor로 성공 상태를 만든 것이 아니다.

| 경로 | 전체 시간 | 전체 거리 | outbound | return |
|---|---:|---:|---:|---:|
| Operator delivery | `7,840.705 s` | `1,786.452243 m` | `2,789.613 s / 889.573874 m` | `5,051.089 s / 896.878369 m` |
| Operator recall | `7,669.826 s` | `1,723.057324 m` | `2,573.926 s / 858.350562 m` | `5,095.902 s / 864.706762 m` |
| Guest recall | `7,803.845 s` | `1,722.789852 m` | `2,591.052 s / 858.190109 m` | `5,212.793 s / 864.599743 m` |

사이트별 정확한 시간·거리·Drop Zone 오차는 다음 파일에 있다.

- [`current/operator_delivery/summary/camping_site_metrics.md`](current/operator_delivery/summary/camping_site_metrics.md)
- [`current/operator_recall/summary/camping_site_metrics.md`](current/operator_recall/summary/camping_site_metrics.md)
- [`current/guest_recall/summary/camping_site_metrics.md`](current/guest_recall/summary/camping_site_metrics.md)

## authority별 의미

- Operator delivery: production destination workflow로 사이트를 선택하고 도착 뒤
  Robot UI Return을 사용한다.
- Operator recall: `/ui/camping_site_recall`의 typed recall 경로로 출발하고 도착 뒤
  Return을 사용한다. 최신 develop의 “현재 out pose에서 바로 복귀 경로 생성” 동작을
  포함한다.
- Guest recall: 표시 중인 Guest browser에서 `navigate`와 `usage_complete`를 누른다.
  evidence runner는 exact `guest:usage_complete:site=B#:g=N` source를 현재 사이트와
  generation nonce까지 대조한다.

`server`, `bridge`, `pacer`, `spawn`, `camrod-site-geometry`, UI/spectator 단계는
스스로 주행하지 않는다. `camping-sites*` 또는 아래 evidence runner가 실제 motion
authority다.

## 재실행 순서

각 명령은 별도 terminal에서 앞 단계의 ready 메시지를 본 뒤 실행한다.

```bash
cd /home/hong/camrod_ws/src
export CARLA_RENDER_MODE=onscreen

./scripts/virtual_carla/site_access.sh server
./scripts/virtual_carla/site_access.sh bridge
./scripts/virtual_carla/site_access.sh pacer
./scripts/virtual_carla/site_access.sh spawn
./scripts/virtual_carla/site_access.sh camrod-site-geometry
```

표시 화면이 필요한 authority는 별도 terminal에서 연다.

```bash
./scripts/virtual_carla/site_access.sh operator-ui
./scripts/virtual_carla/site_access.sh guest-ui
./scripts/virtual_carla/site_access.sh spectator
```

전체 증거 runner의 공통 사이트 값은 다음과 같다.

```bash
sites=B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13
```

각 runner는 반드시 서로 다른 빈 절대 `--output-root`를 사용한다.

```bash
./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority operator --mission-intent delivery --sites "$sites" \
  --output-root /absolute/evidence/operator_delivery

./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority operator --mission-intent recall --sites "$sites" \
  --output-root /absolute/evidence/operator_recall

./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority guest --mission-intent recall --sites "$sites" \
  --output-root /absolute/evidence/guest_recall \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority
```

runner 종료 뒤 최종 `PASS` manifest에 다시 묶는 post-final strict 검증 예시는 다음과
같다.

```bash
python3 scripts/virtual_carla/validate_site_evidence_collection.py \
  --input-root /absolute/evidence/guest_recall \
  --output-dir /absolute/evidence/guest_recall.strict-final \
  --sites "$sites" --authority guest --mission-intent recall
```

## B2/B12 경사 판정

최종 Guest B2와 B12 실제 왕복에서 최대 pitch는 약 `11.6°`였다. B12 outbound는
최대 `11.622° / 8.051 N·m`, return 도로·경사 구간은 `11.609° / 7.588 N·m`였고
4/4 wheel 접지를 유지했다. 토크 활성 상태에서 속도 `0.02 m/s` 미만인 최장 구간은
outbound `0.7 s`, return 도로 구간 `0.1 s`였으며 STALL/RECOVERY 전환은 0건이다.
따라서 최종 v15 실행에서는 턱을 삭제하거나 차체를 순간 이동하지 않고 실제 토크로
왕복했다.

상세 provenance, commit 경계, 수동 4WS와 sensor 판정은
[`../../VIRTUAL_CARLA_FINAL_VALIDATION.md`](../../VIRTUAL_CARLA_FINAL_VALIDATION.md)를
따른다.
