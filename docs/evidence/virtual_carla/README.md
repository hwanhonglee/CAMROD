# CAMROD × CARLA 증거 인덱스

최종 승인 자료의 단일 진입점은 [`current/`](current/)다. 이 디렉터리는 Woraksan
site-geometry v15에서 실제 `vehicle.ranger.default`를 구동한 exact-v27 결과만 담는다.
합성 그림이나 fake sensor 결과가 아니라 표시 중인 CarlaUE4와 production Robot/Guest
UI의 X11 캡처, 실제 wheel telemetry 요약, native mission report를 함께 보존한다.

전체 결과와 bundle별 실행 commit, B2/B12 경사 분석, 실행 명령은
[`../../VIRTUAL_CARLA_FINAL_VALIDATION.md`](../../VIRTUAL_CARLA_FINAL_VALIDATION.md),
develop 대비 변경 경계는
[`../../VIRTUAL_CARLA_DEVELOP_PARITY_AUDIT.md`](../../VIRTUAL_CARLA_DEVELOP_PARITY_AUDIT.md)를
따른다.

## 현재 v27 승인 결과

| 사용자 경로 | 결과 | 전체 시간 | 전체 거리 | 자료 |
|---|---:|---:|---:|---|
| Operator delivery + Return | **13/13 PASS** | `7,840.705 s` | `1,786.452243 m` | [`current/operator_delivery/`](current/operator_delivery/) |
| Operator typed recall + Return | **13/13 PASS** | `7,669.826 s` | `1,723.057324 m` | [`current/operator_recall/`](current/operator_recall/) |
| visible Guest recall + 이용완료 | **13/13 PASS** | `7,803.845 s` | `1,722.789852 m` | [`current/guest_recall/`](current/guest_recall/) |
| visible Robot UI manual 4WS | **4/4 PASS** | straight/turn/crab/zero-turn | 실제 UI 입력 | [`current/manual_4ws/`](current/manual_4ws/) |

세 mission 경로는 모두 collision 0, 모든 wheel 접지, B1~B13 최종
`PARKED + CHARGING`이다. Guest 결과는 visible page의 `navigate -> usage_complete`를
사용하고 `guest:usage_complete:site=B#:g=N` source를 site/nonce까지 검증했다.
sensor audit은 actual stream `36/36`, CARLA sensor actor `13/13`, fake/dummy owner 0이다.

`current/`에는 다음 자료가 있다.

- 실제 X11 contact-sheet PNG 43개
- 실제 motion GIF 43개
- physical-wheel summary PNG 43개
- 총 PNG 86개, GIF 43개
- site/matrix/runtime/sensor/capture/wheel manifest와 strict-validation 보고서
- root `SHA256SUMS` 528개

원본 MP4, raw wheel/ROS JSONL, runtime log는 Git 용량을 줄이기 위해 선별본에서
제외했다. 원본 파일의 byte 수와 SHA-256은 각 manifest에 남아 있고 host-local 원본은
`ranger-carla-4ws-pipeline/.work/evidence/v27_final/`에 있다.

## 빠른 탐색

- 사이트별 시간·거리: 각 bundle의 `summary/camping_site_metrics.md`
- 사이트별 실제 화면: `current/<bundle>/sites/B1..B13/visual/`
- 사이트별 네 wheel 측정: `current/<bundle>/sites/B1..B13/wheel_summary/`
- 수동 4WS 화면/측정: `current/manual_4ws/scenarios/{straight,turn,crab,zero_turn}/`
- B1~B13 통합 판정표: [`CAMPING_SITE_MATRIX.md`](CAMPING_SITE_MATRIX.md)
- UI 창 구성: [`UI_COMPOSITION.md`](UI_COMPOSITION.md)

## 무결성 확인

```bash
cd /home/hong/camrod_ws/src/docs/evidence/virtual_carla/current
sha256sum -c SHA256SUMS

find . -type f -name '*.png' | wc -l   # 86
find . -type f -name '*.gif' | wc -l   # 43
find . \( -type l -o -name '*.mp4' -o -name '*.jsonl' -o -name '*.log' \) -print
# 마지막 명령 출력은 0개여야 한다.
```

## 역사 자료

`current/` 밖의 `00_*`~`08_*`, 2026-08 JSON/PNG, 2026-09-01 smoke는 과거
tuned·개발 과정 자료다. 현재 v27 승인에 합산하지 않는다. 과거 재현이 필요할 때만
파일명에 적힌 날짜와 당시 JSON을 함께 사용한다.
