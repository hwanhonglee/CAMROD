# CARLA 시험 자료 — 게시 범위

<!-- HH_261103 - Preserve measured evidence without treating historical host paths or gates as portable runtime setup. -->

이 폴더와 11월 2일 `live_carla`는 실제 CARLA 시험의 원본 기록이다.
당시 `/home/hong/...` 경로·커밋 전 상태·실행 명령은 **그 시점의 이력**이며,
다른 호스트에서 그대로 실행할 설치 안내가 아니다. 재현은 별도
`camrod_carla_service_test/README.md`와 `tools/overlay.sh` 안내를 사용하고
해당 호스트의 게이트를 새로 만든다.

## Git에 포함한 자료

- 실제 PNG/JPEG와 GIF, 시험 결과·해시 JSON, XML, 관측·시각화 스크립트.
- 원시 JSONL의 무손실 gzip 사본(`.jsonl.gz`). 원시 내용을 편집하지 않았다.
- 당시 장면·게이트 영수증 사본은 `live_carla/provenance`에 보관했다.
  역사적 증빙일 뿐 현재 호스트의 구동 허가로 재사용할 수 없다.

분석 도구가 비압축 JSONL을 요구하면 저장소 루트에서 필요한 파일을 푼다.

```bash
gzip --keep --decompress docs/evidence/service_usability_20261103/live_carla/sensors/nearfield_actual_03.jsonl.gz
```

## 실행 호스트에만 보존한 자료

대용량 연속 MP4 3개는 일반 Git 객체로 중복 게시하지 않았다. 실행 호스트의
원본 시험 폴더에 보존했으며 파일 해시·크기와 선택 구간은 관련 manifest에 남았다.
자동 도킹 연속 GIF와 단계별 PNG/GIF는 Git에 포함했다. 비연속 요약 GIF를
전체 연속 영상이나 실제 제동 시험으로 해석하면 안 된다.

근거리 비교는 정지 상태의 입력·격자 관측이다. 20–30cm 배치의 기대 반응과
실물 충전기·차량 제동 성능을 통과로 판정하지 않았다.
