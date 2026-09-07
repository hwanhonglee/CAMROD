# CAMROD x CARLA final evidence index

이 디렉터리는 최종 acceptance를 통과한 실제 CARLA 주행 자료만 허용 목록으로 선별한
인덱스다. 모든 mission bundle은 B1~B13 strict validation과 원본 artifact hash를
대조했고, matrix symlink는 저장소 안의 일반 파일로 복사했다.

| 시나리오 | 결과 | 전체 시간 | 전체 거리 | 자료 |
|---|---:|---:|---:|---|
| Operator delivery + Return | 13/13 PASS | 7840.705 s | 1786.452243 m | [operator_delivery/](operator_delivery/) |
| Operator recall + Return | 13/13 PASS | 7669.826 s | 1723.057324 m | [operator_recall/](operator_recall/) |
| Guest UI recall + usage complete | 13/13 PASS | 7803.845 s | 1722.789852 m | [guest_recall/](guest_recall/) |
| Visible Robot UI manual 4WS | 4/4 PASS | - | - | [manual_4ws/](manual_4ws/) |

각 사이트/수동 시나리오 폴더에는 실제 화면 `PNG`, 실제 motion `GIF`, 4개 wheel 측정
요약이 있다. raw JSONL, MP4, 로그는 제외했으며 `curation_manifest.json`이 선별 규칙과
provenance를 기록한다.

```bash
sha256sum -c SHA256SUMS
```
