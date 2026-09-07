# Operator recall + Return — final accepted evidence

이 bundle은 Woraksan CARLA에서 B1~B13을 실제 주행한 최종 PASS 자료를 선별한
저장소다. 원본 runtime evidence는 변경하지 않았고 matrix symlink는 일반 JSON 파일로
역참조 복사했다.

## 결과

- B1~B13: `13/13 PASS`
- 전체 경과 시간: `7669.826 s`
- 전체 odometry 거리: `1723.057324 m`
- outbound: `2573.926 s`, `858.350562 m`
- return: `5095.902 s`, `864.706762 m`
- strict-validation 묶음: `2`개, 전체 사이트와 PNG/GIF/matrix/wheel hash 대조 완료

사이트별 시간·거리·Drop Zone 오차는
[`summary/camping_site_metrics.md`](summary/camping_site_metrics.md)에 있다.

## 보존 범위

각 `sites/B1`~`sites/B13`에는 site/matrix/capture/physical-wheel manifest,
실제 CARLA와 UI를 함께 담은 `representative_contact_sheet.png`,
`representative_motion.gif`, wheel JSON/CSV/PNG가 있다. `provenance/`에는 원본 runner
manifest와 독립 strict-validator 결과가 있다.

## 제외 범위

용량이 큰 raw wheel/ROS JSONL, 원본 MP4, runtime log, ffprobe/command 중간 파일은
의도적으로 복사하지 않았다. 이 파일들의 원본 hash와 byte 수는 보존된 manifest가
계속 기록한다.

## 무결성 확인

```bash
sha256sum -c SHA256SUMS
```
