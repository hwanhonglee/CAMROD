# Test Result Index

<!-- HH_260806 - Index committed simulation and field-test result assets. -->

이 디렉터리는 재현 조건, 원본 데이터 식별자, 측정 수치, 판정을 함께
보존하는 테스트 결과 전용 공간이다. 일반 설명용 그림은
`docs/assets/module-guides/`에 두고, 특정 실행에서 측정한 결과는 여기에
실험별 디렉터리로 보관한다.

| Date | Experiment | Scope | Main result |
|---|---|---|---|
| 2026-08-06 | [Route boundary recovery](route-boundary-recovery-20260806/README.md) | AMD64 simulation, lanelet margin/retry/body clearance | First stop was margin-only; repeated retries found no crab; downstream physical-body contact makes forced passage unsafe |
| 2026-08-06 | [Provisional robot boundary adjustment](robot-boundary-adjustment-20260806/README.md) | Preserve old boundary, subtract 0.10 m per side, replay plus fresh full-bringup simulation | Normal route, margin stop/crab, and physical-body no-motion policy pass in AMD64 sim; complete physical survey remains required |

각 실험 디렉터리는 다음 파일을 기본으로 가진다.

- `README.md`: 목적, 조건, 절차, 결과 해석
- `result.json`: 기계 판독용 수치와 원본 SHA-256
- numbered PNG/GIF: 실행 순서에 맞춘 시각 결과

대용량 rosbag은 기본적으로 커밋하지 않는다. 커밋하지 않은 원본의 경로와
DB3 SHA-256은 각 `result.json`에 남겨 결과의 출처를 식별한다.
