# Push 사전 점검 — 원격 변경 없음

2026-10-02에 `git push --dry-run origin develop:develop virtual/carla:virtual/carla`를
실행했다. **로컬 pre-push 훅에서 차단**됐으며 실제 push는 실행하지 않았다.

```
Push blocked: this ref contains pre-cleanup attribution history.
Migrate local work onto the rewritten remote history before pushing.
```

## 현재 참조

| 참조 | 사전 점검 시 커밋 |
| --- | --- |
| 로컬 develop | `e2069007a` (기능 커밋 `11034aa6c`, 이전 주석 커밋 `15277ab32` 포함) |
| 로컬 virtual/carla 소스 통합 | `c8035a892` (증빙 문서는 후속 커밋) |
| 원격 develop | `fe6f7815fc13eee1b482bee9e5d2b3badddec9a0` |
| 원격 virtual/carla | `d3ea6c4eeafd9b4bb0a17d9366c7afdd01654364` |
| v2.3.0 | 로컬·원격 모두 생성하지 않음 |

## 차단 원인

로컬 `.git/hooks/pre_push_identity_guard.py`는 전송할 새 커밋뿐 아니라 모든 조상을 검사한다.
동일 조건으로 읽기 전용 검사를 수행한 결과:

- `origin/develop`: 기존 검사 규칙과 충돌하는 커밋 **37개**.
- `origin/virtual/carla`: **0개**.
- 로컬 `develop`, `virtual/carla`: 각각 **37개**, 최신 기능 커밋에서 추가된 것이 아님.
- 최근 해당 커밋은 `c72327c38090`, `97912a7c46e0`, `bf52fea5d961`이며
  과거 커밋 메시지의 공동 작성자 trailer 규칙 때문에 걸린다.
- `origin/develop`이 이미 이 이력을 포함하므로 단순히 최신 원격 develop에 이식해도
  지금의 전체 조상 검사와 충돌한다.

훅 해제/우회, 작성자 삭제, 과거 커밋 재작성, force-push는 하지 않았다. 원격 이력 보존과
훅 정책을 어떻게 맞출지는 별도 확인이 필요하다. 자의적으로 바꾸지 않는다.

## 별도 릴리스 보류 사항

[시뮬레이터 거리 시간축 문제](release_v230_simulation_distance_clock_issue_20261002.md)가
확인됐다. 기록이 남는 것과 거리값이 정확한 것은 구별해야 한다. 해당 검증 전
`v2.3.0`을 완료 릴리스로 생성하지 않는다.

## 파일 선정

[소스 허용 목록](release_virtual_carla_source_allowlist_v230_20261002.txt)을 검토하여 선택 커밋했다.
필요한 GLB·도색 PNG는 포함했다. 실제/시뮬레이터 DB, writer lock, 고빈도 JSONL,
원본 MP4, 214MiB ZIP, 외부 라이브러리 및 이번 범위 밖 과거 주석 변경은 포함하지 않았다.
제외된 로컬 파일과 기존 변경은 삭제하지 않았다.
`origin/virtual/carla..virtual/carla`에서 50MB를 넘는 신규 Git blob은 없었다.

## 승인·검증 뒤에만 실행할 명령

아래는 지금 실행하지 않았다. 거리 검증, 최종 커밋 검토, 훅 정책 해결이 선행돼야 한다.
기존 태그를 이동하거나 force 옵션을 사용하지 않는다.

```bash
cd /home/hong/camrod_ws/src
git fetch origin develop virtual/carla
git push --dry-run origin develop:develop virtual/carla:virtual/carla
# 검증된 최종 develop 커밋을 확인한 뒤에만:
git tag -a v2.3.0 develop -m "CAMROD v2.3.0: validated core UI and mission recording"
git push origin develop:develop refs/tags/v2.3.0
git push origin virtual/carla:virtual/carla
```
