# Claude Code 작업 컨텍스트 동기화

다른 PC(집)에서 Claude Code 작업을 이어가기 위한 메모리/대화 백업.

## 구성

- `memory/` — 프로젝트 메모리 (프로젝트 개요, 패키지 구조, 안전 경로, 빌드/실행,
  필드 베이스라인, **전조등/방향지시등 설계 결정**)
- `sessions/` — 대화 세션 원본. **git 제외됨(.gitignore)** — 공개 저장소라 개인정보
  (이메일/로컬 경로) 노출 방지. 필요 시 USB/개인 클라우드로 직접 전달할 것.

## 집 PC에서 복원하는 법

워크스페이스를 클론한 뒤, **워크스페이스 루트**(src의 부모, 예: `~/camrod_develop`)에서:

```bash
# 1. 프로젝트 슬러그 계산 (현재 경로의 / 를 - 로 치환한 값)
SLUG=$(pwd | sed 's|/|-|g')

# 2. 메모리 복원
mkdir -p ~/.claude/projects/$SLUG/memory
cp src/.claude/memory/*.md ~/.claude/projects/$SLUG/memory/

# 3. (선택) 대화 세션 복원 → claude 실행 후 /resume 으로 선택
cp src/.claude/sessions/*.jsonl ~/.claude/projects/$SLUG/
```

이후 워크스페이스 루트에서 `claude`를 실행하면 메모리가 자동 로드된다.
세션 이어가기는 같은 폴더에서 `claude --resume` 실행 후 목록에서 선택.

> 주의: 세션 복원은 Claude Code 버전이 크게 다르면 실패할 수 있음.
> 그 경우에도 메모리 + `camrod_platform/docs/lights-design-doc.html`(설계서+TODO)만으로
> 작업 재개에 충분하다.

## 최신화

이쪽 PC에서 작업 후 다시 동기화하려면:

```bash
cp ~/.claude/projects/$(pwd | sed 's|/|-|g')/memory/*.md src/.claude/memory/
```
