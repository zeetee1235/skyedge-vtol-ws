# 문서 인덱스

이 폴더의 문서 목록과 각 문서의 용도를 정리합니다.

새 문서를 추가할 때는 아래 표와 [추가 방법](#새-문서-추가하는-방법)을 참고하세요.

---

## 문서 목록

| 문서 | 대상 독자 | 내용 |
|------|-----------|------|
| [vtol_demo_quickstart.md](vtol_demo_quickstart.md) | 전체 | VTOL 시뮬 데모 실행 명령어 모음 (처음부터 종료까지) |
| [architecture.md](architecture.md) | 전체 | 패키지 구조, 토픽 목록, 데이터 흐름 |
| [simulation.md](simulation.md) | 제어·비전·시뮬 담당 | PX4 + Gazebo 시뮬 실행 방법 (Docker / 네이티브) |
| [docker.md](docker.md) | 전체 | Docker Compose 서비스 구성 및 사용법 |
| [foxglove_setup.md](foxglove_setup.md) | 전체 | Foxglove 브릿지 설정 및 모니터링 방법 |
| [ros2_study_links.md](ros2_study_links.md) | 신규 팀원 | ROS2·PX4·비전 학습 링크 모음 |
| [git_basics.md](git_basics.md) | Git 입문자 | Git 기초 개념 및 하루 작업 흐름 |
| [github_issues_guide.md](github_issues_guide.md) | 전체 | GitHub Issue 작성 방법 및 PR 연결 |
| [tdd_guide.md](tdd_guide.md) | 구현 담당자 | 테스트를 읽고 구현하는 방법 |
| [test_spec.md](test_spec.md) | 전체 | 현재 테스트 정책, 레이어 구조, 합격 기준 |

> 협업 절차(브랜치 전략, PR 규칙)는 루트의 [CONTRIBUTING.md](../CONTRIBUTING.md)를 참고하세요.

---

## 처음 왔다면 읽는 순서

1. [../README.md](../README.md) — 프로젝트 전체 개요
2. [../CONTRIBUTING.md](../CONTRIBUTING.md) — 브랜치·PR 작업 절차
3. [git_basics.md](git_basics.md) — Git이 처음이라면
4. [architecture.md](architecture.md) — 패키지 구조 파악
5. 담당 파트 문서 (아래 참고)

### 담당별 추천 시작점

| 담당 | 먼저 읽을 문서 |
|------|----------------|
| 제어 A / B | architecture.md → simulation.md |
| 비전 A / B | architecture.md → simulation.md → foxglove_setup.md |
| HW / 통신 | architecture.md → docker.md |
| 총괄 | architecture.md → docker.md → simulation.md |
| 신규 팀원 | git_basics.md → github_issues_guide.md → ros2_study_links.md |
| 테스트 처음 하는 팀원 | test_spec.md → tdd_guide.md |

---

## 새 문서 추가하는 방법

1. `docs/` 폴더에 마크다운 파일 추가
   - 파일명은 영문 소문자 + 하이픈 사용 (예: `px4_params.md`)
   - 첫 줄은 `# 문서 제목` 형식으로 시작
2. **이 파일(README.md)** 표에 항목 추가
3. **루트 README.md** 의 "문서 목록" 섹션에도 추가
4. 관련 문서가 있으면 상호 링크 추가
5. PR 설명에 추가한 문서 명시

### 문서 작성 원칙

- 대상 독자를 문서 첫 부분에 명시
- 명령어는 복사해서 바로 실행할 수 있게 작성
- "적당히", "알아서" 같은 표현 대신 구체적인 예시 제시
- 새 도구·환경 변경이 생기면 관련 문서도 함께 업데이트
