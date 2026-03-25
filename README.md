# VTOL 프로젝트

[![Build Check](https://github.com/zeetee1235/skyedge-vtol-ws/actions/workflows/build_check.yml/badge.svg?branch=main)](https://github.com/zeetee1235/skyedge-vtol-ws/actions/workflows/build_check.yml)
[![Python Lint Check](https://github.com/zeetee1235/skyedge-vtol-ws/actions/workflows/lint_check.yml/badge.svg?branch=main)](https://github.com/zeetee1235/skyedge-vtol-ws/actions/workflows/lint_check.yml)

> **⚠️ NOTICE:** This repository is for **Paichai University SkyEdge Club members only**. Please do not create a pull request unless you are an active member of the SkyEdge club.

VTOL 기체를 이용한 자율 임무 수행 시스템

이 저장소는 ROS 2 Humble 기반의 워크스페이스이며, 비전 인식, 항법 제어, 정밀 착륙, 하드웨어 제어, 텔레메트리 기능을 패키지 단위로 나누어 관리합니다.

## 이 문서를 먼저 읽는 사람

이 README는 아래 사람을 기준으로 작성했습니다.

- 프로젝트를 처음 받는 팀원
- Git과 GitHub가 익숙하지 않은 팀원
- 전체 구조만 빠르게 보고 싶은 팀장 / 리뷰어

처음이라면 아래 순서로 읽는 것을 권장합니다.

1. 이 README로 전체 구조 파악
2. [CONTRIBUTING.md](CONTRIBUTING.md) 로 작업 절차 확인
3. [docs/git_basics.md](docs/git_basics.md) 로 Git 기초 확인
4. 필요한 세부 문서 확인

## 프로젝트 목적

이 프로젝트의 목표는 VTOL 기체를 이용해 자율 임무를 수행할 수 있는 소프트웨어 스택을 구성하는 것입니다.

예를 들어 아래 기능이 포함됩니다.

- 객체 인식
- ArUco 기반 위치 인식
- 웨이포인트 기반 이동
- 정밀 착륙
- 집게발 제어
- LTE 텔레메트리 전송
- Foxglove 기반 모니터링

## 시스템 구성

| 패키지 | 역할 | 담당 |
|--------|------|------|
| `vtol_bringup` | 전체 실행 통합 | 총괄 |
| `vtol_vision_yolo` | YOLO 객체 인식 | 비전 A |
| `vtol_vision_aruco` | ArUco 마커 인식 | 비전 B |
| `vtol_control_nav` | GPS/웨이포인트 제어 | 제어 A |
| `vtol_control_task` | 정밀 착륙 | 제어 B |
| `vtol_hw_gripper` | 집게발 제어 | HW A |
| `vtol_comm_lte` | LTE 텔레메트리 | 통신 B |

패키지 간 관계는 [docs/architecture.md](docs/architecture.md) 문서에서 더 자세히 볼 수 있습니다.

## 작업 전 꼭 알아둘 점

- 안정 버전 브랜치는 `main` 입니다.
- 일상 개발 기준 브랜치는 `develop` 입니다.
- 개인 작업은 보통 `develop`에서 `feature-...` 브랜치를 만들어 진행합니다.
- 다만 문서 수정/경미한 수정은 상황에 따라 `develop`에 바로 반영할 수 있습니다.

브랜치 운영 규칙은 아래와 같습니다.

| 브랜치 | 직접 push | PR | 리뷰 | CI |
|--------|-----------|---------|---------------|--------------|
| `main` | 원칙 불가 (긴급 hotfix 예외) | 필수 | 권장 | 필수 |
| `develop` | 가능 | 권장 | 권장 | 권장 |

Git이 익숙하지 않다면 아래 문서를 먼저 읽는 것을 강력히 권장합니다.

- [CONTRIBUTING.md](CONTRIBUTING.md)
- [docs/git_basics.md](docs/git_basics.md)
- [docs/README.md](docs/README.md)

## 빠른 시작

### 1. 저장소 받기

현재 저장소 주소 예시:

```bash
git clone https://github.com/zeetee1235/skyedge-vtol-ws.git
cd skyedge-vtol-ws
git switch develop
```

`git switch` 가 익숙하지 않다면 아래 명령도 가능합니다.

```bash
git checkout develop
```

### 2. 환경 준비 방법 선택

이 저장소는 크게 두 가지 방식으로 준비할 수 있습니다.

### 방법 A. 자동 셋업 스크립트 사용

Ubuntu 22.04 / ROS 2 Humble 기준으로 의존성을 설치하고 빌드까지 진행하는 방식입니다.

```bash
bash setup.sh
source ~/.bashrc
```

`setup.sh` 는 다음 작업을 수행합니다.

- ROS 2 Humble 설치 확인 및 설치
- 필요한 ROS 패키지 설치
- Python 패키지 설치
- `rosdep` 초기화 및 의존성 설치
- `colcon build --symlink-install` 실행
- `.bashrc` 에 ROS 및 워크스페이스 source 추가

### 방법 B. Docker 사용

로컬 환경 오염을 줄이고 팀 공통 환경을 맞추고 싶다면 Docker 방식을 사용할 수 있습니다.

시뮬레이션:

```bash
docker compose -f docker/docker-compose.yml --profile sim up
```

실기체:

```bash
docker compose -f docker/docker-compose.yml --profile real up
```

Foxglove 브릿지만 실행:

```bash
docker compose -f docker/docker-compose.yml up foxglove_bridge
```

### 3. 빌드

로컬 환경 기준:

```bash
colcon build --symlink-install
source install/setup.bash
```

새 터미널을 열 때마다 아래 명령이 필요할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 4. 실행

실행 방법은 사용 환경에 따라 다릅니다.

- 시뮬레이션은 Docker `sim` 프로필 사용
- 실기체는 Docker `real` 프로필 사용
- 모니터링은 Foxglove 브릿지 사용

Foxglove 연결 방법은 [docs/foxglove_setup.md](docs/foxglove_setup.md) 문서를 참고하세요.

## 브랜치 전략

```text
main          ← 안정 버전만 유지
  └── develop ← 팀 통합 개발
        └── feature/<패키지명>-<기능명> ← 개인 작업
```

간단한 원칙:

- 큰 작업/기능 작업은 `feature/...` 브랜치 + PR을 기본으로 합니다.
- 작은 수정(오탈자, 문서, 단순 설정)은 `develop` 직접 반영도 허용합니다.
- `main` 직접 반영은 긴급 상황에서만 최소 범위로 처리합니다.

예시:

```bash
git switch develop
git pull
git switch -c feature/vtol-vision-yolo-confidence-filter
```

자세한 작업 절차는 [CONTRIBUTING.md](CONTRIBUTING.md) 를 참고하세요.

## 처음 기여하는 사람을 위한 가장 짧은 작업 순서

```bash
git switch develop
git pull
git switch -c feature/<작업이름>
# 작업
git status
git add .
git commit -m "feat: 설명"
git push -u origin feature/<작업이름>
```

그다음 GitHub에서 `develop` 대상으로 Pull Request를 생성합니다.

## 자동 검사

이 저장소는 GitHub Actions로 아래 항목을 자동 검사합니다.

- ROS 2 워크스페이스 `colcon build` 검증
- `docker/Dockerfile` 기준 Docker 이미지 빌드 검증
- `python -m unittest discover -s test -v` 기준 기본 테스트 검증

즉, `main` 또는 `develop` 대상으로 push / PR을 올리면 최소한 아래를 자동으로 확인합니다.

- 워크스페이스가 실제로 빌드되는지
- Docker 이미지가 실제로 만들어지는지
- 기본 테스트 세트가 깨지지 않았는지

초보자를 위한 운영 원칙:

- 기본 브랜치(`main`, `develop`)는 가능한 한 항상 초록 상태를 유지합니다.
- 아직 구현하지 않은 기능 테스트는 기본 스위트에서 명확한 이유와 함께 `skip` 처리합니다.
- 구현을 시작한 담당자는 `skip` 을 제거하고, 구현과 테스트를 함께 초록으로 만드는 것을 목표로 합니다.

## 문서 목록

전체 문서 인덱스는 [docs/README.md](docs/README.md)를 참고하세요.

| 문서 | 내용 |
|------|------|
| [CONTRIBUTING.md](CONTRIBUTING.md) | 브랜치 전략, PR 작업 절차 |
| [docs/vtol_demo_quickstart.md](docs/vtol_demo_quickstart.md) | VTOL 데모를 처음부터 끝까지 실행하는 명령어 모음 |
| [docs/architecture.md](docs/architecture.md) | 패키지 구조, 토픽 목록, PX4 통신 구조 |
| [docs/simulation.md](docs/simulation.md) | PX4 + Gazebo 시뮬 실행 방법 |
| [docs/docker.md](docs/docker.md) | Docker Compose 서비스 구성 및 사용법 |
| [docs/foxglove_setup.md](docs/foxglove_setup.md) | Foxglove 브릿지 설정 및 모니터링 |
| [docs/git_basics.md](docs/git_basics.md) | Git 기초 (입문자용) |
| [docs/github_issues_guide.md](docs/github_issues_guide.md) | GitHub Issue 작성 방법 |
| [docs/ros2_study_links.md](docs/ros2_study_links.md) | ROS2·PX4·비전 학습 링크 |
| [docs/tdd_guide.md](docs/tdd_guide.md) | 테스트를 처음 만지는 팀원을 위한 TDD 안내 |
| [docs/test_spec.md](docs/test_spec.md) | 현재 테스트 정책, 레이어 구조, 합격 기준 |
