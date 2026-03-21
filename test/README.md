# 테스트 안내

## 구조

```
test/
  mock_ros2.py          ← ROS2 mock 헬퍼 (단위 테스트 공용)
  contract/             ← 구조 계약 테스트 (항상 PASS 유지)
    test_spec_contract.py         명세 문서 + 코드 구조 검증
    test_node_entrypoints.py      모든 노드 main() 진입점 검증
  unit/                 ← TDD 단위 테스트 (ROS2 불필요)
    test_waypoint_nav.py   ✅ 구현 완료 → PASS
    test_vision_aruco.py   ⏸ 구현 전  → skip  (비전 B 담당)
    test_vision_yolo.py    ⏸ 구현 전  → skip  (비전 A 담당)
    test_control_task.py   ⏸ 구현 전  → skip  (제어 B 담당)
    test_hw_gripper.py     ⏸ 구현 전  → skip  (HW A 담당)
    test_comm_lte.py       ⏸ 구현 전  → skip  (통신 B 담당)
  integration/          ← E2E 테스트 (SIM_RUNNING=1 필요, 기본 skip)
    test_tc003_takeoff.py       TC-003 PX4 토픽·노드 확인
    test_tc007_full_mission.py  TC-007 전 노드 alive·setpoint 주파수
```

## 실행 방법

```bash
# 전체 (contract + unit, integration은 자동 skip)
python -m unittest discover -s test -v

# 계층별
python -m unittest discover -s test/contract -v
python -m unittest discover -s test/unit -v

# 시뮬 실행 중일 때 integration 포함
SIM_RUNNING=1 python -m unittest discover -s test -v
```

## TDD 작업 흐름

1. `test/unit/` 에서 본인 담당 파일 열기 (skip 상태)
2. 파일 안의 `@unittest.skip(...)` 줄 제거
3. 각 파일 상단 **구현 체크리스트** 항목 구현
4. `python -m unittest test.unit.test_<이름> -v` → PASS 확인
5. 전체 재실행 후 PR 제출

## 명세 연결

| 테스트 | 관련 TC |
|--------|---------|
| `test_waypoint_nav.py` | TC-003, TC-004, TC-005, TC-006 |
| `test_vision_aruco.py` | TC-002, TC-007 |
| `test_vision_yolo.py` | TC-002, TC-007 |
| `test_control_task.py` | TC-006, TC-007 |
| `test_hw_gripper.py` | TC-007 |
| `test_comm_lte.py` | TC-002 |
| `test_tc003_takeoff.py` | TC-001, TC-003 |
| `test_tc007_full_mission.py` | TC-007 |
