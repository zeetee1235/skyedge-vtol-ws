# Integration 테스트

실제 시뮬레이션 스택이 실행 중일 때만 동작합니다.

## 실행 조건

```bash
# 1. 시뮬 스택 올리기
docker compose -f docker/docker-compose.yml --profile sim up -d

# 2. 안정화 대기 (약 15초)
sleep 15

# 3. 테스트 실행
SIM_RUNNING=1 python -m unittest discover -s test/integration -v
```

## 테스트 목록

| 파일 | 관련 명세 | 내용 |
|------|-----------|------|
| `test_tc003_takeoff.py` | TC-003 | PX4 토픽 수신, 노드 실행 확인 |
| `test_tc007_full_mission.py` | TC-007 | setpoint 퍼블리시 주파수, 전 노드 alive |

## SIM_RUNNING 없이 실행하면?

모든 테스트가 `skip` 처리됩니다. CI에서 기본 `python -m unittest discover -s test -v` 실행 시 integration 테스트는 자동으로 스킵됩니다.
