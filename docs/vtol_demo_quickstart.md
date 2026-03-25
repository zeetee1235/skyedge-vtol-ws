# VTOL 데모 퀵스타트 (명령어만)

## 1) 최초 1회

```bash
git clone https://github.com/zeetee1235/skyedge-vtol-ws.git
cd skyedge-vtol-ws
docker compose -f docker/docker-compose.yml build vtol_sim
```

## 2) 실행 (Gazebo GUI + 자동 비행)

```bash
./sim.sh --down
./sim.sh --gui
```

## 3) 진행 로그 확인

```bash
docker compose -f docker/docker-compose.yml --profile sim logs -f vtol_sim
```

## 4) 웨이포인트 진행만 필터링

```bash
docker compose -f docker/docker-compose.yml --profile sim logs -f vtol_sim | grep -E "TAKEOFF|TRANSITION_TO_FW|NAVIGATE|WP [0-9]+ / 5|→ WP "
```

## 5) 상태값 확인 (선택)

```bash
docker compose -f docker/docker-compose.yml --profile sim exec -T vtol_sim bash -lc 'source /opt/ros/humble/setup.bash && source /vtol_ws/install/setup.bash && ros2 topic echo /drone1/fmu/out/vehicle_local_position_v1 --once'
```

## 6) 종료

```bash
./sim.sh --down
```
