#!/bin/bash
# =============================================
# VTOL 시뮬레이션 실행 스크립트
# 사용법:
#   ./sim.sh            # headless (기본)
#   ./sim.sh --gui      # Gazebo GUI 포함
#   ./sim.sh --down     # 전체 종료
# =============================================

set -e

COMPOSE="docker compose -f docker/docker-compose.yml --profile sim"
MODE="headless"

kickstart_mission() {
  echo "[sim] PX4 시뮬 파라미터 적용"
  $COMPOSE exec -T px4_sitl /bin/bash -lc "\
    /root/px4/build/bin/px4-param set COM_DISARM_PRFLT -1; \
    /root/px4/build/bin/px4-param set COM_DISARM_LAND -1; \
    /root/px4/build/bin/px4-param set COM_ARM_WO_GPS 1; \
    /root/px4/build/bin/px4-param set NAV_RCL_ACT 0; \
    /root/px4/build/bin/px4-param set NAV_DLL_ACT 0; \
    /root/px4/build/bin/px4-param set COM_RC_IN_MODE 4" || true

  echo "[sim] 미션 시작 보강 루프 (ARM/OFFBOARD/TAKEOFF, 60초)"
  (
    for _ in $(seq 1 60); do
      $COMPOSE exec -T px4_sitl /bin/bash -lc "\
        /root/px4/build/bin/px4-commander arm -f; \
        /root/px4/build/bin/px4-commander mode offboard; \
        /root/px4/build/bin/px4-commander takeoff" >/dev/null 2>&1 || true
      sleep 1
    done
  ) &
}

for arg in "$@"; do
  case $arg in
    --gui)     MODE="gui" ;;
    --down)    MODE="down" ;;
    --help|-h)
      echo "사용법: $0 [--gui | --down]"
      exit 0 ;;
  esac
done

case $MODE in
  down)
    echo "[sim] 시뮬레이션 종료 중..."
    $COMPOSE down
    exit 0
    ;;

  gui)
    echo "[sim] GUI 모드로 시작합니다 (Gazebo 창이 뜹니다)"
    xhost +local:docker 2>/dev/null || true
    $COMPOSE up -d px4_sitl xrce_agent vtol_sim
    echo "[sim] PX4 부팅 대기 중 (15초)..."
    sleep 15
    kickstart_mission
    if command -v gz >/dev/null 2>&1; then
      echo "[sim] 호스트 Gazebo Sim GUI 실행 (gz sim -g)"
      gz sim -g
    elif command -v gzclient >/dev/null 2>&1; then
      echo "[sim] 호스트 gzclient 실행 (구버전 Gazebo Classic)"
      gzclient
    else
      echo "[sim] 호스트에 gz/gzclient 없음 → 컨테이너 GUI로 시도"
      echo "[sim] (Ubuntu 예: sudo apt install gz-sim8)"
      $COMPOSE up gzclient
    fi
    ;;

  headless)
    echo "[sim] Headless 모드로 시작합니다 (Gazebo 창 없음)"
    $COMPOSE up -d px4_sitl xrce_agent vtol_sim
    echo "[sim] PX4 부팅 대기 중 (15초)..."
    sleep 15
    kickstart_mission
    $COMPOSE logs -f vtol_sim
    ;;
esac
