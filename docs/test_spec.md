# 테스트 명세서 v0.0.1

> VTOL 드론 프로젝트 — Gazebo 시뮬레이션 기반 초기 검증 기준 문서

---

## 목차

1. [목적 및 범위](#1-목적-및-범위)
2. [테스트 환경](#2-테스트-환경)
3. [테스트 레이어 구조](#3-테스트-레이어-구조)
4. [노드별 단위 테스트 명세](#4-노드별-단위-테스트-명세)
5. [통합 테스트 케이스 (TC)](#5-통합-테스트-케이스-tc)
6. [성공 기준 요약](#6-성공-기준-요약)
7. [이후 확장 계획](#7-이후-확장-계획)
8. [결과 기록 형식](#8-결과-기록-형식)

---

## 핵심 원칙

초보자를 위해 이 프로젝트는 아래 원칙을 따릅니다.

1. 기본 브랜치(`main`, `develop`)는 항상 초록 상태를 유지합니다.
2. 아직 구현하지 않은 기능 테스트는 실패 상태로 두지 않고, 이유가 적힌 `@unittest.skip` 으로 둡니다.
3. 담당자가 구현을 시작하면 해당 테스트의 `skip` 을 제거하고 구현과 함께 초록으로 바꿉니다.

처음 참여한 팀원은 아래 명령 하나부터 실행하면 됩니다.

```bash
python -m unittest discover -s test -v
```

기대 결과: contract PASS + waypoint_nav PASS + 나머지 **skip** (빨간색 없음)

---

## 1. 목적 및 범위

### 목적

실제 기체 투입 전, **Gazebo + PX4 SITL + ROS2** 환경에서 기본 비행·통신·인터페이스가
정상 동작하는지 검증한다.

현 단계 목표:
- 기본 비행 루프 (이륙 → 이동 → 착륙) 자동화 확인
- ROS2 토픽 인터페이스 정상 연결 확인
- 이후 비전 / 그리퍼 / 임무 로직 확장을 위한 노드 구조 사전 검증

### 포함 범위

| 항목 | 설명 |
|------|------|
| Gazebo 기체 스폰 | PX4 SITL standard_vtol 모델 |
| ROS2 노드 통신 | uXRCE-DDS 브리지를 통한 PX4 ↔ ROS2 토픽 |
| 자동 이륙 | Offboard + TrajectorySetpoint |
| Waypoint 이동 | NED 좌표 기반 단일/복수 |
| 자동 착륙 | VehicleCommand NAV_LAND |
| 노드 단위 테스트 | mock_ros2 기반, ROS2 설치 불필요 |

### 제외 범위 (현 단계)

- 정밀 객체 인식 및 타겟 추적
- Gripper 실기체 제어
- VTOL MC ↔ FW 전환 정밀 검증
- 강풍 / 센서 노이즈 / 복합 장애물 환경
- 실환경 오차 보정

---

## 2. 테스트 환경

### 소프트웨어 구성

| 구성 요소 | 버전/종류 | 비고 |
|-----------|-----------|------|
| OS | Ubuntu 22.04 LTS | |
| ROS2 | Humble | `ros:humble` Docker 이미지 |
| PX4 SITL | main 브랜치 | `jonasvautherin/px4-gazebo-headless` |
| 기체 모델 | standard_vtol | Gazebo Classic |
| DDS 브리지 | Micro XRCE-DDS Agent v2.4.2 | UDP 포트 8888 |
| 토픽 네임스페이스 | `/drone1` | `PX4_UXRCE_DDS_NS=drone1` |
| 테스트 프레임워크 | Python unittest | mock_ros2.py |

### ROS2 토픽 인터페이스

#### 제어 입력 (`/drone1/fmu/in/`)

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `offboard_control_mode` | `OffboardControlMode` | Offboard 모드 유지 (10 Hz 이상 필요) |
| `trajectory_setpoint` | `TrajectorySetpoint` | 위치 목표 전송 (NED, m) |
| `vehicle_command` | `VehicleCommand` | ARM / 모드 전환 / 착륙 명령 |

#### 상태 출력 (`/drone1/fmu/out/`)

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `vehicle_status` | `VehicleStatus` | arming_state, nav_state |
| `vehicle_local_position` | `VehicleLocalPosition` | NED 위치 (x, y, z) |

#### 내부 노드 간 토픽

| 토픽 | 방향 | 용도 |
|------|------|------|
| `/drone1/camera/image_raw` | PX4 → Vision 노드 | 카메라 입력 |
| `/vtol/aruco/pose` | ArucoDetect → PrecisionTask | ArUco 마커 위치 |
| `/vtol/yolo/detections` | YoloDetect → PrecisionTask | YOLO 탐지 결과 |
| `/vtol/gripper/command` | PrecisionTask → HwGripper | 그리퍼 열기/닫기 |

### 초기 월드 조건

- 평지, 장애물 없음
- 이륙 지점 1개 (홈 위치)
- Waypoint 2개 이상 (NED 좌표)
- 착륙 지점: 마지막 waypoint 또는 홈 복귀

---

## 3. 테스트 레이어 구조

```
test/
  mock_ros2.py            ROS2·px4_msgs mock 헬퍼 (단위 테스트 공용)
  contract/               구조 계약 테스트 — 항상 PASS 유지 필수
    test_spec_contract.py       명세 문서 + 핵심 파일 구조 검증
    test_node_entrypoints.py    모든 노드 main() 진입점 형식 검증
  unit/                   TDD 단위 테스트 — ROS2 설치 불필요
    test_waypoint_nav.py    ✅ 구현 완료 (PASS 유지)
    test_vision_aruco.py    ⏸ 구현 전 skip → 비전 B 담당
    test_vision_yolo.py     ⏸ 구현 전 skip → 비전 A 담당
    test_control_task.py    ⏸ 구현 전 skip → 제어 B 담당
    test_hw_gripper.py      ⏸ 구현 전 skip → HW A 담당
    test_comm_lte.py        ⏸ 구현 전 skip → 통신 B 담당
  integration/            E2E 테스트 — SIM_RUNNING=1 환경에서만 실행
    test_tc003_takeoff.py       TC-003 PX4 토픽·노드 확인
    test_tc007_full_mission.py  TC-007 전 노드 alive·setpoint 주파수
```

### 레이어별 특성

| 레이어 | 실행 조건 | 속도 | 의존성 |
|--------|-----------|------|--------|
| contract | 항상 | < 1 s | 파일 시스템만 |
| unit | 항상 | < 5 s | Python + mock_ros2 |
| integration | `SIM_RUNNING=1` | 수십 초 | Docker Compose + 실행 중인 시뮬 |

### TDD 작업 흐름

```
[현재 기본 상태]                 [기능 구현 시작 후 목표]
contract: PASS                 contract: PASS 유지
unit waypoint_nav: PASS        unit 담당 영역: PASS
unit 나머지 5파일: skip     →  skip 제거 후 PASS
integration: skip              integration: PASS (SIM_RUNNING=1)
```

각 담당자 작업 순서:
1. `test/unit/` 에서 본인 담당 파일 확인
2. 파일 상단의 `skip` 이유를 읽고, 구현을 시작할 때 그 `skip` 을 제거
3. 파일 상단 **구현 체크리스트** 항목을 순서대로 구현
4. `python -m unittest test.unit.test_<파일명> -v` 로 개별 확인
5. 전체 테스트 재실행 후 PR 제출

---

## 4. 노드별 단위 테스트 명세

### 4-1. WaypointNavNode (vtol_control_nav) ✅ 구현 완료

**파일:** `test/unit/test_waypoint_nav.py`
**관련 TC:** TC-003, TC-004, TC-005, TC-006

#### 테스트 항목 (22개)

| 클래스 | 메서드 | 검증 내용 |
|--------|--------|-----------|
| TestInit | `test_initial_state_is_idle` | 초기 상태 = IDLE |
| TestInit | `test_publishes_offboard_control_mode` | `/drone1/fmu/in/offboard_control_mode` 퍼블리셔 존재 |
| TestInit | `test_publishes_trajectory_setpoint` | `/drone1/fmu/in/trajectory_setpoint` 퍼블리셔 존재 |
| TestInit | `test_publishes_vehicle_command` | `/drone1/fmu/in/vehicle_command` 퍼블리셔 존재 |
| TestInit | `test_subscribes_vehicle_status` | `/drone1/fmu/out/vehicle_status` 구독 존재 |
| TestInit | `test_subscribes_vehicle_local_position` | `/drone1/fmu/out/vehicle_local_position` 구독 존재 |
| TestInit | `test_takeoff_z_is_negative_ned` | `_takeoff_z < 0` (NED 좌표계 상향 = 음수) |
| TestInit | `test_cruise_z_is_negative_ned` | `_cruise_z < 0` |
| TestInit | `test_has_at_least_two_waypoints` | waypoint 2개 이상 정의 |
| TestReached | `test_true_when_at_target` | 목표에 도달했을 때 True |
| TestReached | `test_false_when_far` | 멀리 있을 때 False |
| TestReached | `test_boundary_exactly_at_threshold` | 거리 == threshold 일 때 False (strict `<`) |
| TestStateMachine | `test_stays_idle_before_10_cycles` | 10회 미만 루프 → IDLE 유지 |
| TestStateMachine | `test_transitions_to_arming_after_10_cycles` | 10회 루프 후 → ARMING |
| TestStateMachine | `test_transitions_to_takeoff_when_armed` | ARMED 상태 수신 → TAKEOFF |
| TestStateMachine | `test_stays_arming_when_not_armed` | STANDBY 상태 → ARMING 유지 |
| TestStateMachine | `test_transitions_to_navigate_at_takeoff_altitude` | 이륙 고도 도달 → NAVIGATE |
| TestStateMachine | `test_advances_waypoint_when_reached` | waypoint 도달 → `_wp_idx` 증가 |
| TestStateMachine | `test_transitions_to_land_after_last_waypoint` | 마지막 waypoint 도달 → LAND |
| TestStateMachine | `test_transitions_to_done_from_land` | LAND 상태 루프 → DONE |
| TestVTOLCommands | `test_vtol_to_fw_callable` | `_cmd_vtol_to_fw()` 메서드 존재 |
| TestVTOLCommands | `test_vtol_to_mc_callable` | `_cmd_vtol_to_mc()` 메서드 존재 |

#### 상태 머신 흐름

```
IDLE (10 tick 대기)
  └─→ ARMING      : arm 명령 전송, 상태 폴링
        └─→ TAKEOFF   : offboard 모드 + 이륙 setpoint
              └─→ NAVIGATE : waypoint 순차 방문
                    └─→ LAND     : 착륙 명령
                          └─→ DONE
```

---

### 4-2. ArucoDetectNode (vtol_vision_aruco) ⏸ 구현 전 (skip)

**파일:** `test/unit/test_vision_aruco.py`
**담당자:** 비전 B
**관련 TC:** TC-002, TC-007

#### 구현 체크리스트

- [ ] `/drone1/camera/image_raw` (`sensor_msgs/Image`) 구독
- [ ] `cv2.aruco` 로 마커 검출 (사전 딕셔너리: `DICT_4X4_50` 권장)
- [ ] 카메라 캘리브레이션 파라미터 적용 (`camera_matrix`, `dist_coeffs`)
- [ ] `/vtol/aruco/pose` (`geometry_msgs/PoseStamped`) 퍼블리시
- [ ] `vtol.aruco_marker_id` 파라미터 선언 (기본값 0)

#### 테스트 항목 (5개)

| 메서드 | 검증 내용 | 합격 기준 |
|--------|-----------|-----------|
| `test_subscribes_to_camera_image` | 카메라 토픽 구독 | `_subscriptions`에 키 존재 |
| `test_publishes_aruco_pose` | pose 퍼블리셔 | `_publishers`에 키 존재 |
| `test_declares_marker_id_parameter` | marker_id 파라미터 | `_parameters`에 키 존재 |
| `test_image_callback_exists` | 콜백 callable | `callback` is callable |
| `test_pose_published_when_marker_detected` | 마커 탐지 시 publish | `pub.publish.called == True` |

---

### 4-3. YoloDetectNode (vtol_vision_yolo) ⏸ 구현 전 (skip)

**파일:** `test/unit/test_vision_yolo.py`
**담당자:** 비전 A
**관련 TC:** TC-002, TC-007

#### 구현 체크리스트

- [ ] `/drone1/camera/image_raw` 구독
- [ ] `ultralytics` YOLO 모델 로드 (`yolo11n.pt` 또는 커스텀)
- [ ] 바운딩 박스 추론 후 결과 가공
- [ ] `vtol.yolo_confidence_threshold` 파라미터로 결과 필터링 (기본값 0.5)
- [ ] `/vtol/yolo/detections` (`vision_msgs/Detection2DArray`) 퍼블리시

#### 테스트 항목 (5개)

| 메서드 | 검증 내용 | 합격 기준 |
|--------|-----------|-----------|
| `test_subscribes_to_camera_image` | 카메라 토픽 구독 | `_subscriptions`에 키 존재 |
| `test_publishes_yolo_detections` | detections 퍼블리셔 | `_publishers`에 키 존재 |
| `test_declares_confidence_threshold_parameter` | confidence 파라미터 | `_parameters`에 키 존재 |
| `test_image_callback_is_callable` | 콜백 callable | `callback` is callable |
| `test_detections_published_on_image_received` | 이미지 수신 시 publish | `pub.publish.called == True` |

---

### 4-4. PrecisionTaskNode (vtol_control_task) ⏸ 구현 전 (skip)

**파일:** `test/unit/test_control_task.py`
**담당자:** 제어 B
**관련 TC:** TC-006, TC-007

#### 구현 체크리스트

- [ ] `/vtol/aruco/pose` 구독
- [ ] `/vtol/yolo/detections` 구독
- [ ] `/drone1/fmu/in/trajectory_setpoint` 퍼블리시 (정밀 하강 setpoint)
- [ ] `/vtol/gripper/command` (`std_msgs/Int32`) 퍼블리시
- [ ] `vtol.landing_descent_altitude` 파라미터 선언 (기본값 1.0 m)
- [ ] 상태 머신: `IDLE → DESCEND → HOVER → GRIP → DONE`

#### 테스트 항목 (7개)

| 메서드 | 검증 내용 | 합격 기준 |
|--------|-----------|-----------|
| `test_subscribes_to_aruco_pose` | ArUco pose 구독 | `_subscriptions`에 키 존재 |
| `test_subscribes_to_yolo_detections` | YOLO detections 구독 | `_subscriptions`에 키 존재 |
| `test_publishes_gripper_command` | 그리퍼 명령 퍼블리셔 | `_publishers`에 키 존재 |
| `test_publishes_trajectory_setpoint` | setpoint 퍼블리셔 | `_publishers`에 키 존재 |
| `test_declares_descent_altitude_parameter` | descent_altitude 파라미터 | `_parameters`에 키 존재 |
| `test_has_state_attribute` | `_state` 속성 | `hasattr(node, '_state')` |
| `test_gripper_open_on_command` | 그리퍼 열기 호출 시 publish | `pub.publish.called == True` |

---

### 4-5. ArduinoCmdNode (vtol_hw_gripper) ⏸ 구현 전 (skip)

**파일:** `test/unit/test_hw_gripper.py`
**담당자:** HW A
**관련 TC:** TC-007

#### 구현 체크리스트

- [ ] `/vtol/gripper/command` (`std_msgs/Int32`) 구독
- [ ] 구독 콜백에서 `pyserial`로 시리얼 명령 전송 (`/dev/ttyACM0` 기본)
- [ ] `vtol.gripper_open_angle` / `vtol.gripper_close_angle` 파라미터 적용
- [ ] 시리얼 포트 없을 때 graceful 처리 (예외 → 로그, 크래시 금지)

#### 테스트 항목 (5개)

| 메서드 | 검증 내용 | 합격 기준 |
|--------|-----------|-----------|
| `test_subscribes_to_gripper_command` | 그리퍼 명령 구독 | `_subscriptions`에 키 존재 |
| `test_declares_open_angle_parameter` | open_angle 파라미터 | `_parameters`에 키 존재 |
| `test_declares_close_angle_parameter` | close_angle 파라미터 | `_parameters`에 키 존재 |
| `test_command_callback_is_callable` | 콜백 callable | `callback` is callable |
| `test_serial_send_called_on_command` | send_serial 계열 메서드 존재 | `getattr` 성공 |

---

### 4-6. TelemetryNode (vtol_comm_lte) ⏸ 구현 전 (skip)

**파일:** `test/unit/test_comm_lte.py`
**담당자:** 통신 B
**관련 TC:** TC-002

#### 구현 체크리스트

- [ ] `/drone1/fmu/out/vehicle_status` 구독
- [ ] `/drone1/fmu/out/vehicle_local_position` 구독
- [ ] `vtol.lte_gcs_ip` / `vtol.lte_gcs_port` 파라미터 적용
- [ ] `create_timer`로 주기 전송 등록 (1 Hz 권장)
- [ ] `send_telemetry` 또는 `_send_to_gcs` 메서드로 UDP 패킷 전송
- [ ] 전송 실패 시 예외 처리 (소켓 에러 → 로그)

#### 테스트 항목 (6개)

| 메서드 | 검증 내용 | 합격 기준 |
|--------|-----------|-----------|
| `test_subscribes_to_vehicle_status` | vehicle_status 구독 | `_subscriptions`에 키 존재 |
| `test_subscribes_to_vehicle_local_position` | local_position 구독 | `_subscriptions`에 키 존재 |
| `test_declares_gcs_ip_parameter` | lte_gcs_ip 파라미터 | `_parameters`에 키 존재 |
| `test_declares_gcs_port_parameter` | lte_gcs_port 파라미터 | `_parameters`에 키 존재 |
| `test_has_send_telemetry_method` | 전송 메서드 존재 | `getattr` 성공 |
| `test_timer_registered_for_periodic_send` | 타이머 등록 | `len(_timer_callbacks) > 0` |

---

## 5. 통합 테스트 케이스 (TC)

> 통합 테스트는 `SIM_RUNNING=1` 환경에서만 실행됩니다.
> 시뮬 실행 방법은 [simulation.md](simulation.md) 참고.

---

### TC-001. 시뮬레이션 실행 확인

**목적:** Gazebo, PX4 SITL, ROS2 노드가 동시에 정상 실행되는지 확인
**관련 테스트 파일:** `test/integration/test_tc003_takeoff.py`

**사전 조건:**
- Docker Compose sim 프로파일 기동 완료
- `MicroXRCEAgent` 프로세스 실행 중

**절차:**
1. `docker compose --profile sim up -d` 실행
2. 30초 대기
3. `ros2 topic list` 로 PX4 토픽 확인
4. `ros2 node list` 로 ROS2 노드 확인

**합격 기준:**
- `/drone1/fmu` 접두사 토픽이 ROS2에서 보임
- `waypoint_nav` 노드가 `ros2 node list` 에 출력됨
- 치명적 에러 없이 30초 이상 유지됨

---

### TC-002. 상태 수신 확인

**목적:** ROS2에서 기체의 위치/자세/상태를 읽을 수 있는지 확인
**관련 노드:** `TelemetryNode` (vtol_comm_lte)
**관련 테스트:** `test_comm_lte.py` (unit)

**사전 조건:** TC-001 합격

**합격 기준:**

| 토픽 | 조건 |
|------|------|
| `vehicle_status` | `ros2 topic hz` ≥ 1 Hz, arming_state 값 유효 |
| `vehicle_local_position` | `ros2 topic hz` ≥ 10 Hz, x/y/z 값 유효 |
| `camera/image_raw` | 카메라 활성화 시 수신 가능 (Phase 2) |

---

### TC-003. 자동 이륙 테스트

**목적:** Offboard 모드에서 기체가 지정 고도까지 자동 이륙하는지 확인
**관련 테스트 파일:** `test/integration/test_tc003_takeoff.py`
**관련 노드:** `WaypointNavNode`

**사전 조건:** TC-001 합격, 시뮬 실행 중

**입력 파라미터:**

| 파라미터 | 값 | 비고 |
|----------|-----|------|
| `takeoff_altitude` | 5.0 m | global_params.yaml |
| `IDLE_CYCLES_BEFORE_ARM` | 10 | 10 tick 후 ARM 시작 |

**절차:**
1. `WaypointNavNode` 시작
2. 10 tick (1 s) 후 ARM + Offboard 전환 자동 실행
3. `trajectory_setpoint.z = -takeoff_altitude` 전송
4. `vehicle_local_position.z ≤ -takeoff_altitude + tolerance` 감지

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 이륙 시작 | ARM 후 10초 이내 |
| 목표 고도 도달 | 5.0 m ± 0.5 m |
| 안정 유지 | 3초 이상 고도 유지 |
| setpoint 주파수 | ≥ 10 Hz (Offboard 모드 유지 조건) |

---

### TC-004. 단일 Waypoint 이동 테스트

**목적:** 이륙 후 단일 목표 지점으로 정상 이동하는지 확인
**관련 노드:** `WaypointNavNode`
**관련 unit 테스트:** `TestStateMachine.test_advances_waypoint_when_reached`

**사전 조건:** TC-003 합격

**입력:**
- waypoint: `(5.0, 0.0, -5.0)` NED (북쪽 5 m, 고도 5 m)

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 이동 시작 | setpoint 갱신 후 10초 이내 이동 시작 |
| 도달 판정 | 목표 반경 **1.0 m** 이내 진입 |
| 도달 후 안정 | 3초 이상 반경 내 유지 |

---

### TC-005. 복수 Waypoint 이동 테스트

**목적:** 2개 이상 waypoint를 순서대로 방문하는지 확인
**관련 노드:** `WaypointNavNode`
**관련 unit 테스트:** `TestStateMachine.test_transitions_to_land_after_last_waypoint`

**사전 조건:** TC-004 합격

**입력:**
```yaml
waypoints:
  - [5.0,  0.0, -5.0]   # WP1: 북쪽 5 m
  - [5.0,  5.0, -5.0]   # WP2: 북동쪽 5 m
```

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 순서 유지 | WP1 → WP2 순서 이탈 없음 |
| 각 WP 도달 | 반경 1.0 m 이내 진입 |
| 미션 중 안정성 | 제어 불능 (`nav_state` 이상) 발생 없음 |

---

### TC-006. 자동 착륙 테스트

**목적:** 마지막 waypoint 도달 후 자동으로 착륙하는지 확인
**관련 노드:** `WaypointNavNode`
**관련 unit 테스트:** `TestStateMachine.test_transitions_to_done_from_land`

**사전 조건:** TC-005 합격

**절차:**
1. 마지막 waypoint 도달 → LAND 상태 전이
2. `VehicleCommand.VEHICLE_CMD_NAV_LAND` 전송
3. `vehicle_local_position.z` 감소 모니터링
4. 착륙 감지 (고도 ≤ 0.2 m 또는 `landed_state` 값 확인)

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 착륙 명령 전송 | LAND 상태 진입 후 1 s 이내 |
| 하강 시작 | 명령 후 5초 이내 |
| 지면 도달 | 전복/추락 없이 접지 |
| 착륙 후 상태 | arming_state 변화 또는 고도 안정 |

---

### TC-007. 간단 통합 미션 테스트

**목적:** 전체 임무 흐름이 수동 개입 없이 완료되는지 확인
**관련 테스트 파일:** `test/integration/test_tc007_full_mission.py`
**관련 노드:** 전체 (WaypointNav, YoloDetect, ArucoDetect, PrecisionTask)

**사전 조건:** TC-001 ~ TC-006 전부 합격

**시나리오:**

```
[START]
  1. 기체 스폰 (Gazebo)
  2. ROS2 노드 전체 기동 (sim_launch.py)
  3. 자동 이륙 (고도 5 m)
  4. WP1 이동 (북쪽 5 m)
  5. 원점 복귀 (0, 0, -5)
  6. 착륙
[END]
```

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 전체 시나리오 완료 | 수동 개입 없이 DONE 상태 도달 |
| 노드 생존성 | 미션 내내 전체 노드 alive (`ros2 node list` 확인) |
| setpoint 주파수 | `trajectory_setpoint` ≥ 5 Hz |
| `offboard_control_mode` 주파수 | ≥ 5 Hz |
| 충돌/추락 | 없음 |

---

## 6. 성공 기준 요약

| 레이어 | 테스트 수 | 현재 상태 | 목표 |
|--------|-----------|-----------|------|
| contract | 8개 | ✅ PASS | PASS 유지 |
| unit waypoint_nav | 22개 | ✅ PASS | PASS 유지 |
| unit vision_aruco | 5개 | ⏸ skip | 비전 B 구현 후 PASS |
| unit vision_yolo | 5개 | ⏸ skip | 비전 A 구현 후 PASS |
| unit control_task | 7개 | ⏸ skip | 제어 B 구현 후 PASS |
| unit hw_gripper | 5개 | ⏸ skip | HW A 구현 후 PASS |
| unit comm_lte | 6개 | ⏸ skip | 통신 B 구현 후 PASS |
| integration TC-003 | 4개 | ⏸ skip | SIM_RUNNING=1 시 PASS |
| integration TC-007 | 3개 | ⏸ skip | SIM_RUNNING=1 시 PASS |

---

## 7. 이후 확장 계획

| Phase | 항목 | 연관 노드 |
|-------|------|-----------|
| **1 (현재)** | 기본 비행 루프, 노드 구조 확립 | WaypointNavNode |
| **2** | 카메라 센서 활성화, 객체 검출 토픽 연결 | YoloDetectNode, ArucoDetectNode |
| **3** | Gripper 메커니즘, 물체 집기/내려놓기 | ArduinoCmdNode, PrecisionTaskNode |
| **4** | 임무 전체 통합 (이륙 → 탐지 → 집기 → 드롭 → 착륙) | 전체 |
| **5** | 장애물 환경, 센서 노이즈, 실제 기체 이전 검증 | 실기체 + real_launch.py |

---

## 8. 결과 기록 형식

```text
[Test ID]    TC-004
[Test Name]  단일 waypoint 이동 테스트
[Date]       2026-03-22
[Env]        Gazebo Classic + PX4 SITL standard_vtol + ROS2 Humble
[Tester]     홍길동
[Result]     PASS / FAIL
[Issues]
  - 예: waypoint 도달 전 yaw 흔들림 발생 (진폭 ±15°)
  - 예: 도달 판정 반경 1.0 m 달성하는 데 18초 소요
[Notes]
  - 예: MPC_XY_P 게인 0.95 → 1.1 로 조정 후 개선
  - 예: 재검증 예정 (TC-004-retest)
```

---

## 관련 문서

| 문서 | 내용 |
|------|------|
| [simulation.md](simulation.md) | Docker Compose 시뮬 실행 방법 |
| [architecture.md](architecture.md) | 패키지 구조 및 토픽 목록 |
| [docker.md](docker.md) | Docker 서비스 구성표 및 명령어 |
| [test/README.md](../test/README.md) | 테스트 실행 방법 (빠른 참조) |
