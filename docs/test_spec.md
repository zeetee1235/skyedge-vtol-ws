# 테스트 명세서 v0.2.0

> VTOL 드론 프로젝트 — Gazebo 시뮬레이션 기반 초기 검증 기준 문서

---

## 핵심 원칙

1. 기본 브랜치(`main`, `develop`)는 항상 초록 상태를 유지합니다.
2. 아직 구현하지 않은 기능 테스트는 `@unittest.skip`으로 처리합니다.
3. 담당자가 구현을 시작하면 `skip`을 제거하고 구현과 함께 PASS로 만듭니다.

```bash
python3 -m unittest discover -s test -v
# 기대: 76개 실행, 17개 skip, 빨간색 없음
# contract PASS + waypoint_nav(59개) PASS + vision skip
```

---

## 1. 목적 및 범위

**목적:** 실기체 투입 전 Gazebo + PX4 SITL + ROS2 환경에서 기본 비행·통신·인터페이스 검증

**포함 범위:**

| 항목 | 설명 |
|------|------|
| Gazebo 기체 스폰 | PX4 SITL standard_vtol 모델 |
| ROS2 노드 통신 | uXRCE-DDS 브리지를 통한 PX4 ↔ ROS2 토픽 |
| 자동 이륙 | Offboard + TrajectorySetpoint |
| Waypoint 이동 | NED 좌표 기반 복수 웨이포인트 |
| 자동 착륙 | VTOL MC 전환 → VehicleCommand NAV_LAND |
| 단위 테스트 | mock_ros2 기반, ROS2 설치 불필요 |

**제외 범위 (현 단계):** 정밀 객체 인식, 그리퍼 실기체 제어, 강풍/센서 노이즈 환경

---

## 2. 테스트 환경

| 구성 요소 | 버전/종류 |
|-----------|-----------|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble |
| PX4 SITL | `jonasvautherin/px4-gazebo-headless` |
| 기체 모델 | standard_vtol (Gazebo Classic) |
| DDS 브리지 | Micro XRCE-DDS Agent (UDP 8888) |
| 토픽 네임스페이스 | `/drone1` (`PX4_UXRCE_DDS_NS=drone1`) |
| 테스트 프레임워크 | Python unittest + mock_ros2 |

### PX4 토픽 인터페이스

**제어 입력 (`/drone1/fmu/in/`)**

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `offboard_control_mode` | `OffboardControlMode` | Offboard 모드 유지 (10 Hz 이상) |
| `trajectory_setpoint` | `TrajectorySetpoint` | 위치 목표 전송 (NED, m) |
| `vehicle_command` | `VehicleCommand` | ARM / 모드 전환 / 착륙 명령 |

**상태 출력 (`/drone1/fmu/out/`)**

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `vehicle_status_v1` | `VehicleStatus` | arming_state, nav_state |
| `vehicle_local_position_v1` | `VehicleLocalPosition` | NED 위치 (x, y, z) |
| `vehicle_land_detected` | `VehicleLandDetected` | 착륙 감지 |

**노드 간 토픽**

| 토픽 | 발행자 | 용도 |
|------|--------|------|
| `/vtol/yolo/detections` | `yolo_detect` | YOLO 탐지 결과 |
| `/vtol/aruco/pose` | `aruco_detect` | ArUco 마커 위치 |

---

## 3. 테스트 레이어 구조

```
test/
  mock_ros2.py                ROS2·px4_msgs mock 헬퍼
  contract/
    test_spec_contract.py     명세 문서 + 핵심 파일 구조 검증
    test_node_entrypoints.py  노드 main() 진입점 형식 검증
  unit/
    test_waypoint_nav.py      ✅ 구현 완료
    test_vision_aruco.py      ⏸ skip → vtol_vision 구현 시 활성화
    test_vision_yolo.py       ⏸ skip → vtol_vision 구현 시 활성화
  integration/
    test_tc003_takeoff.py     TC-003 (SIM_RUNNING=1)
    test_tc007_full_mission.py TC-007 (SIM_RUNNING=1)
```

| 레이어 | 실행 조건 | 속도 | 의존성 |
|--------|-----------|------|--------|
| contract | 항상 | < 1 s | 파일 시스템만 |
| unit | 항상 | < 5 s | Python + mock_ros2 |
| integration | `SIM_RUNNING=1` | 수십 초 | 실행 중인 시뮬 |

---

## 4. 노드별 단위 테스트 명세

### 4-1. WaypointNavNode (vtol) ✅ 구현 완료

**파일:** `test/unit/test_waypoint_nav.py` | **관련 TC:** TC-003, TC-004, TC-005, TC-006

| 클래스 | 메서드 | 검증 내용 |
|--------|--------|-----------|
| TestInit | `test_initial_state_is_idle` | 초기 상태 = IDLE |
| TestInit | `test_publishes_offboard_control_mode` | offboard_control_mode 퍼블리셔 존재 |
| TestInit | `test_publishes_trajectory_setpoint` | trajectory_setpoint 퍼블리셔 존재 |
| TestInit | `test_publishes_vehicle_command` | vehicle_command 퍼블리셔 존재 |
| TestInit | `test_subscribes_vehicle_status` | vehicle_status_v1 구독 존재 |
| TestInit | `test_subscribes_vehicle_local_position` | vehicle_local_position_v1 구독 존재 |
| TestInit | `test_takeoff_z_is_negative_ned` | `_takeoff_z < 0` (NED 상향 = 음수) |
| TestInit | `test_waypoints_loaded` | 웨이포인트 1개 이상 로드, 전부 NED z < 0 |
| TestWaypointParsing | `test_parse_waypoints_filters_invalid_entries` | 잘못된 배열 항목 필터링 |
| TestWaypointParsing | `test_parse_waypoints_handles_string_format` | `"x,y,z"` 문자열 형식 파싱 |
| TestWaypointParsing | `test_parse_waypoints_filters_invalid_string` | 잘못된 문자열 필터링 |
| TestWaypointParsing | `test_parse_waypoints_mixed_string_and_array` | 문자열·배열 혼합 파싱 |
| TestWaypointParsing | `test_gps_conversion_sets_first_waypoint_origin` | GPS → NED 변환, 첫 점 = 원점 |
| TestMissionPlanner | `test_invalid_gps_filtered` | 범위 초과 GPS 필터링 + gps_error 플래그 |
| TestMissionPlanner | `test_empty_waypoints_returns_default` | 빈 웨이포인트 → 기본 목표 반환 |
| TestMissionPlanner | `test_local_ned_z_forced_negative` | local_ned 프레임 z 음수 강제 변환 |
| TestTrajectoryPlanner | `test_point_jump_returns_target` | point_jump: 즉시 목표 반환 |
| TestTrajectoryPlanner | `test_linear_limits_step` | linear: 스텝 상한 준수 |
| TestTrajectoryPlanner | `test_mpc_moves_toward_target_bounded` | mpc: 목표 방향 이동 + 상한 준수 |
| TestReached | `test_true_when_at_target` | 목표 도달 시 True |
| TestReached | `test_false_when_far` | 멀리 있을 때 False |
| TestReached | `test_boundary_exactly_at_threshold` | 거리 == threshold → False (strictly less) |
| TestStateMachine | `test_state_handler_map_has_core_states` | 핵심 상태 핸들러 등록 확인 |
| TestStateMachine | `test_stays_idle_before_10_cycles` | 10회 미만 → IDLE 유지 |
| TestStateMachine | `test_transitions_to_arming_after_10_cycles` | 10회 후 → ARMING |
| TestStateMachine | `test_transitions_to_takeoff_when_armed` | ARMED → TAKEOFF |
| TestStateMachine | `test_arming_retry_sends_arm_command` | ARMING 20사이클마다 ARM 명령 재전송 |
| TestStateMachine | `test_arming_no_retry_before_20_cycles` | 20사이클 미만에서는 재전송 없음 |
| TestStateMachine | `test_takeoff_goes_to_transition_to_fw` | 이륙 고도 도달 → TRANSITION_TO_FW |
| TestStateMachine | `test_takeoff_timeout_goes_to_transition_to_fw` | 이륙 타임아웃 → TRANSITION_TO_FW 강제 전환 |
| TestStateMachine | `test_transition_to_fw_goes_to_navigate_on_timeout` | FW 전환 타임아웃 → NAVIGATE |
| TestStateMachine | `test_transitions_to_mc_after_last_waypoint` | 마지막 WP 도달 → TRANSITION_TO_MC |
| TestStateMachine | `test_transition_to_mc_goes_to_land_on_timeout` | MC 전환 타임아웃 → LAND |
| TestStateMachine | `test_land_goes_to_landing_confirm` | LAND → LANDING_CONFIRM |
| TestStateMachine | `test_landing_confirm_goes_to_done_when_disarmed` | LANDING_CONFIRM → DONE |
| TestStateMachine | `test_comm_loss_enters_failsafe` | status + local_pos 모두 두절 → FAILSAFE_LAND |
| TestStateMachine | `test_single_topic_loss_does_not_enter_failsafe` | 단일 토픽 끊김은 failsafe 미진입 |
| TestStateMachine | `test_waypoint_unreachable_skips_waypoint` | WP 타임아웃 → 다음 WP 스킵 |
| TestLandingCondition | `test_is_landed_by_low_alt_and_speed` | 저고도 + 저속 → 착지 판정 |
| TestLandingCondition | `test_not_landed_when_fast` | 고속 비행 중 → 착지 미판정 |
| TestLandingCondition | `test_not_landed_on_low_hover_by_default` | kinematic fallback 비활성 시 미판정 |
| TestLandingCondition | `test_is_landed_when_standby` | ARMING_STATE_STANDBY(disarm) → 착지 판정 |
| TestSITLBehavior | `test_force_arm_param2_in_sim` | 시뮬 모드: ARM param2 = 21196.0 (강제 ARM) |
| TestSITLBehavior | `test_no_force_arm_without_sim` | 비시뮬 모드: ARM param2 = 0.0 |
| TestSITLBehavior | `test_arm_keepalive_fires_when_disarmed_in_navigate` | SITL: NAVIGATE 중 비armed 시 keepalive 발동 |
| TestSITLBehavior | `test_arm_keepalive_skipped_in_idle` | IDLE 상태에서는 keepalive 스킵 |
| TestSITLBehavior | `test_arm_keepalive_skipped_when_already_armed` | 이미 armed 시 keepalive 스킵 |
| TestVTOLCommands | `test_vtol_to_fw_callable` | `_cmd_vtol_to_fw()` 존재 |
| TestVTOLCommands | `test_vtol_to_mc_callable` | `_cmd_vtol_to_mc()` 존재 |
| TestVTOLCommands | `test_vtol_to_fw_sends_param1_4` | FW 전환 명령 param1 = 4.0 |
| TestVTOLCommands | `test_vtol_to_mc_sends_param1_3` | MC 전환 명령 param1 = 3.0 |

---

### 4-2. ArucoDetectNode (vtol_vision) ⏸ 구현 전

**파일:** `test/unit/test_vision_aruco.py` | **구현 파일:** `src/vtol_vision/src/aruco_detect_node.py`

**구현 체크리스트:**

- [ ] `/drone1/camera/image_raw` (`sensor_msgs/Image`) 구독
- [ ] `cv2.aruco`로 마커 검출 (`DICT_4X4_50` 권장)
- [ ] `/vtol/aruco/pose` (`geometry_msgs/PoseStamped`) 퍼블리시
- [ ] `vtol.aruco_marker_id` 파라미터 선언 (기본값 0)

| 메서드 | 검증 내용 |
|--------|-----------|
| `test_subscribes_to_camera_image` | 카메라 토픽 구독 존재 |
| `test_publishes_aruco_pose` | pose 퍼블리셔 존재 |
| `test_declares_marker_id_parameter` | marker_id 파라미터 존재 |
| `test_image_callback_exists` | 콜백 callable |
| `test_pose_published_when_marker_detected` | 마커 탐지 시 publish 호출 |

---

### 4-3. YoloDetectNode (vtol_vision) ⏸ 구현 전

**파일:** `test/unit/test_vision_yolo.py` | **구현 파일:** `src/vtol_vision/src/yolo_detect_node.py`

**구현 체크리스트:**

- [ ] `/drone1/camera/image_raw` 구독
- [ ] `ultralytics` YOLO 모델 로드
- [ ] `vtol.yolo_confidence_threshold` 파라미터로 결과 필터링 (기본값 0.5)
- [ ] `/vtol/yolo/detections` (`vision_msgs/Detection2DArray`) 퍼블리시

| 메서드 | 검증 내용 |
|--------|-----------|
| `test_subscribes_to_camera_image` | 카메라 토픽 구독 존재 |
| `test_publishes_yolo_detections` | detections 퍼블리셔 존재 |
| `test_declares_confidence_threshold_parameter` | confidence 파라미터 존재 |
| `test_image_callback_is_callable` | 콜백 callable |
| `test_detections_published_on_image_received` | 이미지 수신 시 publish 호출 |

---

## 5. 통합 테스트 케이스 (TC)

> 통합 테스트는 `SIM_RUNNING=1` 환경에서만 실행됩니다.
> 시뮬 실행 방법 → [simulation.md](simulation.md)

---

### TC-001. 시뮬레이션 실행 확인

**목적:** PX4 SITL, ROS2 노드가 정상 기동되는지 확인

**합격 기준:**
- `/drone1/fmu` 접두사 토픽이 ROS2에서 보임
- `waypoint_nav` 노드가 `ros2 node list`에 출력됨
- 치명적 에러 없이 30초 이상 유지됨

---

### TC-002. 상태 수신 확인

**목적:** ROS2에서 기체의 위치/상태를 읽을 수 있는지 확인

| 토픽 | 조건 |
|------|------|
| `vehicle_status_v1` | `ros2 topic hz` ≥ 1 Hz |
| `vehicle_local_position_v1` | `ros2 topic hz` ≥ 10 Hz |

---

### TC-003. 자동 이륙 테스트

**목적:** Offboard 모드에서 지정 고도까지 자동 이륙

**합격 기준:**

| 항목 | 기준 |
|------|------|
| 이륙 시작 | ARM 후 10초 이내 |
| 목표 고도 도달 | 10.0 m ± 1.0 m |
| setpoint 주파수 | ≥ 10 Hz |

---

### TC-004. 단일 Waypoint 이동

**목적:** 이륙 후 단일 목표 지점으로 이동

**합격 기준:** 목표 반경 2.0 m 이내 진입 후 3초 유지

---

### TC-005. 복수 Waypoint 이동

**목적:** 복수 웨이포인트를 순서대로 방문

**합격 기준:** 모든 웨이포인트 순차 방문 후 TRANSITION_TO_MC 진입

---

### TC-006. 착륙 테스트

**목적:** MC 전환 후 자동 착륙 및 LANDING_CONFIRM

**합격 기준:** LAND 명령 후 2분 이내 LANDING_CONFIRM → DONE 전이

---

### TC-007. 전체 미션 통합 테스트

**목적:** 이륙 → 웨이포인트 순회 → 착륙 전체 흐름

**파일:** `test/integration/test_tc007_full_mission.py`

**합격 기준:**

| 항목 | 기준 |
|------|------|
| trajectory_setpoint 주파수 | ≥ 5 Hz |
| offboard_control_mode 주파수 | ≥ 5 Hz |
| waypoint_nav 노드 alive | `ros2 node list`에 존재 |

---

## 6. 성공 기준 요약

| 레이어 | 기준 |
|--------|------|
| contract | 전체 PASS 필수 (CI 차단) |
| unit waypoint_nav | 전체 PASS 필수 |
| unit vision (aruco/yolo) | 구현 완료 후 PASS |
| integration | `SIM_RUNNING=1` 환경에서 TC-001 ~ TC-007 PASS |
