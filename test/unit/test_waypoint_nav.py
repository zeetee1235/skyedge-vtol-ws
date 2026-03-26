"""
unit › vtol › WaypointNavNode

관련 명세: TC-003 자동 이륙 / TC-004~005 웨이포인트 이동 / TC-006 착륙
상태: 구현 완료 → 전체 PASS 필요
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# ── mock 주입 (노드 import 전) ────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install, _VehicleStatus, _VehicleCommand
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol' / 'src'))
from waypoint_nav_node import MissionPlanner, TrajectoryPlanner, WaypointNavNode


# ── 헬퍼 ─────────────────────────────────────────────────────────
def _make_pos(x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0):
    p = MagicMock()
    p.x, p.y, p.z = x, y, z
    p.vx, p.vy, p.vz = vx, vy, vz
    return p


def _arm_publish_count(node):
    """vehicle_command 퍼블리셔에서 ARM 명령 발행 횟수를 반환."""
    pub_cmd = node._publishers['/drone1/fmu/in/vehicle_command']
    return sum(
        1 for c in pub_cmd.publish.call_args_list
        if hasattr(c[0][0], 'command')
        and c[0][0].command == _VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    )


class TestInit(unittest.TestCase):
    """초기화: 토픽 연결 검증"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_initial_state_is_idle(self):
        self.assertEqual(self.node._state, WaypointNavNode._IDLE)

    def test_publishes_offboard_control_mode(self):
        self.assertIn('/drone1/fmu/in/offboard_control_mode', self.node._publishers)

    def test_publishes_trajectory_setpoint(self):
        self.assertIn('/drone1/fmu/in/trajectory_setpoint', self.node._publishers)

    def test_publishes_vehicle_command(self):
        self.assertIn('/drone1/fmu/in/vehicle_command', self.node._publishers)

    def test_subscribes_vehicle_status(self):
        subs = self.node._subscriptions
        self.assertTrue(
            '/drone1/fmu/out/vehicle_status' in subs or
            '/drone1/fmu/out/vehicle_status_v1' in subs,
        )

    def test_subscribes_vehicle_local_position(self):
        subs = self.node._subscriptions
        self.assertTrue(
            '/drone1/fmu/out/vehicle_local_position' in subs or
            '/drone1/fmu/out/vehicle_local_position_v1' in subs,
        )

    def test_takeoff_z_is_negative_ned(self):
        self.assertLess(self.node._takeoff_z, 0)

    def test_waypoints_loaded(self):
        self.assertGreaterEqual(len(self.node._waypoints), 1)
        # NED 좌표계에서 고도는 음수
        for _, _, z in self.node._waypoints:
            self.assertLess(z, 0.0, 'NED z는 음수여야 합니다 (고도 방향)')


class TestWaypointParsing(unittest.TestCase):
    """파라미터 waypoint 파싱 로직"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_parse_waypoints_filters_invalid_entries(self):
        raw = [[1, 2, 3], [4, 5], ['a', 1, 2], [7.1, 8.2, 9.3]]
        parsed = self.node._parse_waypoints(raw)
        self.assertEqual(parsed, [(1.0, 2.0, 3.0), (7.1, 8.2, 9.3)])

    def test_parse_waypoints_handles_string_format(self):
        """ROS2 파라미터 제약 우회용 "x,y,z" 문자열 형식 파싱"""
        raw = ['1.0,2.0,3.0', '4.5,5.5,6.5']
        parsed = self.node._parse_waypoints(raw)
        self.assertEqual(parsed, [(1.0, 2.0, 3.0), (4.5, 5.5, 6.5)])

    def test_parse_waypoints_filters_invalid_string(self):
        """잘못된 문자열 형식은 필터링"""
        raw = ['1.0,2.0', 'a,b,c', '7.0,8.0,9.0']
        parsed = self.node._parse_waypoints(raw)
        self.assertEqual(parsed, [(7.0, 8.0, 9.0)])

    def test_parse_waypoints_mixed_string_and_array(self):
        """문자열 형식과 배열 형식 혼합 파싱"""
        raw = ['1.0,2.0,3.0', [4.0, 5.0, 6.0]]
        parsed = self.node._parse_waypoints(raw)
        self.assertEqual(parsed, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])

    def test_gps_conversion_sets_first_waypoint_origin(self):
        gps = [(37.0, 127.0, 30.0), (37.00001, 127.0, 30.0)]
        local = self.node._gps_waypoints_to_local_ned(gps)
        self.assertAlmostEqual(local[0][0], 0.0, places=6)
        self.assertAlmostEqual(local[0][1], 0.0, places=6)
        self.assertLess(local[0][2], 0.0)
        self.assertGreater(local[1][0], 0.0)


class TestMissionPlanner(unittest.TestCase):
    def test_invalid_gps_filtered(self):
        planner = MissionPlanner(cruise_z=-30.0)
        mission, gps_error = planner.build_mission(
            frame='gps',
            raw_waypoints=[[999.0, 127.0, 30.0], [37.0, 127.0, 30.0]],
        )
        self.assertTrue(gps_error)
        self.assertEqual(len(mission), 1)

    def test_empty_waypoints_returns_default(self):
        planner = MissionPlanner(cruise_z=-30.0)
        mission, gps_error = planner.build_mission(frame='local_ned', raw_waypoints=[])
        self.assertEqual(mission, [(0.0, 0.0, -30.0)])
        self.assertFalse(gps_error)

    def test_local_ned_z_forced_negative(self):
        """local_ned 프레임에서 양수 고도값도 NED 음수로 변환"""
        planner = MissionPlanner(cruise_z=-30.0)
        mission, _ = planner.build_mission(
            frame='local_ned',
            raw_waypoints=[[10.0, 20.0, 30.0]],
        )
        self.assertLess(mission[0][2], 0.0)


class TestTrajectoryPlanner(unittest.TestCase):
    def test_point_jump_returns_target(self):
        planner = TrajectoryPlanner(mode='point_jump', max_step_m=1.0)
        sp = planner.next_setpoint((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
        self.assertEqual(sp, (10.0, 0.0, 0.0))

    def test_linear_limits_step(self):
        planner = TrajectoryPlanner(mode='linear', max_step_m=1.0)
        sp = planner.next_setpoint((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
        self.assertAlmostEqual(sp[0], 1.0, places=6)

    def test_mpc_moves_toward_target_bounded(self):
        # 실제 MPC는 부드러운 가속 프로파일을 따르므로 첫 스텝은 max_step_m보다
        # 작을 수 있다. 핵심 보장 사항: (1) 목표 방향으로 이동, (2) 상한 준수.
        planner = TrajectoryPlanner(mode='mpc', max_step_m=1.0)
        sp = planner.next_setpoint((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
        self.assertGreater(sp[0], 0.0)               # 목표 방향으로 이동
        self.assertLessEqual(sp[0], 1.0 + 1e-9)      # max_step_m 상한 준수
        self.assertAlmostEqual(sp[1], 0.0, places=6) # y축 변화 없음
        self.assertAlmostEqual(sp[2], 0.0, places=6) # z축 변화 없음


class TestReached(unittest.TestCase):
    """도달 판정 로직"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_true_when_at_target(self):
        self.node._local_pos = _make_pos(0.0, 0.0, -5.0)
        self.assertTrue(self.node._reached(0.0, 0.0, -5.0, thr=0.5))

    def test_false_when_far(self):
        self.node._local_pos = _make_pos(100.0, 100.0, 0.0)
        self.assertFalse(self.node._reached(0.0, 0.0, -5.0, thr=2.0))

    def test_boundary_exactly_at_threshold(self):
        self.node._local_pos = _make_pos(2.0, 0.0, 0.0)
        # 거리 == threshold → False (strictly less)
        self.assertFalse(self.node._reached(0.0, 0.0, 0.0, thr=2.0))


class TestStateMachine(unittest.TestCase):
    """상태 머신 전이 검증 (TC-003 / TC-004 / TC-006)"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_state_handler_map_has_core_states(self):
        self.assertIn(WaypointNavNode._IDLE, self.node._state_handlers)
        self.assertIn(WaypointNavNode._NAVIGATE, self.node._state_handlers)
        self.assertIn(WaypointNavNode._LAND, self.node._state_handlers)

    def test_stays_idle_before_10_cycles(self):
        for _ in range(9):
            self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._IDLE)

    def test_transitions_to_arming_after_10_cycles(self):
        for _ in range(10):
            self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._ARMING)

    def test_transitions_to_takeoff_when_armed(self):
        self.node._state = WaypointNavNode._ARMING
        self.node._status = MagicMock()
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_ARMED
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._TAKEOFF)

    def test_arming_retry_sends_arm_command(self):
        """ARMING 상태에서 20사이클마다 ARM 명령을 재전송"""
        self.node._state = WaypointNavNode._ARMING
        self.node._arming_retry_cnt = 19  # 다음 +1로 20이 되어 재시도 발동
        arm_before = _arm_publish_count(self.node)
        self.node._control_loop()
        self.assertGreater(_arm_publish_count(self.node), arm_before)

    def test_arming_no_retry_before_20_cycles(self):
        """ARMING 상태에서 20사이클 미만에서는 ARM 명령 재전송 없음"""
        self.node._state = WaypointNavNode._ARMING
        self.node._arming_retry_cnt = 0
        arm_before = _arm_publish_count(self.node)
        # 19사이클 실행: 마지막 사이클이 19가 되어 20 미만 → 재시도 없음
        for _ in range(19):
            self.node._arming_retry_cnt += 1
        # 직접 핸들러 1회 호출 (20번째 실행 직전 상태 확인)
        self.node._arming_retry_cnt = 18
        self.node._control_loop()  # → 19, 재시도 없음
        self.assertEqual(_arm_publish_count(self.node), arm_before)

    def test_takeoff_goes_to_transition_to_fw(self):
        self.node._state = WaypointNavNode._TAKEOFF
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._TRANSITION_TO_FW)

    def test_takeoff_timeout_goes_to_transition_to_fw(self):
        """이륙 타임아웃 시 고도 미달이어도 TRANSITION_TO_FW로 강제 전환"""
        self.node._state = WaypointNavNode._TAKEOFF
        self.node._local_pos = _make_pos(0.0, 0.0, 0.0)  # 이륙 고도 미달
        self.node._takeoff_hold_cnt = self.node._takeoff_timeout_cycles
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._TRANSITION_TO_FW)

    def test_transition_to_fw_goes_to_navigate_on_timeout(self):
        self.node._state = WaypointNavNode._TRANSITION_TO_FW
        self.node._transition_wait_cnt = self.node._transition_timeout_cycles
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._NAVIGATE)

    def test_transitions_to_mc_after_last_waypoint(self):
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._wp_idx = len(self.node._waypoints) - 1
        self.node._local_pos = _make_pos(*self.node._waypoints[-1])
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._TRANSITION_TO_MC)

    def test_transition_to_mc_goes_to_land_on_timeout(self):
        self.node._state = WaypointNavNode._TRANSITION_TO_MC
        self.node._transition_wait_cnt = self.node._transition_timeout_cycles
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._LAND)

    def test_land_goes_to_landing_confirm(self):
        self.node._state = WaypointNavNode._LAND
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._LANDING_CONFIRM)

    def test_landing_confirm_goes_to_done_when_disarmed(self):
        self.node._state = WaypointNavNode._LANDING_CONFIRM
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_STANDBY
        for _ in range(self.node._landing_confirm_required_cycles):
            self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._DONE)

    def test_comm_loss_enters_failsafe(self):
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._seen_status = True
        self.node._seen_local_pos = True
        self.node._last_status_loop = 0
        self.node._last_local_pos_loop = 0
        self.node._loop_cnt = self.node._comm_loss_timeout_cycles + 1
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._FAILSAFE_LAND)

    def test_single_topic_loss_does_not_enter_failsafe(self):
        """status만 끊겨도 local_pos가 살아있으면 failsafe 진입하지 않음"""
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._seen_status = True
        self.node._seen_local_pos = True
        self.node._last_status_loop = 0
        # local_pos는 최근 수신
        self.node._last_local_pos_loop = self.node._comm_loss_timeout_cycles + 2
        self.node._loop_cnt = self.node._comm_loss_timeout_cycles + 1
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
        self.node._control_loop()
        self.assertNotEqual(self.node._state, WaypointNavNode._FAILSAFE_LAND)

    def test_waypoint_unreachable_skips_waypoint(self):
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._wp_idx = 0
        self.node._wp_progress_cnt = self.node._wp_unreachable_timeout_cycles
        self.node._local_pos = _make_pos(999.0, 999.0, -999.0)
        self.node._control_loop()
        self.assertGreaterEqual(self.node._wp_idx, 1)


class TestLandingCondition(unittest.TestCase):
    def setUp(self):
        self.node = WaypointNavNode()

    def test_is_landed_by_low_alt_and_speed(self):
        self.node._use_kinematic_landing_fallback = True
        self.node._status.arming_state = 999
        self.node._local_pos = _make_pos(0.0, 0.0, -0.1, vx=0.1, vy=0.1, vz=0.1)
        self.assertTrue(self.node._is_landed())

    def test_not_landed_when_fast(self):
        self.node._use_kinematic_landing_fallback = True
        self.node._status.arming_state = 999
        self.node._local_pos = _make_pos(0.0, 0.0, -0.1, vx=2.0, vy=0.0, vz=0.0)
        self.assertFalse(self.node._is_landed())

    def test_not_landed_on_low_hover_by_default(self):
        self.node._status.arming_state = 999
        self.node._local_pos = _make_pos(0.0, 0.0, -0.1, vx=0.0, vy=0.0, vz=0.0)
        self.assertFalse(self.node._is_landed())

    def test_is_landed_when_standby(self):
        """ARMING_STATE_STANDBY(disarm) 상태도 착지 판정"""
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_STANDBY
        self.node._local_pos = _make_pos(0.0, 0.0, 0.0)
        self.assertTrue(self.node._is_landed())


class TestSITLBehavior(unittest.TestCase):
    """SITL 환경 전용 동작 검증"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_force_arm_param2_in_sim(self):
        """시뮬레이션 모드에서 ARM 명령 param2 = 21196.0 (강제 ARM)"""
        self.node._use_sim = True
        self.node._cmd_arm()
        pub_cmd = self.node._publishers['/drone1/fmu/in/vehicle_command']
        msg = pub_cmd.publish.call_args[0][0]
        self.assertEqual(msg.param2, 21196.0)

    def test_no_force_arm_without_sim(self):
        """비 시뮬레이션 모드에서 ARM 명령 param2 = 0.0"""
        self.node._use_sim = False
        self.node._cmd_arm()
        pub_cmd = self.node._publishers['/drone1/fmu/in/vehicle_command']
        msg = pub_cmd.publish.call_args[0][0]
        self.assertEqual(msg.param2, 0.0)

    def test_arm_keepalive_fires_when_disarmed_in_navigate(self):
        """SITL: NAVIGATE 중 disarm 시 20사이클마다 ARM keepalive 발동"""
        self.node._use_sim = True
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_STANDBY  # not armed
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
        self.node._arm_keepalive_cnt = 19  # 다음 +1로 20이 되어 keepalive 발동
        arm_before = _arm_publish_count(self.node)
        self.node._control_loop()
        self.assertGreater(_arm_publish_count(self.node), arm_before)

    def test_arm_keepalive_skipped_in_idle(self):
        """IDLE 상태에서는 ARM keepalive 발동 안 함 (pre_arm 로직과 분리)"""
        self.node._use_sim = True
        self.node._state = WaypointNavNode._IDLE
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_STANDBY
        self.node._arm_keepalive_cnt = 19
        # IDLE에서 keepalive 제외 조건 검증: _control_loop의 keepalive 블록이 스킵됨
        # _handle_idle이 ARM 보내기 전(pre_arm_cnt < 10) 확인
        arm_before = _arm_publish_count(self.node)
        self.node._control_loop()  # pre_arm_cnt = 1, ARM 아직 안 보냄
        self.assertEqual(_arm_publish_count(self.node), arm_before)

    def test_arm_keepalive_skipped_when_already_armed(self):
        """이미 armed 상태에서는 keepalive ARM 명령 발동 안 함"""
        self.node._use_sim = True
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_ARMED
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
        self.node._arm_keepalive_cnt = 19
        arm_before = _arm_publish_count(self.node)
        self.node._control_loop()
        self.assertEqual(_arm_publish_count(self.node), arm_before)


class TestVTOLCommands(unittest.TestCase):
    """VTOL 전환 명령 검증"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_vtol_to_fw_callable(self):
        self.assertTrue(callable(getattr(self.node, '_cmd_vtol_to_fw', None)))

    def test_vtol_to_mc_callable(self):
        self.assertTrue(callable(getattr(self.node, '_cmd_vtol_to_mc', None)))

    def test_vtol_to_fw_sends_param1_4(self):
        """고정익 전환 명령: param1 = 4.0"""
        self.node._cmd_vtol_to_fw()
        pub_cmd = self.node._publishers['/drone1/fmu/in/vehicle_command']
        msg = pub_cmd.publish.call_args[0][0]
        self.assertEqual(msg.command, _VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION)
        self.assertEqual(msg.param1, 4.0)

    def test_vtol_to_mc_sends_param1_3(self):
        """멀티콥터 전환 명령: param1 = 3.0"""
        self.node._cmd_vtol_to_mc()
        pub_cmd = self.node._publishers['/drone1/fmu/in/vehicle_command']
        msg = pub_cmd.publish.call_args[0][0]
        self.assertEqual(msg.command, _VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION)
        self.assertEqual(msg.param1, 3.0)


if __name__ == '__main__':
    unittest.main()
