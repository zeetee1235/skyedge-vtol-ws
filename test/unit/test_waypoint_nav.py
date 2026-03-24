"""
unit › vtol_control_nav › WaypointNavNode

관련 명세: TC-003 자동 이륙 / TC-004~005 웨이포인트 이동 / TC-006 착륙
상태: 구현 완료 → 전체 PASS 필요
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# ── mock 주입 (노드 import 전) ────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install, _VehicleStatus
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_control_nav' / 'src'))
from waypoint_nav_node import MissionPlanner, TrajectoryPlanner, WaypointNavNode


# ── 헬퍼 ─────────────────────────────────────────────────────────
def _make_pos(x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0):
    p = MagicMock()
    p.x, p.y, p.z = x, y, z
    p.vx, p.vy, p.vz = vx, vy, vz
    return p


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
        self.assertIn('/drone1/fmu/out/vehicle_status', self.node._subscriptions)

    def test_subscribes_vehicle_local_position(self):
        self.assertIn('/drone1/fmu/out/vehicle_local_position', self.node._subscriptions)

    def test_takeoff_z_is_negative_ned(self):
        self.assertLess(self.node._takeoff_z, 0)

    def test_waypoints_loaded(self):
        self.assertGreaterEqual(len(self.node._waypoints), 1)


class TestWaypointParsing(unittest.TestCase):
    """파라미터 waypoint 파싱 로직"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_parse_waypoints_filters_invalid_entries(self):
        raw = [[1, 2, 3], [4, 5], ['a', 1, 2], [7.1, 8.2, 9.3]]
        parsed = self.node._parse_waypoints(raw)
        self.assertEqual(parsed, [(1.0, 2.0, 3.0), (7.1, 8.2, 9.3)])

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

    def test_takeoff_goes_to_transition_to_fw(self):
        self.node._state = WaypointNavNode._TAKEOFF
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
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


class TestVTOLCommands(unittest.TestCase):
    """VTOL 전환 명령 메서드 존재 검증"""

    def setUp(self):
        self.node = WaypointNavNode()

    def test_vtol_to_fw_callable(self):
        self.assertTrue(callable(getattr(self.node, '_cmd_vtol_to_fw', None)))

    def test_vtol_to_mc_callable(self):
        self.assertTrue(callable(getattr(self.node, '_cmd_vtol_to_mc', None)))


if __name__ == '__main__':
    unittest.main()
