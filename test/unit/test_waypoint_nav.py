"""
unit › vtol_control_nav › WaypointNavNode

관련 명세: TC-003 자동 이륙 / TC-004~005 웨이포인트 이동 / TC-006 착륙
상태: 구현 완료 → 전체 PASS 필요
"""
import sys
import math
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# ── mock 주입 (노드 import 전) ────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install, _VehicleStatus
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_control_nav' / 'src'))
from waypoint_nav_node import WaypointNavNode


# ── 헬퍼 ─────────────────────────────────────────────────────────
def _make_pos(x=0.0, y=0.0, z=0.0):
    p = MagicMock()
    p.x, p.y, p.z = x, y, z
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

    def test_cruise_z_is_negative_ned(self):
        self.assertLess(self.node._cruise_z, 0)

    def test_has_at_least_two_waypoints(self):
        self.assertGreaterEqual(len(self.node._waypoints), 2)


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

    def test_stays_arming_when_not_armed(self):
        self.node._state = WaypointNavNode._ARMING
        self.node._status = MagicMock()
        self.node._status.arming_state = _VehicleStatus.ARMING_STATE_STANDBY
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._ARMING)

    def test_transitions_to_navigate_at_takeoff_altitude(self):
        self.node._state = WaypointNavNode._TAKEOFF
        self.node._local_pos = _make_pos(0.0, 0.0, self.node._takeoff_z)
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._NAVIGATE)

    def test_advances_waypoint_when_reached(self):
        self.node._state = WaypointNavNode._NAVIGATE
        wp = self.node._waypoints[0]
        self.node._local_pos = _make_pos(*wp)
        self.node._control_loop()
        self.assertEqual(self.node._wp_idx, 1)

    def test_transitions_to_land_after_last_waypoint(self):
        self.node._state = WaypointNavNode._NAVIGATE
        self.node._wp_idx = len(self.node._waypoints) - 1
        self.node._local_pos = _make_pos(*self.node._waypoints[-1])
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._LAND)

    def test_transitions_to_done_from_land(self):
        self.node._state = WaypointNavNode._LAND
        self.node._control_loop()
        self.assertEqual(self.node._state, WaypointNavNode._DONE)


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
