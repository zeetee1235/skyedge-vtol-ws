"""
unit › vtol_control_task › PrecisionTaskNode

관련 명세: TC-006 착륙 / TC-007 통합 미션
담당자:   제어 B

구현 전 상태: skip — 아래 @unittest.skip 제거 후 구현 시작
구현 후 목표: 전체 PASS

구현 체크리스트:
  [ ] /vtol/aruco/pose 구독
  [ ] /vtol/yolo/detections 구독
  [ ] /drone1/fmu/in/trajectory_setpoint 퍼블리시 (정밀 하강 setpoint)
  [ ] /vtol/gripper/command 퍼블리시
  [ ] vtol.landing_descent_altitude 파라미터 사용
  [ ] 상태 머신: IDLE → DESCEND → HOVER → GRIP → DONE
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_SKIP_REASON = (
    "vtol_control_task 노드는 아직 구현 전입니다. "
    "초보자용 기본 브랜치를 항상 초록 상태로 유지하기 위해 현재는 skip 합니다. "
    "제어 B 구현을 시작할 때 이 skip 을 제거하고 테스트를 함께 녹색으로 바꾸세요."
)

sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_control_task' / 'src'))
from precision_task_node import PrecisionTaskNode


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestPrecisionTaskInit(unittest.TestCase):
    """초기화: 토픽 연결 검증"""

    def setUp(self):
        self.node = PrecisionTaskNode()

    def test_subscribes_to_aruco_pose(self):
        self.assertIn('/vtol/aruco/pose', self.node._subscriptions,
                      "ArUco pose 구독 없음")

    def test_subscribes_to_yolo_detections(self):
        self.assertIn('/vtol/yolo/detections', self.node._subscriptions,
                      "YOLO detections 구독 없음")

    def test_publishes_gripper_command(self):
        self.assertIn('/vtol/gripper/command', self.node._publishers,
                      "그리퍼 명령 퍼블리셔 없음")

    def test_publishes_trajectory_setpoint(self):
        self.assertIn('/drone1/fmu/in/trajectory_setpoint', self.node._publishers,
                      "정밀 착륙 setpoint 퍼블리셔 없음")

    def test_declares_descent_altitude_parameter(self):
        self.assertIn('vtol.landing_descent_altitude', self.node._parameters,
                      "landing_descent_altitude 파라미터 선언 없음")


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestPrecisionTaskStateMachine(unittest.TestCase):
    """상태 머신 기본 전이 검증"""

    def setUp(self):
        self.node = PrecisionTaskNode()

    def test_has_state_attribute(self):
        self.assertTrue(hasattr(self.node, '_state'),
                        "상태 머신 _state 속성 없음")

    def test_gripper_open_on_command(self):
        """그리퍼 열기 명령 퍼블리시 검증"""
        self.assertIn('/vtol/gripper/command', self.node._publishers)
        pub_mock = self.node._publishers['/vtol/gripper/command']
        # 노드에 open_gripper() 또는 동등한 메서드가 있어야 함
        open_method = getattr(self.node, 'open_gripper', None) or \
                      getattr(self.node, '_open_gripper', None) or \
                      getattr(self.node, '_cmd_gripper_open', None)
        self.assertIsNotNone(open_method,
                             "open_gripper 계열 메서드 없음")
        open_method()
        self.assertTrue(pub_mock.publish.called,
                        "open_gripper 호출 후 publish 미실행")


if __name__ == '__main__':
    unittest.main()
