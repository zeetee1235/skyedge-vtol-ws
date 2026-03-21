"""
unit › vtol_hw_gripper › ArduinoCmdNode

관련 명세: TC-007 통합 미션
담당자:   HW A

구현 전 상태: skip — 아래 @unittest.skip 제거 후 구현 시작
구현 후 목표: 전체 PASS

구현 체크리스트:
  [ ] /vtol/gripper/command (std_msgs/Int32) 구독
  [ ] 구독 콜백에서 pyserial로 시리얼 명령 전송
  [ ] vtol.gripper_open_angle / vtol.gripper_close_angle 파라미터 적용
  [ ] 시리얼 포트가 없을 때 graceful 처리 (예외 → 로그)
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

_SKIP_REASON = (
    "vtol_hw_gripper 노드는 아직 구현 전입니다. "
    "초보자용 기본 브랜치를 항상 초록 상태로 유지하기 위해 현재는 skip 합니다. "
    "HW A 구현을 시작할 때 이 skip 을 제거하고 테스트를 함께 녹색으로 바꾸세요."
)

sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_hw_gripper' / 'src'))
from arduino_cmd_node import ArduinoCmdNode


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestGripperInit(unittest.TestCase):
    """초기화: 토픽 연결 및 파라미터 검증"""

    def setUp(self):
        self.node = ArduinoCmdNode()

    def test_subscribes_to_gripper_command(self):
        self.assertIn('/vtol/gripper/command', self.node._subscriptions,
                      "그리퍼 명령 구독 없음 — create_subscription 추가 필요")

    def test_declares_open_angle_parameter(self):
        self.assertIn('vtol.gripper_open_angle', self.node._parameters,
                      "gripper_open_angle 파라미터 선언 없음")

    def test_declares_close_angle_parameter(self):
        self.assertIn('vtol.gripper_close_angle', self.node._parameters,
                      "gripper_close_angle 파라미터 선언 없음")


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestGripperSerial(unittest.TestCase):
    """시리얼 명령 전송 검증"""

    def setUp(self):
        self.node = ArduinoCmdNode()

    def test_command_callback_is_callable(self):
        self.assertIn('/vtol/gripper/command', self.node._subscriptions,
                      "구독 없음 — 콜백 확인 불가")
        callback, _ = self.node._subscriptions['/vtol/gripper/command']
        self.assertTrue(callable(callback))

    def test_serial_send_called_on_command(self):
        """그리퍼 명령 수신 시 시리얼 write 또는 동등 메서드가 호출되어야 함"""
        send_method = getattr(self.node, 'send_serial', None) or \
                      getattr(self.node, '_send_serial', None) or \
                      getattr(self.node, '_write_serial', None)
        self.assertIsNotNone(send_method,
                             "send_serial 계열 메서드 없음")


if __name__ == '__main__':
    unittest.main()
