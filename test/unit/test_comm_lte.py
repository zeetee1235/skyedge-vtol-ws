"""
unit › vtol_comm_lte › TelemetryNode

관련 명세: TC-002 상태 수신
담당자:   통신 B

구현 전 상태: skip — 아래 @unittest.skip 제거 후 구현 시작
구현 후 목표: 전체 PASS

구현 체크리스트:
  [ ] /drone1/fmu/out/vehicle_status 구독
  [ ] /drone1/fmu/out/vehicle_local_position 구독
  [ ] vtol.lte_gcs_ip / vtol.lte_gcs_port 파라미터 적용
  [ ] 주기적으로 GCS에 UDP 패킷 전송
  [ ] 전송 실패 시 예외 처리 (소켓 에러 → 로그)
"""
import sys
import unittest
from pathlib import Path

_SKIP_REASON = (
    "vtol_comm_lte 노드는 아직 구현 전입니다. "
    "초보자용 기본 브랜치를 항상 초록 상태로 유지하기 위해 현재는 skip 합니다. "
    "통신 B 구현을 시작할 때 이 skip 을 제거하고 테스트를 함께 녹색으로 바꾸세요."
)

sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_comm_lte' / 'src'))
from telemetry_node import TelemetryNode


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestTelemetryInit(unittest.TestCase):
    """초기화: 토픽 연결 및 파라미터 검증"""

    def setUp(self):
        self.node = TelemetryNode()

    def test_subscribes_to_vehicle_status(self):
        self.assertIn('/drone1/fmu/out/vehicle_status', self.node._subscriptions,
                      "vehicle_status 구독 없음")

    def test_subscribes_to_vehicle_local_position(self):
        self.assertIn('/drone1/fmu/out/vehicle_local_position', self.node._subscriptions,
                      "vehicle_local_position 구독 없음")

    def test_declares_gcs_ip_parameter(self):
        self.assertIn('vtol.lte_gcs_ip', self.node._parameters,
                      "lte_gcs_ip 파라미터 선언 없음")

    def test_declares_gcs_port_parameter(self):
        self.assertIn('vtol.lte_gcs_port', self.node._parameters,
                      "lte_gcs_port 파라미터 선언 없음")


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestTelemetrySend(unittest.TestCase):
    """GCS 전송 로직 검증"""

    def setUp(self):
        self.node = TelemetryNode()

    def test_has_send_telemetry_method(self):
        send = getattr(self.node, 'send_telemetry', None) or \
               getattr(self.node, '_send_telemetry', None) or \
               getattr(self.node, '_send_to_gcs', None)
        self.assertIsNotNone(send,
                             "send_telemetry 계열 메서드 없음")

    def test_timer_registered_for_periodic_send(self):
        self.assertGreater(len(self.node._timer_callbacks), 0,
                           "주기 전송용 타이머 없음 — create_timer 추가 필요")


if __name__ == '__main__':
    unittest.main()
