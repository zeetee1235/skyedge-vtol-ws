"""
unit › vtol_vision_aruco › ArucoDetectNode

관련 명세: TC-002 상태 수신 / TC-007 통합 미션
담당자:   비전 B

구현 전 상태: skip — 아래 @unittest.skip 제거 후 구현 시작
구현 후 목표: 전체 PASS

구현 체크리스트:
  [ ] /drone1/camera/image_raw 구독
  [ ] cv2.aruco 로 마커 검출
  [ ] 카메라 캘리브레이션 파라미터 적용
  [ ] /vtol/aruco/pose (geometry_msgs/PoseStamped) 퍼블리시
  [ ] vtol.aruco_marker_id 파라미터 사용
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_SKIP_REASON = (
    "vtol_vision_aruco 노드는 아직 구현 전입니다. "
    "초보자용 기본 브랜치를 항상 초록 상태로 유지하기 위해 현재는 skip 합니다. "
    "비전 B 구현을 시작할 때 이 skip 을 제거하고 테스트를 함께 녹색으로 바꾸세요."
)

sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_vision_aruco' / 'src'))
from aruco_detect_node import ArucoDetectNode


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestArucoInit(unittest.TestCase):
    """초기화: 토픽 연결 검증"""

    def setUp(self):
        self.node = ArucoDetectNode()

    def test_subscribes_to_camera_image(self):
        self.assertIn('/drone1/camera/image_raw', self.node._subscriptions,
                      "카메라 토픽 구독 없음 — create_subscription 추가 필요")

    def test_publishes_aruco_pose(self):
        self.assertIn('/vtol/aruco/pose', self.node._publishers,
                      "ArUco pose 퍼블리셔 없음 — create_publisher 추가 필요")

    def test_declares_marker_id_parameter(self):
        self.assertIn('vtol.aruco_marker_id', self.node._parameters,
                      "aruco_marker_id 파라미터 선언 없음")


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestArucoCallback(unittest.TestCase):
    """이미지 콜백 동작 검증"""

    def setUp(self):
        self.node = ArucoDetectNode()

    def test_image_callback_exists(self):
        """카메라 구독 콜백이 callable 해야 함"""
        self.assertIn('/drone1/camera/image_raw', self.node._subscriptions,
                      "구독 없음 — 콜백 확인 불가")
        callback, _ = self.node._subscriptions['/drone1/camera/image_raw']
        self.assertTrue(callable(callback),
                        "이미지 콜백이 callable 하지 않음")

    def test_pose_published_when_marker_detected(self):
        """마커가 검출되면 /vtol/aruco/pose 를 퍼블리시해야 함"""
        self.assertIn('/vtol/aruco/pose', self.node._publishers,
                      "퍼블리셔 없음 — 퍼블리시 동작 확인 불가")
        pub_mock = self.node._publishers['/vtol/aruco/pose']

        # 이미지 콜백 호출 시뮬레이션
        fake_image = MagicMock()
        callback, _ = self.node._subscriptions.get('/drone1/camera/image_raw', (None, None))
        if callback:
            callback(fake_image)

        self.assertTrue(pub_mock.publish.called,
                        "마커 탐지 후 pose 퍼블리시가 호출되지 않음")


if __name__ == '__main__':
    unittest.main()
