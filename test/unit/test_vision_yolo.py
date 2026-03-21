"""
unit › vtol_vision_yolo › YoloDetectNode

관련 명세: TC-002 상태 수신 / TC-007 통합 미션
담당자:   비전 A

구현 전 상태: skip — 아래 @unittest.skip 제거 후 구현 시작
구현 후 목표: 전체 PASS

구현 체크리스트:
  [ ] /drone1/camera/image_raw 구독
  [ ] ultralytics YOLO 모델 로드
  [ ] 바운딩 박스 추론
  [ ] vtol.yolo_confidence_threshold 파라미터로 필터링
  [ ] /vtol/yolo/detections (vision_msgs/Detection2DArray) 퍼블리시
"""
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_SKIP_REASON = (
    "vtol_vision_yolo 노드는 아직 구현 전입니다. "
    "초보자용 기본 브랜치를 항상 초록 상태로 유지하기 위해 현재는 skip 합니다. "
    "비전 A 구현을 시작할 때 이 skip 을 제거하고 테스트를 함께 녹색으로 바꾸세요."
)

sys.path.insert(0, str(Path(__file__).parents[1]))
from mock_ros2 import install
install()

sys.path.insert(0, str(Path(__file__).parents[2] / 'src' / 'vtol_vision_yolo' / 'src'))
from yolo_detect_node import YoloDetectNode


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestYoloInit(unittest.TestCase):
    """초기화: 토픽 연결 검증"""

    def setUp(self):
        self.node = YoloDetectNode()

    def test_subscribes_to_camera_image(self):
        self.assertIn('/drone1/camera/image_raw', self.node._subscriptions,
                      "카메라 토픽 구독 없음 — create_subscription 추가 필요")

    def test_publishes_yolo_detections(self):
        self.assertIn('/vtol/yolo/detections', self.node._publishers,
                      "YOLO 탐지 결과 퍼블리셔 없음 — create_publisher 추가 필요")

    def test_declares_confidence_threshold_parameter(self):
        self.assertIn('vtol.yolo_confidence_threshold', self.node._parameters,
                      "yolo_confidence_threshold 파라미터 선언 없음")


@unittest.skip(_SKIP_REASON)
@unittest.skip('미구현 — 구현 시작 시 이 줄을 제거하세요')
class TestYoloCallback(unittest.TestCase):
    """이미지 콜백 동작 검증"""

    def setUp(self):
        self.node = YoloDetectNode()

    def test_image_callback_is_callable(self):
        self.assertIn('/drone1/camera/image_raw', self.node._subscriptions,
                      "구독 없음 — 콜백 확인 불가")
        callback, _ = self.node._subscriptions['/drone1/camera/image_raw']
        self.assertTrue(callable(callback))

    def test_detections_published_on_image_received(self):
        """이미지 수신 시 /vtol/yolo/detections 를 퍼블리시해야 함"""
        self.assertIn('/vtol/yolo/detections', self.node._publishers,
                      "퍼블리셔 없음")
        pub_mock = self.node._publishers['/vtol/yolo/detections']

        fake_image = MagicMock()
        callback, _ = self.node._subscriptions.get('/drone1/camera/image_raw', (None, None))
        if callback:
            callback(fake_image)

        self.assertTrue(pub_mock.publish.called,
                        "이미지 수신 후 detections 퍼블리시가 호출되지 않음")


if __name__ == '__main__':
    unittest.main()
