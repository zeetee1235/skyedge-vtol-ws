#!/usr/bin/env python3
"""
패키지명: vtol_vision_aruco
노드명:   aruco_detect_node
담당자:   비전 B 담당
설명:     ArUco 마커 인식 및 카메라 캘리브레이션 기반 pose 퍼블리시
"""

import rclpy
from rclpy.node import Node


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')
        self.get_logger().info('ArucoDetectNode 시작됨')

        # TODO: 구현 필요
        # - 카메라 토픽 구독
        # - ArUco 마커 인식 (cv2.aruco)
        # - 카메라 캘리브레이션 적용
        # - /vtol/aruco/pose 퍼블리시


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
