#!/usr/bin/env python3
"""
패키지명: vtol_vision_yolo
노드명:   yolo_detect_node
담당자:   비전 A 담당
설명:     YOLO v8/v11 기반 객체 인식 및 바운딩 박스 퍼블리시
"""

import rclpy
from rclpy.node import Node


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')
        self.get_logger().info('YoloDetectNode 시작됨')

        # TODO: 구현 필요
        # - 카메라 토픽 구독
        # - YOLO 추론
        # - 바운딩 박스 퍼블리시


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
