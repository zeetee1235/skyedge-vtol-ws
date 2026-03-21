#!/usr/bin/env python3
"""
패키지명: vtol_control_task
노드명:   precision_task_node
담당자:   제어 B 담당
설명:     정밀 착륙, 객체 추적 하강, 2~3m 고도 유지
"""

import rclpy
from rclpy.node import Node


class PrecisionTaskNode(Node):
    def __init__(self):
        super().__init__('precision_task_node')
        self.get_logger().info('PrecisionTaskNode 시작됨')

        # TODO: 구현 필요
        # - /vtol/aruco/pose 구독
        # - /vtol/yolo/detections 구독
        # - 2~3m 고도 유지 로직
        # - 정밀 착륙 하강 제어
        # - /vtol/gripper/command 퍼블리시


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionTaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
