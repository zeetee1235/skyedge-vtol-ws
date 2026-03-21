#!/usr/bin/env python3
"""
패키지명: vtol_hw_gripper
노드명:   arduino_cmd_node
담당자:   HW A 담당
설명:     아두이노 직렬 통신을 통한 집게발 상하강/파지 제어
"""

import rclpy
from rclpy.node import Node


class ArduinoCmdNode(Node):
    def __init__(self):
        super().__init__('arduino_cmd_node')
        self.get_logger().info('ArduinoCmdNode 시작됨')

        # TODO: 구현 필요
        # - /vtol/gripper/command 구독 (Int32)
        # - pyserial로 아두이노 직렬 통신
        # - gripper_open_angle / gripper_close_angle 파라미터 적용


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
