#!/usr/bin/env python3
"""
패키지명: vtol_comm_lte
노드명:   telemetry_node
담당자:   통신 B 담당
설명:     LTE 텔레메트리를 통한 GCS 데이터 송수신 및 상태 모니터링
"""

import rclpy
from rclpy.node import Node


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.get_logger().info('TelemetryNode 시작됨')

        # TODO: 구현 필요
        # - /drone1/fmu/out/monitoring 구독
        # - LTE UDP 소켓으로 GCS에 상태 전송 (lte_gcs_ip:lte_gcs_port)
        # - 배터리, 속도, 위치 등 핵심 텔레메트리 데이터 포맷팅


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
