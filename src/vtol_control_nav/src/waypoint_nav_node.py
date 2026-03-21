#!/usr/bin/env python3
"""
패키지명: vtol_control_nav
노드명:   waypoint_nav_node
담당자:   제어 A 담당
설명:     GPS 웨이포인트 기반 VTOL 순항 비행 제어 (PX4 uXRCE-DDS)

상태 머신:
  IDLE → ARMING → TAKEOFF → NAVIGATE → LAND → DONE

PX4 토픽 네임스페이스: /{drone_id}/fmu/in|out/...
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


# PX4 uXRCE-DDS 필수 QoS
_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class WaypointNavNode(Node):
    """PX4 VTOL 웨이포인트 비행 노드 (Offboard 포지션 제어)"""

    _IDLE = 'IDLE'
    _ARMING = 'ARMING'
    _TAKEOFF = 'TAKEOFF'
    _NAVIGATE = 'NAVIGATE'
    _LAND = 'LAND'
    _DONE = 'DONE'

    def __init__(self):
        super().__init__('waypoint_nav_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('vtol.drone_id', 'drone1')
        self.declare_parameter('vtol.takeoff_altitude', 5.0)
        self.declare_parameter('vtol.cruise_altitude', 30.0)

        drone_id = self.get_parameter('vtol.drone_id').value
        takeoff_alt = self.get_parameter('vtol.takeoff_altitude').value
        cruise_alt = self.get_parameter('vtol.cruise_altitude').value

        # NED 좌표계: 위쪽이 음수
        self._takeoff_z = -abs(takeoff_alt)
        self._cruise_z = -abs(cruise_alt)

        ns = f'/{drone_id}'

        # ── Publishers ────────────────────────────────────────────
        self._pub_offboard = self.create_publisher(
            OffboardControlMode,
            f'{ns}/fmu/in/offboard_control_mode',
            _PX4_QOS,
        )
        self._pub_setpoint = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint',
            _PX4_QOS,
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand,
            f'{ns}/fmu/in/vehicle_command',
            _PX4_QOS,
        )

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            VehicleStatus,
            f'{ns}/fmu/out/vehicle_status',
            self._cb_status,
            _PX4_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position',
            self._cb_local_pos,
            _PX4_QOS,
        )

        # ── 상태 변수 ─────────────────────────────────────────────
        self._status = VehicleStatus()
        self._local_pos = VehicleLocalPosition()
        self._state = self._IDLE
        self._pre_arm_cnt = 0   # Offboard 모드 전환 전 setpoint 최소 10회 필요

        # 웨이포인트 (NED local frame, 미터)
        # 실제 운영 시에는 GPS → local 변환(NavSatFix → LocalPosition) 필요
        self._waypoints: list[tuple[float, float, float]] = [
            (0.0, 0.0, self._cruise_z),   # WP0: 이륙 지점 상공
            (100.0, 0.0, self._cruise_z),   # WP1
            (100.0, 100.0, self._cruise_z),   # WP2
            (0.0, 100.0, self._cruise_z),   # WP3
        ]
        self._wp_idx = 0

        self._timer = self.create_timer(0.1, self._control_loop)  # 10 Hz
        self.get_logger().info(f'WaypointNavNode 시작 (ns={ns})')

    # ── 콜백 ─────────────────────────────────────────────────────

    def _cb_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
        self._local_pos = msg

    # ── 제어 루프 (10 Hz) ─────────────────────────────────────────

    def _control_loop(self) -> None:
        if self._state == self._IDLE:
            # Offboard 모드 진입 전 setpoint를 연속으로 전송해야 함 (PX4 규칙)
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, self._takeoff_z)
            self._pre_arm_cnt += 1
            if self._pre_arm_cnt >= 10:
                self._cmd_arm()
                self._cmd_set_offboard_mode()
                self._state = self._ARMING
                self.get_logger().info('→ ARMING')

        elif self._state == self._ARMING:
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, self._takeoff_z)
            if self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._state = self._TAKEOFF
                self.get_logger().info('→ TAKEOFF')

        elif self._state == self._TAKEOFF:
            self._send_offboard_mode()
            self._send_setpoint(0.0, 0.0, self._takeoff_z)
            if self._reached(0.0, 0.0, self._takeoff_z, thr=0.5):
                self._state = self._NAVIGATE
                self.get_logger().info(f'→ NAVIGATE  WP 0 / {len(self._waypoints)}')

        elif self._state == self._NAVIGATE:
            wp = self._waypoints[self._wp_idx]
            self._send_offboard_mode()
            self._send_setpoint(*wp)
            if self._reached(*wp):
                self._wp_idx += 1
                if self._wp_idx >= len(self._waypoints):
                    self._state = self._LAND
                    self.get_logger().info('모든 웨이포인트 완료 → LAND')
                else:
                    self.get_logger().info(f'→ WP {self._wp_idx}')

        elif self._state == self._LAND:
            self._cmd_land()
            self._state = self._DONE
            self.get_logger().info('→ DONE')

    # ── 도달 판정 ─────────────────────────────────────────────────

    def _reached(self, x: float, y: float, z: float, thr: float = 2.0) -> bool:
        p = self._local_pos
        return math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2) < thr

    # ── OffboardControlMode / TrajectorySetpoint 퍼블리시 ─────────

    def _send_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = self._us_now()
        self._pub_offboard.publish(msg)

    def _send_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self._us_now()
        self._pub_setpoint.publish(msg)

    # ── VehicleCommand ────────────────────────────────────────────

    def _cmd_arm(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _cmd_set_offboard_mode(self) -> None:
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, PX4 custom_mode=6(Offboard)
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       param1=1.0, param2=6.0)

    def _cmd_vtol_to_fw(self) -> None:
        """VTOL → 고정익 전환 (순항 고도 도달 후 호출)"""
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)

    def _cmd_vtol_to_mc(self) -> None:
        """VTOL → 멀티콥터 전환 (착륙 전 호출)"""
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)

    def _cmd_land(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0,
                  param3: float = 0.0, param4: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._us_now()
        self._pub_cmd.publish(msg)

    def _us_now(self) -> int:
        """현재 시각 (마이크로초) — PX4 타임스탬프 형식"""
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
