"""
ROS2 mock 헬퍼 — ROS2 설치 없이 노드 단위 테스트 실행용

각 테스트 파일 상단에서:
    from mock_ros2 import install; install()
를 호출하면 rclpy, px4_msgs 등을 mock 처리합니다.
"""
import sys
from unittest.mock import MagicMock


class MockNode:
    """create_publisher / create_subscription 호출을 추적하는 경량 Node mock"""

    def __init__(self, name: str = 'mock_node'):
        self._name = name
        self._publishers: dict[str, MagicMock] = {}       # topic → Mock
        self._subscriptions: dict[str, tuple] = {}        # topic → (callback, Mock)
        self._parameters: dict = {}
        self._timer_callbacks: list = []

    def declare_parameter(self, name: str, default=None):
        self._parameters.setdefault(name, default)

    def get_parameter(self, name: str):
        m = MagicMock()
        m.value = self._parameters.get(name)
        return m

    def create_publisher(self, msg_type, topic: str, qos=None):
        pub = MagicMock()
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos=None):
        sub = MagicMock()
        self._subscriptions[topic] = (callback, sub)
        return sub

    def create_timer(self, period, callback):
        self._timer_callbacks.append(callback)
        return MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.nanoseconds = 1_000_000_000
        return clock

    def get_logger(self):
        return MagicMock()


class _VehicleStatus:
    """PX4 VehicleStatus 상수 (실제 px4_msgs 값과 동일)"""
    ARMING_STATE_STANDBY = 1
    ARMING_STATE_ARMED   = 2


class _VehicleCommand:
    """PX4 VehicleCommand 상수 (실제 px4_msgs 값과 동일)"""
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_DO_SET_MODE          = 176
    VEHICLE_CMD_DO_VTOL_TRANSITION   = 3000
    VEHICLE_CMD_NAV_LAND             = 21


def install() -> None:
    """sys.modules 에 ROS2 mock 주입. 반드시 노드 import 전에 호출."""
    _m = MagicMock

    # ── rclpy 코어 ──────────────────────────────────────────────
    sys.modules['rclpy']      = _m()
    sys.modules['rclpy.node'] = _m(Node=MockNode)
    sys.modules['rclpy.qos']  = _m(
        QoSProfile=_m,
        ReliabilityPolicy=_m(),
        HistoryPolicy=_m(),
        DurabilityPolicy=_m(),
    )

    # ── px4_msgs ─────────────────────────────────────────────────
    px4_msg = _m()
    px4_msg.VehicleStatus         = _VehicleStatus
    px4_msg.VehicleCommand        = _VehicleCommand
    px4_msg.OffboardControlMode   = _m
    px4_msg.TrajectorySetpoint    = _m
    px4_msg.VehicleLocalPosition  = _m
    sys.modules['px4_msgs']     = _m()
    sys.modules['px4_msgs.msg'] = px4_msg

    # ── 기타 메시지 패키지 ────────────────────────────────────────
    for mod in [
        'sensor_msgs', 'sensor_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'vision_msgs', 'vision_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'diagnostic_msgs', 'diagnostic_msgs.msg',
        'cv_bridge', 'tf2_ros',
    ]:
        sys.modules.setdefault(mod, _m())
