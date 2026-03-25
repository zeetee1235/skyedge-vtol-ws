#!/usr/bin/env python3
"""
패키지명: vtol_control_nav
노드명:   waypoint_nav_node
담당자:   제어 A 담당
설명:     VTOL 웨이포인트 기반 순항 비행 제어 (PX4 uXRCE-DDS)

상태 머신:
  IDLE → ARMING → TAKEOFF → TRANSITION_TO_FW → NAVIGATE
       → TRANSITION_TO_MC → LAND → LANDING_CONFIRM → DONE

PX4 토픽 네임스페이스: /{drone_id}/fmu/in|out/...
"""

import math
from typing import Callable, Iterable

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from trajectory_planners import create_trajectory_planner
from precision_landing import BasePrecisionLandingController
from mission_executor import BaseMissionExecutor


# PX4 uXRCE-DDS 필수 QoS
_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MissionPlanner:
    """YAML/파라미터 waypoint를 내부 미션 리스트로 관리."""

    def __init__(self, cruise_z: float):
        self._cruise_z = cruise_z

    def build_mission(
        self,
        frame: str,
        raw_waypoints: Iterable[Iterable[float]],
    ) -> tuple[list[tuple[float, float, float]], bool]:
        parsed = self._parse_waypoints(raw_waypoints)
        if not parsed:
            return [(0.0, 0.0, self._cruise_z)], False

        frame_l = frame.lower()
        if frame_l == 'gps':
            mission, has_gps_error = self._gps_waypoints_to_local_ned(parsed)
            if mission:
                return mission, has_gps_error
            return [(0.0, 0.0, self._cruise_z)], True

        # local_ned 또는 알 수 없는 frame은 local로 처리
        return ([(x, y, -abs(z)) for x, y, z in parsed], False)

    def _parse_waypoints(self, raw_waypoints: Iterable[Iterable[float]]) -> list[tuple[float, float, float]]:
        parsed: list[tuple[float, float, float]] = []
        if not isinstance(raw_waypoints, (list, tuple)):
            return parsed

        for item in raw_waypoints:
            # ROS2 파라미터 제약(중첩 배열 미지원)을 위해 "x,y,z" 문자열도 허용
            if isinstance(item, str):
                parts = [p.strip() for p in item.split(',')]
                if len(parts) != 3:
                    continue
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                except (TypeError, ValueError):
                    continue
                parsed.append((x, y, z))
                continue

            if not isinstance(item, (list, tuple)) or len(item) != 3:
                continue
            try:
                x, y, z = float(item[0]), float(item[1]), float(item[2])
            except (TypeError, ValueError):
                continue
            parsed.append((x, y, z))

        return parsed

    def _gps_waypoints_to_local_ned(
        self,
        gps_waypoints: list[tuple[float, float, float]],
    ) -> tuple[list[tuple[float, float, float]], bool]:
        """GPS(lat, lon, alt) -> local NED(north, east, down)."""
        valid: list[tuple[float, float, float]] = []
        has_gps_error = False
        for lat, lon, alt in gps_waypoints:
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                has_gps_error = True
                continue
            valid.append((lat, lon, alt))

        if not valid:
            return [], True

        lat0, lon0, _ = valid[0]
        lat0_rad = math.radians(lat0)
        earth_r = 6_378_137.0

        local: list[tuple[float, float, float]] = []
        for lat, lon, alt in valid:
            d_lat = math.radians(lat - lat0)
            d_lon = math.radians(lon - lon0)
            north = d_lat * earth_r
            east = d_lon * earth_r * math.cos(lat0_rad)
            down = -abs(alt)
            local.append((north, east, down))

        return local, has_gps_error


class TrajectoryPlanner:
    """하위호환 래퍼: 실제 구현은 trajectory_planners 패키지에 위임."""

    def __init__(
        self,
        mode: str,
        max_step_m: float,
        mpc_cfg: dict[str, float] | None = None,
    ):
        self._mode = mode
        self._max_step_m = max_step_m
        self._mpc_cfg = dict(mpc_cfg or {})
        self._planner = create_trajectory_planner(
            planner_type=self._mode,
            max_step_m=self._max_step_m,
            mpc_cfg=self._mpc_cfg,
        )
        self._planner.update_config(max_step_m=self._max_step_m, **self._mpc_cfg)

    def update_config(
        self,
        mode: str,
        max_step_m: float,
        mpc_cfg: dict[str, float] | None = None,
    ) -> None:
        self._mode = mode
        self._max_step_m = max_step_m
        if mpc_cfg is not None:
            self._mpc_cfg = dict(mpc_cfg)

        # 모드 변경 시 구현체를 교체
        self._planner = create_trajectory_planner(
            planner_type=self._mode,
            max_step_m=self._max_step_m,
            mpc_cfg=self._mpc_cfg,
        )
        self._planner.update_config(max_step_m=self._max_step_m, **self._mpc_cfg)

    def reset_segment(self) -> None:
        self._planner.reset_segment()

    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return self._planner.next_setpoint(current=current, target=target)


class WaypointNavNode(Node):
    """PX4 VTOL 웨이포인트 비행 노드 (Offboard 포지션 제어)"""

    _IDLE = 'IDLE'
    _ARMING = 'ARMING'
    _TAKEOFF = 'TAKEOFF'
    _TRANSITION_TO_FW = 'TRANSITION_TO_FW'
    _NAVIGATE = 'NAVIGATE'
    _TRANSITION_TO_MC = 'TRANSITION_TO_MC'
    _LAND = 'LAND'
    _LANDING_CONFIRM = 'LANDING_CONFIRM'
    _FAILSAFE_LAND = 'FAILSAFE_LAND'
    _DONE = 'DONE'

    # ── 확장 상태 (컨트롤러/실행기 미설정 시 pass-through) ────────────
    # 정밀 착륙: TRANSITION_TO_MC 완료 후 LAND 전 삽입
    #   구현: BasePrecisionLandingController 를 _precision_landing_ctrl 에 할당
    _PRECISION_LAND = 'PRECISION_LAND'
    # 웨이포인트 임무: 각 WP 정상 도달 후 다음 WP 진행 전 삽입
    #   구현: BaseMissionExecutor 를 _mission_executor 에 할당
    _MISSION_EXEC = 'MISSION_EXEC'

    _CTRL_DT = 0.1  # 10 Hz

    def __init__(self):
        super().__init__('waypoint_nav_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('vtol.drone_id', 'drone1')
        self.declare_parameter('vtol.use_sim', True)
        self.declare_parameter('vtol.takeoff_altitude', 5.0)
        self.declare_parameter('vtol.cruise_altitude', 30.0)
        self.declare_parameter('vtol.max_altitude_m', 40.0)
        self.declare_parameter('vtol.takeoff_timeout_sec', 35.0)

        self.declare_parameter('vtol.waypoint_frame', 'gps')
        # ROS2 파라미터 타입 추론 이슈를 피하기 위해 문자열 배열 기본값 사용
        self.declare_parameter('vtol.waypoints', ['0.0,0.0,30.0'])
        self.declare_parameter('vtol.dynamic_waypoint_update', True)
        self.declare_parameter('vtol.dynamic_reload_period_sec', 1.0)

        self.declare_parameter('vtol.waypoint_reached_threshold', 2.0)
        self.declare_parameter('vtol.transition_timeout_sec', 8.0)
        self.declare_parameter('vtol.waypoint_unreachable_timeout_sec', 45.0)

        # 신규 키(vtol.trajectory.type) + 기존 키(vtol.trajectory_mode) 병행 지원
        self.declare_parameter('vtol.trajectory.type', '')
        self.declare_parameter('vtol.trajectory_mode', 'smoothstep')
        self.declare_parameter('vtol.trajectory_max_step_m', 2.0)

        # MPC stub 파라미터
        self.declare_parameter('vtol.mpc.horizon_steps', 15)
        self.declare_parameter('vtol.mpc.dt', 0.1)
        self.declare_parameter('vtol.mpc.w_pos', 1.0)
        self.declare_parameter('vtol.mpc.w_vel', 0.2)
        self.declare_parameter('vtol.mpc.w_u', 0.05)

        self.declare_parameter('vtol.comm_loss_timeout_sec', 2.0)
        self.declare_parameter('vtol.gps_error_failsafe', True)

        self.declare_parameter('vtol.landing_confirm_alt_threshold', 0.4)
        self.declare_parameter('vtol.landing_confirm_speed_threshold', 0.5)
        self.declare_parameter('vtol.landing_confirm_hold_sec', 1.0)
        self.declare_parameter('vtol.landing_confirm_use_kinematic_fallback', False)

        # 기본 파라미터 로드
        drone_id = self.get_parameter('vtol.drone_id').value
        self._use_sim = bool(self.get_parameter('vtol.use_sim').value)
        takeoff_alt = float(self.get_parameter('vtol.takeoff_altitude').value)
        cruise_alt = float(self.get_parameter('vtol.cruise_altitude').value)
        max_alt_m = float(self.get_parameter('vtol.max_altitude_m').value)

        self._takeoff_z = -abs(takeoff_alt)
        self._cruise_z = -abs(cruise_alt)
        self._max_alt_z = -abs(max_alt_m)
        # 이륙/순항 목표도 최대 고도 제한을 넘지 않도록 초기값부터 클램프
        self._takeoff_z = max(self._takeoff_z, self._max_alt_z)
        self._cruise_z = max(self._cruise_z, self._max_alt_z)

        self._wp_reached_thr = float(self.get_parameter('vtol.waypoint_reached_threshold').value)
        self._landing_alt_thr = float(self.get_parameter('vtol.landing_confirm_alt_threshold').value)
        self._landing_speed_thr = float(self.get_parameter('vtol.landing_confirm_speed_threshold').value)
        self._use_kinematic_landing_fallback = bool(
            self.get_parameter('vtol.landing_confirm_use_kinematic_fallback').value,
        )
        self._gps_error_failsafe = bool(self.get_parameter('vtol.gps_error_failsafe').value)

        transition_timeout_sec = float(self.get_parameter('vtol.transition_timeout_sec').value)
        landing_confirm_hold_sec = float(self.get_parameter('vtol.landing_confirm_hold_sec').value)
        waypoint_unreachable_timeout_sec = float(
            self.get_parameter('vtol.waypoint_unreachable_timeout_sec').value,
        )
        self._comm_loss_timeout_sec = float(self.get_parameter('vtol.comm_loss_timeout_sec').value)

        self._transition_timeout_cycles = max(1, int(math.ceil(transition_timeout_sec / self._CTRL_DT)))
        takeoff_timeout_sec = float(self.get_parameter('vtol.takeoff_timeout_sec').value)
        self._takeoff_timeout_cycles = max(1, int(math.ceil(takeoff_timeout_sec / self._CTRL_DT)))
        self._landing_confirm_required_cycles = max(
            1,
            int(math.ceil(landing_confirm_hold_sec / self._CTRL_DT)),
        )
        self._wp_unreachable_timeout_cycles = max(
            1,
            int(math.ceil(waypoint_unreachable_timeout_sec / self._CTRL_DT)),
        )
        self._comm_loss_timeout_cycles = max(1, int(math.ceil(self._comm_loss_timeout_sec / self._CTRL_DT)))

        self._dynamic_waypoint_update = bool(self.get_parameter('vtol.dynamic_waypoint_update').value)
        dynamic_reload_sec = float(self.get_parameter('vtol.dynamic_reload_period_sec').value)
        self._dynamic_reload_cycles = max(1, int(math.ceil(dynamic_reload_sec / self._CTRL_DT)))

        planner_type, traj_step_m, mpc_cfg = self._read_trajectory_config_from_params()

        # 미션/플래너
        self._mission_planner = MissionPlanner(cruise_z=self._cruise_z)
        self._trajectory_planner = TrajectoryPlanner(
            mode=planner_type,
            max_step_m=traj_step_m,
            mpc_cfg=mpc_cfg,
        )
        self._waypoint_frame = 'gps'
        self._waypoints: list[tuple[float, float, float]] = [(0.0, 0.0, self._cruise_z)]
        self._gps_error_detected = False
        self._mission_signature: tuple | None = None
        self._reload_mission_from_params(force=True, apply_inflight=False)

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
            f'{ns}/fmu/out/vehicle_status_v1',
            self._cb_status,
            _PX4_QOS,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position_v1',
            self._cb_local_pos,
            _PX4_QOS,
        )
        self.create_subscription(
            VehicleLandDetected,
            f'{ns}/fmu/out/vehicle_land_detected',
            self._cb_land_detected,
            _PX4_QOS,
        )

        # ── 상태 변수 ─────────────────────────────────────────────
        self._status = VehicleStatus()
        self._local_pos = VehicleLocalPosition()
        self._land_detected = VehicleLandDetected()

        self._state = self._IDLE
        self._pre_arm_cnt = 0
        self._arming_retry_cnt = 0
        self._arm_keepalive_cnt = 0
        self._transition_wait_cnt = 0
        self._takeoff_hold_cnt = 0
        self._landing_confirm_cnt = 0
        self._wp_progress_cnt = 0

        self._wp_idx = 0
        self._landing_hold_sp = self._waypoints[-1]
        self._failsafe_reason = ''
        self._failsafe_cmd_sent = False

        # ── 확장 포인트: 외부에서 할당하여 기능 활성화 ─────────────────
        # 정밀 착륙 컨트롤러 (BasePrecisionLandingController 구현체)
        self._precision_landing_ctrl: BasePrecisionLandingController | None = None
        # 웨이포인트 임무 실행기 (BaseMissionExecutor 구현체)
        self._mission_executor: BaseMissionExecutor | None = None

        self._loop_cnt = 0
        self._last_status_loop = 0
        self._last_local_pos_loop = 0
        self._seen_status = False
        self._seen_local_pos = False
        self._last_reload_loop = 0

        self._state_handlers: dict[str, Callable[[], None]] = {
            self._IDLE: self._handle_idle,
            self._ARMING: self._handle_arming,
            self._TAKEOFF: self._handle_takeoff,
            self._TRANSITION_TO_FW: self._handle_transition_to_fw,
            self._NAVIGATE: self._handle_navigate,
            self._TRANSITION_TO_MC: self._handle_transition_to_mc,
            self._LAND: self._handle_land,
            self._LANDING_CONFIRM: self._handle_landing_confirm,
            self._FAILSAFE_LAND: self._handle_failsafe_land,
            self._DONE: self._handle_done,
            self._PRECISION_LAND: self._handle_precision_land,
            self._MISSION_EXEC: self._handle_mission_exec,
        }

        self._timer = self.create_timer(self._CTRL_DT, self._control_loop)
        self.get_logger().info(
            f'WaypointNavNode 시작 (ns={ns}, frame={self._waypoint_frame}, waypoints={len(self._waypoints)})',
        )

    # ── 콜백 ─────────────────────────────────────────────────────

    def _cb_status(self, msg: VehicleStatus) -> None:
        self._status = msg
        self._seen_status = True
        self._last_status_loop = self._loop_cnt

    def _cb_local_pos(self, msg: VehicleLocalPosition) -> None:
        self._local_pos = msg
        self._seen_local_pos = True
        self._last_local_pos_loop = self._loop_cnt

    def _cb_land_detected(self, msg: VehicleLandDetected) -> None:
        self._land_detected = msg

    # ── 하위호환 헬퍼 (기존 테스트/호출부) ───────────────────────────

    def _parse_waypoints(self, raw_waypoints: Iterable[Iterable[float]]) -> list[tuple[float, float, float]]:
        return self._mission_planner._parse_waypoints(raw_waypoints)

    def _gps_waypoints_to_local_ned(
        self,
        gps_waypoints: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        mission, _ = self._mission_planner._gps_waypoints_to_local_ned(gps_waypoints)
        return mission

    # ── 미션/파라미터 로드 ─────────────────────────────────────────

    def _read_trajectory_config_from_params(self) -> tuple[str, float, dict[str, float]]:
        planner_type = str(self.get_parameter('vtol.trajectory.type').value or '').strip()
        if not planner_type:
            planner_type = str(self.get_parameter('vtol.trajectory_mode').value)

        max_step = float(self.get_parameter('vtol.trajectory_max_step_m').value)
        mpc_cfg = {
            'horizon_steps': int(self.get_parameter('vtol.mpc.horizon_steps').value),
            'dt': float(self.get_parameter('vtol.mpc.dt').value),
            'w_pos': float(self.get_parameter('vtol.mpc.w_pos').value),
            'w_vel': float(self.get_parameter('vtol.mpc.w_vel').value),
            'w_u': float(self.get_parameter('vtol.mpc.w_u').value),
        }
        return planner_type, max_step, mpc_cfg

    def _reload_mission_from_params(self, force: bool, apply_inflight: bool) -> bool:
        frame = str(self.get_parameter('vtol.waypoint_frame').value).lower()
        raw_waypoints = self.get_parameter('vtol.waypoints').value
        signature = (frame, str(raw_waypoints))
        if not force and signature == self._mission_signature:
            return False

        mission, gps_error = self._mission_planner.build_mission(frame, raw_waypoints)
        self._mission_signature = signature
        self._waypoint_frame = frame
        self._gps_error_detected = gps_error

        self._waypoints = mission
        self._landing_hold_sp = self._waypoints[-1]

        planner_type, max_step, mpc_cfg = self._read_trajectory_config_from_params()
        self._trajectory_planner.update_config(
            mode=planner_type,
            max_step_m=max_step,
            mpc_cfg=mpc_cfg,
        )
        self._trajectory_planner.reset_segment()

        if apply_inflight:
            self._wp_idx = 0
            self._wp_progress_cnt = 0
            if self._state in (self._TRANSITION_TO_FW, self._NAVIGATE, self._TRANSITION_TO_MC):
                self._state = self._NAVIGATE

        if self._gps_error_detected:
            self.get_logger().warn('GPS waypoint 일부/전체가 유효하지 않습니다. 미션을 보정하여 로드했습니다.')

        self.get_logger().info(
            f'미션 로드 완료 (frame={self._waypoint_frame}, count={len(self._waypoints)}, inflight={apply_inflight})',
        )
        return True

    def _sync_runtime_updates(self) -> None:
        if not self._dynamic_waypoint_update:
            return
        if (self._loop_cnt - self._last_reload_loop) < self._dynamic_reload_cycles:
            return

        self._last_reload_loop = self._loop_cnt
        changed = self._reload_mission_from_params(force=False, apply_inflight=True)
        if not changed:
            planner_type, max_step, mpc_cfg = self._read_trajectory_config_from_params()
            self._trajectory_planner.update_config(
                mode=planner_type,
                max_step_m=max_step,
                mpc_cfg=mpc_cfg,
            )

    # ── 제어 루프/헬스체크 ─────────────────────────────────────────

    def _control_loop(self) -> None:
        self._loop_cnt += 1
        self._sync_runtime_updates()

        # SITL에서는 preflight 제약으로 간헐적 자동 disarm이 발생할 수 있어 arm 유지 재시도
        if self._use_sim and self._state not in (self._IDLE, self._DONE, self._LANDING_CONFIRM):
            self._arm_keepalive_cnt += 1
            if self._status.arming_state != VehicleStatus.ARMING_STATE_ARMED and self._arm_keepalive_cnt % 20 == 0:
                self._cmd_arm()
                self._cmd_set_offboard_mode()
                self.get_logger().warn(
                    f'ARM keepalive 재시도 (state={self._state}, arming_state={self._status.arming_state})',
                )

        if self._check_faults_and_enter_failsafe():
            return

        handler = self._state_handlers.get(self._state)
        if handler is None:
            self.get_logger().error(f'알 수 없는 상태: {self._state}. FAILSAFE_LAND로 전환')
            self._enter_failsafe('unknown_state')
            return
        handler()

    def _check_faults_and_enter_failsafe(self) -> bool:
        if self._state in (self._DONE, self._LAND, self._LANDING_CONFIRM, self._FAILSAFE_LAND):
            return False

        if self._gps_error_detected and self._gps_error_failsafe and self._state != self._IDLE:
            self._enter_failsafe('gps_error')
            return True

        status_stale = self._seen_status and (
            (self._loop_cnt - self._last_status_loop) > self._comm_loss_timeout_cycles
        )
        local_pos_stale = self._seen_local_pos and (
            (self._loop_cnt - self._last_local_pos_loop) > self._comm_loss_timeout_cycles
        )

        # 단일 토픽 일시 끊김(특히 status)에는 즉시 failsafe 진입하지 않고,
        # 핵심 텔레메트리(status+local_position) 모두가 끊겼을 때만 진입한다.
        if status_stale and local_pos_stale:
            self._enter_failsafe('telemetry_timeout')
            return True

        return False

    def _enter_failsafe(self, reason: str) -> None:
        self._failsafe_reason = reason
        self._failsafe_cmd_sent = False
        self._state = self._FAILSAFE_LAND
        self.get_logger().error(f'FAILSAFE 진입: {reason}')

    # ── 상태 핸들러 ───────────────────────────────────────────────

    def _handle_idle(self) -> None:
        self._send_offboard_mode()
        self._send_setpoint(0.0, 0.0, self._takeoff_z)
        self._pre_arm_cnt += 1
        if self._pre_arm_cnt >= 10:
            self._cmd_arm()
            self._cmd_set_offboard_mode()
            self._arming_retry_cnt = 0
            self._state = self._ARMING
            self.get_logger().info('→ ARMING')

    def _handle_arming(self) -> None:
        self._send_offboard_mode()
        self._send_setpoint(0.0, 0.0, self._takeoff_z)
        if self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self._takeoff_hold_cnt = 0
            self._state = self._TAKEOFF
            self.get_logger().info('→ TAKEOFF')
            return

        # PX4 preflight 조건이 늦게 충족되는 경우를 대비해 ARM/OFFBOARD를 주기 재시도
        self._arming_retry_cnt += 1
        if self._arming_retry_cnt % 20 == 0:  # 2초 간격
            self._cmd_arm()
            self._cmd_set_offboard_mode()
            self.get_logger().warn(
                'ARMING 재시도: 아직 ARMED 아님 '
                f'(arming_state={self._status.arming_state}, '
                f'pre_flight_checks_pass={getattr(self._status, "pre_flight_checks_pass", False)})',
            )

    def _handle_takeoff(self) -> None:
        self._send_offboard_mode()
        self._send_setpoint(0.0, 0.0, self._takeoff_z)
        self._takeoff_hold_cnt += 1
        reached_takeoff = self._reached(0.0, 0.0, self._takeoff_z, thr=1.0)
        timeout_takeoff = self._takeoff_hold_cnt >= self._takeoff_timeout_cycles
        if reached_takeoff or timeout_takeoff:
            if timeout_takeoff and not reached_takeoff:
                self.get_logger().warn('TAKEOFF 고도 도달 확인 타임아웃. 고정익 전환을 시도합니다.')
            self._cmd_vtol_to_fw()
            self._transition_wait_cnt = 0
            self._state = self._TRANSITION_TO_FW
            self.get_logger().info('→ TRANSITION_TO_FW')

    def _handle_transition_to_fw(self) -> None:
        hold_sp = self._waypoints[min(self._wp_idx, len(self._waypoints) - 1)]
        self._send_offboard_mode()
        self._send_setpoint(*hold_sp)
        self._transition_wait_cnt += 1

        if self._is_fixed_wing_mode() or self._transition_wait_cnt >= self._transition_timeout_cycles:
            if self._transition_wait_cnt >= self._transition_timeout_cycles:
                self.get_logger().warn('고정익 전환 확인 타임아웃. NAVIGATE로 진행합니다.')
            self._wp_progress_cnt = 0
            self._trajectory_planner.reset_segment()
            self._state = self._NAVIGATE
            self.get_logger().info(f'→ NAVIGATE  WP {self._wp_idx} / {len(self._waypoints)}')

    def _advance_to_next_wp(self, current_wp: tuple[float, float, float]) -> None:
        """wp_idx 를 증가시키고 다음 상태(NAVIGATE 또는 TRANSITION_TO_MC)로 전이합니다."""
        self._wp_idx += 1
        if self._wp_idx >= len(self._waypoints):
            self._landing_hold_sp = current_wp
            self._cmd_vtol_to_mc()
            self._transition_wait_cnt = 0
            self._state = self._TRANSITION_TO_MC
            self.get_logger().info('모든 웨이포인트 완료 → TRANSITION_TO_MC')
        else:
            self.get_logger().info(f'→ WP {self._wp_idx}')

    def _handle_navigate(self) -> None:
        wp = self._waypoints[self._wp_idx]
        cur = (
            float(getattr(self._local_pos, 'x', 0.0)),
            float(getattr(self._local_pos, 'y', 0.0)),
            float(getattr(self._local_pos, 'z', self._takeoff_z)),
        )
        sp = self._trajectory_planner.next_setpoint(cur, wp)

        self._send_offboard_mode()
        self._send_setpoint(*sp)

        if self._reached(*wp, thr=self._wp_reached_thr):
            self._wp_progress_cnt = 0
            self._trajectory_planner.reset_segment()
            if self._mission_executor is not None:
                # WP 정상 도달 → 임무 실행 후 다음 WP로
                self._mission_executor.reset()
                self._state = self._MISSION_EXEC
                self.get_logger().info(f'WP {self._wp_idx} 도달 → MISSION_EXEC')
                return
            self._advance_to_next_wp(wp)
            return

        self._wp_progress_cnt += 1
        if self._wp_progress_cnt >= self._wp_unreachable_timeout_cycles:
            self._wp_progress_cnt = 0
            self.get_logger().warn(f'WP {self._wp_idx} 도달 타임아웃. 다음 WP로 스킵합니다.')
            self._trajectory_planner.reset_segment()
            # 타임아웃 스킵은 임무 미실행 (정상 도달 아님)
            self._advance_to_next_wp(wp)

    def _handle_transition_to_mc(self) -> None:
        self._send_offboard_mode()
        self._send_setpoint(*self._landing_hold_sp)
        self._transition_wait_cnt += 1

        if self._is_multicopter_mode() or self._transition_wait_cnt >= self._transition_timeout_cycles:
            if self._transition_wait_cnt >= self._transition_timeout_cycles:
                self.get_logger().warn('멀티콥터 전환 확인 타임아웃. 착륙 절차로 진행합니다.')
            if self._precision_landing_ctrl is not None:
                self._precision_landing_ctrl.reset()
                self._state = self._PRECISION_LAND
                self.get_logger().info('→ PRECISION_LAND')
            else:
                self._state = self._LAND
                self.get_logger().info('→ LAND')

    def _handle_land(self) -> None:
        self._cmd_land()
        self._landing_confirm_cnt = 0
        self._state = self._LANDING_CONFIRM
        self.get_logger().info('→ LANDING_CONFIRM')

    def _handle_landing_confirm(self) -> None:
        if self._is_landed():
            self._landing_confirm_cnt += 1
            if self._landing_confirm_cnt >= self._landing_confirm_required_cycles:
                self._state = self._DONE
                self.get_logger().info('→ DONE')
        else:
            self._landing_confirm_cnt = 0

    def _handle_failsafe_land(self) -> None:
        if not self._failsafe_cmd_sent:
            self._cmd_land()
            self._failsafe_cmd_sent = True
            self.get_logger().error(f'FAILSAFE LAND 명령 전송 완료 (reason={self._failsafe_reason})')
        self._landing_confirm_cnt = 0
        self._state = self._LANDING_CONFIRM

    def _handle_precision_land(self) -> None:
        """정밀 착륙 단계. 컨트롤러 미설정 시 LAND 로 즉시 전환 (pass-through).
        실제 구현은 BasePrecisionLandingController 를 상속하여 주입합니다."""
        if self._precision_landing_ctrl is None:
            self._state = self._LAND
            self.get_logger().info('→ LAND (정밀 착륙 컨트롤러 없음, 건너뜀)')
            return
        sp = self._precision_landing_ctrl.tick(self._local_pos)
        if sp is not None:
            self._send_offboard_mode()
            self._send_setpoint(*sp)
        if self._precision_landing_ctrl.is_complete():
            self._state = self._LAND
            self.get_logger().info('→ LAND (정밀 착륙 완료)')

    def _handle_mission_exec(self) -> None:
        """웨이포인트 임무 실행 단계. 실행기 미설정 또는 완료 시 다음 WP 로 즉시 전환 (pass-through).
        실제 구현은 BaseMissionExecutor 를 상속하여 주입합니다."""
        wp = self._waypoints[self._wp_idx]
        self._send_offboard_mode()
        self._send_setpoint(*wp)   # 현재 WP 위치 홀드
        if self._mission_executor is None or self._mission_executor.is_complete():
            self._advance_to_next_wp(wp)
            return
        self._mission_executor.tick(self._wp_idx, wp, self._local_pos)

    def _handle_done(self) -> None:
        return

    # ── 판정 ─────────────────────────────────────────────────────

    def _reached(self, x: float, y: float, z: float, thr: float = 2.0) -> bool:
        p = self._local_pos
        return math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2) < thr

    def _is_landed(self) -> bool:
        landed = getattr(self._land_detected, 'landed', None)
        if isinstance(landed, bool):
            return landed

        arming_state = getattr(self._status, 'arming_state', None)
        if arming_state == getattr(VehicleStatus, 'ARMING_STATE_STANDBY', None):
            return True

        if not self._use_kinematic_landing_fallback:
            return False

        z = abs(getattr(self._local_pos, 'z', 999.0))
        vx = getattr(self._local_pos, 'vx', 999.0)
        vy = getattr(self._local_pos, 'vy', 999.0)
        vz = getattr(self._local_pos, 'vz', 999.0)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        return z <= self._landing_alt_thr and speed <= self._landing_speed_thr

    def _is_fixed_wing_mode(self) -> bool:
        vehicle_type = getattr(self._status, 'vehicle_type', None)
        fw_type = getattr(VehicleStatus, 'VEHICLE_TYPE_FIXED_WING', None)
        return vehicle_type is not None and fw_type is not None and vehicle_type == fw_type

    def _is_multicopter_mode(self) -> bool:
        vehicle_type = getattr(self._status, 'vehicle_type', None)
        mc_type = getattr(VehicleStatus, 'VEHICLE_TYPE_ROTARY_WING', None)
        return vehicle_type is not None and mc_type is not None and vehicle_type == mc_type

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
        # NED 기준 z는 음수일수록 고도가 높다. 최대고도(max_altitude_m) 초과 상승 방지.
        z_clamped = max(z, self._max_alt_z)
        msg.position = [x, y, z_clamped]
        msg.yaw = yaw
        msg.timestamp = self._us_now()
        self._pub_setpoint.publish(msg)

    # ── VehicleCommand ────────────────────────────────────────────

    def _cmd_arm(self) -> None:
        # SITL 데모 환경에서는 preflight 미통과 시에도 강제 arm을 허용해 자동 비행을 진행
        force_code = 21196.0 if self._use_sim else 0.0
        self._send_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=force_code,
        )

    def _cmd_set_offboard_mode(self) -> None:
        self._send_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def _cmd_vtol_to_fw(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)

    def _cmd_vtol_to_mc(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)

    def _cmd_land(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _send_cmd(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
    ) -> None:
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
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
