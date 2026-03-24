"""
정밀 착륙 컨트롤러 추상 인터페이스.

구현 예시:
    - ArUco 마커 기반 정밀 착륙 (vtol.aruco_marker_id 파라미터 참조)
    - 비전 기반 랜딩 패드 추적

사용 방법:
    1. BasePrecisionLandingController 를 상속하여 구현
    2. WaypointNavNode._precision_landing_ctrl 에 인스턴스 할당

    node = WaypointNavNode()
    node._precision_landing_ctrl = MyPrecisionLander(...)

상태 머신 연동:
    TRANSITION_TO_MC 완료
        → ctrl is not None: PRECISION_LAND (매 루프 tick() 호출)
        → ctrl is None:     LAND (바로 착륙 명령)
    PRECISION_LAND
        → is_complete() == True: LAND
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from px4_msgs.msg import VehicleLocalPosition


class BasePrecisionLandingController(ABC):
    """
    정밀 착륙 컨트롤러 인터페이스.

    PRECISION_LAND 상태 진입 시 reset() 이 한 번 호출되고,
    이후 매 제어 루프(10 Hz)마다 tick() → is_complete() 순으로 호출됩니다.
    is_complete() 가 True 를 반환하면 노드는 LAND 상태로 전이합니다.
    """

    @abstractmethod
    def reset(self) -> None:
        """
        PRECISION_LAND 상태 진입 시 1회 호출.
        내부 상태(추적 오프셋, 타임아웃 카운터 등)를 초기화합니다.
        """

    @abstractmethod
    def tick(self, local_pos: 'VehicleLocalPosition') -> tuple[float, float, float] | None:
        """
        매 제어 루프에서 호출. 다음 위치 셋포인트를 반환합니다.

        반환값:
            (north_m, east_m, down_m) — NED 위치 셋포인트.
            None — 셋포인트 없음 (예: 외부 제어 위임, 착륙 명령만 사용).

        구현 예:
            비전 피드백으로 착륙 지점 중심을 추적하며 서서히 하강.
        """

    @abstractmethod
    def is_complete(self) -> bool:
        """
        정밀 착륙 완료 여부.
        True 반환 시 노드가 LAND 로 전이합니다.

        완료 기준 예시:
            - 마커 중심에서 수평 오차 < 0.3 m 이면서 고도 < 2.0 m
            - 타임아웃 초과 (안전 폴백)
        """
