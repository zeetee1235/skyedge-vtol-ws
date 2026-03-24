"""
웨이포인트 도달 후 임무 실행기 추상 인터페이스.

구현 예시:
    - 그리퍼 개폐 (투하/수거)
    - 특정 좌표 정밀 호버링 후 촬영
    - 조명/페이로드 제어

사용 방법:
    1. BaseMissionExecutor 를 상속하여 구현
    2. WaypointNavNode._mission_executor 에 인스턴스 할당

    node = WaypointNavNode()
    node._mission_executor = MyMissionExecutor(...)

상태 머신 연동:
    NAVIGATE (WP 도달)
        → executor is not None: MISSION_EXEC (reset() 호출 후 진입)
        → executor is None:     다음 WP 또는 TRANSITION_TO_MC 로 즉시 전진
    MISSION_EXEC
        → is_complete() == True: 다음 WP 또는 TRANSITION_TO_MC

주의:
    - WP 도달 타임아웃 스킵(unreachable skip) 시에는 MISSION_EXEC 에 진입하지 않습니다.
      정상 도달 시에만 임무가 실행됩니다.
    - tick() 은 매 10 Hz 루프에서 호출됩니다. 블로킹 작업은 별도 스레드로 처리하세요.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from px4_msgs.msg import VehicleLocalPosition


class BaseMissionExecutor(ABC):
    """
    웨이포인트 임무 실행기 인터페이스.

    WP 도달 시 reset() 이 한 번 호출되고,
    이후 매 제어 루프(10 Hz)마다 tick() → is_complete() 순으로 호출됩니다.
    is_complete() 가 True 를 반환하면 노드는 다음 WP 또는 TRANSITION_TO_MC 로 전이합니다.
    """

    @abstractmethod
    def reset(self) -> None:
        """
        MISSION_EXEC 상태 진입 시 1회 호출.
        내부 상태(타이머, 완료 플래그, 하드웨어 상태 등)를 초기화합니다.
        """

    @abstractmethod
    def tick(
        self,
        waypoint_idx: int,
        waypoint: tuple[float, float, float],
        local_pos: 'VehicleLocalPosition',
    ) -> None:
        """
        매 제어 루프에서 호출. 임무 로직을 1 스텝 진행합니다.

        Args:
            waypoint_idx : 현재 도달한 WP 인덱스 (0-based).
            waypoint     : 현재 WP 좌표 (north_m, east_m, down_m).
            local_pos    : 현재 기체 위치·속도 정보.

        구현 예:
            그리퍼 개폐 명령 퍼블리시, 타이머 경과 확인.
        """

    @abstractmethod
    def is_complete(self) -> bool:
        """
        임무 완료 여부.
        True 반환 시 노드가 다음 WP 또는 착륙 절차로 전이합니다.

        완료 기준 예시:
            - 그리퍼 닫힘 확인 + 1 s 대기
            - 촬영 완료 토픽 수신
            - 타임아웃 초과 (안전 폴백)
        """
