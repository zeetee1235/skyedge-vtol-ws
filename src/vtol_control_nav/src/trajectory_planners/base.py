from __future__ import annotations

from abc import ABC, abstractmethod


class BaseTrajectoryPlanner(ABC):
    @abstractmethod
    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        raise NotImplementedError

    def update_config(self, **kwargs) -> None:
        return

    def reset_segment(self) -> None:
        return
