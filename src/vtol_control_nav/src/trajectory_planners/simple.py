from __future__ import annotations

import math

from .base import BaseTrajectoryPlanner


def distance3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def move_toward(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    max_step: float,
) -> tuple[float, float, float]:
    dist = distance3(current, target)
    if dist < 1e-6 or max_step <= 0.0:
        return target
    if dist <= max_step:
        return target

    scale = max_step / dist
    return (
        current[0] + (target[0] - current[0]) * scale,
        current[1] + (target[1] - current[1]) * scale,
        current[2] + (target[2] - current[2]) * scale,
    )


class PointJumpPlanner(BaseTrajectoryPlanner):
    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return target


class LinearPlanner(BaseTrajectoryPlanner):
    def __init__(self, max_step_m: float):
        self._max_step_m = max(0.1, max_step_m)

    def update_config(self, **kwargs) -> None:
        self._max_step_m = max(0.1, float(kwargs.get('max_step_m', self._max_step_m)))

    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return move_toward(current, target, self._max_step_m)


class SmoothstepPlanner(BaseTrajectoryPlanner):
    def __init__(self, max_step_m: float):
        self._max_step_m = max(0.1, max_step_m)
        self._seg_start: tuple[float, float, float] | None = None
        self._seg_target: tuple[float, float, float] | None = None
        self._seg_len = 1.0

    def update_config(self, **kwargs) -> None:
        self._max_step_m = max(0.1, float(kwargs.get('max_step_m', self._max_step_m)))

    def reset_segment(self) -> None:
        self._seg_start = None
        self._seg_target = None
        self._seg_len = 1.0

    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if self._seg_target != target or self._seg_start is None:
            self._seg_start = current
            self._seg_target = target
            self._seg_len = max(distance3(self._seg_start, self._seg_target), 1e-6)

        dist_to_target = distance3(current, target)
        progress = max(0.0, min(1.0, 1.0 - (dist_to_target / self._seg_len)))
        s = progress * progress * (3.0 - 2.0 * progress)

        smooth = (
            self._seg_start[0] + (target[0] - self._seg_start[0]) * s,
            self._seg_start[1] + (target[1] - self._seg_start[1]) * s,
            self._seg_start[2] + (target[2] - self._seg_start[2]) * s,
        )
        return move_toward(current, smooth, self._max_step_m)
