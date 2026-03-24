from __future__ import annotations

from .base import BaseTrajectoryPlanner
from .mpc_planner import MPCPlanner
from .simple import LinearPlanner, PointJumpPlanner, SmoothstepPlanner


def create_trajectory_planner(
    planner_type: str,
    max_step_m: float,
    mpc_cfg: dict[str, float] | None = None,
) -> BaseTrajectoryPlanner:
    t = planner_type.lower().strip()
    if t == 'point_jump':
        return PointJumpPlanner()
    if t == 'linear':
        return LinearPlanner(max_step_m=max_step_m)
    if t == 'mpc':
        cfg = mpc_cfg or {}
        return MPCPlanner(
            max_step_m=max_step_m,
            horizon_steps=int(cfg.get('horizon_steps', 15)),
            dt=float(cfg.get('dt', 0.1)),
            w_pos=float(cfg.get('w_pos', 1.0)),
            w_vel=float(cfg.get('w_vel', 0.2)),
            w_u=float(cfg.get('w_u', 0.05)),
        )
    return SmoothstepPlanner(max_step_m=max_step_m)
