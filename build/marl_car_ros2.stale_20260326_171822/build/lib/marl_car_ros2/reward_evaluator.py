from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from .observation_builder import ObservationSnapshot


@dataclass
class RewardConfig:
    step_penalty_scale: float = -1.0
    progress_scale: float = 4.0
    accel_penalty_scale: float = -0.3
    clearance_bonus_scale: float = 0.2
    max_scan_range: float = 8.0


class RewardEvaluator:
    """
    Training/evaluation utility.
    Runtime control path must not depend on this module.
    """

    def __init__(self, cfg: RewardConfig) -> None:
        self.cfg = cfg
        self._prev_goal_dist = None
        self._prev_exec_speed = 0.0

    def reset(self) -> None:
        self._prev_goal_dist = None
        self._prev_exec_speed = 0.0

    def evaluate(self, *, snapshot: ObservationSnapshot, dt: float) -> Dict[str, float]:
        progress = 0.0
        if self._prev_goal_dist is not None:
            progress = self._prev_goal_dist - snapshot.goal_dist
        self._prev_goal_dist = snapshot.goal_dist

        planner_reward = (self.cfg.step_penalty_scale * dt) + (self.cfg.progress_scale * progress)

        accel = (snapshot.linear_speed - self._prev_exec_speed) / max(dt, 1e-3)
        self._prev_exec_speed = snapshot.linear_speed
        smooth_penalty = self.cfg.accel_penalty_scale * abs(accel)
        clearance_bonus = self.cfg.clearance_bonus_scale * min(
            snapshot.min_range / max(self.cfg.max_scan_range, 1e-6),
            1.0,
        )
        chassis_reward = smooth_penalty + clearance_bonus
        return {"planner": float(planner_reward), "chassis": float(chassis_reward)}
