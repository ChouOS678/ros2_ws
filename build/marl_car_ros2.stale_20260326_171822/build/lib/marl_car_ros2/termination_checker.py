from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from .observation_builder import ObservationSnapshot


@dataclass
class TerminationConfig:
    collision_range: float = 0.22
    goal_tolerance: float = 0.25
    stuck_speed_eps: float = 0.03
    stuck_steps_limit: int = 30
    stuck_goal_dist_min: float = 0.5


class TerminationChecker:
    """
    Training/evaluation utility.
    Runtime control path must not depend on this module.
    """

    def __init__(self, cfg: TerminationConfig) -> None:
        self.cfg = cfg
        self._stuck_counter = 0

    def reset(self) -> None:
        self._stuck_counter = 0

    def evaluate(self, *, snapshot: ObservationSnapshot) -> Tuple[bool, Dict[str, float]]:
        done = False
        info: Dict[str, float] = {
            "goal_dist": float(snapshot.goal_dist),
            "min_range": float(snapshot.min_range),
            "stuck": 0.0,
        }

        if snapshot.min_range <= self.cfg.collision_range:
            done = True
            info["termination"] = "collision"
        elif snapshot.goal_dist < self.cfg.goal_tolerance:
            done = True
            info["termination"] = "goal_reached"

        if abs(snapshot.linear_speed) < self.cfg.stuck_speed_eps and snapshot.goal_dist > self.cfg.stuck_goal_dist_min:
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0

        if self._stuck_counter >= self.cfg.stuck_steps_limit:
            info["stuck"] = 1.0

        return done, info
