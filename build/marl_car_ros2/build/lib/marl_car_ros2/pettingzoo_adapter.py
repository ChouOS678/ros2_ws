from __future__ import annotations

from typing import Dict, Optional

import gymnasium as gym
import numpy as np
import rclpy
from pettingzoo import ParallelEnv

from .marl_env_wrapper import JointAction, Ros2MarlEnvWrapper


class Ros2CarParallelEnv(ParallelEnv):
    metadata = {"name": "ros2_car_parallel_v0"}

    def __init__(self, max_steps: int = 1200) -> None:
        super().__init__()
        self.max_steps = max_steps
        self.possible_agents = ["planner", "chassis"]
        self.agents = self.possible_agents[:]
        self._step_count = 0

        if not rclpy.ok():
            rclpy.init()
        self.wrapper_node = Ros2MarlEnvWrapper()

        obs_dim = 6 + self.wrapper_node.scan_bins
        self._obs_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
        self._planner_action_space = gym.spaces.Box(
            low=np.array([0.0], dtype=np.float32),
            high=np.array([self.wrapper_node.max_speed], dtype=np.float32),
            dtype=np.float32,
        )
        self._chassis_action_space = gym.spaces.Box(
            low=np.array([-self.wrapper_node.max_omega], dtype=np.float32),
            high=np.array([self.wrapper_node.max_omega], dtype=np.float32),
            dtype=np.float32,
        )

    def observation_space(self, agent: str):
        return self._obs_space

    def action_space(self, agent: str):
        if agent == "planner":
            return self._planner_action_space
        if agent == "chassis":
            return self._chassis_action_space
        raise KeyError(f"unknown agent: {agent}")

    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None):
        del seed, options
        self.agents = self.possible_agents[:]
        self._step_count = 0
        obs = self.wrapper_node.reset()
        observations = {a: obs.copy() for a in self.agents}
        infos = {a: {} for a in self.agents}
        return observations, infos

    def step(self, actions: Dict[str, np.ndarray]):
        self._step_count += 1
        planner_act = actions.get("planner", np.array([0.0], dtype=np.float32))
        chassis_act = actions.get("chassis", np.array([0.0], dtype=np.float32))

        joint = JointAction(
            planner_target_speed=float(np.asarray(planner_act).reshape(-1)[0]),
            chassis_target_omega=float(np.asarray(chassis_act).reshape(-1)[0]),
        )

        rclpy.spin_once(self.wrapper_node, timeout_sec=0.0)
        obs, reward_dict, done, info = self.wrapper_node.step(joint)

        observations = {a: obs.copy() for a in self.agents}
        rewards = {"planner": reward_dict["planner"], "chassis": reward_dict["chassis"]}
        terminations = {a: done for a in self.agents}
        trunc = self._step_count >= self.max_steps
        truncations = {a: trunc for a in self.agents}
        infos = {a: dict(info) for a in self.agents}
        if done or trunc:
            self.agents = []
        return observations, rewards, terminations, truncations, infos

    def close(self):
        if self.wrapper_node is not None:
            self.wrapper_node.destroy_node()
            self.wrapper_node = None
        if rclpy.ok():
            rclpy.shutdown()

