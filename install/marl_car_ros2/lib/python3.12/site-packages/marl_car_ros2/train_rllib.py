from __future__ import annotations

from ray import tune
from ray.rllib.algorithms.ppo import PPOConfig
from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
from ray.tune.registry import register_env

from .pettingzoo_adapter import Ros2CarParallelEnv


def env_creator(env_config):
    max_steps = int(env_config.get("max_steps", 1200))
    return ParallelPettingZooEnv(Ros2CarParallelEnv(max_steps=max_steps))


def main() -> None:
    env_name = "ros2_car_parallel_env"
    register_env(env_name, env_creator)

    cfg = (
        PPOConfig()
        .environment(env=env_name, env_config={"max_steps": 800})
        .framework("torch")
        .rollouts(num_rollout_workers=0)
        .resources(num_gpus=0)
        .multi_agent(
            policies={"planner", "chassis"},
            policy_mapping_fn=lambda agent_id, *args, **kwargs: agent_id,
        )
    )

    tune.run(
        "PPO",
        name="marl_car_ppo",
        stop={"training_iteration": 5},
        config=cfg.to_dict(),
        checkpoint_at_end=True,
    )

